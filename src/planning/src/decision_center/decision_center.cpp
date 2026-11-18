#include "decision_center.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Planning
{
    DecisionCenter::DecisionCenter()
    {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "decision_center created");

        decision_config_ = std::make_unique<ConfigReader>();
        decision_config_->read_decision_config();

        road_half_width_ = decision_config_->pnc_map().road_half_width_;
        speed_limit_ = decision_config_->pnc_map().speed_limit_ / kPlanningCycle; // m/帧 -> m/s
        safe_dis_l_ = decision_config_->decision().safe_dis_l_;
        safe_dis_s_ = decision_config_->decision().safe_dis_s_;

        // 换道过渡长度: 至少为纵向安全距离, 且不少于2s的行驶距离, 避免横向加速度过大
        transition_length_ = std::max(safe_dis_s_, 2.0 * speed_limit_);
    }

    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car,
                                            const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        bypass_intervals_.clear();

        const double car_half_w = car->width() / 2.0;
        const double corridor = car_half_w + safe_dis_l_; // 本车道中心(l=0)两侧需要的空间
        const double need_room = car->width() + safe_dis_l_; // 绕行一侧至少需要的空间
        const double look_ahead = std::max(3.0 * transition_length_, speed_limit_ * 10.0);

        // obses 已按 s 从小到大排序
        for (const auto &obs : obses)
        {
            // 只对静止障碍物做绕行决策, 动态障碍物交给速度决策(跟车/让行)
            if (std::fabs(obs->speed()) > kStaticSpeed)
            {
                continue;
            }

            const double half_len = obs->half_extent_s() + car->length() / 2.0 + 1.0;
            const double s_begin = obs->s() - half_len;
            const double s_end = obs->s() + half_len;
            if (s_end < car->s() || s_begin > car->s() + look_ahead)
            {
                continue; // 已经驶过, 或太远
            }

            // 是否挡住本车道
            const double obs_left = obs->l() + obs->half_extent_l();
            const double obs_right = obs->l() - obs->half_extent_l();
            if (obs_right >= corridor || obs_left <= -corridor)
            {
                continue;
            }

            // 左右两侧可用空间
            const double left_room = l_max() - obs_left;
            const double right_room = obs_right - l_min();
            const double left_target = std::min(obs_left + safe_dis_l_ + car_half_w, l_max() - car_half_w);
            const double right_target = std::max(obs_right - safe_dis_l_ - car_half_w, l_min() + car_half_w);

            BypassInterval interval;
            interval.obs_id = obs->id();
            interval.s_begin = s_begin;
            interval.s_end = s_end;

            const bool left_ok = left_room >= need_room;
            const bool right_ok = right_room >= need_room;
            if (left_ok && (!right_ok || std::fabs(left_target) <= std::fabs(right_target)))
            {
                interval.target_l = left_target;
            }
            else if (right_ok)
            {
                interval.target_l = right_target;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("decision_center"),
                            "obs %d blocks the road, no room to bypass, stop before it", obs->id());
                continue; // 无法绕行: 速度决策会让主车停在它前面
            }

            // 与上一个区间重叠时合并(同侧取偏移更大者)
            if (!bypass_intervals_.empty() && interval.s_begin - bypass_intervals_.back().s_end < 1.0)
            {
                auto &last = bypass_intervals_.back();
                if (last.target_l * interval.target_l >= 0.0)
                {
                    last.s_end = std::max(last.s_end, interval.s_end);
                    last.target_l = (std::fabs(interval.target_l) > std::fabs(last.target_l)) ? interval.target_l : last.target_l;
                    continue;
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                        "path decision: bypass obs %d on the %s, target_l = %.2f, s: [%.2f, %.2f]",
                        obs->id(), interval.target_l > 0.0 ? "left" : "right", interval.target_l,
                        interval.s_begin, interval.s_end);
            bypass_intervals_.emplace_back(interval);
        }
    }

    void DecisionCenter::make_speed_decision(const LocalPath &path, const std::vector<double> &path_s,
                                             const std::shared_ptr<VehicleBase> &car,
                                             const std::vector<std::shared_ptr<VehicleBase>> &obses,
                                             const double &dt, const int &steps)
    {
        speed_constraints_.clear();
        obs_info_.clear();
        if (path.local_path.size() < 2)
        {
            return;
        }

        // 主车的名义运动(从当前速度以最大加速度加速到限速), 用于判断能否抢行
        auto nominal_time_to_reach = [&](const double &s_target) -> double
        {
            double s = 0.0;
            double v = std::max(0.0, car->speed());
            for (int k = 0; k <= steps; k++)
            {
                if (s >= s_target)
                {
                    return k * dt;
                }
                const double v_next = std::min(speed_limit_, v + kMaxAccel * dt);
                s += 0.5 * (v + v_next) * dt;
                v = v_next;
            }
            return std::numeric_limits<double>::max();
        };

        for (const auto &obs : obses)
        {
            SpeedConstraint constraint;
            constraint.obs_id = obs->id();
            constraint.active.assign(steps + 1, false);
            constraint.s_upper.assign(steps + 1, std::numeric_limits<double>::max());
            constraint.v_obs.assign(steps + 1, 0.0);

            const double corridor = (car->width() + obs->width()) / 2.0 + kCollisionBuffer;
            const double margin = (car->length() + obs->length()) / 2.0 + safe_dis_s_;

            // 障碍物匀速直线预测, 每个时间步投影到路径上
            const double x0 = obs->loc_point().pose.position.x;
            const double y0 = obs->loc_point().pose.position.y;
            const double vx = obs->speed() * std::cos(obs->theta());
            const double vy = obs->speed() * std::sin(obs->theta());

            std::vector<bool> in_path(steps + 1, false);
            std::vector<double> s_pred(steps + 1, 0.0);
            std::vector<double> v_along(steps + 1, 0.0);
            int k_in = -1;
            int k_out = -1;
            double s_min_in = std::numeric_limits<double>::max();
            double s_max_in = -std::numeric_limits<double>::max();
            for (int k = 0; k <= steps; k++)
            {
                const double t = k * dt;
                double s = 0.0, l = 0.0;
                const bool inside = Curve::project_to_path(path, path_s, x0 + vx * t, y0 + vy * t, s, l);
                if (inside && std::fabs(l) < corridor)
                {
                    in_path[k] = true;
                    s_pred[k] = s;
                    // 障碍物速度在路径切向的分量
                    size_t idx = 0;
                    while (idx + 1 < path_s.size() && path_s[idx + 1] < s)
                    {
                        idx++;
                    }
                    v_along[k] = obs->speed() * std::cos(Curve::NormalizeAngle(obs->theta() - path.local_path[idx].theta));
                    if (k_in < 0)
                    {
                        k_in = k;
                    }
                    k_out = k;
                    s_min_in = std::min(s_min_in, s);
                    s_max_in = std::max(s_max_in, s);
                }
            }

            // 填写绘图信息
            ObsInfo info;
            info.obs_length = obs->length();
            info.obs_width = obs->width();
            info.l = obs->l();
            info.s = obs->s();
            info.s_2path = obs->s_2path();
            info.ds_dt_2path = obs->ds_dt_2path();
            info.t_in = (k_in >= 0) ? k_in * dt : -1.0;
            info.t_out = (k_out >= 0) ? k_out * dt : -1.0;
            obs->set_time_param(info.t_in, info.t_out);

            if (k_in < 0)
            {
                constraint.type = SpeedDecisionType::IGNORE; // 不会进入路径
            }
            else
            {
                const bool along_path = std::fabs(obs->speed()) <= kStaticSpeed ||
                                        std::fabs(v_along[k_in]) >= 0.7 * std::fabs(obs->speed());
                if (k_in == 0 || along_path)
                {
                    // 静止或同向/逆向行驶在路径上: 跟车(静止时即停车)
                    constraint.type = SpeedDecisionType::FOLLOW;
                    for (int k = 0; k <= steps; k++)
                    {
                        if (in_path[k])
                        {
                            constraint.active[k] = true;
                            constraint.s_upper[k] = s_pred[k] - margin;
                            constraint.v_obs[k] = std::max(0.0, v_along[k]);
                        }
                    }
                }
                else
                {
                    // 横穿: 能否在它进入前通过?
                    const double pass_margin = (car->length() + obs->length()) / 2.0 + 1.0;
                    const double t_pass = nominal_time_to_reach(s_max_in + pass_margin);
                    if (t_pass + 0.5 < k_in * dt)
                    {
                        constraint.type = SpeedDecisionType::OVERTAKE;
                    }
                    else
                    {
                        constraint.type = SpeedDecisionType::YIELD;
                        for (int k = 0; k <= k_out; k++)
                        {
                            constraint.active[k] = true;
                            constraint.s_upper[k] = s_min_in - margin;
                            constraint.v_obs[k] = 0.0;
                        }
                    }
                }
            }

            if (constraint.type != SpeedDecisionType::IGNORE)
            {
                RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                            "speed decision: obs %d -> %s, t_in = %.1f, t_out = %.1f",
                            obs->id(),
                            constraint.type == SpeedDecisionType::FOLLOW ? "FOLLOW" : (constraint.type == SpeedDecisionType::YIELD ? "YIELD" : "OVERTAKE"),
                            info.t_in, info.t_out);
            }

            obs_info_.emplace_back(info);
            speed_constraints_.emplace_back(constraint);
        }
    }

} // namespace Planning
