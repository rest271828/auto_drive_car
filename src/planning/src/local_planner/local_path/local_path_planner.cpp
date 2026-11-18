#include "local_path_planner.h"

#include <algorithm>
#include <cmath>

namespace Planning
{
    LocalPathPlanner::LocalPathPlanner() // 局部路径规划器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_planner created");

        local_path_config_ = std::make_unique<ConfigReader>();
        local_path_config_->read_local_path_config();
        local_path_config_->read_local_speeds_config();
        curve_type_ = static_cast<PolynomialType>(std::clamp(local_path_config_->local_path().curve_type_, 0, 2));
        path_size_ = std::max(2, local_path_config_->local_path().path_size_);

        local_path_smoother_ = std::make_shared<LocalPathSmoother>();
    }

    std::vector<LocalPathPlanner::Knot> LocalPathPlanner::build_knots(const std::shared_ptr<VehicleBase> &car,
                                                                      const std::shared_ptr<DecisionCenter> &decision,
                                                                      const double &s_end) const
    {
        const double trans = decision->transition_length();
        const double min_trans = 0.5 * trans; // 已经很靠近障碍物时允许更短的过渡

        std::vector<Knot> knots;
        knots.push_back({car->s(), car->l(), car->dl_ds(), car->ddl_ds()}); // 起点: 主车当前状态

        auto push = [&](const double &s, const double &l)
        {
            if (s > knots.back().s + 0.5)
            {
                knots.push_back({s, l, 0.0, 0.0});
            }
        };

        for (const auto &interval : decision->bypass_intervals())
        {
            if (interval.s_end <= car->s())
            {
                continue;
            }
            const double trans_begin = interval.s_begin - trans;

            if (trans_begin > knots.back().s + 0.5)
            {
                // 距离障碍物还远: 先回到本车道中心(若不在中心), 再保持, 再开始换道
                if (std::fabs(knots.back().l) > 0.05 && knots.back().s + trans < trans_begin)
                {
                    push(knots.back().s + trans, 0.0);
                }
                push(trans_begin, knots.back().s + trans < trans_begin ? 0.0 : knots.back().l);
            }

            // 到达绕行偏移, 并保持到障碍物之后
            // 若已经很靠近障碍物, 过渡长度按剩余横向偏移量缩短; 已在目标偏移上时直接保持
            const double remain_ratio = std::clamp(std::fabs(interval.target_l - knots.back().l) / 1.0, 0.0, 1.0);
            push(std::max(interval.s_begin, knots.back().s + min_trans * remain_ratio), interval.target_l);
            push(interval.s_end, interval.target_l);
        }

        // 最后回到本车道中心
        if (std::fabs(knots.back().l) > 0.05 || knots.size() == 1)
        {
            push(knots.back().s + trans, 0.0);
        }
        // 保证覆盖整段路径
        push(std::max(s_end, knots.back().s + 1.0), 0.0);
        return knots;
    }

    LocalPath LocalPathPlanner::create_local_path(const Referline &refer_line,
                                                  const std::shared_ptr<VehicleBase> &car,
                                                  const std::shared_ptr<DecisionCenter> &decision)
    {
        local_path_.header = refer_line.header;
        local_path_.local_path.clear();
        if (refer_line.refer_line.size() < 2)
        {
            return local_path_;
        }

        // 路径长度: 覆盖速度规划时域内以限速行驶的距离, 但不超过参考线
        const double horizon = local_path_config_->local_speeds().speed_size_ * kPlanningCycle;
        const double desired_len = std::max(30.0, decision->speed_limit() * horizon);
        const double available_len = refer_line.refer_line.back().rs - car->s();
        const double path_len = std::min(desired_len, available_len);
        if (path_len < 1.0)
        {
            RCLCPP_WARN(rclcpp::get_logger("local_path"), "reach the end of reference line");
            return local_path_;
        }
        const double ds = path_len / (path_size_ - 1);

        // 关键点之间用多项式连接
        const auto knots = build_knots(car, decision, car->s() + path_len);
        std::vector<PolynomialCurve> segments(knots.size() - 1);
        for (size_t i = 0; i + 1 < knots.size(); i++)
        {
            segments[i].fit(curve_type_,
                            knots[i].l, knots[i].dl, knots[i].ddl,
                            knots[i + 1].l, knots[i + 1].dl, knots[i + 1].ddl,
                            knots[i + 1].s - knots[i].s);
        }

        size_t seg = 0;
        LocalPathPoint point;
        point.pose.header = refer_line.header;
        for (int i = 0; i < path_size_; i++)
        {
            const double s = car->s() + i * ds;
            while (seg + 1 < segments.size() && s > knots[seg + 1].s)
            {
                seg++;
            }
            const double x = std::clamp(s - knots[seg].s, 0.0, segments[seg].length());
            const double l = segments[seg].value(x);
            const double dl_ds = segments[seg].d1(x);
            const double ddl_ds = segments[seg].d2(x);

            // 参考线上同s处的参考点
            double rx = 0.0, ry = 0.0, rtheta = 0.0, rkappa = 0.0, rdkappa = 0.0;
            Curve::interpolate_refer_point(refer_line, s, rx, ry, rtheta, rkappa, rdkappa);

            // frenet -> 笛卡尔
            double px = 0.0, py = 0.0, theta = 0.0, speed = 0.0, a = 0.0, kappa = 0.0;
            Curve::frenet_to_cartesian(s, 0.0, 0.0, l, dl_ds, ddl_ds,
                                       s, rx, ry, rtheta, rkappa, rdkappa,
                                       px, py, theta, speed, a, kappa);

            point.pose.pose.position.x = px;
            point.pose.pose.position.y = py;
            point.pose.pose.position.z = 0.0;
            Curve::set_pose_yaw(point.pose, theta);
            point.s = s;
            point.l = l;
            point.dl_ds = dl_ds;
            point.ddl_ds = ddl_ds;
            point.theta = theta;
            point.kappa = kappa;
            point.rs = s;
            point.rtheta = rtheta;
            point.rkappa = rkappa;
            point.rdkappa = rdkappa;
            local_path_.local_path.emplace_back(point);
        }

        // 曲率变化率
        const auto path_s = Curve::path_arc_length(local_path_);
        for (size_t i = 0; i < local_path_.local_path.size(); i++)
        {
            const size_t j = (i + 1 < local_path_.local_path.size()) ? i + 1 : i;
            const size_t k = (j == i) ? i - 1 : i;
            const double d = path_s[j] - path_s[k];
            local_path_.local_path[i].dkappa = (d > Curve::kMathEpsilon)
                                                   ? (local_path_.local_path[j].kappa - local_path_.local_path[k].kappa) / d
                                                   : 0.0;
        }

        local_path_smoother_->smooth_local_path(local_path_);
        return local_path_;
    }

    Path LocalPathPlanner::path_to_rviz() const
    {
        Path path;
        path.header = local_path_.header;
        for (const auto &point : local_path_.local_path)
        {
            PoseStamped pose = point.pose;
            pose.header = path.header;
            path.poses.emplace_back(pose);
        }
        return path;
    }

} // namespace Planning
