#include "local_speeds_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Planning
{
    LocalSpeedsPlanner::LocalSpeedsPlanner() // 速度规划器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_planner created");

        local_speeds_config_ = std::make_unique<ConfigReader>();
        local_speeds_config_->read_local_speeds_config();
        speed_size_ = std::max(2, local_speeds_config_->local_speeds().speed_size_);

        local_speeds_smoother_ = std::make_shared<LocalSpeedsSmoother>();
    }

    LocalSpeeds LocalSpeedsPlanner::create_local_speeds(const LocalPath &path, const std::vector<double> &path_s,
                                                        const std::shared_ptr<VehicleBase> &car,
                                                        const std::shared_ptr<DecisionCenter> &decision)
    {
        local_speeds_.header = path.header;
        local_speeds_.local_speeds.clear();
        if (path.local_path.size() < 2 || path_s.size() != path.local_path.size())
        {
            return local_speeds_;
        }

        const double dt = this->dt();
        const double b = DecisionCenter::kComfortDecel;
        const double path_len = path_s.back();

        // 弯道限速: v <= sqrt(a_lat / |kappa|)
        std::vector<double> v_curve(path_s.size());
        for (size_t i = 0; i < path_s.size(); i++)
        {
            const double kappa = std::fabs(path.local_path[i].kappa);
            v_curve[i] = (kappa > 1.0e-4) ? std::sqrt(kMaxLatAccel / kappa) : 1.0e3;
        }

        double s = 0.0;
        double v = std::clamp(car->speed(), 0.0, decision->speed_limit());
        double a_last = car->acceleration();
        const auto &constraints = decision->speed_constraints();

        LocalSpeedsPoint point;
        for (int k = 0; k < speed_size_; k++)
        {
            // 当前时刻允许的最大速度: 道路限速、弯道限速、路径终点、障碍物ST约束
            double v_allow = decision->speed_limit();

            for (size_t i = 0; i < path_s.size(); i++)
            {
                if (path_s[i] >= s)
                {
                    v_allow = std::min(v_allow, std::sqrt(v_curve[i] * v_curve[i] + 2.0 * b * (path_s[i] - s)));
                }
            }

            v_allow = std::min(v_allow, std::sqrt(2.0 * b * std::max(0.0, path_len - s)));

            for (const auto &c : constraints)
            {
                for (int j = k; j < static_cast<int>(c.active.size()); j++)
                {
                    if (!c.active[j])
                    {
                        continue;
                    }
                    const double gap = c.s_upper[j] - s;
                    const double v_c = (gap <= 0.0) ? 0.0 : std::sqrt(c.v_obs[j] * c.v_obs[j] + 2.0 * b * gap);
                    v_allow = std::min(v_allow, v_c);
                }
            }

            const double a = std::clamp((v_allow - v) / dt, -kMaxDecel, DecisionCenter::kMaxAccel);
            const double v_next = std::max(0.0, v + a * dt);

            point.t = k * dt;
            point.s_2path = s;
            point.ds_dt_2path = v;
            point.dds_dt_2path = a;
            point.speed = v;
            point.acceleration = a;
            point.dacceleration = (k == 0) ? (a - a_last) / dt : (a - local_speeds_.local_speeds.back().acceleration) / dt;
            local_speeds_.local_speeds.emplace_back(point);

            s = std::min(path_len, s + 0.5 * (v + v_next) * dt);
            v = v_next;
        }

        local_speeds_smoother_->smooth_local_speeds(local_speeds_);
        return local_speeds_;
    }

} // namespace Planning
