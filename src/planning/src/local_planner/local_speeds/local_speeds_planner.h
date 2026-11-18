#ifndef LOCAL_SPEED_PLANNER_H_
#define LOCAL_SPEED_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "base_msgs/msg/local_speeds_point.hpp"
#include "config_reader.h"
#include "polynomial_curve.h"
#include "decision_center.h"
#include "local_speeds_smoother.h"

#include <memory>
#include <vector>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::LocalSpeeds;
    using base_msgs::msg::LocalSpeedsPoint;

    class LocalSpeedsPlanner // 速度规划器
    {
    public:
        LocalSpeedsPlanner();

        // 在ST约束下, 前向积分生成 s(t), v(t), a(t)
        LocalSpeeds create_local_speeds(const LocalPath &path, const std::vector<double> &path_s,
                                        const std::shared_ptr<VehicleBase> &car,
                                        const std::shared_ptr<DecisionCenter> &decision);

        inline double dt() const { return kPlanningCycle; } // 速度点时间间隔
        inline int speed_size() const { return speed_size_; }

        constexpr static double kMaxDecel = 6.0;     // 最大减速度(m/s^2)
        constexpr static double kMaxLatAccel = 2.5;  // 最大横向加速度(m/s^2), 用于弯道限速

    private:
        std::unique_ptr<ConfigReader> local_speeds_config_;
        std::shared_ptr<LocalSpeedsSmoother> local_speeds_smoother_;
        int speed_size_ = 0;
        LocalSpeeds local_speeds_;
    };
} // namespace Planning
#endif // LOCAL_SPEED_PLANNER_H_
