#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "config_reader.h"
#include "main_car_info.h"
#include "local_trajectory_combiner.h"

namespace Planning
{
    using base_msgs::msg::LocalTrajectory;
    using geometry_msgs::msg::TransformStamped;

    // 主车运动指令: 订阅规划轨迹, 按时间沿轨迹推进主车, 以 map -> base_footprint 的tf发布主车位姿
    class CarMoveCmd : public rclcpp::Node
    {
    public:
        CarMoveCmd();

    private:
        void trajectory_callback(const LocalTrajectory::SharedPtr msg);
        void timer_callback();

    private:
        std::shared_ptr<MainCar> car_;
        std::string map_frame_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<LocalTrajectory>::SharedPtr trajectory_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        LocalTrajectory trajectory_;
        rclcpp::Time trajectory_time_;
        bool has_trajectory_ = false;
    };
} // namespace Planning
#endif // CAR_MOVE_CMD_H_
