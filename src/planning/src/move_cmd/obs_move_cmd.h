#ifndef OBS_MOVE_CMD_H_
#define OBS_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "config_reader.h"
#include "obs_car_info.h"

#include <vector>

namespace Planning
{
    using geometry_msgs::msg::TransformStamped;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    // 障碍物运动指令: 障碍物按配置的初速度沿初始航向匀速直线运动, 以tf发布位姿,
    // 并按配置中的长宽发布立方体Marker(挂在各自的tf坐标下), 供rviz显示
    class ObsMoveCmd : public rclcpp::Node
    {
    public:
        ObsMoveCmd();

    private:
        void timer_callback();

    private:
        std::vector<std::shared_ptr<ObsCar>> obses_;
        std::string map_frame_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        int tick_ = 0;
        rclcpp::Time start_time_;
    };
} // namespace Planning
#endif // OBS_MOVE_CMD_H_
