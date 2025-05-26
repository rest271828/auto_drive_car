#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "global_planner_normal.h"
#include "base_msgs/srv/global_path_service.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"


namespace Planning
{
    using base_msgs::srv::GlobalPathService;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;
    
    class GlobalPathServer : public rclcpp::Node
    {
    public:
    GlobalPathServer();


    private:

    };
}  // namespace Planning
#endif  // GLOBAL_PATH_SERVER_H_
