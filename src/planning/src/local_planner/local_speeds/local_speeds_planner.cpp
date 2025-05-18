#include "local_speeds_planner.h"

namespace Planning
{
    Planning::LocalSpeedsPlanner::LocalSpeedsPlanner()  //局部速度规划器
    {
        RCLCPP_INFO(rclcpp::get_logger(""), "LocalSpeedsPlanner initialized");
    }
}