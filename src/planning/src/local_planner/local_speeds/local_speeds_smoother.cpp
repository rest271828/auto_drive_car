#include "local_speeds_smoother.h"

namespace Planning
{
    Planning::LocalSpeedsSmoother::LocalSpeedsSmoother()    //局部速度平滑器  
    {
        RCLCPP_INFO(rclcpp::get_logger(""), "LocalSpeedsPlanner initialized");
    }
}