#include "local_path_smoother.h"

namespace Planning
{
    Planning::LocalPathSmoother::LocalPathSmoother()    //局部路径平滑器  
    {
        RCLCPP_INFO(rclcpp::get_logger(""), "LocalSpeedsPlanner initialized");
    }
}