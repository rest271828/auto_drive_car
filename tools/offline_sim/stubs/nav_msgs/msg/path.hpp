#pragma once
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace nav_msgs::msg
{
    struct Path { std_msgs::msg::Header header; std::vector<geometry_msgs::msg::PoseStamped> poses; };
}
