#pragma once
#include "std_msgs/msg/header.hpp"
namespace geometry_msgs::msg
{
    struct Point { double x = 0.0, y = 0.0, z = 0.0; };
    struct Quaternion { double x = 0.0, y = 0.0, z = 0.0, w = 1.0; };
    struct Pose { Point position; Quaternion orientation; };
    struct PoseStamped { std_msgs::msg::Header header; Pose pose; };
}
