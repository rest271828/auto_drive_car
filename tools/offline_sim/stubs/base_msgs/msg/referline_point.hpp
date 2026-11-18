#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace base_msgs::msg { struct ReferlinePoint { geometry_msgs::msg::PoseStamped pose; double rs = 0, rtheta = 0, rkappa = 0, rdkappa = 0; }; }
