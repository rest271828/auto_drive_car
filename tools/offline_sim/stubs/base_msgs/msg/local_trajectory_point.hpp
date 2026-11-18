#pragma once
#include "base_msgs/msg/local_path_point.hpp"
#include "base_msgs/msg/local_speeds_point.hpp"
namespace base_msgs::msg { struct LocalTrajectoryPoint { LocalPathPoint path_point; LocalSpeedsPoint speed_point; }; }
