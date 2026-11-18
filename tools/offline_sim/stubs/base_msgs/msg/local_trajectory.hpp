#pragma once
#include <vector>
#include "base_msgs/msg/local_trajectory_point.hpp"
namespace base_msgs::msg { struct LocalTrajectory { std_msgs::msg::Header header; std::vector<LocalTrajectoryPoint> local_trajectory; }; }
