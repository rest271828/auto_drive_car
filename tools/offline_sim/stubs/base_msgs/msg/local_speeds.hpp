#pragma once
#include <vector>
#include "std_msgs/msg/header.hpp"
#include "base_msgs/msg/local_speeds_point.hpp"
namespace base_msgs::msg { struct LocalSpeeds { std_msgs::msg::Header header; std::vector<LocalSpeedsPoint> local_speeds; }; }
