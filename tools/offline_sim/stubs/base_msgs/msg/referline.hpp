#pragma once
#include <vector>
#include "base_msgs/msg/referline_point.hpp"
namespace base_msgs::msg { struct Referline { std_msgs::msg::Header header; std::vector<ReferlinePoint> refer_line; }; }
