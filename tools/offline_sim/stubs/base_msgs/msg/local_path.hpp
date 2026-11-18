#pragma once
#include <vector>
#include "base_msgs/msg/local_path_point.hpp"
namespace base_msgs::msg { struct LocalPath { std_msgs::msg::Header header; std::vector<LocalPathPoint> local_path; }; }
