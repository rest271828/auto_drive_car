#pragma once
#include <string>
namespace builtin_interfaces::msg
{
    struct Time
    {
        int sec = 0;
        unsigned nanosec = 0;
    };
}
namespace std_msgs::msg
{
    struct Header
    {
        builtin_interfaces::msg::Time stamp;
        std::string frame_id;
    };
}
