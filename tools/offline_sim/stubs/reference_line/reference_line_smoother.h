// 离线仿真: 用空实现替代 OSQP 平滑(本机没有 OsqpEigen)
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
namespace Planning
{
    class ReferenceLineSmoother
    {
    public:
        void smooth_reference_line(base_msgs::msg::Referline &) {}
    };
}
