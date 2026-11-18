// 离线仿真用的 rclcpp 极简替身: 只提供日志宏、Logger、Clock/Time
#pragma once
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include "std_msgs/msg/header.hpp"

namespace rclcpp
{
    struct Logger
    {
        std::string name;
    };
    inline Logger get_logger(const std::string &name) { return Logger{name}; }

    inline bool log_enabled(int level)
    {
        static const int min_level = std::getenv("SIM_LOG") ? std::atoi(std::getenv("SIM_LOG")) : 2;
        return level >= min_level;
    }

    struct Time
    {
        double t = 0.0;
        operator builtin_interfaces::msg::Time() const
        {
            builtin_interfaces::msg::Time out;
            out.sec = static_cast<int>(t);
            out.nanosec = static_cast<unsigned>((t - out.sec) * 1e9);
            return out;
        }
        double seconds() const { return t; }
    };

    struct Clock
    {
        Time now() const { return Time{}; }
    };
} // namespace rclcpp

#define SIM_LOG_IMPL(level, tag, logger, ...)                    \
    do                                                           \
    {                                                            \
        if (rclcpp::log_enabled(level))                          \
        {                                                        \
            std::fprintf(stderr, "[%s] [%s] ", tag, (logger).name.c_str()); \
            std::fprintf(stderr, __VA_ARGS__);                   \
            std::fprintf(stderr, "\n");                          \
        }                                                        \
    } while (0)

#define RCLCPP_DEBUG(logger, ...) SIM_LOG_IMPL(0, "DEBUG", logger, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) SIM_LOG_IMPL(1, "INFO", logger, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) SIM_LOG_IMPL(2, "WARN", logger, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) SIM_LOG_IMPL(3, "ERROR", logger, __VA_ARGS__)
