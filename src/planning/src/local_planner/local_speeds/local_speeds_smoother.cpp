#include "local_speeds_smoother.h"

namespace Planning
{
    LocalSpeedsSmoother::LocalSpeedsSmoother() // 速度平滑器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_smoother created");
    }

    void LocalSpeedsSmoother::smooth_local_speeds(LocalSpeeds &local_speeds)
    {
        (void)local_speeds; // TODO: 基于ST约束的QP速度平滑
    }

} // namespace Planning
