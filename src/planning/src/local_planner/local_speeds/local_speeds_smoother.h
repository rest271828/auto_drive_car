#ifndef LOCAL_SPEED_SMOOTHER_H_
#define LOCAL_SPEED_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::LocalSpeeds;

    class LocalSpeedsSmoother // 速度平滑器
    {
    public:
        LocalSpeedsSmoother();

        // 目前速度规划已限制加速度, 这里预留给后续的QP平滑
        void smooth_local_speeds(LocalSpeeds &local_speeds);

    private:
    };
} // namespace Planning
#endif // LOCAL_SPEED_SMOOTHER_H_
