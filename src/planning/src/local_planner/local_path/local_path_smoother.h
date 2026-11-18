#ifndef LOCAL_PATH_SMOOTHER_H_
#define LOCAL_PATH_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "config_reader.h"


namespace Planning
{
    using base_msgs::msg::LocalPath;

    class LocalPathSmoother // 局部路径平滑器
    {
    public:
        LocalPathSmoother();

        // 目前五次多项式已保证 l, l', l'' 连续, 这里预留给后续的QP平滑(如 piecewise-jerk)
        void smooth_local_path(LocalPath &local_path);

    private:
    };
} // namespace Planning
#endif // LOCAL_PATH_SMOOTHER_H_
