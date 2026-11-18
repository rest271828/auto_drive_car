#include "local_path_smoother.h"

namespace Planning
{
    LocalPathSmoother::LocalPathSmoother() // 局部路径平滑器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_smoother created");
    }

    void LocalPathSmoother::smooth_local_path(LocalPath &local_path)
    {
        (void)local_path; // TODO: piecewise-jerk QP 平滑(复用 OsqpEigen)
    }

} // namespace Planning
