#ifndef LOCAL_TRAJECTORY_COMBINER_H_
#define LOCAL_TRAJECTORY_COMBINER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "base_msgs/msg/local_trajectory_point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "config_reader.h"
#include "curve.h"

#include <vector>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::LocalSpeeds;
    using base_msgs::msg::LocalTrajectory;
    using base_msgs::msg::LocalTrajectoryPoint;
    using nav_msgs::msg::Path;

    class LocalTrajectoryCombiner // 轨迹合成器
    {
    public:
        LocalTrajectoryCombiner();

        // 按速度点的 s_2path 在路径上插值, 合成带时间的轨迹
        LocalTrajectory combine_trajectory(const LocalPath &path, const std::vector<double> &path_s,
                                           const LocalSpeeds &speeds);

        Path trajectory_to_rviz() const; // 转换成rviz的Path格式

        // 按时间在轨迹上插值(t 相对轨迹起点); 供规划节点更新主车状态、运动节点驱动主车
        static LocalTrajectoryPoint interpolate_by_time(const LocalTrajectory &trajectory, const double &t);

    private:
        LocalTrajectory local_trajectory_;
    };
} // namespace Planning
#endif // LOCAL_TRAJECTORY_COMBINER_H_
