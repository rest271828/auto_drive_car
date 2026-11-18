#include "local_trajectory_combiner.h"

#include <algorithm>
#include <cmath>

namespace Planning
{
    LocalTrajectoryCombiner::LocalTrajectoryCombiner() // 轨迹合成器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_trajectory"), "local_trajectory_combiner created");
    }

    LocalTrajectory LocalTrajectoryCombiner::combine_trajectory(const LocalPath &path, const std::vector<double> &path_s,
                                                                const LocalSpeeds &speeds)
    {
        local_trajectory_.header = path.header;
        local_trajectory_.local_trajectory.clear();
        if (path.local_path.size() < 2 || path_s.size() != path.local_path.size())
        {
            return local_trajectory_;
        }

        LocalTrajectoryPoint point;
        size_t idx = 0;
        for (const auto &speed_point : speeds.local_speeds)
        {
            const double s = std::clamp(speed_point.s_2path, 0.0, path_s.back());
            while (idx + 2 < path_s.size() && path_s[idx + 1] < s)
            {
                idx++;
            }
            const auto &p0 = path.local_path[idx];
            const auto &p1 = path.local_path[idx + 1];
            const double seg = path_s[idx + 1] - path_s[idx];
            const double r = (seg > Curve::kMathEpsilon) ? std::clamp((s - path_s[idx]) / seg, 0.0, 1.0) : 0.0;

            point.path_point = p0;
            point.path_point.pose.pose.position.x = p0.pose.pose.position.x + r * (p1.pose.pose.position.x - p0.pose.pose.position.x);
            point.path_point.pose.pose.position.y = p0.pose.pose.position.y + r * (p1.pose.pose.position.y - p0.pose.pose.position.y);
            point.path_point.theta = Curve::NormalizeAngle(p0.theta + r * Curve::NormalizeAngle(p1.theta - p0.theta));
            Curve::set_pose_yaw(point.path_point.pose, point.path_point.theta);
            point.path_point.kappa = p0.kappa + r * (p1.kappa - p0.kappa);
            point.path_point.dkappa = p0.dkappa + r * (p1.dkappa - p0.dkappa);
            point.path_point.s = p0.s + r * (p1.s - p0.s);
            point.path_point.l = p0.l + r * (p1.l - p0.l);
            point.path_point.dl_ds = p0.dl_ds + r * (p1.dl_ds - p0.dl_ds);
            point.path_point.ddl_ds = p0.ddl_ds + r * (p1.ddl_ds - p0.ddl_ds);
            point.path_point.ds_dt = speed_point.speed;
            point.path_point.dds_dt = speed_point.acceleration;
            point.speed_point = speed_point;
            local_trajectory_.local_trajectory.emplace_back(point);
        }
        return local_trajectory_;
    }

    Path LocalTrajectoryCombiner::trajectory_to_rviz() const
    {
        Path path;
        path.header = local_trajectory_.header;
        for (const auto &point : local_trajectory_.local_trajectory)
        {
            PoseStamped pose = point.path_point.pose;
            pose.header = path.header;
            path.poses.emplace_back(pose);
        }
        return path;
    }

    LocalTrajectoryPoint LocalTrajectoryCombiner::interpolate_by_time(const LocalTrajectory &trajectory, const double &t)
    {
        const auto &pts = trajectory.local_trajectory;
        if (pts.empty())
        {
            return LocalTrajectoryPoint();
        }
        if (t <= pts.front().speed_point.t)
        {
            return pts.front();
        }
        if (t >= pts.back().speed_point.t)
        {
            return pts.back();
        }

        size_t i = 0;
        while (i + 1 < pts.size() && pts[i + 1].speed_point.t < t)
        {
            i++;
        }
        const auto &p0 = pts[i];
        const auto &p1 = pts[i + 1];
        const double dt = p1.speed_point.t - p0.speed_point.t;
        const double r = (dt > Curve::kMathEpsilon) ? (t - p0.speed_point.t) / dt : 0.0;

        LocalTrajectoryPoint p = p0;
        p.path_point.pose.pose.position.x += r * (p1.path_point.pose.pose.position.x - p0.path_point.pose.pose.position.x);
        p.path_point.pose.pose.position.y += r * (p1.path_point.pose.pose.position.y - p0.path_point.pose.pose.position.y);
        p.path_point.theta = Curve::NormalizeAngle(p0.path_point.theta + r * Curve::NormalizeAngle(p1.path_point.theta - p0.path_point.theta));
        Curve::set_pose_yaw(p.path_point.pose, p.path_point.theta);
        p.path_point.kappa += r * (p1.path_point.kappa - p0.path_point.kappa);
        p.path_point.dkappa += r * (p1.path_point.dkappa - p0.path_point.dkappa);
        p.speed_point.t = t;
        p.speed_point.s_2path += r * (p1.speed_point.s_2path - p0.speed_point.s_2path);
        p.speed_point.speed += r * (p1.speed_point.speed - p0.speed_point.speed);
        p.speed_point.acceleration += r * (p1.speed_point.acceleration - p0.speed_point.acceleration);
        return p;
    }

} // namespace Planning
