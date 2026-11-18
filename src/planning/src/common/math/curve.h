#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <vector>
#include <limits>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    class Curve // 曲线
    {
    public:
        Curve() = default;

        static double NormalizeAngle(const double &angle); // 角度归一化到[-pi, pi]

        // 四元数(仅yaw) <-> 航向角
        static double yaw_from_pose(const PoseStamped &pose);
        static void set_pose_yaw(PoseStamped &pose, const double &yaw);

        // 笛卡尔转frenet
        static void cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                        const double &speed, const double &a, const double &kappa, // 输入1: 目标点在笛卡尔下的参数:
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2: 目标点在参考线
                                        double &s, double &ds_dt, double &dds_dt,
                                        double &l, double &dl_ds, double &dl_dt, double &ddl_ds, double &ddl_dt); // 输出: 目标点在

        // frenet转笛卡尔
        static void frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                        const double &l, const double &dl_ds, const double &ddl_ds, // 输入1: 目标点在frenet下的参数
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2: 目标点在参考线
                                        double &x, double &y, double &theta,
                                        double &speed, double &a, double &kappa); // 输出: 目标点在笛卡尔下的参数

        constexpr static double kMathEpsilon = 1.0e-6; // 小数

        // 找匹配点下标
        static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point);
        static int find_match_point(const Referline &refer_line, const PoseStamped &target_point);

        // 找到投影点
        static void find_projection_point(const Referline &path, const PoseStamped &target_point,
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa); // 输出：投影点的rs,rx,ry,rtheta,rkappa,rdkappa

        // 按给定s在参考线上线性插值出参考点
        static void interpolate_refer_point(const Referline &refer_line, const double &s,
                                            double &rx, double &ry,
                                            double &rtheta, double &rkappa, double &rdkappa);

        // 计算投影点参数
        static void cal_projection_param(Referline &refer_line);

        // 局部路径的累计弧长(从路径起点开始, 第一个点为0)
        static std::vector<double> path_arc_length(const LocalPath &path);

        // 点向局部路径(折线)投影, 得到沿路径的s和带符号的横向偏移l(左正右负)
        // 若投影点落在路径首尾之外, 返回false(此时s, l仍给出外推值)
        static bool project_to_path(const LocalPath &path, const std::vector<double> &path_s,
                                    const double &x, const double &y,
                                    double &s, double &l);
    };
} // namespace Planning
#endif // CURVE_H_
