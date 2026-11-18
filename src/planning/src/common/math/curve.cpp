#include "curve.h"
#include <algorithm>

namespace Planning
{
        double Curve::NormalizeAngle(const double &angle)
        {
                double a = std::fmod(angle + M_PI, 2.0 * M_PI);
                if (a < 0.0)
                {
                        a += (2.0 * M_PI);
                }
                return a - M_PI;
        }

        void Curve::cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                        const double &speed, const double &a, const double &kappa, // 输入1: 目标点在笛卡尔下的参数:
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2: 目标点在参考线
                                        double &s, double &ds_dt, double &dds_dt,
                                        double &l, double &dl_ds, double &dl_dt, double &ddl_ds, double &ddl_dt)
        {
                // 计算s
                s = rs;
                
                // 计算l
                const double dx = x - rx;
                const double dy = y - ry;

                const double cos_theta_r = std::cos(rtheta);
                const double sin_theta_r = std::sin(rtheta);

                const double cross_r_x = cos_theta_r * dy - sin_theta_r * dx;
                l = std::copysign(std::hypot(dx, dy), cross_r_x);

                // 计算l' = dl/ds
                const double delta_theta = theta - rtheta;
                const double tan_delta_theta = std::tan(delta_theta);
                const double cos_delta_theta = std::cos(delta_theta);
                const double sin_delta_theta = std::sin(delta_theta);
                const double one_minus_kappa_l = 1 - rkappa * l;
                dl_ds = one_minus_kappa_l * tan_delta_theta;

                // 计算l'' = d(dl)/ds
                const double kappa_l_prime = rdkappa * l + rkappa * dl_ds;
                const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * kappa - rkappa;
                ddl_ds = -kappa_l_prime * tan_delta_theta +
                         one_minus_kappa_l / (cos_delta_theta * cos_delta_theta) * delta_theta_prime;

                // 计算ds/dt
                ds_dt = speed * cos_delta_theta / one_minus_kappa_l;

                // 计算d(ds)/dt
                dds_dt = (a * cos_delta_theta - (ds_dt * ds_dt) * (dl_ds * delta_theta_prime - kappa_l_prime)) / one_minus_kappa_l;

                // 计算dl_dt
                dl_dt = speed * sin_delta_theta;

                // 计算ddl_dt
                ddl_dt = a * sin_delta_theta;
        }

        void Curve::frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                        const double &l, const double &dl_ds, const double &ddl_ds, // 输入1: 目标点在frenet下的参数
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2: 目标点在参考线
                                        double &x, double &y, double &theta,
                                        double &speed, double &a, double &kappa)
        {
                (void)s;
                (void)rs; // s与rs在投影点处相等, 这里不再校验

                const double cos_theta_r = std::cos(rtheta);
                const double sin_theta_r = std::sin(rtheta);

                // 位置: 沿参考线法向偏移l
                x = rx - sin_theta_r * l;
                y = ry + cos_theta_r * l;

                // 航向角
                const double one_minus_kappa_r_l = 1.0 - rkappa * l;
                const double tan_delta_theta = dl_ds / one_minus_kappa_r_l;
                const double delta_theta = std::atan2(dl_ds, one_minus_kappa_r_l);
                const double cos_delta_theta = std::cos(delta_theta);
                theta = NormalizeAngle(delta_theta + rtheta);

                // 曲率
                const double kappa_r_l_prime = rdkappa * l + rkappa * dl_ds;
                kappa = (((ddl_ds + kappa_r_l_prime * tan_delta_theta) * cos_delta_theta * cos_delta_theta) /
                             one_minus_kappa_r_l +
                         rkappa) *
                        cos_delta_theta / one_minus_kappa_r_l;

                // 速度
                const double d_dot = dl_ds * ds_dt;
                speed = std::sqrt(one_minus_kappa_r_l * one_minus_kappa_r_l * ds_dt * ds_dt + d_dot * d_dot);

                // 加速度
                const double delta_theta_prime = one_minus_kappa_r_l / cos_delta_theta * kappa - rkappa;
                a = dds_dt * one_minus_kappa_r_l / cos_delta_theta +
                    ds_dt * ds_dt / cos_delta_theta * (dl_ds * delta_theta_prime - kappa_r_l_prime);
        }

        double Curve::yaw_from_pose(const PoseStamped &pose)
        {
                const auto &q = pose.pose.orientation;
                const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                return std::atan2(siny_cosp, cosy_cosp);
        }

        void Curve::set_pose_yaw(PoseStamped &pose, const double &yaw)
        {
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = std::sin(yaw / 2.0);
                pose.pose.orientation.w = std::cos(yaw / 2.0);
        }

        int Curve::find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point)
        {
                const int path_size = path.poses.size();
                if (path_size <= 1)
                {
                        return path_size - 1;
                }

                double min_dis = std::numeric_limits<double>::max();
                int closest_index = -1;
                for (int i = 0; i < path_size; i++)
                {
                        double dis = std::hypot(
                            path.poses[i].pose.position.x - target_point.pose.position.x,
                            path.poses[i].pose.position.y - target_point.pose.position.y);

                        if (dis < min_dis)
                        {
                                if (abs(last_match_point_index - i) > 100)
                                {
                                        continue;
                                }
                                min_dis = dis;
                                closest_index = i;
                        }
                }
                return closest_index;
        }
        // 找匹配点下标（参考线）
        int Curve::find_match_point(const Referline &refer_line, const PoseStamped &target_point)
        {
                const int size = refer_line.refer_line.size();
                double min_dis = std::numeric_limits<double>::max();
                int closest_index = -1;
                for (int i = 0; i < size; i++)
                {
                        const double dis = std::hypot(
                            refer_line.refer_line[i].pose.pose.position.x - target_point.pose.position.x,
                            refer_line.refer_line[i].pose.pose.position.y - target_point.pose.position.y);
                        if (dis < min_dis)
                        {
                                min_dis = dis;
                                closest_index = i;
                        }
                }
                return closest_index;
        }

        void Curve::find_projection_point(const Referline &path, const PoseStamped &target_point,
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa)
        {
                const int match_index = find_match_point(path, target_point);
                if (match_index < 0)
                {
                        rs = rx = ry = rtheta = rkappa = rdkappa = 0.0;
                        return;
                }

                // 以匹配点为基准, 沿匹配点切线方向近似投影(匹配点间距很小时误差可忽略)
                const auto &match_point = path.refer_line[match_index];
                const double mx = match_point.pose.pose.position.x;
                const double my = match_point.pose.pose.position.y;
                const double dx = target_point.pose.position.x - mx;
                const double dy = target_point.pose.position.y - my;
                const double ds = std::cos(match_point.rtheta) * dx + std::sin(match_point.rtheta) * dy;

                rs = match_point.rs + ds;
                rx = mx + ds * std::cos(match_point.rtheta);
                ry = my + ds * std::sin(match_point.rtheta);
                rtheta = NormalizeAngle(match_point.rtheta + match_point.rkappa * ds);
                rkappa = match_point.rkappa + match_point.rdkappa * ds;
                rdkappa = match_point.rdkappa;
        }

        void Curve::interpolate_refer_point(const Referline &refer_line, const double &s,
                                            double &rx, double &ry,
                                            double &rtheta, double &rkappa, double &rdkappa)
        {
                const auto &pts = refer_line.refer_line;
                const int size = pts.size();
                if (size == 0)
                {
                        rx = ry = rtheta = rkappa = rdkappa = 0.0;
                        return;
                }

                // 二分查找s所在区间 [i, i+1]
                int lo = 0;
                int hi = size - 1;
                if (s <= pts[0].rs)
                {
                        hi = std::min(1, size - 1);
                }
                else if (s >= pts[size - 1].rs)
                {
                        lo = std::max(0, size - 2);
                }
                else
                {
                        while (hi - lo > 1)
                        {
                                const int mid = (lo + hi) / 2;
                                if (pts[mid].rs <= s)
                                {
                                        lo = mid;
                                }
                                else
                                {
                                        hi = mid;
                                }
                        }
                }

                const auto &p0 = pts[lo];
                const auto &p1 = pts[hi];
                const double seg = p1.rs - p0.rs;
                const double ratio = (seg > kMathEpsilon) ? (s - p0.rs) / seg : 0.0;

                rx = p0.pose.pose.position.x + ratio * (p1.pose.pose.position.x - p0.pose.pose.position.x);
                ry = p0.pose.pose.position.y + ratio * (p1.pose.pose.position.y - p0.pose.pose.position.y);
                rtheta = NormalizeAngle(p0.rtheta + ratio * NormalizeAngle(p1.rtheta - p0.rtheta));
                rkappa = p0.rkappa + ratio * (p1.rkappa - p0.rkappa);
                rdkappa = p0.rdkappa + ratio * (p1.rdkappa - p0.rdkappa);
        }

        // 计算投影点参数
        void Curve::cal_projection_param(Referline &refer_line)
        {
                const int path_size = refer_line.refer_line.size();
                if (path_size < 3)
                {
                        RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
                        return;
                }
                // 计算rs
                double rs = 0.0;
                for (int i = 0; i < path_size; i++)
                {
                        if (i == 0)
                        {
                                rs = 0.0;
                        }
                        else
                        {
                                rs += std::hypot(
                                    refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                    refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                                refer_line.refer_line[i].rs = rs;
                        }
                        refer_line.refer_line[i].rs = rs;
                }

                // 计算航向角
                for (int i = 0; i < path_size; i++)
                {
                        if (i < path_size - 1)
                        {
                                refer_line.refer_line[i].rtheta = std::atan2(
                                    refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                    refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                        }
                        else
                        {
                                refer_line.refer_line[i].rtheta = std::atan2(

                                    refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,

                                    refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                        }
                }

                // 计算曲率
                for (int i = 0; i < path_size; i++)
                {
                        if (i < path_size - 1)
                        {
                                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                                if (dis <= kMathEpsilon)
                                {
                                        refer_line.refer_line[i].rkappa = 0.0;
                                }
                                else
                                {
                                        refer_line.refer_line[i].rkappa = NormalizeAngle(refer_line.refer_line[i + 1].rtheta - refer_line.refer_line[i].rtheta) / dis;
                                }
                        }
                        else
                        {
                                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                                if (dis <= kMathEpsilon)
                                {
                                        refer_line.refer_line[i].rkappa = 0.0;
                                }
                                else
                                {
                                        refer_line.refer_line[i].rkappa = NormalizeAngle(refer_line.refer_line[i].rtheta - refer_line.refer_line[i - 1].rtheta) / dis;
                                }
                        }
                }
                // 计算曲率变化率
                for (int i = 0; i < path_size; i++)
                {
                        if (i < path_size - 1)
                        {
                                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                                if (dis <= kMathEpsilon)
                                {
                                        refer_line.refer_line[i].rdkappa = 0.0;
                                }
                                else
                                {
                                        refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i + 1].rkappa - refer_line.refer_line[i].rkappa) / dis;
                                }
                        }
                        else
                        {
                                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                                if (dis <= kMathEpsilon)
                                {
                                        refer_line.refer_line[i].rdkappa = 0.0;
                                }
                                else
                                {
                                        refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i].rkappa - refer_line.refer_line[i - 1].rkappa) / dis;
                                }
                        }
                }
        }

        std::vector<double> Curve::path_arc_length(const LocalPath &path)
        {
                std::vector<double> s_list;
                s_list.reserve(path.local_path.size());
                double s = 0.0;
                for (size_t i = 0; i < path.local_path.size(); i++)
                {
                        if (i > 0)
                        {
                                s += std::hypot(
                                    path.local_path[i].pose.pose.position.x - path.local_path[i - 1].pose.pose.position.x,
                                    path.local_path[i].pose.pose.position.y - path.local_path[i - 1].pose.pose.position.y);
                        }
                        s_list.emplace_back(s);
                }
                return s_list;
        }

        bool Curve::project_to_path(const LocalPath &path, const std::vector<double> &path_s,
                                    const double &x, const double &y,
                                    double &s, double &l)
        {
                const int size = path.local_path.size();
                if (size < 2 || static_cast<int>(path_s.size()) != size)
                {
                        s = 0.0;
                        l = std::numeric_limits<double>::max();
                        return false;
                }

                // 在所有线段上找最近的投影
                double best_dis = std::numeric_limits<double>::max();
                bool inside = false;
                for (int i = 0; i < size - 1; i++)
                {
                        const double x0 = path.local_path[i].pose.pose.position.x;
                        const double y0 = path.local_path[i].pose.pose.position.y;
                        const double x1 = path.local_path[i + 1].pose.pose.position.x;
                        const double y1 = path.local_path[i + 1].pose.pose.position.y;
                        const double seg_x = x1 - x0;
                        const double seg_y = y1 - y0;
                        const double seg_len = std::hypot(seg_x, seg_y);
                        if (seg_len < kMathEpsilon)
                        {
                                continue;
                        }
                        const double ux = seg_x / seg_len;
                        const double uy = seg_y / seg_len;
                        double t = (x - x0) * ux + (y - y0) * uy; // 沿线段的投影长度
                        const double lat = ux * (y - y0) - uy * (x - x0); // 左正右负

                        bool seg_inside = true;
                        double t_clamped = t;
                        if (i == 0 && t < 0.0)
                        {
                                seg_inside = false; // 落在路径起点之前
                        }
                        else if (i == size - 2 && t > seg_len)
                        {
                                seg_inside = false; // 落在路径终点之后
                        }
                        else
                        {
                                t_clamped = std::clamp(t, 0.0, seg_len);
                        }

                        const double px = x0 + t_clamped * ux;
                        const double py = y0 + t_clamped * uy;
                        const double dis = std::hypot(x - px, y - py);
                        if (dis < best_dis)
                        {
                                best_dis = dis;
                                s = path_s[i] + (seg_inside ? t_clamped : t);
                                l = seg_inside ? std::copysign(dis, lat) : lat;
                                inside = seg_inside;
                        }
                }
                return inside;
        }

} // namespace Planning
