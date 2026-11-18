// 不依赖 ROS 的闭环仿真: 复用 planning 包里的真实算法代码(参考线/决策/路径/速度/轨迹),
// 用同样的流程逐帧运行, 主车严格沿上一帧轨迹走 0.1s, 障碍物匀速直线运动, 统计碰撞与通行情况。
//   用法: ./offline_sim <配置文件名> [输出csv]
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "config_reader.h"
#include "curve.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"

using namespace Planning;

// 复刻 pnc_map_creator(直道/S弯) + global_planner_normal 的几何, 生成全局路径
static Path make_global_path(const ConfigReader &cfg)
{
    const auto &m = cfg.pnc_map();
    const double hw = m.road_half_width_;
    const double step = m.segment_len_;
    double x = -3.0, y = hw / 2.0, theta = 0.0;
    std::vector<std::array<double, 2>> mid, right;

    auto straight = [&](double length)
    {
        double len = 0.0;
        while (len < length)
        {
            mid.push_back({x, y});
            right.push_back({x, y - hw});
            len += step;
            x += step;
        }
    };
    auto arc = [&](double angle, double flag)
    {
        double t = 0.0;
        while (t < angle)
        {
            mid.push_back({x, y});
            right.push_back({x + hw * std::sin(theta), y - hw * std::cos(theta)});
            x += step * std::cos(theta);
            y += step * std::sin(theta);
            t += 0.01;
            theta += 0.01 * flag;
        }
    };

    if (m.type_ == 1)
    {
        straight(m.road_length_ / 3.0);
        arc(M_PI_2, 1.0);
        arc(M_PI_2, -1.0);
    }
    else
    {
        straight(m.road_length_);
    }
    if (mid.size() % 2 == 1)
    {
        mid.pop_back();
    }

    Path path;
    path.header.frame_id = m.frame_;
    for (size_t i = 0; i < mid.size(); i++)
    {
        PoseStamped p;
        p.pose.position.x = (mid[i][0] + right[i][0]) / 2.0;
        p.pose.position.y = (mid[i][1] + right[i][1]) / 2.0;
        path.poses.push_back(p);
    }
    return path;
}

// 两个有向矩形之间是否重叠(分离轴定理)
static bool boxes_overlap(double x1, double y1, double th1, double l1, double w1,
                          double x2, double y2, double th2, double l2, double w2)
{
    const double axes[4][2] = {{std::cos(th1), std::sin(th1)}, {-std::sin(th1), std::cos(th1)},
                               {std::cos(th2), std::sin(th2)}, {-std::sin(th2), std::cos(th2)}};
    const double dx = x2 - x1, dy = y2 - y1;
    for (const auto &ax : axes)
    {
        const double proj_d = std::fabs(dx * ax[0] + dy * ax[1]);
        const double r1 = l1 / 2 * std::fabs(std::cos(th1) * ax[0] + std::sin(th1) * ax[1]) +
                          w1 / 2 * std::fabs(-std::sin(th1) * ax[0] + std::cos(th1) * ax[1]);
        const double r2 = l2 / 2 * std::fabs(std::cos(th2) * ax[0] + std::sin(th2) * ax[1]) +
                          w2 / 2 * std::fabs(-std::sin(th2) * ax[0] + std::cos(th2) * ax[1]);
        if (proj_d > r1 + r2)
        {
            return false;
        }
    }
    return true;
}

// 两个有向矩形之间的最小距离(不重叠时 = 顶点到对方边的最小距离)
static double boxes_distance(double x1, double y1, double th1, double l1, double w1,
                             double x2, double y2, double th2, double l2, double w2)
{
    auto corners = [](double x, double y, double th, double l, double w)
    {
        std::array<std::array<double, 2>, 4> c;
        const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, -1, 1};
        for (int i = 0; i < 4; i++)
        {
            const double lx = sx[i] * l / 2, ly = sy[i] * w / 2;
            c[i] = {x + lx * std::cos(th) - ly * std::sin(th), y + lx * std::sin(th) + ly * std::cos(th)};
        }
        return c;
    };
    auto seg_dist = [](const std::array<double, 2> &p, const std::array<double, 2> &a, const std::array<double, 2> &b)
    {
        const double vx = b[0] - a[0], vy = b[1] - a[1];
        const double t = std::clamp(((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / (vx * vx + vy * vy), 0.0, 1.0);
        return std::hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy);
    };
    const auto c1 = corners(x1, y1, th1, l1, w1), c2 = corners(x2, y2, th2, l2, w2);
    double d = 1e9;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            d = std::min(d, seg_dist(c1[i], c2[j], c2[(j + 1) % 4]));
            d = std::min(d, seg_dist(c2[i], c1[j], c1[(j + 1) % 4]));
        }
    }
    return d;
}

static void set_pose(const std::shared_ptr<VehicleBase> &v, double x, double y, double theta)
{
    PoseStamped p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    Curve::set_pose_yaw(p, theta);
    v->update_location(p);
}

int main(int argc, char **argv)
{
    if (argc >= 2)
    {
        setenv("PLANNING_CONFIG", argv[1], 1);
    }
    const std::string csv_path = argc >= 3 ? argv[2] : "";

    ConfigReader cfg;
    cfg.read_planning_process_config();
    const Path global_path = make_global_path(cfg);
    const double obs_dis = cfg.process().obs_dis_;

    auto car = std::make_shared<MainCar>();
    std::vector<std::shared_ptr<VehicleBase>> obses_spawn;
    std::vector<std::array<double, 4>> obs_init; // x, y, theta, speed
    for (int i = 1; i <= 3; i++)
    {
        auto obs = std::make_shared<ObsCar>(i);
        obs_init.push_back({obs->loc_point().pose.position.x, obs->loc_point().pose.position.y, obs->theta(), obs->speed()});
        obses_spawn.push_back(obs);
    }

    ReferencelineCreator refer_line_creator;
    auto decision = std::make_shared<DecisionCenter>();
    LocalPathPlanner path_planner;
    LocalSpeedsPlanner speeds_planner;
    LocalTrajectoryCombiner combiner;

    std::ofstream csv;
    if (!csv_path.empty())
    {
        csv.open(csv_path);
        csv << "t,x,y,theta,v,a,s,l,o1x,o1y,o2x,o2y,o3x,o3y\n";
    }

    const PoseStamped goal = global_path.poses.back();
    LocalTrajectory last_traj;
    bool has_traj = false;
    int collisions = 0;
    double min_clear = 1e9;
    double max_abs_lat_acc = 0.0, max_decel = 0.0, max_planning_ms = 0.0;
    int stopped_cycles = 0;
    const int max_cycles = 4000;
    int cycle = 0;
    double travelled = 0.0;
    double last_x = car->loc_point().pose.position.x, last_y = car->loc_point().pose.position.y;

    for (; cycle < max_cycles; cycle++)
    {
        const double t_now = cycle * kPlanningCycle;

        // ---- "car_move_cmd": 主车沿上一帧轨迹走一个周期 ----
        if (has_traj)
        {
            const auto p = LocalTrajectoryCombiner::interpolate_by_time(last_traj, kPlanningCycle);
            set_pose(car, p.path_point.pose.pose.position.x, p.path_point.pose.pose.position.y, p.path_point.theta);
            car->update_motion(p.speed_point.speed, p.speed_point.acceleration, p.path_point.kappa, p.path_point.dkappa);
            max_abs_lat_acc = std::max(max_abs_lat_acc, std::fabs(p.speed_point.speed * p.speed_point.speed * p.path_point.kappa));
            max_decel = std::max(max_decel, -p.speed_point.acceleration);
        }
        // ---- "obs_move_cmd": 障碍物匀速直线 ----
        for (size_t i = 0; i < obses_spawn.size(); i++)
        {
            const auto &o = obs_init[i];
            set_pose(obses_spawn[i], o[0] + o[3] * std::cos(o[2]) * t_now, o[1] + o[3] * std::sin(o[2]) * t_now, o[2]);
        }

        // ---- 碰撞检测 ----
        const double cx = car->loc_point().pose.position.x, cy = car->loc_point().pose.position.y;
        travelled += std::hypot(cx - last_x, cy - last_y);
        last_x = cx;
        last_y = cy;
        for (const auto &obs : obses_spawn)
        {
            const double ox = obs->loc_point().pose.position.x, oy = obs->loc_point().pose.position.y;
            if (boxes_overlap(cx, cy, car->theta(), car->length(), car->width(),
                              ox, oy, obs->theta(), obs->length(), obs->width()))
            {
                collisions++;
                std::fprintf(stderr, "COLLISION at t=%.1f with obs %d (car %.2f,%.2f obs %.2f,%.2f)\n",
                             t_now, obs->id(), cx, cy, ox, oy);
            }
            else
            {
                min_clear = std::min(min_clear, boxes_distance(cx, cy, car->theta(), car->length(), car->width(),
                                                               ox, oy, obs->theta(), obs->length(), obs->width()));
            }
        }

        // ---- 规划主流程(与 planning_process::planning_callback 一致) ----
        const auto wall0 = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<VehicleBase>> obses;
        for (const auto &obs : obses_spawn)
        {
            if (std::hypot(obs->loc_point().pose.position.x - cx, obs->loc_point().pose.position.y - cy) <= obs_dis)
            {
                obses.push_back(obs);
            }
        }
        const auto refer_line = refer_line_creator.creat_reference_line(global_path, car->loc_point());
        if (refer_line.refer_line.empty())
        {
            std::fprintf(stderr, "reference line empty\n");
            break;
        }
        car->vehicle_cartesian_to_frenet(refer_line);
        std::vector<std::shared_ptr<VehicleBase>> obses_on_refer;
        for (const auto &obs : obses)
        {
            if (obs->vehicle_cartesian_to_frenet(refer_line))
            {
                obses_on_refer.push_back(obs);
            }
        }
        std::sort(obses_on_refer.begin(), obses_on_refer.end(),
                  [](const auto &a, const auto &b) { return a->s() < b->s(); });
        decision->make_path_decision(car, obses_on_refer);
        const auto local_path = path_planner.create_local_path(refer_line, car, decision);
        if (local_path.local_path.size() < 2)
        {
            std::fprintf(stderr, "t=%.1f local path empty -> stop\n", t_now);
            break;
        }
        const auto path_s = Curve::path_arc_length(local_path);
        for (const auto &obs : obses)
        {
            obs->vehicle_cartesian_to_frenet_2path(local_path, path_s);
        }
        decision->make_speed_decision(local_path, path_s, car, obses, speeds_planner.dt(), speeds_planner.speed_size());
        const auto speeds = speeds_planner.create_local_speeds(local_path, path_s, car, decision);
        last_traj = combiner.combine_trajectory(local_path, path_s, speeds);
        has_traj = !last_traj.local_trajectory.empty();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall0).count();
        max_planning_ms = std::max(max_planning_ms, ms);

        if (csv)
        {
            csv << t_now << "," << cx << "," << cy << "," << car->theta() << "," << car->speed() << ","
                << car->acceleration() << "," << car->s() << "," << car->l();
            for (const auto &obs : obses_spawn)
            {
                csv << "," << obs->loc_point().pose.position.x << "," << obs->loc_point().pose.position.y;
            }
            csv << "\n";
        }

        if (car->speed() < 0.05)
        {
            stopped_cycles++;
        }
        if (std::hypot(goal.pose.position.x - cx, goal.pose.position.y - cy) < 2.0 && car->speed() < 0.1)
        {
            break;
        }
    }

    const double cx = car->loc_point().pose.position.x, cy = car->loc_point().pose.position.y;
    std::printf("RESULT config=%s cycles=%d sim_time=%.1fs travelled=%.1fm end=(%.1f,%.1f) dist_to_goal=%.1fm "
                "collisions=%d min_clearance=%.2fm max_lat_acc=%.2f max_decel=%.2f stopped_cycles=%d max_plan=%.2fms\n",
                std::getenv("PLANNING_CONFIG"), cycle, cycle * kPlanningCycle, travelled, cx, cy,
                std::hypot(goal.pose.position.x - cx, goal.pose.position.y - cy),
                collisions, min_clear, max_abs_lat_acc, max_decel, stopped_cycles, max_planning_ms);
    return collisions > 0 ? 1 : 0;
}
