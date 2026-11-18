#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/obs_info.hpp"
#include "config_reader.h"
#include "curve.h"
#include "main_car_info.h"
#include "obs_car_info.h"

#include <memory>
#include <vector>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::ObsInfo;

    // 路径决策结果: 在 [s_begin, s_end] 区间内, 路径需横向偏移到 target_l 以绕开障碍物
    struct BypassInterval
    {
        int obs_id = 0;
        double s_begin = 0.0;  // 参考线s, 开始需要保持偏移的位置
        double s_end = 0.0;    // 参考线s, 可以结束偏移的位置
        double target_l = 0.0; // 绕行时的横向偏移
    };

    enum class SpeedDecisionType // 速度决策类型
    {
        IGNORE = 0,   // 不影响(未进入路径, 或可以抢先通过)
        FOLLOW = 1,   // 跟随/停车: 障碍物在路径上, 主车不能超过它
        YIELD = 2,    // 让行: 障碍物会横穿路径, 主车在它离开之前不能越过穿越点
        OVERTAKE = 3, // 抢行: 主车能在障碍物进入路径前通过
    };

    // 速度约束: 第k个时间步, 主车沿路径的 s 不能超过 s_upper[k], 该处障碍物沿路径速度为 v_obs[k]
    struct SpeedConstraint
    {
        int obs_id = 0;
        SpeedDecisionType type = SpeedDecisionType::IGNORE;
        std::vector<bool> active;
        std::vector<double> s_upper;
        std::vector<double> v_obs;
    };

    class DecisionCenter // 决策中心
    {
    public:
        DecisionCenter();

        // 路径决策: 对挡住本车道的静止障碍物决定向左/向右绕行
        void make_path_decision(const std::shared_ptr<VehicleBase> &car,
                                const std::vector<std::shared_ptr<VehicleBase>> &obses);

        // 速度决策: 预测障碍物运动并投影到路径上, 生成ST约束, 并填写绘图用的障碍物信息
        void make_speed_decision(const LocalPath &path, const std::vector<double> &path_s,
                                 const std::shared_ptr<VehicleBase> &car,
                                 const std::vector<std::shared_ptr<VehicleBase>> &obses,
                                 const double &dt, const int &steps);

        inline const std::vector<BypassInterval> &bypass_intervals() const { return bypass_intervals_; }
        inline const std::vector<SpeedConstraint> &speed_constraints() const { return speed_constraints_; }
        inline const std::vector<ObsInfo> &obs_info() const { return obs_info_; }

        // 道路横向边界(相对全局路径/参考线): 右车道中心为0, 道路范围 [-半宽/2, 1.5*半宽]
        inline double l_min() const { return -0.5 * road_half_width_; }
        inline double l_max() const { return 1.5 * road_half_width_; }
        inline double speed_limit() const { return speed_limit_; }               // m/s
        inline double transition_length() const { return transition_length_; } // 换道过渡长度
        inline double safe_dis_l() const { return safe_dis_l_; }
        inline double safe_dis_s() const { return safe_dis_s_; }

        constexpr static double kStaticSpeed = 0.3;      // 低于该速度视为静止障碍物(m/s)
        constexpr static double kCollisionBuffer = 0.3;  // 速度规划判断"在路径上"时额外的横向缓冲(m)
        constexpr static double kComfortDecel = 2.0;     // 舒适减速度(m/s^2)
        constexpr static double kMaxAccel = 1.5;         // 最大加速度(m/s^2)

    private:
        std::unique_ptr<ConfigReader> decision_config_;
        double road_half_width_ = 0.0;
        double speed_limit_ = 0.0;
        double safe_dis_l_ = 0.0;
        double safe_dis_s_ = 0.0;
        double transition_length_ = 0.0;

        std::vector<BypassInterval> bypass_intervals_;
        std::vector<SpeedConstraint> speed_constraints_;
        std::vector<ObsInfo> obs_info_;
    };
} // namespace Planning
#endif // DECISION_CENTER_H_
