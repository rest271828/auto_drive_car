#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "base_msgs/msg/plot_info.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning
{
    using namespace std::chrono_literals;
    using base_msgs::msg::LocalTrajectory;
    using base_msgs::msg::PlotInfo;
    using base_msgs::msg::PNCMap;
    using base_msgs::srv::GlobalPathService;
    using base_msgs::srv::PNCMapService;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;
    using tf2_ros::Buffer;
    using tf2_ros::TransformListener;

    class PlanningProcess : public rclcpp::Node // 规划总流程
    {
    public:
        PlanningProcess();
        bool process(); // 总流程

    private:
        bool planning_init();
        bool get_location(const std::shared_ptr<VehicleBase> &vehicle);
        void update_car_motion(); // 用上一帧轨迹更新主车的速度、加速度、曲率

        template <typename T>
        bool connect_server(const T &client);
        bool map_request();
        bool global_path_request();
        void planning_callback(); // 总流程回调函数

    public:
        inline PNCMap pnc_map() const { return pnc_map_; }
        inline Path global_path() { return global_path_; }

    private:
        std::unique_ptr<ConfigReader> process_config_; // 配置
        std::shared_ptr<VehicleBase> car_;             // 主车信息

        std::vector<std::shared_ptr<VehicleBase>> obses_spawn_; // 所有的障碍物
        std::vector<std::shared_ptr<VehicleBase>> obses_;       // 要考虑的障碍物
        double obs_dis_ = 0.0;

        std::unique_ptr<Buffer> buffer_;                             // tf缓冲区
        std::shared_ptr<TransformListener> tf_listener_;             // tf监听器

        PNCMap pnc_map_;                                                  // PNCMap
        Path global_path_;                                                // 全局路径
        rclcpp::Client<PNCMapService>::SharedPtr map_client_;             // PNCMap服务客户端
        rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_; // 全局路径服务客户端

        std::shared_ptr<ReferencelineCreator> refer_line_creator_; // 参考线生成器
        rclcpp::Publisher<Path>::SharedPtr refer_line_pub_;        // 参考线发布器

        std::shared_ptr<DecisionCenter> decision_;                // 决策中心
        std::shared_ptr<LocalPathPlanner> local_path_planner_;    // 局部路径规划器
        std::shared_ptr<LocalSpeedsPlanner> local_speeds_planner_; // 速度规划器
        std::shared_ptr<LocalTrajectoryCombiner> trajectory_combiner_; // 轨迹合成器

        rclcpp::Publisher<Path>::SharedPtr local_path_pub_;                  // 局部路径(rviz)
        rclcpp::Publisher<Path>::SharedPtr local_trajectory_rviz_pub_;       // 轨迹(rviz)
        rclcpp::Publisher<LocalTrajectory>::SharedPtr local_trajectory_pub_; // 轨迹(给运动节点)
        rclcpp::Publisher<PlotInfo>::SharedPtr plot_info_pub_;               // 绘图信息

        LocalTrajectory last_trajectory_;    // 上一帧轨迹
        rclcpp::Time last_trajectory_time_;  // 上一帧轨迹的生成时间
        bool has_trajectory_ = false;

        rclcpp::TimerBase::SharedPtr timer_; // 规划定时器
    };
} // namespace Planning
#endif // PLANNING_PROCESS_H_