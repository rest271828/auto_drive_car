#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "global_planner_normal.h"

namespace Planning
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    using base_msgs::srv::GlobalPathService; // 使用base_msgs中的PNCMap消息类型
    using geometry_msgs::msg::Point;         // 使用geometry_msgs中的Point消息类型
    using visualization_msgs::msg::Marker;   // 使用visualization_msgs中的Marker消息类型

    class GlobalPathServer : public rclcpp::Node
    {
    public:
        GlobalPathServer();

    private:
        void response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                           std::shared_ptr<GlobalPathService::Response> response); // 响应全局路径服务的回调函数
        Marker path2marker(const Path &path);                                                      // 将全局路径转换为Marker消息的函数

    private:
        std::shared_ptr<GlobalPlannerBase> global_planner_base_;           // 全局路径规划器指针
        rclcpp::Publisher<Path>::SharedPtr global_path_pub_;               // 全局路径发布器
        rclcpp::Publisher<Marker>::SharedPtr global_path_rviz_pub_;        // 全局路径Marker发布器
        rclcpp::Service<GlobalPathService>::SharedPtr global_path_server_; // 全局路径服务
    };
} // namespace Planning
#endif // GLOBAL_PATH_SERVER_H_
