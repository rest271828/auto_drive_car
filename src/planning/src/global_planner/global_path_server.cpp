#include "global_path_server.h"

namespace Planning
{
    GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "global_path_server_node created");
        global_path_pub_ = this->create_publisher<Path>("global_path", 10);             // 创建全局路径发布器
        global_path_rviz_pub_ = this->create_publisher<Marker>("global_path_rviz", 10); // 创建全局路径Rviz发布器

        global_path_server_ = this->create_service<GlobalPathService>(
            "global_path_server",
            std::bind(&GlobalPathServer::response_global_path_callback, this, _1, _2)); // 创建全局路径服务
    }

    void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request, std::shared_ptr<GlobalPathService::Response> response)
    {
        // 接受请求，多态
        switch (request->global_planner_type)
        {
        case static_cast<int>(GlobalPlannerType::NORMAL):
            global_planner_base_=std::make_shared<GlobalPlannerNormal>(); // 创建普通全局路径规划器
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid global planner type");
            break;
        }

        // 判断是否为空
        if(request->pnc_map.midline.points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty PNCMap");
            return;
        }

        // 搜索全局路径
        const auto global_path = global_planner_base_->search_global_path(request->pnc_map);
        response->global_path = global_path; // 设置响应中的全局路径

        // 发布全局路径
        global_path_pub_->publish(global_path); // 发布全局路径
        RCLCPP_INFO(this->get_logger(), "global path published");

        // 发布全局路径Rviz
        const auto global_path_rviz = path2marker(global_path);
        global_path_rviz_pub_->publish(global_path_rviz); // 发布全局路径Rviz
        RCLCPP_INFO(this->get_logger(), "global path rviz published");
    }

    Marker GlobalPathServer::path2marker(const Path &path)
    {
        Marker path_rviz_;
        path_rviz_.header = path.header;
        path_rviz_.ns = "global_path";
        path_rviz_.id = 0;
        path_rviz_.action = Marker::ADD;
        path_rviz_.type = Marker::LINE_STRIP;      // 连续的线条
        path_rviz_.scale.x = 0.05;                 // 线段宽度
        path_rviz_.color.a = 1.0;                  // 不透明度
        path_rviz_.color.r = 0.8;                  // 红色
        path_rviz_.color.g = 0.0;                  // 绿色
        path_rviz_.color.b = 0.0;                  // 蓝色
        path_rviz_.lifetime = rclcpp::Duration::max(); // 无限时间
        path_rviz_.frame_locked = true;            // 锁定坐标系
    
        Point p_tmp;
        for(const auto &pose : path.poses)
        {
            p_tmp.x = pose.pose.position.x;
            p_tmp.y = pose.pose.position.y;
            p_tmp.z = pose.pose.position.z; // 建议补充z轴坐标，避免高度异常
            path_rviz_.points.emplace_back(p_tmp);
        }
    
        return path_rviz_;
    }

} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}