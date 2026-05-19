#include "planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(), "planning_process created");

        // 读取配置文件
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        obs_dis_ = process_config_->process().obs_dis_;

        map_client_ = this->create_client<PNCMapService>("pnc_map_server");
        global_path_client_ = this->create_client<GlobalPathService>("global_path_server");
    }

    bool PlanningProcess::process() // 总流程
    {
        // 阻塞一秒等rviz和xacro先启动
        rclcpp::Rate rate(1.0);
        rate.sleep();

        if(!planning_init())
        {
            RCLCPP_ERROR(this->get_logger(), "planning init failed!");
            return false;
        }
        return true;
    }

    bool PlanningProcess::planning_init() // 流程初始化
    {
        // 生成车辆

        // 连接地图服务器
        if(!connect_server(map_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "Map server connect failed!");
        }
        // 获取地图
        if(!map_request())
        {
            RCLCPP_ERROR(this->get_logger(), "Map request and response failed!");
        }
        // 连接全局路径服务器
        if(!connect_server(global_path_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "Global path server connect failed!");
        }

        // 获取全局路径
        if(!global_path_request())
        {
            RCLCPP_ERROR(this->get_logger(), "Global path request and response failed!");
        }
        return true;
    }

    template <typename T>
    bool PlanningProcess::connect_server(const T &client)
    {
        std::string server_name;
        if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>)
        {
            server_name = "pnc_map";
        }
        else if (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
        {
            server_name = "global_path";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "wrong client type!");
            return false;
        }

        // 等待服务器
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server." ,server_name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again..." ,server_name.c_str());
        }

        return true;
    }

    bool PlanningProcess::map_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending map request");

        // 构造请求
        auto request = std::make_shared<PNCMapService::Request>();
        request->map_type = process_config_->pnc_map().type_;

        // 获取响应
        auto result_future = map_client_->async_send_request(request);

        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Map received successfully");
            pnc_map_ = result_future.get()->pnc_map;// 获取响应中的地图数据
            return true;
        } 
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive map response");
            return false;
        }
        return false;
    }

    bool PlanningProcess::global_path_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending map request");

        // 构造请求
        auto request = std::make_shared<GlobalPathService::Request>();
        request->global_planner_type = process_config_->global_path().type_;
        request->pnc_map = pnc_map_;

        // 获取响应
        auto result_future = global_path_client_->async_send_request(request);

        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Map received successfully");
            global_path_ = result_future.get()->global_path;
            return true;
        } 
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive map response");
            return false;
        }
        return false;
    }

} // namespace Planning