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

        // 创建车辆和障碍物(它们的位姿由 car_move_cmd / obs_move_cmd 以tf的形式发布)
        car_ = std::make_shared<MainCar>();
        for (int i = 0; i < 3; i++)
        {
            auto obs = std::make_shared<ObsCar>(i+1);
            obses_spawn_.emplace_back(obs);
        }

        // 创建监听器，绑定主车缓存对象
        buffer_ = std::make_unique<Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);

        // 创建地图和全局路径客户端
        map_client_ = this->create_client<PNCMapService>("pnc_map_server");
        global_path_client_ = this->create_client<GlobalPathService>("global_path_server");

        // 创建参考线和参考线的发布器
        refer_line_creator_ = std::make_shared<ReferencelineCreator>();
        refer_line_pub_ = this->create_publisher<Path>("reference_line", 10);

        // 决策、局部路径、速度、轨迹
        decision_ = std::make_shared<DecisionCenter>();
        local_path_planner_ = std::make_shared<LocalPathPlanner>();
        local_speeds_planner_ = std::make_shared<LocalSpeedsPlanner>();
        trajectory_combiner_ = std::make_shared<LocalTrajectoryCombiner>();

        local_path_pub_ = this->create_publisher<Path>("local_path", 10);
        local_trajectory_rviz_pub_ = this->create_publisher<Path>("local_trajectory_rviz", 10);
        local_trajectory_pub_ = this->create_publisher<LocalTrajectory>("local_trajectory", 10);
        plot_info_pub_ = this->create_publisher<PlotInfo>("plot_info", 10);
    }

    bool PlanningProcess::process() // 总流程
    {
        // 阻塞1s，等待rviz2和xacro模型先启动
        rclcpp::Rate rate(1.0);
        rate.sleep();

        // 初始化
        if (!planning_init())
        {
            RCLCPP_ERROR(this->get_logger(), "planning init failed!");
            return false;
        }

        // 进入规划主流程
        timer_ = this->create_wall_timer(
            0.1s, // 0.1秒为周期
            std::bind(&PlanningProcess::planning_callback, this));

        return true;
    }

    bool PlanningProcess::planning_init() // 流程初始化
    {
        // 连接地图服务器
        if (!connect_server(map_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "Map server connect failed!");
        }
        // 获取地图
        if (!map_request())
        {
            RCLCPP_ERROR(this->get_logger(), "Map request and response failed!");
        }
        // 连接全局路径服务器
        if (!connect_server(global_path_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "Global path server connect failed!");
        }

        // 获取全局路径
        if (!global_path_request())
        {
            RCLCPP_ERROR(this->get_logger(), "Global path request and response failed!");
        }
        return true;
    }

    bool PlanningProcess::get_location(const std::shared_ptr<VehicleBase> &vehicle)
    {
        try
        {
            PoseStamped point;
            auto ts = buffer_->lookupTransform(process_config_->pnc_map().frame_, vehicle->child_frame(), tf2::TimePointZero);
            point.header = ts.header;
            point.pose.position.x = ts.transform.translation.x;
            point.pose.position.y = ts.transform.translation.y;
            point.pose.position.z = ts.transform.translation.z;
            point.pose.orientation.x = ts.transform.rotation.x;
            point.pose.orientation.y = ts.transform.rotation.y;
            point.pose.orientation.z = ts.transform.rotation.z;
            point.pose.orientation.w = ts.transform.rotation.w;
            vehicle->update_location(point);
            return true;
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Lookup %s failed: %s", vehicle->child_frame().c_str(), e.what());
            return false;
        }
    }

    void PlanningProcess::update_car_motion()
    {
        if (!has_trajectory_)
        {
            return; // 第一帧沿用配置中的初速度
        }
        const double t = (this->now() - last_trajectory_time_).seconds();
        const auto p = LocalTrajectoryCombiner::interpolate_by_time(last_trajectory_, t);
        car_->update_motion(p.speed_point.speed, p.speed_point.acceleration,
                            p.path_point.kappa, p.path_point.dkappa);
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
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", server_name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again...", server_name.c_str());
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

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Map received successfully");
            pnc_map_ = result_future.get()->pnc_map; // 获取响应中的地图数据
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive map response");
            return false;
        }
        return false;
    }

    bool PlanningProcess::global_path_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending global path request");

        // 构造请求
        auto request = std::make_shared<GlobalPathService::Request>();
        request->global_planner_type = process_config_->global_path().type_;
        request->pnc_map = pnc_map_;

        // 获取响应
        auto result_future = global_path_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Map received successfully");
            global_path_ = result_future.get()->global_path;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive map response");
            return false;
        }
        return false;
    }

    void PlanningProcess::planning_callback()
    {
        const auto start_time = this->get_clock()->now();

        // 监听车辆定位
        if (!get_location(car_))
        {
            return; // 主车tf还没发布(car_move_cmd未启动)
        }
        update_car_motion();

        // 障碍物定位, 只保留一定范围内的
        obses_.clear();
        for (const auto &obs : obses_spawn_)
        {
            if (!get_location(obs))
            {
                continue;
            }
            const double dis = std::hypot(obs->loc_point().pose.position.x - car_->loc_point().pose.position.x,
                                          obs->loc_point().pose.position.y - car_->loc_point().pose.position.y);
            if (dis <= obs_dis_)
            {
                obses_.emplace_back(obs);
            }
        }

        // 参考线
        const auto refer_line = refer_line_creator_->creat_reference_line(global_path_, car_->loc_point());
        if (refer_line.refer_line.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "reference line empty!");
            return;
        }
        const auto refer_line_rviz = refer_line_creator_->referline_to_rviz(); // 生成rviz用的参考线
        refer_line_pub_->publish(refer_line_rviz);                             // 发布参考线

        // 主车和障碍物向参考线投影
        car_->vehicle_cartesian_to_frenet(refer_line);
        std::vector<std::shared_ptr<VehicleBase>> obses_on_refer;
        for (const auto &obs : obses_)
        {
            if (obs->vehicle_cartesian_to_frenet(refer_line))
            {
                obses_on_refer.emplace_back(obs); // 只有投影落在参考线范围内的才参与路径决策
            }
        }

        // 障碍物按s值排序
        std::sort(obses_on_refer.begin(), obses_on_refer.end(),
                  [](const std::shared_ptr<VehicleBase> &a, const std::shared_ptr<VehicleBase> &b)
                  { return a->s() < b->s(); });

        // 路径决策
        decision_->make_path_decision(car_, obses_on_refer);

        // 路径规划
        const auto local_path = local_path_planner_->create_local_path(refer_line, car_, decision_);
        if (local_path.local_path.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "local path empty!");
            return;
        }
        local_path_pub_->publish(local_path_planner_->path_to_rviz());
        const auto path_s = Curve::path_arc_length(local_path);

        // 障碍物向路径投影
        for (const auto &obs : obses_)
        {
            obs->vehicle_cartesian_to_frenet_2path(local_path, path_s);
        }

        // 速度决策
        decision_->make_speed_decision(local_path, path_s, car_, obses_,
                                       local_speeds_planner_->dt(), local_speeds_planner_->speed_size());

        // 速度规划
        const auto local_speeds = local_speeds_planner_->create_local_speeds(local_path, path_s, car_, decision_);

        // 合成轨迹
        auto trajectory = trajectory_combiner_->combine_trajectory(local_path, path_s, local_speeds);
        const auto plan_time = this->now();
        trajectory.header.stamp = plan_time;
        local_trajectory_pub_->publish(trajectory);
        local_trajectory_rviz_pub_->publish(trajectory_combiner_->trajectory_to_rviz());

        // 更新绘图信息
        PlotInfo plot_info;
        plot_info.header = trajectory.header;
        plot_info.trajectory_info = trajectory;
        plot_info.obs_info = decision_->obs_info();
        plot_info_pub_->publish(plot_info);

        // 更新车辆信息: 记录本帧轨迹, 下一帧据此更新主车速度等状态
        last_trajectory_ = trajectory;
        last_trajectory_time_ = plan_time;
        has_trajectory_ = !trajectory.local_trajectory.empty();

        RCLCPP_INFO(this->get_logger(), "---------car state: loc: (%.2f, %.2f), s: %.2f, l: %.2f, speed: %.2f, a: %.2f, theta: %.2f, kappa: %.2f",
                    car_->loc_point().pose.position.x,
                    car_->loc_point().pose.position.y,
                    car_->s(), car_->l(),
                    car_->speed(), car_->acceleration(),
                    car_->theta(), car_->kappa());

        const auto end_time = this->get_clock()->now();
        const double planning_total_time = end_time.seconds() - start_time.seconds();
        RCLCPP_INFO(this->get_logger(), "planning total time: %fms", planning_total_time * 1000);

        // 防止系统卡死
        if (planning_total_time > 1.0)
        {
            RCLCPP_ERROR(this->get_logger(), "planning time too long!");
            rclcpp::shutdown();
        }
    }

} // namespace Planning