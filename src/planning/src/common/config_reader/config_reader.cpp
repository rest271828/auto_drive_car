#include "config_reader.h"

namespace Planning
{
    ConfigReader::ConfigReader()    //配置文件读取类构造函数
    {
        //获取目录路径 workspace/install/planning/share/planning/
        std::string planning_share_directory = ament_index_cpp::get_package_share_directory("planning");

        //获取配置文件路径 workspace/install/planning/share/planning/config/config.yaml
        planning_config = YAML::LoadFile(planning_share_directory + "/config/planning_static_obs_config.yaml");
    }

        /****************vehicle****************/
    void ConfigReader::read_vehicle_config(VehicleStruct &vehicle, const std::string &name)
    {
        vehicle.id_ = planning_config["vehicle"][name]["id"].as<int>();
        vehicle.frame_ = planning_config["vehicle"][name]["frame"].as<std::string>();
        vehicle.length_ = planning_config["vehicle"][name]["length"].as<double>();
        vehicle.width_ = planning_config["vehicle"][name]["width"].as<double>();
        vehicle.pose_x_ = planning_config["vehicle"][name]["pose_x"].as<double>();
        vehicle.pose_y_ = planning_config["vehicle"][name]["pose_y"].as<double>();
        vehicle.pose_theta_ = planning_config["vehicle"][name]["pose_theta"].as<double>();
        vehicle.speed_ori_ = planning_config["vehicle"][name]["speed_ori"].as<double>();

        if(name !=  "main_car")
        {
            obs_pair_.emplace(vehicle.id_, vehicle); //将车辆信息存入map
        }
    }

    void ConfigReader::read_vehicles_config()
    { 
        try
        {
            read_pnc_map_config();
            read_vehicle_config(main_car_, "main_car");
            read_vehicle_config(obs_car1_, "obs_car1");
            read_vehicle_config(obs_car2_, "obs_car2");
            read_vehicle_config(obs_car3_, "obs_car3");

        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_vehicles_config error: %s", e.what());
        }
        
    }
    /****************pnc_map****************/
    void ConfigReader::read_pnc_map_config()
    {   
        try{
            pnc_map_.frame_ = planning_config["pnc_map"]["frame"].as<std::string>();
            pnc_map_.type_ = planning_config["pnc_map"]["type"].as<int>();
            pnc_map_.road_length_ = planning_config["pnc_map"]["road_length"].as<double>();
            pnc_map_.road_half_width_ = planning_config["pnc_map"]["road_half_width"].as<double>();
            pnc_map_.segment_len_ = planning_config["pnc_map"]["segment_len"].as<double>();
            pnc_map_.speed_limit_ = planning_config["pnc_map"]["speed_limit"].as<double>();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_pnc_map_config error: %s", e.what());
        }
    }
    /****************global_path****************/
    void ConfigReader::read_global_path_config()
    {
        try
        {
            global_path_.type_ = planning_config["global_path"]["type"].as<int>();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_global_path_config error: %s", e.what());
        }
    }
    /****************refer_line****************/
    void ConfigReader::read_refer_line_config()
    {
        try
        {
            read_pnc_map_config();
            refer_line_.type_ = planning_config["refer_line"]["type"].as<int>();
            refer_line_.front_size_ = planning_config["refer_line"]["front_size"].as<int>();
            refer_line_.back_size_ = planning_config["refer_line"]["back_size"].as<int>();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_refer_line_config error: %s", e.what());
        }
    }
    /****************local_path****************/
    void ConfigReader::read_local_path_config()
    {
        try
        {   read_refer_line_config();
            read_refer_line_config();
            local_path_.curve_type_ = planning_config["local_path"]["curve_type"].as<int>();
            local_path_.path_size_ = planning_config["local_path"]["path_size"].as<int>();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_local_path_config error: %s", e.what());
        }
    }
    /****************local_speed****************/
    void ConfigReader::read_local_speeds_config()
    {
        try
        {
            read_pnc_map_config();
            local_speeds_.speed_size_ = planning_config["local_speeds"]["speed_size"].as<int>();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_local_speeds_config error: %s", e.what());
        }
    }
    /****************decision****************/
    void ConfigReader::read_decision_config()
    {
        try
        {
            read_pnc_map_config();
            read_refer_line_config();
            read_local_path_config();
            read_local_speeds_config();
            decision_.safe_dis_l = planning_config["decision"]["safe_dis_l"].as<double>();
            decision_.safe_dis_r = planning_config["decision"]["safe_dis_r"].as<double>();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_decision_config error: %s", e.what());
        }
    }

    /****************planning_process****************/
    void ConfigReader::read_planning_process_config()
    {
        try
        {
            read_pnc_map_config();
            read_global_path_config();
            read_vehicles_config();
            process_.obs_dis_ = planning_config["planning_process"]["obs_dis"].as<double>();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_planning_process_config error: %s", e.what());
        }
    }
    /****************move_cmd****************/
    void ConfigReader::read_move_cmd_config()
    {
        try
        {
            read_pnc_map_config();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to read_move_cmd_config error: %s", e.what());
        }
    }

}