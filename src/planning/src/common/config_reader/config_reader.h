#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>

namespace Planning
{
    struct VehicleStruct //车辆信息
    {
        int id_ = 0;                //车辆ID
        std::string frame_ = "";    //车辆坐标系
        double length_ = 0.0;       //车辆长度
        double width_ = 0.0;        //车辆宽度
        double pose_x_ = 0.0;       //车辆高度
        double pose_y_ = 0.0;       //车辆宽度
        double pose_theta_ = 0.0;   //车辆航向角
        double speed_ori_ = 0.0;    //车辆原始速度
    };

    struct PNCMapStruct //pnc地图信息
    {
        std::string frame_ = "";//地图坐标系
        int type_ = 0;          //地图类型
        double road_length_ = 0.0;   //地图长度
        double road_half_width_ = 0.0;    //地图宽度
        double segment_len_ = 0.0;          //道路半宽
        double speed_limit_ = 0.0;         //速度限制
    };

    struct GlobalPathStruct //全局路径信息
    {
        int type_ = 0;          //路径类型
    };

    struct ReferLineStruct //参考线信息
    {
        int type_ = 0;          //路径类型
        int front_size_ = 0;     //前方参考线点数
        int back_size_ = 0;      //后方参考线点数
    };

    struct  LocalPathStruct //局部路径信息
    {
        int curve_type_ = 0;          //路径类型
        int path_size_ = 0;          //路径点数
    };

    struct LocalSpeedsStruct //局部速度信息
    {
        int speed_size_ = 0;    //路径速度
    };

    struct DecisionStruct //决策信息
    {
        double safe_dis_l = 0.0;   //侧向安全距离
        double safe_dis_r = 0.0;   //纵向安全距离
    };

    struct ProcessStruct //处理信息
    {
        double obs_dis_ = 0.0;   //障碍物距离
    };
    
    class ConfigReader      //配置文件读取器
    {
    public:
        ConfigReader() ;

        void read_vehicle_config(VehicleStruct &vehicle, const std::string &name);
        void read_vehicles_config();
        inline VehicleStruct main_car() const {return main_car_;}
        inline VehicleStruct obs_car1() const {return obs_car1_;}
        inline VehicleStruct obs_car2() const {return obs_car2_;}
        inline VehicleStruct obs_car3() const {return obs_car3_;}
        inline std::unordered_map<int, VehicleStruct> obs_pair() const {return obs_pair_;}
        void read_pnc_map_config();
        inline PNCMapStruct pnc_map() const {return pnc_map_;}

        void read_global_path_config();
        inline GlobalPathStruct global_path() const {return global_path_;}

        void read_refer_line_config();
        inline ReferLineStruct refer_line() const {return refer_line_;}

        void read_local_path_config();
        inline LocalPathStruct local_path() const {return local_path_;}

        void read_local_speeds_config();
        inline LocalSpeedsStruct local_speeds() const {return local_speeds_;}

        void read_decision_config();
        inline DecisionStruct decision() const {return decision_;}

        void read_planning_process_config();
        inline ProcessStruct process() const {return process_;}

        void read_move_cmd_config();

    private:
        YAML::Node planning_config; //配置文件节点

        VehicleStruct main_car_; //主车信息
        VehicleStruct obs_car1_;  
        VehicleStruct obs_car2_;  //障碍物信息
        VehicleStruct obs_car3_;
        std::unordered_map<int, VehicleStruct> obs_pair_; //障碍物信息

        //pnc_map
        PNCMapStruct pnc_map_;
        //global_path
        GlobalPathStruct global_path_;
        //refer_line
        ReferLineStruct refer_line_;
        //local_path
        LocalPathStruct local_path_;
        //decision
        DecisionStruct decision_;
        //local_speeds
        LocalSpeedsStruct local_speeds_;
        //process
        ProcessStruct process_;

    };
}  // namespace Planning
#endif  // CONFIG_READER_H_
