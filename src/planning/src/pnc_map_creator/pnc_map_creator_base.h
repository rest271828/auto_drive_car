#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    enum class PNCMapType       //地图类型
    {
        STRAIGHT,
        STURN,
    };

    class PNCMapCreatorBase
    {
    public:
        virtual PNCMap create_pnc_map() = 0;        //创建地图
        inline PNCMap pnc_map() const { return pnc_map_; }      //获取地图
        inline MarkerArray pnc_map_markerarray() const { return pnc_map_markerarray_; }     //获取地图可视化
        virtual ~PNCMapCreatorBase() {};        //

    private:

    protected:
        std::unique_ptr<ConfigReader> pnc_map_config_;  //配置
        int map_type_ = 0; //地图类型
        PNCMap pnc_map_; //地图
        MarkerArray pnc_map_markerarray_; //rviz可视化

        Point p_mid_, pl_, pr_;
        double theta_current_ = 0.0; //当前角度
        double len_step_ = 0.0; //道路长度
        double theta_step_ = 0.0;//道路角度

    };
}  // namespace Planning
#endif  // PNC_MAP_CREATOR_BASE_H_
