#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    enum class PNCMapType
    {
        STRAIGHT,
        STURN,
    };

    class PNCMapCreatorBase // pnc_map创建器基类
    {
    public:
        virtual PNCMap create_pnc_map() = 0;                                             // 创建pnc_map的纯虚函数
        inline PNCMap pnc_map() const { return pnc_map_; }                               // 获取pnc_map
        inline MarkerArray pnc_map_marker_array() const { return pnc_map_markerarray_; } // 获取pnc_map的markerarray
        virtual ~PNCMapCreatorBase() {};                                                 // 虚析构函数

    protected:
        std::unique_ptr<ConfigReader> pnc_map_config_; // pnc_map配置读取器
        int map_type_ = 0;                               // 地图类型
        PNCMap pnc_map_;                                 // pnc_map
        MarkerArray pnc_map_markerarray_;                // pnc_map的markerarray

        Point p_mid_, pl_, pr_;        // 车道中心点、左边界点、右边界点
        double theta_current_ = 0.0; // 当前车道的航向角
        double len_step_ = 0.0;      // 车道点之间的距离
        double theta_step_ = 0.0;    // 航向角步长
    };
} // namespace Planning
#endif // PNC_MAP_CREATOR_BASE_H_
