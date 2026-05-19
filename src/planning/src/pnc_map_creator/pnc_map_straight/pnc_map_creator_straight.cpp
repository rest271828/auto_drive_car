#include "pnc_map_creator_straight.h"

namespace Planning
{
    PNCMapCreatorStraight::PNCMapCreatorStraight() // 直道地图
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_creator created");
        
        pnc_map_config_ = std::make_unique<ConfigReader>(); // 创建配置读取器
        pnc_map_config_->read_pnc_map_config();
        map_type_ = static_cast<int>(PNCMapType::STRAIGHT); // 获取地图类型

        // 初始化地图
        p_mid_.x = -3.0;
        p_mid_.y = pnc_map_config_->pnc_map().road_half_width_ / 2.0;

        // 长度步长
        len_step_ = pnc_map_config_->pnc_map().segment_len_;

        // 初始化
        init_pnc_map();

    }

    PNCMap PNCMapCreatorStraight::create_pnc_map()
    {
        draw_straight_x(pnc_map_.road_length, 1.0);

        // 保证pnc_map_.midline.points为偶数,否则rviz显示不出
        if (pnc_map_.midline.points.size() % 2 == 1)
        {
            pnc_map_.midline.points.pop_back();
        }
    
        // 把所有marker放入pnc_map_markerarray_中
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.midline);
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.left_boundary);
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.right_boundary);

        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map created, midline points: %ld", pnc_map_.midline.points.size());
        return pnc_map_;
    }

    void PNCMapCreatorStraight::init_pnc_map()
    {
        pnc_map_.header.frame_id = pnc_map_config_->pnc_map().frame_;
        pnc_map_.header.stamp = rclcpp::Clock().now();
        pnc_map_.road_length = pnc_map_config_->pnc_map().road_length_;
        pnc_map_.road_half_width = pnc_map_config_->pnc_map().road_half_width_;

        // 中心线格式
        pnc_map_.midline.header = pnc_map_.header;
        pnc_map_.midline.ns = "pnc_map";
        pnc_map_.midline.id = 0;
        pnc_map_.midline.action = Marker::ADD;
        pnc_map_.midline.type = Marker::LINE_LIST;
        pnc_map_.midline.scale.x = 0.05;// 线宽
        pnc_map_.midline.color.a = 1.0;// 不透明
        pnc_map_.midline.color.r = 0.7;// 红色
        pnc_map_.midline.color.g = 0.7;// 绿色
        pnc_map_.midline.color.b = 0.7;// 蓝色

        // 左边界格式
        pnc_map_.left_boundary = pnc_map_.midline;
        pnc_map_.left_boundary.id = 1;
        pnc_map_.left_boundary.type = Marker::LINE_STRIP;
        pnc_map_.left_boundary.color.r = 1.0;// 红色
        pnc_map_.left_boundary.color.g = 0.0;// 绿色
        pnc_map_.left_boundary.color.b = 0.0;// 蓝色

        // 右边界格式
        pnc_map_.right_boundary = pnc_map_.left_boundary;
        pnc_map_.right_boundary.id = 2;

    }

    void PNCMapCreatorStraight::draw_straight_x(const double &length, const double &plus_flag, const double &ratio)
    {
        double len_tmp = 0.0;
        while (len_tmp < length)
        {
            pl_.x = p_mid_.x;
            pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_;
            pr_.x = p_mid_.x;
            pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_;

            pnc_map_.midline.points.emplace_back(p_mid_);
            pnc_map_.left_boundary.points.emplace_back(pl_);
            pnc_map_.right_boundary.points.emplace_back(pr_);

            len_tmp += len_step_ * ratio;
            p_mid_.x += len_step_ * plus_flag;
        }
    }

} // namespace Planning