#include "obs_move_cmd.h"

#include <cmath>

namespace Planning
{
    ObsMoveCmd::ObsMoveCmd() : Node("obs_move_cmd_node") // 障碍物运动指令
    {
        RCLCPP_INFO(this->get_logger(), "obs_move_cmd_node created");

        for (int i = 0; i < 3; i++)
        {
            obses_.emplace_back(std::make_shared<ObsCar>(i + 1));
        }
        ConfigReader config;
        config.read_pnc_map_config();
        map_frame_ = config.pnc_map().frame_;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        marker_pub_ = this->create_publisher<MarkerArray>("obs_markers", 10);
        start_time_ = this->now();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                         std::bind(&ObsMoveCmd::timer_callback, this));
    }

    void ObsMoveCmd::timer_callback()
    {
        const auto now = this->now();
        const double t = (now - start_time_).seconds();
        for (const auto &obs : obses_)
        {
            TransformStamped ts;
            ts.header.stamp = now;
            ts.header.frame_id = map_frame_;
            ts.child_frame_id = obs->child_frame();
            ts.transform.translation.x = obs->loc_point().pose.position.x + obs->speed() * std::cos(obs->theta()) * t;
            ts.transform.translation.y = obs->loc_point().pose.position.y + obs->speed() * std::sin(obs->theta()) * t;
            ts.transform.translation.z = 0.0;
            ts.transform.rotation = obs->loc_point().pose.orientation;
            tf_broadcaster_->sendTransform(ts);
        }

        // 每0.5s发布一次障碍物外形
        if (tick_++ % 25 != 0)
        {
            return;
        }
        MarkerArray markers;
        for (const auto &obs : obses_)
        {
            Marker m;
            m.header.stamp = now;
            m.header.frame_id = obs->child_frame();
            m.ns = "obs_car";
            m.id = obs->id();
            m.type = Marker::CUBE;
            m.action = Marker::ADD;
            m.frame_locked = true; // 跟随tf运动
            m.pose.position.z = 0.6;
            m.pose.orientation.w = 1.0;
            m.scale.x = obs->length();
            m.scale.y = obs->width();
            m.scale.z = 1.2;
            m.color.r = 0.2;
            m.color.g = 0.2;
            m.color.b = 0.8;
            m.color.a = 0.9;
            markers.markers.emplace_back(m);
        }
        marker_pub_->publish(markers);
    }

} // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::ObsMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
