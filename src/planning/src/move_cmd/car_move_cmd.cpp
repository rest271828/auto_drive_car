#include "car_move_cmd.h"

namespace Planning
{
    CarMoveCmd::CarMoveCmd() : Node("car_move_cmd_node") // 主车运动指令
    {
        RCLCPP_INFO(this->get_logger(), "car_move_cmd_node created");

        car_ = std::make_shared<MainCar>(); // 读取主车初始位姿
        ConfigReader config;
        config.read_pnc_map_config();
        map_frame_ = config.pnc_map().frame_;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        trajectory_sub_ = this->create_subscription<LocalTrajectory>(
            "local_trajectory", 10,
            std::bind(&CarMoveCmd::trajectory_callback, this, std::placeholders::_1));

        // 50Hz发布主车位姿
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                         std::bind(&CarMoveCmd::timer_callback, this));
    }

    void CarMoveCmd::trajectory_callback(const LocalTrajectory::SharedPtr msg)
    {
        if (msg->local_trajectory.empty())
        {
            return;
        }
        trajectory_ = *msg;
        trajectory_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
        has_trajectory_ = true;
    }

    void CarMoveCmd::timer_callback()
    {
        TransformStamped ts;
        ts.header.stamp = this->now();
        ts.header.frame_id = map_frame_;
        ts.child_frame_id = car_->child_frame();

        if (has_trajectory_)
        {
            // 按收到轨迹后经过的时间, 在轨迹上插值得到当前位姿
            const double t = (this->now() - trajectory_time_).seconds();
            const auto p = LocalTrajectoryCombiner::interpolate_by_time(trajectory_, t);
            ts.transform.translation.x = p.path_point.pose.pose.position.x;
            ts.transform.translation.y = p.path_point.pose.pose.position.y;
            ts.transform.rotation = p.path_point.pose.pose.orientation;
        }
        else
        {
            // 还没有轨迹: 停在初始位姿
            ts.transform.translation.x = car_->loc_point().pose.position.x;
            ts.transform.translation.y = car_->loc_point().pose.position.y;
            ts.transform.rotation = car_->loc_point().pose.orientation;
        }
        ts.transform.translation.z = 0.0;
        tf_broadcaster_->sendTransform(ts);
    }

} // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::CarMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
