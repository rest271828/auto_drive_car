#include "car_move_cmd.h"

namespace Planning
{

    Planning::CarMoveCmd::CarMoveCmd():Node("car_move_cmd_node")
    {
        RCLCPP_INFO(rclcpp::get_logger(""), "CarMoveCmd initialized");
    }

} // namespace Planing

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::CarMoveCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}