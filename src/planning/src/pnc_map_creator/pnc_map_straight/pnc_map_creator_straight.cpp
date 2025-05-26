#include "pnc_map_creator_straight.h"

namespace Planning
{
    PNCMapCreatorStraight::PNCMapCreatorStraight()
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PncMapCreatorStraight node has been created.");
    }
    PNCMap PNCMapCreatorStraight::create_pnc_map()
    {
        return PNCMap();
    }
} // namespace Planning
