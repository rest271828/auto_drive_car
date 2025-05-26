#include "pnc_map_creator_sturn.h"

namespace Planning
{
    Planning::PNCMapCreatorSTurn::PNCMapCreatorSTurn()
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PncMapCreatorSTurn node has been created.");
    }

    PNCMap PNCMapCreatorSTurn::create_pnc_map()
    { 
        return pnc_map_;
    }
}//namespace Planning