#include "obs_car_info.h"

namespace Planning { 
    ObsCarInfo::ObsCarInfo()
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "ObsCarInfo node has been created.");
    }
}// namespace Planning