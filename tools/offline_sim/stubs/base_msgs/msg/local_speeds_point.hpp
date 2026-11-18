#pragma once
namespace base_msgs::msg
{
    struct LocalSpeedsPoint { double t = 0, s_2path = 0, ds_dt_2path = 0, dds_dt_2path = 0, speed = 0, acceleration = 0, dacceleration = 0; };
}
