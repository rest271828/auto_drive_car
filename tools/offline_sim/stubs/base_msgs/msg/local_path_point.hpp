#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace base_msgs::msg
{
    struct LocalPathPoint
    {
        geometry_msgs::msg::PoseStamped pose;
        double s = 0, l = 0, ds_dt = 0, dl_ds = 0, dl_dt = 0, dds_dt = 0, ddl_ds = 0, ddl_dt = 0;
        double theta = 0, kappa = 0, dkappa = 0, rs = 0, rtheta = 0, rkappa = 0, rdkappa = 0;
    };
}
