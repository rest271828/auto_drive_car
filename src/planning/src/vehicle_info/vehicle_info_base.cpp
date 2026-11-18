#include "vehicle_info_base.h"

namespace Planning
{
    bool VehicleBase::vehicle_cartesian_to_frenet(const Referline &refer_line)
    {
        if (refer_line.refer_line.size() < 2)
        {
            return false;
        }

        // 找投影点
        double rs = 0.0, rx = 0.0, ry = 0.0, rtheta = 0.0, rkappa = 0.0, rdkappa = 0.0;
        Curve::find_projection_point(refer_line, loc_point_, rs, rx, ry, rtheta, rkappa, rdkappa);

        // 笛卡尔转frenet
        Curve::cartesian_to_frenet(loc_point_.pose.position.x, loc_point_.pose.position.y, theta_,
                                   speed_, acceleration_, kappa_,
                                   rs, rx, ry, rtheta, rkappa, rdkappa,
                                   s_, ds_dt_, dds_dt_, l_, dl_ds_, dl_dt_, ddl_ds_, ddl_dt_);
        heading_to_refer_ = Curve::NormalizeAngle(theta_ - rtheta);

        // 投影点必须落在参考线内部
        return rs >= refer_line.refer_line.front().rs && rs <= refer_line.refer_line.back().rs;
    }

    bool VehicleBase::vehicle_cartesian_to_frenet_2path(const LocalPath &path, const std::vector<double> &path_s)
    {
        const bool inside = Curve::project_to_path(path, path_s,
                                                   loc_point_.pose.position.x, loc_point_.pose.position.y,
                                                   s_2path_, l_2path_);

        // 速度在路径切向的分量
        ds_dt_2path_ = 0.0;
        if (path.local_path.size() >= 2 && !path_s.empty())
        {
            // 找到s_2path所在点的航向
            size_t idx = 0;
            while (idx + 1 < path_s.size() && path_s[idx + 1] < s_2path_)
            {
                idx++;
            }
            const double path_theta = path.local_path[idx].theta;
            ds_dt_2path_ = speed_ * std::cos(Curve::NormalizeAngle(theta_ - path_theta));
        }
        return inside;
    }

} // namespace Planning
