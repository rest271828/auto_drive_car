#include "global_planner_normal.h"

namespace Planning
{
    Planning::GlobalPlannerNormal::GlobalPlannerNormal()        //普通全局路径规划
    {
        RCLCPP_INFO(rclcpp::get_logger("GlobalPlannerNormal"), "GlobalPlannerNormal initialized");
    }
    Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map)
    {
        return global_path_;
    }
}