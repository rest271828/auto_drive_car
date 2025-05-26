#ifndef global_planner_base_h_
#define global_planner_base_h_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "base_msgs/msg/pnc_map.hpp"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;
    
    enum class GlobalPlannerType
    {
        NORMAL
    };

    class GlobalPlannerBase
    {
    public:
        GlobalPlannerBase() ;
        virtual Path search_global_path(const PNCMap &pnc_map) = 0;
        inline Path global_path() const { return global_path_; } //获取全局路径
        virtual ~GlobalPlannerBase() = default;

    private:

    protected:
        std::unique_ptr<ConfigReader> global_planner_config_;  //配置文件读取
        int global_planner_type_ = 0; //全局规划器类型
        Path global_path_; //全局路径
    };
}  // namespace Planning
#endif  // global_planner_base_h_
