#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::PNCMap;          // 使用base_msgs中的PNCMap消息类型
    using geometry_msgs::msg::PoseStamped; // 使用geometry_msgs中的PoseStamped消息类型
    using nav_msgs::msg::Path;             // 使用nav_msgs中的Path消息类型

    enum class GlobalPlannerType // 全局路径规划器类型枚举
    {
        NORMAL = 0, // 普通全局路径规划器
        // 可以添加其他类型的全局路径规划器
    };

    class GlobalPlannerBase // 全局路径规划器基类
    {
    public:
        virtual Path search_global_path(const PNCMap &pnc_map) = 0; // 搜索全局路径的纯虚函数
        inline Path global_path() const { return global_path_; } // 获取全局路径的内联函数
        virtual ~GlobalPlannerBase() {} // 虚析构函数

    protected:
        std::unique_ptr<ConfigReader> global_planner_config_; // 配置读取器指针
        int global_planner_type_ = 0;                        // 全局路径规划器类型
        Path global_path_;                                   // 全局路径
    };
} // namespace Planning
#endif // GLOBAL_PLANNER_BASE_H_
