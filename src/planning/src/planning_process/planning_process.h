#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class PlanningProcess : public rclcpp::Node
    {
    public:
        PlanningProcess();

        bool process();     // 规划总流程

    private:

    };
}  // namespace Planning
#endif  // PLANNING_PROCESS_H_