#include "planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_node")
    {
        RCLCPP_INFO(this->get_logger(), "Planning node created");

        //test
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        auto test = process_config_->process().obs_dis_;
        RCLCPP_INFO(this->get_logger(), "obs_dis: %.2f", test);
    }

    bool PlanningProcess::process()
    {
        return true;
    }
}// namespace Planning