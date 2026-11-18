#ifndef LOCAL_PATH_PLANNER_H_
#define LOCAL_PATH_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_path_point.hpp"
#include "base_msgs/msg/referline.hpp"
#include "nav_msgs/msg/path.hpp"
#include "config_reader.h"
#include "curve.h"
#include "polynomial_curve.h"
#include "main_car_info.h"
#include "decision_center.h"
#include "local_path_smoother.h"

#include <memory>
#include <vector>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::LocalPathPoint;
    using base_msgs::msg::Referline;
    using nav_msgs::msg::Path;

    class LocalPathPlanner // 局部路径规划器
    {
    public:
        LocalPathPlanner();

        // 在frenet坐标系下用多项式(一次/三次/五次)生成 l(s), 再转回笛卡尔坐标
        LocalPath create_local_path(const Referline &refer_line,
                                    const std::shared_ptr<VehicleBase> &car,
                                    const std::shared_ptr<DecisionCenter> &decision);

        Path path_to_rviz() const; // 转换成rviz的Path格式

    private:
        struct Knot // l(s)的关键点
        {
            double s = 0.0;
            double l = 0.0;
            double dl = 0.0;
            double ddl = 0.0;
        };

        std::vector<Knot> build_knots(const std::shared_ptr<VehicleBase> &car,
                                      const std::shared_ptr<DecisionCenter> &decision,
                                      const double &s_end) const;

    private:
        std::unique_ptr<ConfigReader> local_path_config_;
        std::shared_ptr<LocalPathSmoother> local_path_smoother_;
        PolynomialType curve_type_ = PolynomialType::QUINTIC;
        int path_size_ = 0;
        LocalPath local_path_;
    };
} // namespace Planning
#endif // LOCAL_PATH_PLANNER_H_
