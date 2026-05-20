#ifndef TEST_H__
#define TEST_H__

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

namespace Planning
{
class OsqpTest : public rclcpp::Node
{
public:
    OsqpTest();
    void test_problem();

private:

};
}  // namespace Planning
#endif  // TEST_H__
