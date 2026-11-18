#pragma once
#include <cmath>
namespace tf2
{
    class Quaternion
    {
    public:
        void setRPY(double, double, double yaw) { z_ = std::sin(yaw / 2); w_ = std::cos(yaw / 2); }
        double getX() const { return 0.0; }
        double getY() const { return 0.0; }
        double getZ() const { return z_; }
        double getW() const { return w_; }
    private:
        double z_ = 0.0, w_ = 1.0;
    };
}
