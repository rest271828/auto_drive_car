#include "polynomial_curve.h"

namespace Planning
{
    void PolynomialCurve::fit(const PolynomialType &type,
                              const double &x0, const double &dx0, const double &ddx0,
                              const double &x1, const double &dx1, const double &ddx1,
                              const double &length)
    {
        coef_.fill(0.0);
        length_ = length;
        const double T = length;
        if (T <= 1.0e-6)
        {
            coef_[0] = x1;
            return;
        }
        const double T2 = T * T;
        const double T3 = T2 * T;

        switch (type)
        {
        case PolynomialType::LINEAR:
            coef_[0] = x0;
            coef_[1] = (x1 - x0) / T;
            break;

        case PolynomialType::CUBIC:
            // f(0)=x0, f'(0)=dx0, f(T)=x1, f'(T)=dx1
            coef_[0] = x0;
            coef_[1] = dx0;
            coef_[2] = (3.0 * (x1 - x0) - (2.0 * dx0 + dx1) * T) / T2;
            coef_[3] = (-2.0 * (x1 - x0) + (dx0 + dx1) * T) / T3;
            break;

        case PolynomialType::QUINTIC:
        default:
        {
            // 闭式解: 先由起点确定c0~c2, 再解3x3线性方程得到c3~c5
            const double T4 = T3 * T;
            const double T5 = T4 * T;
            coef_[0] = x0;
            coef_[1] = dx0;
            coef_[2] = 0.5 * ddx0;

            const double h0 = x1 - (x0 + dx0 * T + 0.5 * ddx0 * T2); // 位置残差
            const double h1 = dx1 - (dx0 + ddx0 * T);                 // 速度残差
            const double h2 = ddx1 - ddx0;                            // 加速度残差

            coef_[3] = (10.0 * h0 - 4.0 * h1 * T + 0.5 * h2 * T2) / T3;
            coef_[4] = (-15.0 * h0 + 7.0 * h1 * T - h2 * T2) / T4;
            coef_[5] = (6.0 * h0 - 3.0 * h1 * T + 0.5 * h2 * T2) / T5;
            break;
        }
        }
    }

    double PolynomialCurve::value(const double &x) const
    {
        return ((((coef_[5] * x + coef_[4]) * x + coef_[3]) * x + coef_[2]) * x + coef_[1]) * x + coef_[0];
    }

    double PolynomialCurve::d1(const double &x) const
    {
        return (((5.0 * coef_[5] * x + 4.0 * coef_[4]) * x + 3.0 * coef_[3]) * x + 2.0 * coef_[2]) * x + coef_[1];
    }

    double PolynomialCurve::d2(const double &x) const
    {
        return ((20.0 * coef_[5] * x + 12.0 * coef_[4]) * x + 6.0 * coef_[3]) * x + 2.0 * coef_[2];
    }

    double PolynomialCurve::d3(const double &x) const
    {
        return (60.0 * coef_[5] * x + 24.0 * coef_[4]) * x + 6.0 * coef_[3];
    }

} // namespace Planning
