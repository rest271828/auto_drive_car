#ifndef POLYNOMIAL_CURVE_H_
#define POLYNOMIAL_CURVE_H_

#include <array>
#include <cmath>

namespace Planning
{
    enum class PolynomialType // 多项式类型, 与配置 local_path.curve_type 对应
    {
        LINEAR = 0,  // 一次: 只约束起终点的值
        CUBIC = 1,   // 三次: 约束起终点的值和一阶导
        QUINTIC = 2, // 五次: 约束起终点的值、一阶导和二阶导
    };

    // 一维多项式曲线 f(x) = c0 + c1*x + ... + c5*x^5, x ∈ [0, length]
    // 在局部路径中 x 为相对起点的 s 增量, f 为横向偏移 l
    class PolynomialCurve
    {
    public:
        PolynomialCurve() = default;

        // 按边界条件拟合; length必须大于0
        void fit(const PolynomialType &type,
                 const double &x0, const double &dx0, const double &ddx0,
                 const double &x1, const double &dx1, const double &ddx1,
                 const double &length);

        double value(const double &x) const;  // f(x)
        double d1(const double &x) const;     // f'(x)
        double d2(const double &x) const;     // f''(x)
        double d3(const double &x) const;     // f'''(x)
        inline double length() const { return length_; }

    private:
        std::array<double, 6> coef_{}; // c0..c5
        double length_ = 0.0;
    };
} // namespace Planning
#endif // POLYNOMIAL_CURVE_H_
