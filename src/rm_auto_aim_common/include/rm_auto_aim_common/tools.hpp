#pragma once

#include <cmath>

namespace rm_auto_aim::tools
{

constexpr double PI = 3.1415926535897932384626433832795;

/// @brief 由三角形直角边长计算斜边长，同时有三维版的重载
inline double hypot(double x, double y) { return std::sqrt(x * x + y * y); }
inline double hypot(double x, double y, double z) { return std::sqrt(x * x + y * y + z * z); }

/// @brief 返回输入值的平方
template <typename T> constexpr inline T square(T x) { return x * x; }

/// @brief 将弧度值限制在(-pi, pi]
constexpr inline double limitRad(double angle)
{
    while (angle > PI)
        angle -= 2 * PI;
    while (angle <= -PI)
        angle += 2 * PI;
    return angle;
}

constexpr inline double limitMinMax(double input, double min, double max)
{
    if (input < min)
        return min;
    if (input > max)
        return max;
    return input;
}

inline float sigmoid(float x)
{
    if (x > 0.0f)
        return 1.0f / (1.0f + std::exp(-x));
    else {
        float exp_x = std::exp(x);
        return exp_x / (exp_x + 1.0f);
    }
}

} // namespace rm_auto_aim::tools
