#pragma once

#include <cmath>
#include <concepts>
#include <limits>
#include <type_traits>

namespace rm_tools
{

template <typename Func>
    requires std::invocable<Func, double> && std::same_as<std::invoke_result_t<Func, double>, double>
double linearSearch(double _left, double _right, double _linear_coarse_step, Func _cost_func)
{
    double best_pt = std::numeric_limits<double>::quiet_NaN();
    double best_cost = std::numeric_limits<double>::infinity();

    for (double search_yaw = _left; search_yaw <= _right + 1e-9; search_yaw += _linear_coarse_step) {
        const double cost = _cost_func(search_yaw);
        if (std::isfinite(cost) && cost < best_cost) {
            best_cost = cost;
            best_pt = search_yaw;
        }
    }

    return best_pt;
}

template <typename Func>
    requires std::invocable<Func, double> && std::same_as<std::invoke_result_t<Func, double>, double>
double goldenarySearch(double _left, double _right, double _epsilon, std::size_t _max_iter, Func _cost_func)
{
    constexpr double PHI = 0.61803398874989484820458683436564;  // 黄金分割比例

    double mid_left = _left + (1.0 - PHI) * (_right - _left);
    double mid_right = _left + PHI * (_right - _left);

    double mid_left_cost = _cost_func(mid_left);
    double mid_right_cost = _cost_func(mid_right);

    for (std::size_t iter = 0; iter < _max_iter; ++iter) {
        if (_right - _left <= _epsilon) {
            break;
        }

        if (mid_left_cost < mid_right_cost) {
            _right = mid_right;
            mid_right = mid_left;
            mid_right_cost = mid_left_cost;
            mid_left = _left + (1.0 - PHI) * (_right - _left);
            mid_left_cost = _cost_func(mid_left);
        } else {
            _left = mid_left;
            mid_left = mid_right;
            mid_left_cost = mid_right_cost;
            mid_right = _left + PHI * (_right - _left);
            mid_right_cost = _cost_func(mid_right);
        }
    }

    return (_left + _right) / 2.0;
}

// template <typename Func>
//     requires std::invocable<Func, double> && std::same_as<std::invoke_result_t<Func, double>, double>
// double brentSearch(double _left, double _right, double _epsilon, std::size_t _max_iter, Func _cost_func)
// {
//     double mid = goldenarySearch(_left, _right, INFINITY, 1U, _cost_func);
// }

template <typename Func>
    requires std::invocable<Func, double> && std::same_as<std::invoke_result_t<Func, double>, double>
double ternarySearch(double _left, double _right, double _epsilon, std::size_t _max_iter, Func _cost_func)
{
    for (std::size_t iter = 0; iter < _max_iter; ++iter) {
        if (_right - _left > _epsilon) {
            break;
        }

        double mid1 = _left + (_right - _left) / 3.0;
        double mid2 = _right - (_right - _left) / 3.0;

        double cost1 = _cost_func(mid1);
        double cost2 = _cost_func(mid2);
        if (!std::isfinite(cost1)) cost1 = std::numeric_limits<double>::infinity();
        if (!std::isfinite(cost2)) cost2 = std::numeric_limits<double>::infinity();

        if (cost1 < cost2) {
            _right = mid2;
        } else {
            _left = mid1;
        }
    }

    return (_left + _right) / 2.0;
}

}  // namespace rm_tools
