#pragma once

#include <cmath>
#include <concepts>
#include <type_traits>

namespace rm_tools
{

/// @brief 线性搜索下凸函数最小值点
/// @param _linear_coarse_step 搜索步长
/// @param _cost_func 要求其只接受一个double类型参数，并返回cost
/// @return 最小值点
template <typename Func>
    requires std::invocable<Func, double> && std::same_as<std::invoke_result_t<Func, double>, double>
double linearSearch(double _left, double _right, double _linear_coarse_step, Func _cost_func);

/// @brief 黄金分割搜索下凸函数最小值点
/// @param _epsilon 搜索精度
/// @param _max_iter 最大迭代次数
/// @param _cost_func 要求其只接受一个double类型参数，并返回cost
/// @return 最小值点
template <typename Func>
    requires std::invocable<Func, double> && std::same_as<std::invoke_result_t<Func, double>, double>
double goldenarySearch(double _left, double _right, double _epsilon, std::size_t _max_iter, Func _cost_func);

/// @brief 三分搜索下凸函数最小值点
/// @note 现在更推荐使用黄金分割搜索，每轮迭代能减少一次代价函数计算
/// @param _cost_func 要求其只接受一个double类型参数，并返回cost
/// @param _epsilon 搜索精度
/// @return 最小值点
template <typename Func>
    requires std::invocable<Func, double> && std::same_as<std::invoke_result_t<Func, double>, double>
double ternarySearch(double _left, double _right, double _epsilon, std::size_t _max_iter, Func _cost_func);

}  // namespace rm_tools

#include "rm_tools/details/algorithm.tpp"  // 实现