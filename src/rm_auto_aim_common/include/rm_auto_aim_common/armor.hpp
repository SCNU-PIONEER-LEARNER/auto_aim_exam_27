#pragma once

#include <array>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>
#include <vector>

#include "rm_auto_aim_common/common.hpp"

namespace rm_auto_aim
{

constexpr double PI = 3.1415926535897932384626433832795;
constexpr float PI_F = 3.1415926535897932384626433832795f;

// Armor size, Unit: m
constexpr double SMALL_ARMOR_WIDTH = 132.0 / 1000.0;
constexpr double SMALL_ARMOR_HEIGHT = 57.0 / 1000.0; // 55
constexpr double LARGE_ARMOR_WIDTH = 223.0 / 1000.0;
constexpr double LARGE_ARMOR_HEIGHT = 57.0 / 1000.0; // 55

// 15 degree in rad
constexpr double FIFTTEN_DEGREE_RAD = 15 * PI / 180;

/// @brief 尾缀数字代表模型输出的数字id，等同于枚举值，不是裁判系统的id
/// @note 注意与RoboType_i区分

// constexpr std::array ArmorTypeName{
//     "sentry", "one", "two", "three", "four", "five", "outpost", "base", "not_armor",
// };

struct Armor_s {
    Color_i color_;
    RobotType_i type_;
    ArmorSize_i size_;
    float confidence_;
    std::array<cv::Point2f, 4> armor_key_points_;
    
    // 延迟赋值
    std::optional<geometry_msgs::msg::Pose> pose_ = std::nullopt;  // camera_optical_frame系下的
    std::optional<Eigen::Vector3d> ypr_ = std::nullopt;
    std::optional<float> dist_to_img_center_ = std::nullopt;  // 符合人类操作手习惯
    std::optional<double> raw_yaw_ = std::nullopt; 

    // 可能没有
    std::optional<cv::Mat> armor_pattern_ = std::nullopt;   // 传统视觉
    std::optional<std::array<cv::Point2f, 4>> net_raw_kpts_ = std::nullopt;  // 神经网络原始角点(如果使用传统矫正的话) 
    std::optional<cv::Mat> cost_image_ = std::nullopt;      // debug

    template <typename PointType>
    static inline std::vector<PointType> buildObjectPoints(const double _width, const double _height) noexcept
    {
        return {
            PointType(0, _width / 2, _height / 2), PointType(0, -_width / 2, _height / 2),
            PointType(0, -_width / 2, -_height / 2), PointType(0, _width / 2, -_height / 2)};
    }
}; // struct Armor

enum KeyCorner_i : int { LEFT_TOP = 0, RIGHT_TOP, RIGHT_BOTTOM, LEFT_BOTTOM };

// clang-format off
const std::vector<std::tuple<Color_i, RobotType_i, ArmorSize_i>> armor_properties = {
    // 哨兵 - 小装甲
    {RED, SENTRY_7, SMALLARMOR},
    {BLUE, SENTRY_7, SMALLARMOR},
    {EXTINGUISH, SENTRY_7, SMALLARMOR},

    // 英雄 - 小装甲
    {RED, HERO_1, SMALLARMOR},
    {BLUE, HERO_1, SMALLARMOR},
    {EXTINGUISH, HERO_1, SMALLARMOR},

    // 工程 - 小装甲
    {RED, ENGINEER_2, SMALLARMOR},
    {BLUE, ENGINEER_2, SMALLARMOR},
    {EXTINGUISH, ENGINEER_2, SMALLARMOR},

    // 步兵3 - 小装甲
    {RED, INFANTRY_3, SMALLARMOR},
    {BLUE, INFANTRY_3, SMALLARMOR},
    {EXTINGUISH, INFANTRY_3, SMALLARMOR},

    // 步兵4 - 小装甲
    {RED, INFANTRY_4, SMALLARMOR},
    {BLUE, INFANTRY_4, SMALLARMOR},
    {EXTINGUISH, INFANTRY_4, SMALLARMOR},

    // 步兵5 - 小装甲
    {RED, INFANTRY_5, SMALLARMOR},
    {BLUE, INFANTRY_5, SMALLARMOR},
    {EXTINGUISH, INFANTRY_5, SMALLARMOR},

    // 前哨站 - 小装甲
    {RED, OUTPOST_11, SMALLARMOR},
    {BLUE, OUTPOST_11, SMALLARMOR},
    {EXTINGUISH, OUTPOST_11, SMALLARMOR},

    // 基地 - 大装甲
    {RED, BASE_8, LARGEARMOR},
    {BLUE, BASE_8, LARGEARMOR},
    {EXTINGUISH, BASE_8, LARGEARMOR},
    {PURPLE, BASE_8, LARGEARMOR},

    // 基地 - 小装甲
    {RED, BASE_8, SMALLARMOR},
    {BLUE, BASE_8, SMALLARMOR},
    {EXTINGUISH, BASE_8, SMALLARMOR},
    {PURPLE, BASE_8, SMALLARMOR},

    // 步兵3 - 大装甲
    {RED, INFANTRY_3, LARGEARMOR},
    {BLUE, INFANTRY_3, LARGEARMOR},
    {EXTINGUISH, INFANTRY_3, LARGEARMOR},

    // 步兵4 - 大装甲
    {RED, INFANTRY_4, LARGEARMOR},
    {BLUE, INFANTRY_4, LARGEARMOR},
    {EXTINGUISH, INFANTRY_4, LARGEARMOR},

    // 步兵5 - 大装甲
    {RED, INFANTRY_5, LARGEARMOR},
    {BLUE, INFANTRY_5, LARGEARMOR},
    {EXTINGUISH, INFANTRY_5, LARGEARMOR}};
// clang-format on

struct Light_s{
    // Light_s() = default;
    // explicit Light_s(cv::Rect _box, cv::Point2f _top, cv::Point2f _bottom, int _area, float _tilt_angle)
    //     : cv::Rect(_box), top_(_top), bottom_(_bottom), tilt_angle_(_tilt_angle)
    // {
    //     length_ = cv::norm(top_ - bottom_);
    //     width = _area / length_;
    //     center_ = (top_ + bottom_) / 2;
    // }
    Color_i color_;
    cv::Point2f top_, bottom_;
    cv::Point2f center_;
    double length_;
    double width_;
    float tilt_angle_;
};

inline KeyCorner_i & operator++(KeyCorner_i & _rt)
{
    _rt = static_cast<KeyCorner_i>(static_cast<int>(_rt) + 1);
    return _rt;
}

inline KeyCorner_i operator++(KeyCorner_i & _rt, int)
{
    KeyCorner_i old = _rt;
    _rt = static_cast<KeyCorner_i>(static_cast<int>(_rt) + 1);
    return old;
}

} // namespace rm_auto_aim
