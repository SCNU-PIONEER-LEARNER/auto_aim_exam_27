#pragma once

#include <Eigen/Core>
#include <array>
#include <string>
#include <string_view>

#include "rm_interfaces/msg/armor.hpp"

namespace rm_auto_aim
{

enum Color_i { RED = 0, BLUE, EXTINGUISH, PURPLE };
const std::array<std::string, 4> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorSize_i { LARGEARMOR, SMALLARMOR };
const std::array<std::string, 2> ARMOR_TYPES = {"big", "small"};


/// @brief 目标类型的枚举索引
/// @note 索引自带的序号为裁判系统赋予的序号，不是target_的对应索引，索引请以枚举值为准，注意与ArmorType_e区分
enum RobotType_i {
    NONE_0 = 0,
    HERO_1,
    ENGINEER_2,
    INFANTRY_3,
    INFANTRY_4,
    INFANTRY_5, // old
    SENTRY_7,
    BASE_8,
    OUTPOST_11,
    //! 如果有新增目标，那么下面的ARMOR_NAMES数组和parseArmorType函数都要修改

    TARGET_COUNT // 始终放在最后，保证枚举值不会大于数组大小
};

// clang-format off
inline constexpr std::array<std::string_view, TARGET_COUNT> ARMOR_NAMES = {
    "not_armor",  // 0
    "one",
    "two",
    "three",
    "four",
    "five",
    "sentry",
    "base",
    "outpost"
};
// clang-format on

inline constexpr std::array<ArmorSize_i, TARGET_COUNT> ARMOR_SIZE_MIRROR{
    SMALLARMOR, LARGEARMOR, SMALLARMOR, SMALLARMOR, SMALLARMOR,
    SMALLARMOR, SMALLARMOR, SMALLARMOR, SMALLARMOR,
};

// 装甲板类型
enum ArmorsNum_i { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3, BASE_1 = 1 };
// enum class ArmorSize_e { SMALL = 0, LARGE = 1 };

inline RobotType_i parseArmorType(std::string_view _armor_name)
{
    for (std::size_t i = 0; i < ARMOR_NAMES.size(); ++i) {
        if (ARMOR_NAMES[i] == _armor_name) {
            return static_cast<RobotType_i>(i);
        }
    }
    return RobotType_i::NONE_0;
}

inline RobotType_i parseArmorType(const rm_interfaces::msg::Armor & _armor)
{
    return parseArmorType(_armor.number);
}

inline RobotType_i & operator++(RobotType_i & _rt)
{
    _rt = static_cast<RobotType_i>(static_cast<int>(_rt) + 1);
    return _rt;
}
inline RobotType_i operator++(RobotType_i & _rt, int)
{
    RobotType_i old = _rt;
    _rt = static_cast<RobotType_i>(static_cast<int>(_rt) + 1);
    return old;
}

} // namespace rm_auto_aim
