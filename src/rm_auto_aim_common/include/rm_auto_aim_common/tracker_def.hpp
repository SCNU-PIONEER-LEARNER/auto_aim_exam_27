// clang-format off
#pragma once

namespace rm_auto_aim::tracker
{

namespace kf
{

enum KfState_i {
    x = 0,  // x_center
    vx = 1,
    y = 2,  // y_center
    vy = 3,
    z = 4,  // z_center
    vz = 5,
    yaw = 6,
    v_yaw = 7,

    r = 8,  // radius 半径
    dr = 9,
    dz = 10,

    outpost_dz1 = 9,
    outpost_dz2 = 10,

    dr1 = 9,
    dr2 = 10,
    dr3 = 11,

    dz1 = 12,
    dz2 = 13,
    dz3 = 14
};

} // namespace `kf` which for index

// 状态机
enum class State_e {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    CHANGE_TARGET,
};

} // namespace rm_auto_aim
