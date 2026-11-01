#pragma once

// #include的本质是代码的复制粘贴，所以这样写是可以的
// rm_vision_bringup里的只是个符号链接，指向rm_tools/include/rm_tools/compile_settings.hpp
// 写在rm_tools里是因为这些都依赖rm_tools但是不依赖rm_vision_bringup

// 这个静态断言不对，C++14有可以用的方法
// static_assert(__cplusplus >= 201703L, "C++版本过低，请提升至C++17或以上版本");

// !改了这个头文件就一定需要重新编译rm_tools包

// tracker.cpp的卡方检测，61行
#ifndef ENABLE_CHI_SQUARE_TEST__
#define ENABLE_CHI_SQUARE_TEST__ false
#else 
    #error "ENABLE_CHI_SQUARE_TEST__ 宏被占用"
#endif  // ENABLE_CHI_SQUARE_TEST__


// rm_planner.cpp的pitch轴mpc求解器，太多辣标不出来
// 改了这个还要重新编译rm_planner包
#ifndef ENABLE_PITCH_TINY_MPC_SOLVER__
#define ENABLE_PITCH_TINY_MPC_SOLVER__ false
#else
    #error "ENABLE_PITCH_TINY_MPC_SOLVER__ 宏被占用"
#endif  // ENABLE_PITCH_TINY_MPC_SOLVER__

// 检测模式，0单线程，1异步推理，2线程池，3传统视觉
// 改了这个要重新编译armor_detector，做这个主要是可以减少编译时间
#ifndef DETECT_MODE__
#define DETECT_MODE__  3
#else
    #error "DETECT_MODE__ 宏被占用"
#endif  // DETECT_MODE__
