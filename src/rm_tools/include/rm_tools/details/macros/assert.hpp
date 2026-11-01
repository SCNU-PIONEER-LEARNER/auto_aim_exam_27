#pragma once

#include <cassert>
#include <rclcpp/logging.hpp>

#ifndef NDEBUG

#define ASSERT(cond, logger, msg)                          \
    do {                                                   \
        if (!cond) [[unlikely]] {                          \
            RCLCPP_FATAL(rclcpp::get_logger(logger), msg); \
            assert(false && msg);                          \
        }                                                  \
    } while (0)

#endif