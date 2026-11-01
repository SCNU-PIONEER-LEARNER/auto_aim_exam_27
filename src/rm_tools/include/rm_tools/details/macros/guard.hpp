/**
 * @file guard.hpp
 * @brief 提供一组用于前置条件检查的 GUARD 宏。
 *
 * @details
 * 本文件封装了 GUARD_LOG、GUARD_RET_VOID、GUARD_RET_VAL、GUARD_BREAK、
 * GUARD_CONTINUE、GUARD_THROW 等宏，目的是把常见的“判断 + 记录日志 +
 * 执行动作”这类三行以上的样板代码压缩成一行，同时让意图一目了然。
 *
 * 典型的裸写方式如下：
 * @code
 * if (!cond) {
 *     RCLCPP_ERROR(logger, "some message: %d", value);
 *     return;
 * }
 * @endcode
 *
 * 使用 GUARD 宏后：
 * @code
 * GUARD_RET_VOID(cond, "logger_name", "some message: %d", value);
 * @endcode
 *
 * 这样做的好处：
 * - **减少样板代码**：把多行 if 判断压缩为一行，降低阅读负担。
 * - **意图清晰**：宏名直接表达“失败时做什么”（记录、返回、抛出、跳出循环），
 *   读者无需逐行分析 if 块内部逻辑。
 * - **行为一致**：所有检查统一使用 RCLCPP_ERROR 记录，且带有 [[unlikely]]
 *   提示，便于编译器优化分支预测。
 * - **防止误用**：通过 do { ... } while (0) 或 if/else ((void)0) 包裹，
 *   确保宏在任意上下文（如 if-else 链、无花括号语句）中都能安全展开。
 *
 * @note 条件 cond 为“期望成立”的条件，宏内部使用 if (!cond) 判断失败分支。
 *
 * @warning 宏参数中的 msg 与 ... 会被直接展开到 RCLCPP_ERROR 中，
 * 请确保格式串与参数匹配，且 logger 名称有效。
 */
#pragma once


#include <rclcpp/logging.hpp>

#define GUARD_LOG_IMPL(logger, msg, ...) RCLCPP_ERROR(rclcpp::get_logger(logger), msg, ...)

#define GUARD_LOG(cond, logger, msg, ...)    \
    do {                                     \
        if (!cond) [[unlikely]] {            \
            GUARD_LOG_IMPL(logger, msg, ...) \
        }                                    \
    } while (0)

#define GUARD_RET_VOID(cond, logger, msg, ...) \
    do {                                       \
        if (!cond) [[unlikely]] {              \
            GUARD_LOG_IMPL(logger, msg, ...)   \
            return;                            \
        }                                      \
    } while (0)

#define GUARD_RET_VAL(cond, default_val, logger, msg, ...) \
    do {                                                   \
        if (!cond) [[unlikely]] {                          \
            GUARD_LOG_IMPL(logger, msg, ...)               \
            return default_val;                            \
        }                                                  \
    } while (0)

#define GUARD_BREAK(cond, logger, msg, ...) \
    if (!cond) [[unlikely]] {               \
        GUARD_LOG_IMPL(logger, msg, ...);   \
        break;                              \
    } else                                  \
        ((void)0)

#define GUARD_CONTINUE(cond, logger, msg, ...) \
    if (!cond) [[unlikely]] {                  \
        GUARD_LOG_IMPL(logger, msg, ...);      \
        continue;                              \
    } else                                     \
        ((void)0)


#define GUARD_THROW(cond, ex, logger, msg, ...) \
    do {                                        \
        if (!cond) [[unlikely]] {               \
            GUARD_LOG_IMPL(logger, msg, ...);   \
            throw ex;                           \
        }                                       \
    } while (0)
