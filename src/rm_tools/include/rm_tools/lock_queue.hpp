#pragma once

#include <queue>
#include <mutex>
#include <stop_token>
#include <condition_variable>
#include <optional>
#include <utility>

namespace rm_tools
{

enum WhetherBlock {
    BLOCKING,
    NON_BLOCKING,
};

/// @brief 带锁的队列，提供的每一个函数接口都是带锁的，多线程下安全

/// @brief 公共基类，包含共享的数据成员和函数
template <typename T>
class LockedQueueBase
{
protected:
    mutable std::mutex mtx_;
    std::queue<T> q_;

public:
    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return q_.size();
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return q_.empty();
    }

    // 保护性的 push 实现，由子类调用
    template <typename U>
    void push(U && _u)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        q_.push(std::forward<U>(_u));
    }

    // 保护性的 try_pop 实现
    std::optional<T> try_pop()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (q_.empty()) {
            return std::nullopt;
        }
        std::optional<T> value(std::move(q_.front()));
        q_.pop();
        return value;
    }

    LockedQueueBase() = default;
    LockedQueueBase(const LockedQueueBase &) = delete;
    LockedQueueBase & operator=(const LockedQueueBase &) = delete;
};

template <typename T, WhetherBlock block>
class LockedQueue;

// 阻塞版本
template <typename T>
class LockedQueue<T, WhetherBlock::BLOCKING> : private LockedQueueBase<T>
{
    using Base = LockedQueueBase<T>;
    std::condition_variable_any cv_;

public:
    template <typename U>
    void push(U && _u)
    {
        Base::push(std::forward<U>(_u));
        cv_.notify_one();
    }

    std::optional<T> pop(std::stop_token _st)
    {
        std::unique_lock<std::mutex> lock(Base::mtx_);
        cv_.wait(lock, _st, [this] { return !Base::q_.empty(); });
        if (Base::q_.empty()) {
            return std::nullopt;
        } else {
            std::optional<T> value(std::move(Base::q_.front()));
            Base::q_.pop();
            return value;
        }
    }

    ~LockedQueue() { cv_.notify_all(); }

    using Base::try_pop;
    using Base::empty;
    using Base::size;
};

// 非阻塞版本
template <typename T>
class LockedQueue<T, WhetherBlock::NON_BLOCKING> : private LockedQueueBase<T>
{
    using Base = LockedQueueBase<T>;

public:
    template <typename U>
    void push(U && _u)
    {
        Base::push(std::forward<U>(_u));
    }

    std::optional<T> pop() { return Base::try_pop(); }

    using Base::empty;
    using Base::size;
};
} // namespace rm_tools
