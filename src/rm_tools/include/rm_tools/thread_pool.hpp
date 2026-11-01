#ifndef RM_TOOLS__THREAD_POOL_HPP_
#define RM_TOOLS__THREAD_POOL_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

namespace rm_tools
{
template <typename FrameT>
class OrderedQueue
{
public:
  OrderedQueue() : current_id_(0), shutdown_(false) {}

  ~OrderedQueue() { shutdown(); }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
      main_queue_ = std::queue<FrameT>();
      buffer_.clear();
    }
    cond_var_.notify_all();
  }

  void enqueue(const FrameT & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) return;
    if (item.id < current_id_) {
      RCLCPP_WARN(rclcpp::get_logger("ordered_queue"), "too small id!");
      return;
    }

    if (item.id == current_id_) {
      main_queue_.push(item);
      current_id_++;

      auto it = buffer_.find(current_id_);
      while (it != buffer_.end()) {
        main_queue_.push(it->second);
        buffer_.erase(it);
        current_id_++;
        it = buffer_.find(current_id_);
      }

      if (main_queue_.size() >= 1) {
        cond_var_.notify_one();
      }
    } else {
      buffer_[item.id] = item;
    }
  }

  FrameT dequeue()
  {
    std::unique_lock<std::mutex> lock(mutex_);

    cond_var_.wait(lock, [this]() { return shutdown_ || !main_queue_.empty(); });

    if (shutdown_) {
      return FrameT{};
    }

    FrameT item = std::move(main_queue_.front());
    main_queue_.pop();
    return item;
  }

  bool dequeue_timeout(
    FrameT & item, std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!cond_var_.wait_for(
          lock, timeout, [this]() { return shutdown_ || !main_queue_.empty(); })) {
      return false;
    }

    if (shutdown_ || main_queue_.empty()) {
      return false;
    }

    item = std::move(main_queue_.front());
    main_queue_.pop();
    return true;
  }

  bool has_frames_ready() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !main_queue_.empty();
  }

  bool try_dequeue(FrameT & item)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (main_queue_.empty()) {
      return false;
    }
    item = std::move(main_queue_.front());
    main_queue_.pop();
    return true;
  }

  size_t get_size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return main_queue_.size() + buffer_.size();
  }

  void print_status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(
      rclcpp::get_logger("ordered_queue"),
      "OrderedQueue Status: current_id=%d, main_queue=%zu, buffer=%zu", current_id_,
      main_queue_.size(), buffer_.size());
  }

  int current_id_;

private:
  std::queue<FrameT> main_queue_;
  std::unordered_map<int, FrameT> buffer_;
  mutable std::mutex mutex_;
  std::condition_variable cond_var_;
  bool shutdown_;
};

class ThreadPool
{
public:
  ThreadPool(size_t num_threads, size_t max_queue_size)
  : max_queue_size_(max_queue_size), stop(false)
  {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) {
              return;
            }
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();
        }
      });
    }
  }

  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
      tasks = std::queue<std::function<void()>>();
    }
    condition.notify_all();
    for (std::thread & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  template <class F>
  void enqueue(F && f)
  {
    if (!try_enqueue(std::forward<F>(f))) {
      throw std::runtime_error("ThreadPool queue is full");
    }
  }

  template <class F>
  bool try_enqueue(F && f)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      if (tasks.size() >= max_queue_size_) {
        return false;
      }
      tasks.emplace(std::forward<F>(f));
    }
    condition.notify_one();
    return true;
  }

  size_t queue_size() const
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    return tasks.size();
  }

private:
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;
  mutable std::mutex queue_mutex;
  std::condition_variable condition;
  size_t max_queue_size_;
  bool stop;
};
}  // namespace rm_tools

#endif  // RM_TOOLS__THREAD_POOL_HPP_
