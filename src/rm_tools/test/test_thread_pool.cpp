#include "rm_tools/thread_pool.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>

namespace
{
struct QueueItem
{
  int id = 0;
  int value = 0;
};
}  // namespace

TEST(ThreadPool, TryEnqueueReturnsFalseWhenQueueIsFull)
{
  rm_tools::ThreadPool pool(0, 1);

  EXPECT_TRUE(pool.try_enqueue([] {}));
  EXPECT_FALSE(pool.try_enqueue([] {}));
  EXPECT_EQ(pool.queue_size(), 1u);
}

TEST(ThreadPool, ExecutesQueuedTask)
{
  using namespace std::chrono_literals;

  rm_tools::ThreadPool pool(1, 4);
  std::promise<int> done;
  auto future = done.get_future();

  pool.enqueue([&done] { done.set_value(42); });

  ASSERT_EQ(future.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(future.get(), 42);
}

TEST(OrderedQueue, DequeuesItemsInIdOrder)
{
  rm_tools::OrderedQueue<QueueItem> queue;

  queue.enqueue(QueueItem{1, 20});

  QueueItem item;
  EXPECT_FALSE(queue.try_dequeue(item));

  queue.enqueue(QueueItem{0, 10});

  ASSERT_TRUE(queue.try_dequeue(item));
  EXPECT_EQ(item.id, 0);
  EXPECT_EQ(item.value, 10);

  ASSERT_TRUE(queue.try_dequeue(item));
  EXPECT_EQ(item.id, 1);
  EXPECT_EQ(item.value, 20);
}

TEST(OrderedQueue, DequeueTimeoutReturnsFalseWhenEmpty)
{
  rm_tools::OrderedQueue<QueueItem> queue;

  QueueItem item;
  EXPECT_FALSE(queue.dequeue_timeout(item, std::chrono::milliseconds(1)));
}
