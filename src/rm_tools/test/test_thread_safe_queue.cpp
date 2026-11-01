#include "rm_tools/thread_safe_queue.hpp"

#include <gtest/gtest.h>

#include <atomic>

TEST(ThreadSafeQueue, TryPopReturnsFalseWhenEmpty)
{
  rm_tools::ThreadSafeQueue<int> queue(2);

  int value = 0;
  EXPECT_FALSE(queue.try_pop(value));
  EXPECT_FALSE(queue.try_pop().has_value());
  EXPECT_TRUE(queue.empty());
}

TEST(ThreadSafeQueue, CallsFullHandlerAndDropsNewValueWhenFull)
{
  std::atomic<int> full_count{0};
  rm_tools::ThreadSafeQueue<int> queue(2, [&full_count] { full_count++; });

  queue.push(1);
  queue.push(2);
  queue.push(3);

  EXPECT_EQ(full_count.load(), 1);
  EXPECT_EQ(queue.pop(), 1);
  EXPECT_EQ(queue.pop(), 2);
  EXPECT_TRUE(queue.empty());
}

TEST(ThreadSafeQueue, PopWhenFullDropsOldestValue)
{
  rm_tools::ThreadSafeQueue<int, true> queue(2);

  queue.push(1);
  queue.push(2);
  queue.push(3);

  EXPECT_EQ(queue.pop(), 2);
  EXPECT_EQ(queue.pop(), 3);
  EXPECT_TRUE(queue.empty());
}

TEST(ThreadSafeQueue, TryPopIfMoreThanRequiresEnoughBufferedItems)
{
  rm_tools::ThreadSafeQueue<int> queue(3);

  queue.push(1);
  queue.push(2);

  int value = 0;
  EXPECT_FALSE(queue.try_pop_if_more_than(2, value));
  EXPECT_TRUE(queue.try_pop_if_more_than(1, value));
  EXPECT_EQ(value, 1);
  EXPECT_EQ(queue.pop(), 2);
}
