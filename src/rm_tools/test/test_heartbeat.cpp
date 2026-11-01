#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#include "rm_tools/heartbeat.hpp"

TEST(HeartBeatPublisherTest, PublishesIncreasingHeartbeatOnNodeTopic)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("heartbeat_test_node");
  std::vector<int64_t> received;

  auto subscription = node->create_subscription<std_msgs::msg::Int64>(
    "/heartbeat_test_node/heartbeat", 10,
    [&received](const std_msgs::msg::Int64::SharedPtr msg) {received.push_back(msg->data);});

  auto heartbeat =
    rm_tools::HeartBeatPublisher::create(node.get(), std::chrono::milliseconds(10));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (received.size() < 2 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  ASSERT_GE(received.size(), 2u);
  EXPECT_LT(received.front(), received.back());

  heartbeat.reset();
  subscription.reset();
  node.reset();
}
