#include "rm_tools/heartbeat.hpp"

#include <stdexcept>
#include <string>

namespace rm_tools
{

HeartBeatPublisher::SharedPtr HeartBeatPublisher::create(rclcpp::Node * node)
{
  return create(node, std::chrono::seconds(1));
}

HeartBeatPublisher::SharedPtr HeartBeatPublisher::create(
  rclcpp::Node * node, std::chrono::milliseconds period)
{
  return SharedPtr(new HeartBeatPublisher(node, period));
}

HeartBeatPublisher::HeartBeatPublisher(rclcpp::Node * node, std::chrono::milliseconds period)
{
  if (node == nullptr) {
    throw std::invalid_argument("HeartBeatPublisher requires a valid node");
  }
  if (period <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("HeartBeatPublisher period must be positive");
  }

  message_.data = 0;
  const std::string topic_name = std::string(node->get_name()) + "/heartbeat";
  publisher_ = node->create_publisher<std_msgs::msg::Int64>(topic_name, 1);
  timer_ = node->create_wall_timer(period, [this]() {
    message_.data++;
    publisher_->publish(message_);
  });
}

}  // namespace rm_tools
