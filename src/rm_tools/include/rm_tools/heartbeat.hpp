#ifndef RM_TOOLS_HEARTBEAT_HPP_
#define RM_TOOLS_HEARTBEAT_HPP_

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

namespace rm_tools
{

class HeartBeatPublisher
{
public:
  using SharedPtr = std::shared_ptr<HeartBeatPublisher>;

  static SharedPtr create(rclcpp::Node * node);
  static SharedPtr create(rclcpp::Node * node, std::chrono::milliseconds period);

private:
  HeartBeatPublisher(rclcpp::Node * node, std::chrono::milliseconds period);

  std_msgs::msg::Int64 message_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_tools

#endif  // RM_TOOLS_HEARTBEAT_HPP_
