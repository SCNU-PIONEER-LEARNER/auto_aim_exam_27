// Copyright (c) 2026 tomoon
// Licensed under the Apache-2.0 License.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "virtual_low_board/frame_codec.hpp"
#include "virtual_low_board/pseudo_random.hpp"

namespace virtual_low_board
{
/**
  * @brief Virtual lower board: publishes pseudo-random uplink frames, sends only.
  * @details Each timer tick rebuilds frame `index` from the shared profile, packs it
  *          into the 20 byte ReceivePacket layout with CRC16 and publishes the raw
  *          bytes. A consumer can feed them into the serial receive path unchanged.
  */
class VirtualLowBoardNode : public rclcpp::Node
{
public:
  explicit VirtualLowBoardNode(const rclcpp::NodeOptions & options)
  : Node("virtual_low_board", options)
  {
    profile_.seed = static_cast<uint32_t>(declare_parameter("seed", 90));
    profile_.mode = parseDataMode(declare_parameter("data_mode", std::string("smooth")));
    const double rate_hz = declare_parameter("rate_hz", 100.0);
    if (rate_hz <= 0.0) {
      throw std::invalid_argument{"rate_hz must be positive"};
    }

    const auto topic = declare_parameter("packet_topic", std::string("/receive_packet"));
    packet_pub_ =
      create_publisher<std_msgs::msg::UInt8MultiArray>(topic, rclcpp::QoS(10).reliable());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, std::bind(&VirtualLowBoardNode::publishFrame, this));

    RCLCPP_INFO(
      get_logger(), "virtual lower board: '%s' at %.1f Hz, mode=%s, seed=%u", topic.c_str(),
      rate_hz, dataModeName(profile_.mode), profile_.seed);
  }

private:
  /// Build one frame, append CRC16 and publish the raw bytes.
  void publishFrame()
  {
    const auto packet = makeFrame(profile_, frame_index_++);

    std_msgs::msg::UInt8MultiArray message;
    message.data = encodeFrame(packet);
    packet_pub_->publish(message);
  }

  FrameProfile profile_;
  uint64_t frame_index_ = 0;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr packet_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace virtual_low_board

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<virtual_low_board::VirtualLowBoardNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
