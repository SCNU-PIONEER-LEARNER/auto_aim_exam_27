// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.

// std
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_serial_driver
{
class VirtualSerialNode : public rclcpp::Node
{
public:
  explicit VirtualSerialNode(const rclcpp::NodeOptions & options) : Node("serial_driver", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting VirtualSerialNode!");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "gimbal_link";
    task_pub_ = this->create_publisher<std_msgs::msg::String>("task_mode", 10);
    task = this->declare_parameter("task_mode", "aim");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      task = this->get_parameter("task_mode").as_string();
      std_msgs::msg::String task_msg;
      task_msg.data = task;
      task_pub_->publish(task_msg);
      tf2::Quaternion q;
      q.setRPY(roll * M_PI / 180.0, -pitch * M_PI / 180.0, yaw * M_PI / 180.0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom";
      transform_stamped_.child_frame_id = "gimbal_link";
      transform_stamped_.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform_stamped_);
    });
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
  std::string task;
  geometry_msgs::msg::TransformStamped transform_stamped_;
};
}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::VirtualSerialNode)
