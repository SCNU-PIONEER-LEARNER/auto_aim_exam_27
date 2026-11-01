// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{

// todo : 后续并入compile_settings，或者保留在此处
#ifndef IS_SAME_AXIS__
#define IS_SAME_AXIS__ true
#else
  #error "IS_SAME_AXIS__宏被占用"
#endif

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  //for debug
  is_open_fire_control = this->declare_parameter("is_open_fire_control", true);
  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // Create Publisher
  task_pub_ = this->create_publisher<std_msgs::msg::String>("task_mode", 10);
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("latency", 10);
  bullet_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("bullet_speed", 10);
  record_controller_pub_ = this->create_publisher<std_msgs::msg::String>("record_controller", 10);
  // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  aim_time_info_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::TimeInfo>("time_info/aim", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  //Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("tracker/reset");

  //Target change service cilent
  change_target_client_ = this->create_client<std_srvs::srv::Trigger>("tracker/change");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // Create Subscription
  aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCmd>(
    "armor_planner/cmd_gimbal", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendArmorData, this, std::placeholders::_1));

  if(IS_SAME_AXIS__) 
    RCLCPP_WARN(this->get_logger(),"使用共轴TF");
  else 
    RCLCPP_WARN(this->get_logger(),"使用不共轴TF");

}
RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);
      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {
            resetTracker();
          }

          if (packet.change_target) {
            changeTarget();
          }
          std_msgs::msg::String task;
          if (packet.task_mode == 0)
            task.data = "aim";
          else if (packet.task_mode == 1)
            task.data = "small_buff";
          else if (packet.task_mode == 2)
            task.data = "large_buff";
          //保底是自瞄
          else
            task.data = "aim";
          task_pub_->publish(task);

          std_msgs::msg::Float32 bullet_speed;
          bullet_speed.data = packet.bullet_speed;
          bullet_speed_pub_->publish(bullet_speed);
          std_msgs::msg::String record_controller;
          record_controller.data = packet.is_play ? "start" : "stop";
          record_controller_pub_->publish(record_controller);

          #if IS_SAME_AXIS__
          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          
          tf2::Quaternion q;
          q.setRPY(packet.roll, packet.pitch, packet.yaw);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);
          #else
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();

          geometry_msgs::msg::TransformStamped odom2gimbal_yaw;
          odom2gimbal_yaw.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
          odom2gimbal_yaw.header.frame_id = "odom";
          odom2gimbal_yaw.child_frame_id = "gimbal_yaw";
         
          tf2::Quaternion q_odom2gimbal_yaw;
          q_odom2gimbal_yaw.setRPY(0.0, 0.0, packet.yaw);
          odom2gimbal_yaw.transform.rotation = tf2::toMsg(q_odom2gimbal_yaw);
          tf_broadcaster_->sendTransform(odom2gimbal_yaw);
          
          geometry_msgs::msg::TransformStamped gimbal_pitch2link;
          gimbal_pitch2link.header.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
          gimbal_pitch2link.header.frame_id = "gimbal_yaw";
          gimbal_pitch2link.child_frame_id = "gimbal_link";
          
          tf2::Quaternion q_gimbal_pitch2link;
          q_gimbal_pitch2link.setRPY(packet.roll, packet.pitch, 0.0);
          gimbal_pitch2link.transform.rotation = tf2::toMsg(q_gimbal_pitch2link);
          tf_broadcaster_->sendTransform(gimbal_pitch2link);
          #endif
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendArmorData(const auto_aim_interfaces::msg::GimbalCmd::ConstSharedPtr msg)
{
  try {
    SendPacket packet;
    packet.control = msg->control;
    if (!is_open_fire_control)
      packet.fire = 1;
    else
      packet.fire = msg->fire;
    packet.yaw = msg->yaw;
    packet.yaw_vel = msg->yaw_vel;
    packet.yaw_acc = msg->yaw_acc;
    packet.pitch = msg->target_pitch;
    packet.pitch_vel = 0;
    packet.pitch_acc = 0;
    // 20240329 ZY: Eliminate communication latency
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);
    if (msg->control) {
      std_msgs::msg::Float64 latency;
      latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
      RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
      latency_pub_->publish(latency);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or "
        "hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  // 参数表不会存在这个参数。使用calibrate.py启动是为true，其它情况为默认值false
  try {
    this->if_calib = this->declare_parameter<bool>("if_calib", false);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(this->get_logger(), "The varience \"if_calib\" was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  // if (!detector_param_client_->service_is_ready()) {
  //   if (this->if_calib) return;  // 如果正在标定，跳过报错。不需要改参数，calibrate.py自动处理
  //   RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
  //   return;
  // }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void RMSerialDriver::changeTarget()
{
  if (!change_target_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping target change");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  change_target_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Change target!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
