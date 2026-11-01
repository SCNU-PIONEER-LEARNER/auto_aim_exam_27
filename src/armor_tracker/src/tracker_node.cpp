// Copyright 2022 Chen Jun
#include "armor_tracker/tracker_node.hpp"

// STD
#include <cmath>
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  double max_match_angle_diff = this->declare_parameter("tracker.max_match_angle_diff", 10.0);
  tracker_ = std::make_unique<Tracker>(max_match_angle_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  tracker_->init_in_priority =
    this->declare_parameter("init_mode", 0);  //0:center mode 1:prior mode
  tracker_->s2qxyz_max_ = declare_parameter("ekf.sigma2_q_xyz_max", 0.1);
  tracker_->s2qxyz_min_ = declare_parameter("ekf.sigma2_q_xyz_min", 0.05);
  tracker_->s2qyaw_max_ = declare_parameter("ekf.sigma2_q_yaw_max", 10.0);
  tracker_->s2qyaw_min_ = declare_parameter("ekf.sigma2_q_yaw_min", 5.0);
  tracker_->r_py = declare_parameter("ekf.r_py", 4e-3);
  tracker_->r_d = declare_parameter("ekf.r_d", 0.05);
  tracker_->r_yaw = declare_parameter("ekf.r_yaw", 0.02);

  tracker_->yaw_threshold = declare_parameter("yaw_threshold", 60);
  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "tracker/reset", [this](
                       const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });

  // Change target service
  change_target_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "tracker/change", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::CHANGE_TARGET;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Target change!");
      return;
    });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "tracker/target", rclcpp::SensorDataQoS());
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance_;
      }),
    armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->dt = dt_;
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    tracker_->update(armors_msg);

    // Publish Info
    auto x = tracker_->ekf.x[0];
    auto y = tracker_->ekf.x[2];
    auto pd = std::hypot(tracker_->measurement(0) - x, tracker_->measurement(1) - y);   // todo 可以写点没用的优化
    // auto pd = pow(pow(tracker_->measurement(0) - x, 2) + pow(tracker_->measurement(1) - y, 2), 0.5);   
    auto yaw = tracker_->ekf.x[6];
    info_msg.position_diff = pd;
    info_msg.yaw_diff = abs(yaw - tracker_->measurement(3));
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->target_state;  // todo:我的刀盾
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius = state(8);
      target_msg.d_radius = state(9);
      target_msg.dz = state(10);
      target_msg.visiable_num = tracker_->visiable_armor_num;
    } else if (tracker_->tracker_state == Tracker::CHANGE_TARGET) {
      target_msg.tracking = false;
    }
  }

  last_time_ = time;

  target_pub_->publish(target_msg);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
