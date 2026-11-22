// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/cv_detector.hpp"
#include "armor_detector/detector_node.hpp"
#include "rm_tools/debug_gate.hpp"
#include "rm_tools/image_projection.hpp"
#include "rm_tools/math.hpp"
#include "armor_detector/cv_detector.hpp" 
#include "armor_detector/number_classifier.hpp"
#include "rm_tools/math.hpp"


namespace rm_auto_aim
{
std::vector<std::unique_ptr<YOLO>> create_yolov5s(
  const std::string & model_path, int number, double conf_threshold, const std::string & device)
{
  const int throughput_mode = 1;
  std::vector<std::unique_ptr<YOLO>> yolov5s;
  for (int i = 0; i < number; i++) {
    auto yolo_instance =
      std::make_unique<YOLO>(model_path, conf_threshold, device, throughput_mode);

    RCLCPP_INFO(
      rclcpp::get_logger("yolo_factory"), "Created YOLO instance %d with address: %p", i,
      yolo_instance.get());

    yolov5s.push_back(std::move(yolo_instance));
  }
  return yolov5s;
}

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  frame_id = 0;
  frame_id_ = "camera_optical_frame";
  // detect_mode = this->declare_parameter("detect_mode", 0);
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      armor_pose_estimator_ = std::make_unique<ArmorPoseEstimator>(cam_info_);
      armor_pose_estimator_->is_optimize_yaw = this->declare_parameter("is_optimize_yaw", true);
      armor_pose_estimator_->is_use_plus_pnp = this->declare_parameter("is_use_plus_pnp", true);
      armor_pose_estimator_->is_use_ternary = this->declare_parameter("is_use_ternary", true);
      armor_pose_estimator_->is_show_cost = this->declare_parameter("is_show_cost", false);
      cam_info_sub_.reset();
    });
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  this->detect_mode_ = declare_parameter("detect_mode", 0);

  if ((detect_mode_ != 2) && (detect_mode_ != 3))
  initDetector();

  if (detect_mode_ == 1) {
    detect_thread_running_ = true;
    detect_thread_ = std::thread(&ArmorDetectorNode::async_yolo_loop, this);
  }
  if (detect_mode_ == 2) {
    init_pool();
    detect_thread_running_ = true;
    process_thread_ = std::thread(&ArmorDetectorNode::yolo_pool_loop, this);
  }
  if (detect_mode_ == 3) {
    detect_thread_running_ = false;
    cv_Detector_ = make_cv_Detector();
  }

  // Armors Publisher
  armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
    "detector/armors", rclcpp::SensorDataQoS());

  // Transform initialize
  odom_frame_ = this->declare_parameter("target_frame", "odom");
  imu_to_camera_ = Eigen::Matrix3d::Identity();

  // Task subscriber
  is_aim_task_ = true;
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "task_mode", 10, std::bind(&ArmorDetectorNode::taskCallback, this, std::placeholders::_1));
  is_play_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "is_play", 10, std::bind(&ArmorDetectorNode::isPlayCallback, this, std::placeholders::_1));
  // Visualization Marker Publisher
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detector/marker", 10);
  tracker_overlay_sub_ = this->create_subscription<rm_interfaces::msg::ArmorOverlay>(
    "tracker/overlay", 10,
    std::bind(&ArmorDetectorNode::trackerOverlayCallback, this, std::placeholders::_1));

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  if (debugEnabled()) {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
        debug_ = p.as_bool();
        debugEnabled() ? createDebugPublishers() : destroyDebugPublishers();
    });
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  heartbeat_ = rm_tools::HeartBeatPublisher::create(this);
}

ArmorDetectorNode::~ArmorDetectorNode()
{
  if (detect_mode_ == 1) {
    detect_thread_running_ = false;
    if (detect_thread_.joinable()) {
      detect_thread_.join();
    }
  }
  if (detect_mode_ == 2) {
    detect_thread_running_ = false;
    frame_queue.shutdown();
    if (process_thread_.joinable()) {
      process_thread_.join();
    }
    thread_pool_.reset();
  }
}

void ArmorDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  std::string task_mode = task_msg->data;
  const bool was_aim_task = is_aim_task_.load();
  const bool is_aim_task = task_mode == "aim" || task_mode == "auto";
  is_aim_task_ = is_aim_task;
  if (was_aim_task && !is_aim_task && detect_mode_ == 1 && yolo_ != nullptr) {
    yolo_->queue_.clear();
  }
}

void ArmorDetectorNode::isPlayCallback(const std_msgs::msg::Bool::SharedPtr is_play_msg)
{
  is_play_ = is_play_msg->data;
}

bool ArmorDetectorNode::debugEnabled() const
{
  return rm_tools::debug_enabled(debug_, is_play_.load());
}

void ArmorDetectorNode::trackerOverlayCallback(
  const rm_interfaces::msg::ArmorOverlay::SharedPtr overlay_msg)
{
  std::lock_guard<std::mutex> lock(latest_tracker_overlay_mutex_);
  latest_tracker_overlay_ = *overlay_msg;
  has_tracker_overlay_ = true;
}

void ArmorDetectorNode::publishResultImage(
  const std_msgs::msg::Header & header, cv::Mat & result_img)
{
  overlayTrackerData(result_img, header);
  result_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", result_img).toImageMsg());
}

void ArmorDetectorNode::overlayTrackerData(
  cv::Mat & image, const std_msgs::msg::Header & image_header)
{
  if (
    image.empty() || cam_info_ == nullptr || tf2_buffer_ == nullptr ||
    image_header.frame_id.empty()) {
    return;
  }

  rm_interfaces::msg::ArmorOverlay overlay_msg;
  {
    std::lock_guard<std::mutex> lock(latest_tracker_overlay_mutex_);
    if (
      !has_tracker_overlay_ || !latest_tracker_overlay_.tracking ||
      latest_tracker_overlay_.header.frame_id.empty()) {
      return;
    }
    overlay_msg = latest_tracker_overlay_;
  }

  auto camera_model = rm_tools::makeCameraModel(*cam_info_);
  camera_model.image_size = image.size();

  geometry_msgs::msg::TransformStamped overlay_to_image;
  try {
    overlay_to_image = tf2_buffer_->lookupTransform(
      image_header.frame_id, overlay_msg.header.frame_id, image_header.stamp,
      rclcpp::Duration::from_seconds(0.001));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Tracker overlay TF error: %s", ex.what());
    return;
  }

  const auto armor_count = std::min(
    overlay_msg.armor_poses.size(),
    std::min(overlay_msg.armor_widths.size(), overlay_msg.armor_heights.size()));
  for (std::size_t i = 0; i < armor_count; ++i) {
    geometry_msgs::msg::PoseStamped source_pose;
    source_pose.header = overlay_msg.header;
    source_pose.header.stamp = image_header.stamp;
    source_pose.pose = overlay_msg.armor_poses[i];

    geometry_msgs::msg::PoseStamped camera_pose;
    tf2::doTransform(source_pose, camera_pose, overlay_to_image);
    const auto corners = rm_tools::buildRectangleFaceCorners(
      camera_pose.pose, overlay_msg.armor_widths[i], overlay_msg.armor_heights[i]);
    rm_tools::drawProjectedPolygon(image, corners, camera_model, cv::Scalar(255, 0, 255), 2);
  }

}

void ArmorDetectorNode::async_yolo_loop()
{
  while (detect_thread_running_ && rclcpp::ok() && yolo_ != nullptr) {
    if (!is_aim_task_) {
      yolo_->queue_.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto result = yolo_->try_debug_pop();
    if (!result.has_value()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    auto [debug_img, armors, time] = std::move(*result);
    if (!is_aim_task_) {
      continue;
    }
    if (use_traditional) {
      for (auto & armor : armors) {
        detector_->traditional_correct(armor, debug_img);
      }
    }
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(time);
    header.frame_id = frame_id_;
    auto t = (this->now()).seconds() * 1000.0;
    auto time_ms = time / 1e6;
    auto latency = t - time_ms;

    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "async detect latency: %f", latency);

    if (detector_ == nullptr) continue;
    const bool publish_debug = debugEnabled();
    if (publish_debug) {
      detector_->result_img = detector_->draw_result(armors, debug_img, latency);
      publishResultImage(header, detector_->result_img);
      auto all_binary_img = detector_->get_all_binary_img(armors);
      binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_binary_img).toImageMsg());
      cost_img_pub_.publish(
        cv_bridge::CvImage(header, "rgb8", armor_pose_estimator_->img_cost).toImageMsg());
    }

    if (armor_pose_estimator_ != nullptr) {
      armors_msg_.header = header;

      // 过滤颜色
      armors.erase(
        std::remove_if(
          armors.begin(), armors.end(),
          [this](const Armor & armor) { return static_cast<int>(armor.color) != detect_color; }),
        armors.end());
      try {
        rclcpp::Time target_time = header.stamp;
        auto camera2odom = tf2_buffer_->lookupTransform(
          odom_frame_, header.frame_id, target_time, rclcpp::Duration::from_seconds(0.01));

        auto msg_q = camera2odom.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3 tf2_matrix(tf_q);
        imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
          tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
          tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        continue;
      }
      armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);

      if (publish_debug) {
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;
        armor_marker_.header = text_marker_.header = armors_msg_.header;
        for (const auto & armor : armors_msg_.armors) {
          armor_marker_.scale.y = armor.type == std::string("small") ? 0.135 : 0.23;
          armor_marker_.pose = armor.pose;
          armor_marker_.id++;
          text_marker_.pose.position = armor.pose.position;
          text_marker_.id++;
          text_marker_.pose.position.y -= 0.1;
          text_marker_.text = armor.number;
          marker_array_.markers.emplace_back(armor_marker_);
          marker_array_.markers.emplace_back(text_marker_);
        }
        publishMarkers();
      }

      armors_pub_->publish(armors_msg_);
    }
  }
}

void ArmorDetectorNode::yolo_pool_loop()
{
  while (detect_thread_running_ && rclcpp::ok()) {
    Frame process_frame;
    if (!frame_queue.dequeue_timeout(process_frame, std::chrono::milliseconds(20))) {
      continue;
    }
    if (!process_frame.isValid() || !is_aim_task_) {
      continue;
    }
    {
      auto img = process_frame.img;
      auto armors = process_frame.armors;
      auto t = process_frame.t;
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time(t);
      header.frame_id = frame_id_;
      if (use_traditional) {
        for (auto & armor : armors) detector_->traditional_correct(armor, img);
      }
      auto timestamp = rclcpp::Time(t);
      auto end = this->get_clock()->now();
      double latency = (end.seconds() - timestamp.seconds()) * 1000.0;

      if (debugEnabled()) {
        detector_->result_img = detector_->draw_result(armors, img, latency);
        publishResultImage(header, detector_->result_img);
        auto all_binary_img = detector_->get_all_binary_img(armors);
        binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_binary_img).toImageMsg());
        cost_img_pub_.publish(
          cv_bridge::CvImage(header, "rgb8", armor_pose_estimator_->img_cost).toImageMsg());
      }

      if (armor_pose_estimator_ != nullptr) {
        armors_msg_.header = header;
        armors.erase(
          std::remove_if(
            armors.begin(), armors.end(),
            [this](const Armor & armor) { return static_cast<int>(armor.color) != detect_color; }),
          armors.end());

        try {
          rclcpp::Time target_time = header.stamp;
          auto camera2odom = tf2_buffer_->lookupTransform(
            odom_frame_, header.frame_id, target_time, rclcpp::Duration::from_seconds(0.01));

          auto msg_q = camera2odom.transform.rotation;
          tf2::Quaternion tf_q;
          tf2::fromMsg(msg_q, tf_q);
          tf2::Matrix3x3 tf2_matrix(tf_q);
          imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1],
            tf2_matrix.getRow(0)[2], tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1],
            tf2_matrix.getRow(1)[2], tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1],
            tf2_matrix.getRow(2)[2];
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
          continue;
        }
        auto x1 = std::chrono::high_resolution_clock::now();
        armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
        auto x2 = std::chrono::high_resolution_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(x2 - x1);
        //std::cout<<"armor_pose_estimator_latency:"<<total_time.count()/1000.0<<std::endl;

        if (debugEnabled()) {
          marker_array_.markers.clear();
          armor_marker_.id = 0;
          text_marker_.id = 0;
          armor_marker_.header = text_marker_.header = armors_msg_.header;

          for (const auto & armor : armors_msg_.armors) {
            armor_marker_.scale.y = armor.type == std::string("small") ? 0.135 : 0.23;
            armor_marker_.pose = armor.pose;
            armor_marker_.id++;
            text_marker_.pose.position = armor.pose.position;
            text_marker_.id++;
            text_marker_.pose.position.y -= 0.1;
            text_marker_.text = armor.number;
            marker_array_.markers.emplace_back(armor_marker_);
            marker_array_.markers.emplace_back(text_marker_);
          }
          publishMarkers();
        }

        armors_pub_->publish(armors_msg_);
      }
    }
  }
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  if (is_aim_task_) {
    detect_color = this->get_parameter("detect_color").as_int();
    // Convert ROS img to cv::Mat
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    auto timestamp = rclcpp::Time(img_msg->header.stamp);
    int64_t t = timestamp.nanoseconds();
    auto end = this->get_clock()->now();
    double latency = (end.seconds() - timestamp.seconds()) * 1000.0;
    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "imagecallback latency: %f", latency);

    static int detect_count = 0;
    detect_count++;
    if (detect_count > 10000) {
      this->detect_color = this->get_parameter("detect_color").as_int();
      detect_count = 0;
    }

    if (detect_mode_ == 0) {
      single_yolo_process(img, t);
    }
    if (detect_mode_ == 1 && yolo_ != nullptr) {
      yolo_->push(img, timestamp.nanoseconds());
    }
    if (detect_mode_ == 2) {
      yolo_pool_process(img, t);
    }
    if (detect_mode_ == 3) {
      cv_process(img, t);
    }
  }
}

void ArmorDetectorNode::single_yolo_process(const cv::Mat & img, int64_t t)
{
  std::vector<Armor> armors;
  armors = yolo_->detect(img);
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(t);
  header.frame_id = frame_id_;
  if (use_traditional) {
    for (auto & armor : armors) detector_->traditional_correct(armor, img);
  }
  auto timestamp = rclcpp::Time(t);
  auto end = this->get_clock()->now();
  double latency = (end.seconds() - timestamp.seconds()) * 1000.0;
  RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "single yolo detect latency: %f", latency);
  if (debugEnabled()) {
    detector_->result_img = detector_->draw_result(armors, img, latency);
    publishResultImage(header, detector_->result_img);
    auto all_binary_img = detector_->get_all_binary_img(armors);
    binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_binary_img).toImageMsg());
    cost_img_pub_.publish(
      cv_bridge::CvImage(header, "rgb8", armor_pose_estimator_->img_cost).toImageMsg());
  }

  if (armor_pose_estimator_ != nullptr) {
    armors_msg_.header = header;
    armors.erase(
      std::remove_if(
        armors.begin(), armors.end(),
        [this](const Armor & armor) { return static_cast<int>(armor.color) != detect_color; }),
      armors.end());

    try {
      rclcpp::Time target_time = header.stamp;
      auto camera2odom = tf2_buffer_->lookupTransform(
        odom_frame_, header.frame_id, target_time, rclcpp::Duration::from_seconds(0.01));

      auto msg_q = camera2odom.transform.rotation;
      tf2::Quaternion tf_q;
      tf2::fromMsg(msg_q, tf_q);
      tf2::Matrix3x3 tf2_matrix(tf_q);
      imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
        tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
        tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    }
    armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);

    if (debugEnabled()) {
      marker_array_.markers.clear();
      armor_marker_.id = 0;
      text_marker_.id = 0;
      armor_marker_.header = text_marker_.header = armors_msg_.header;

      for (const auto & armor : armors_msg_.armors) {
        armor_marker_.scale.y = armor.type == "small" ? 0.135 : 0.23;
        armor_marker_.pose = armor.pose;
        armor_marker_.id++;
        text_marker_.pose.position = armor.pose.position;
        text_marker_.id++;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.number;
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      }
      publishMarkers();
    }

    armors_pub_->publish(armors_msg_);
  }
}

void ArmorDetectorNode::yolo_pool_process(const cv::Mat & img, int64_t t)
{
  cv::Mat img_clone = img.clone();
  if (img_clone.empty()) {
    std::cout << "the img is empty" << std::endl;
    return;
  }
  if (!thread_pool_->try_enqueue([this, img_clone = std::move(img_clone), t] {
    auto current_frame_id = frame_id.fetch_add(1, std::memory_order_relaxed);
    int yolo_id = -1;
    {
      std::lock_guard<std::mutex> lock(yolo_mutex_);
      for (int i = 0; i < num_yolo_thread; i++) {
        if (!yolo_used[i]) {
          yolo_used[i] = true;
          yolo_id = i;
          break;
        }
      }
    }
    if (yolo_id != -1) {
      Frame frame{current_frame_id, img_clone, t};
      frame.armors = yolos[yolo_id]->detect(frame.img);
      frame_queue.enqueue(frame);
      {
        std::lock_guard<std::mutex> lock(yolo_mutex_);
        yolo_used[yolo_id] = false;
      }
    } else
      RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "YOLO Pool is full!");
  })) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "YOLO task queue is full, dropping frame");
  }
}

void ArmorDetectorNode::cv_process(const cv::Mat & img, int64_t t)
{
  cv_Detector_->binary_thres = get_parameter("binary_thres").as_int();
  // cv_Detector_->detect_color = get_parameter("detect_color").as_int();
  cv_Detector_->detect_color = this->detect_color;
  cv_Detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  auto armors = cv_Detector_->detect(img);

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(t);
  header.frame_id = frame_id_;

  auto end = this->get_clock()->now();
  auto latency = (end.seconds() - rclcpp::Time(t).seconds()) * 1000.0;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  if (debugEnabled()) {
    cv::Mat result_img = cv_Detector_->draw_result(armors, img, latency);  // 变成栈变量自动管理生命周期
    publishResultImage(header, result_img);
    cv::Mat all_binary_img = cv_Detector_->get_all_binary_img(armors);
    binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_binary_img).toImageMsg());
    // debug_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", all_num_img).toImageMsg());  // * 可以用来发任何需要debug的东西

    cv_bin_img_pub_.publish(cv_bridge::CvImage(header, "mono8", cv_Detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
      cv_Detector_->debug_lights.data.begin(), cv_Detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      cv_Detector_->debug_armors.data.begin(), cv_Detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    debug_lights_pub_->publish(cv_Detector_->debug_lights);
    debug_armors_pub_->publish(cv_Detector_->debug_armors);
  }

  if (armor_pose_estimator_ != nullptr) {
    armors_msg_.header = header;
    armors.erase(
      std::remove_if(
        armors.begin(), armors.end(),
        [this](const Armor & armor) { return static_cast<int>(armor.color) != detect_color; }),
      armors.end());

    try {
      rclcpp::Time target_time = header.stamp;
      auto camera2odom = tf2_buffer_->lookupTransform(
        odom_frame_, header.frame_id, target_time, rclcpp::Duration::from_seconds(0.01));

      auto msg_q = camera2odom.transform.rotation;
      tf2::Quaternion tf_q;
      tf2::fromMsg(msg_q, tf_q);
      tf2::Matrix3x3 tf2_matrix(tf_q);
      imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
        tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
        tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    }
    auto x1 = std::chrono::high_resolution_clock::now();
    armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
    auto x2 = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(x2 - x1);
    //std::cout<<"armor_pose_estimator_latency:"<<total_time.count()/1000.0<<std::endl;

    if (debugEnabled()) {
      marker_array_.markers.clear();
      armor_marker_.id = 0;
      text_marker_.id = 0;
      armor_marker_.header = text_marker_.header = armors_msg_.header;

      for (const auto & armor : armors_msg_.armors) {
        armor_marker_.scale.y = armor.type == std::string("small") ? 0.135 : 0.23;
        armor_marker_.pose = armor.pose;
        armor_marker_.id++;
        text_marker_.pose.position = armor.pose.position;
        text_marker_.id++;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.number;
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      }
      publishMarkers();
    }

    armors_pub_->publish(armors_msg_);
  }
}

void ArmorDetectorNode::initDetector()
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("pinyvision_models");
  std::string model_name = this->declare_parameter("model_name", "yolov5.xml");
  auto model_path = pkg_path + "/models/armor/" + model_name;
  std::string device = declare_parameter("device", "CPU");
  auto conf_threshold = this->declare_parameter("confidence_threshole", 0.5);
  yolo_ = nullptr;
  if (detect_mode_ == 0) {
    yolo_ = std::make_unique<YOLOV5>(model_path, conf_threshold, device, LATENCY_MODE);
  }
  if (detect_mode_ == 1) {
    yolo_ = std::make_unique<YOLOV5>(model_path, conf_threshold, device, THROUGHPUT_MODE);
  }
  detector_ = std::make_unique<Detector>();
  detect_color = declare_parameter("detect_color", 0);  //0:red 1:blue
  detector_->binary_thres = this->declare_parameter("binary_threshold", 70);
  use_traditional = this->declare_parameter("use_traditional", true);
  detector_->max_angle_error_ = this->declare_parameter("max_angle_error", 50);
  detector_->min_lightbar_ratio_ = this->declare_parameter("min_lightbar_ratio", 1.5);
  detector_->max_lightbar_ratio_ = this->declare_parameter("max_lightbar_ratio", 20);
  detector_->min_lightbar_length_ = this->declare_parameter("min_lightbar_length", 20);
  detector_->tolerate = this->declare_parameter("tolerate", 0.1);
  detector_->min_l2l_ratio_ = this->declare_parameter("min_l2l_ratio", 0.7);
  detector_->use_pca = this->declare_parameter("use_pca", false);

  RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_path.c_str());
}

void ArmorDetectorNode::init_pool()
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("pinyvision_models");
  std::string model_name = this->declare_parameter("model_name", "yolov5.xml");
  auto model_path = pkg_path + "/models/armor/" + model_name;
  std::string device = this->declare_parameter("device", "CPU");
  num_yolo_thread = this->declare_parameter("num_yolo_thread", 8);
  int max_yolo_queue_size = this->declare_parameter("max_yolo_queue_size", num_yolo_thread);
  auto conf_threshold = this->declare_parameter("confidence_threshole", 0.7);
  yolos = create_yolov5s(model_path, num_yolo_thread, conf_threshold, device);
  yolo_used.resize(num_yolo_thread, false);
  thread_pool_ = std::make_unique<ThreadPool>(
    num_yolo_thread, static_cast<size_t>(std::max(1, max_yolo_queue_size)));
  detector_ = std::make_unique<Detector>();
  detect_color = declare_parameter("detect_color", 0);  //0:red 1:blue
  detector_->binary_thres = this->declare_parameter("binary_threshold", 70);
  detector_->max_angle_error_ = this->declare_parameter("max_angle_error", 50);
  detector_->min_lightbar_ratio_ = this->declare_parameter("min_lightbar_ratio", 1.5);
  detector_->max_lightbar_ratio_ = this->declare_parameter("max_lightbar_ratio", 20);
  detector_->min_lightbar_length_ = this->declare_parameter("min_lightbar_length", 20);
  detector_->tolerate = this->declare_parameter("tolerate", 0.1);
  detector_->min_l2l_ratio_ = this->declare_parameter("min_l2l_ratio", 0.7);
  detector_->use_pca = this->declare_parameter("use_pca", false);
  RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_path.c_str());
}

std::unique_ptr<cv_Detector> ArmorDetectorNode::make_cv_Detector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", static_cast<int>(Color::red), param_desc);
  this->detect_color = static_cast<int>(detect_color);

  cv_Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 35.0),
    .min_fill_ratio = declare_parameter("light.min_fill_ratio", 0.8),
    .min_contour_size = static_cast<size_t>(declare_parameter("light.min_contour_size", 10)),
};

  cv_Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<cv_Detector>(binary_thres, detect_color, l_params, a_params);

  // std::string model_name = this->declare_parameter("model_name", "yolov5.xml"); // 栈变量自动销毁
  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("pinyvision_models");
  auto model_path = pkg_path + "/models/armor/mlp.onnx";
  auto label_path = pkg_path + "/models/armor/label.txt";
  RCLCPP_INFO(this->get_logger(),(std::string("model_path = ") + model_path).c_str());
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

void ArmorDetectorNode::createDebugPublishers()
{
  this->declare_parameter("armor_detector.result_img.jpeg_quality", 10);
  this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
  binary_img_pub_ = image_transport::create_publisher(this, "detector/binary_img");
  result_img_pub_ = image_transport::create_publisher(this, "detector/result_img");
  cost_img_pub_ = image_transport::create_publisher(this, "detector/yaw_cost");

  if (detect_mode_ == 3) {
    cv_bin_img_pub_ = image_transport::create_publisher(this, "detector/bin_img");
    debug_lights_pub_ = this->create_publisher<rm_interfaces::msg::DebugLights>(
      "/detector/debug_lights", rclcpp::SensorDataQoS());
      debug_armors_pub_ = this->create_publisher<rm_interfaces::msg::DebugArmors>(
        "/detector/debug_armors", rclcpp::SensorDataQoS());
        debug_img_pub_ = image_transport::create_publisher(this, "/detector/debug_imgs");
  }
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  binary_img_pub_.shutdown();
  result_img_pub_.shutdown();
  cost_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
