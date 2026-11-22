// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

#include "armor_detector/armor_pose_estimator.hpp"
#include "armor_detector/detector.hpp"
#include "armor_detector/cv_detector.hpp"
// ROS
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armor_overlay.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_tools/heartbeat.hpp"
#include "rm_tools/thread_pool.hpp"
#include "yolo.hpp"
#include "yolov5.hpp"
// STD
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>


namespace rm_auto_aim
{

struct Frame
{
  int id;
  cv::Mat img;
  int64_t t;
  std::vector<Armor> armors;

  Frame() = default;

  Frame(int frame_id, cv::Mat image, int64_t timestamp)
  : id(frame_id), img(std::move(image)), t(timestamp), armors()
  {
  }

  Frame(Frame && other) noexcept = default;

  Frame(const Frame & other)
  : id(other.id), img(other.img.clone()), t(other.t), armors(other.armors)
  {
  }

  Frame & operator=(Frame && other) noexcept = default;

  Frame & operator=(const Frame & other)
  {
    if (this != &other) {
      id = other.id;
      img = other.img.clone();
      t = other.t;
      armors = other.armors;
    }
    return *this;
  }

  bool isValid() const { return !img.empty() && id >= 0; }

  std::string getImageInfo() const
  {
    if (img.empty()) {
      return "Empty image";
    }
    return "Size: " + std::to_string(img.cols) + "x" + std::to_string(img.rows) +
           ", Channels: " + std::to_string(img.channels());
  }
};

using OrderedQueue = rm_tools::OrderedQueue<Frame>;
using ThreadPool = rm_tools::ThreadPool;

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode();

  std::atomic<int> frame_id{0};

private:
  rm_tools::HeartBeatPublisher::SharedPtr heartbeat_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void initDetector();

  std::unique_ptr<cv_Detector> make_cv_Detector();

  // // std::vector<Armor> ArmorDetectorNode::detectArmors(const cv::Mat & img, int64_t t);

  void createDebugPublishers();
  void destroyDebugPublishers();
  void publishResultImage(const std_msgs::msg::Header & header, cv::Mat & result_img);
  void overlayTrackerData(cv::Mat & image, const std_msgs::msg::Header & image_header);

  void publishMarkers();
  int detect_color;
  bool use_traditional;
  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  std::atomic<bool> is_aim_task_{true};
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);
  // Armor Detector
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<YOLOV5> yolo_;
  std::unique_ptr<cv_Detector> cv_Detector_;
  image_transport::Publisher cv_bin_img_pub_;

  void single_yolo_process(const cv::Mat & img, int64_t timestampe);
  void async_yolo_process(const cv::Mat & img, int64_t timestampe);
  void yolo_pool_process(const cv::Mat & img, int64_t timestampe);
  void cv_process(const cv::Mat & img, int64_t t);

  //async yolo
  std::string frame_id_;  // camera_optical_frame
  void async_yolo_loop();
  std::thread detect_thread_;
  std::atomic<bool> detect_thread_running_{false};
  // //thread pool
  //bool use_thread_pool;
  int detect_mode_;
  OrderedQueue frame_queue;
  std::thread process_thread_;
  std::unique_ptr<ThreadPool> thread_pool_;
  std::mutex yolo_mutex_;
  std::vector<std::unique_ptr<YOLO>> yolos;
  int num_yolo_thread;
  std::vector<bool> yolo_used;
  void init_pool();
  void yolo_pool_loop();
  int count = 0;

  const int LATENCY_MODE = 0;
  const int THROUGHPUT_MODE = 1;
  // Detected armors publisher
  rm_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  void trackerOverlayCallback(
    const rm_interfaces::msg::ArmorOverlay::SharedPtr overlay_msg);
  rclcpp::Subscription<rm_interfaces::msg::ArmorOverlay>::SharedPtr tracker_overlay_sub_;
  rm_interfaces::msg::ArmorOverlay latest_tracker_overlay_;
  std::mutex latest_tracker_overlay_mutex_;
  bool has_tracker_overlay_ = false;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<ArmorPoseEstimator> armor_pose_estimator_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Debug information
  bool debug_;
  std::atomic<bool> is_play_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_play_sub_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  void isPlayCallback(const std_msgs::msg::Bool::SharedPtr is_play_msg);
  bool debugEnabled() const;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;
  image_transport::Publisher cost_img_pub_;
  rclcpp::Publisher<rm_interfaces::msg::DebugLights>::SharedPtr debug_lights_pub_;
  rclcpp::Publisher<rm_interfaces::msg::DebugArmors>::SharedPtr debug_armors_pub_;
  image_transport::Publisher debug_img_pub_;

  // ReceiveData subscripiton
  std::string odom_frame_;
  Eigen::Matrix3d imu_to_camera_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
