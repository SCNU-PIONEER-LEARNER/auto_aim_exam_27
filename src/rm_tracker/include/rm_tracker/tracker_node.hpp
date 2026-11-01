#pragma once

#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/tracker_info.hpp"
#include "rm_tracker/tracker/armor_tracker.hpp"

#include "rm_tools/heartbeat.hpp"

namespace rm_auto_aim::tracker
{

using tf2_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Armors>;

class TrackerNode : public rclcpp::Node {
public:
    explicit TrackerNode(const rclcpp::NodeOptions & _options);

    void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr & _armors_msg);

private:
    // params

    double max_armor_distance_;
    double max_armor_z_;

    // ptrs

    ArmorTracker::UniquePtr armor_tracker_;

    // tracker service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_target_srv_;

    // Subscriber with tf2 message_filter
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<rm_interfaces::msg::Armors> armors_sub_;
    std::shared_ptr<tf2_filter> tf2_filter_;

    // Tracker info publisher
    rclcpp::Publisher<rm_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;


    // heartbeat
    rm_tools::HeartBeatPublisher::SharedPtr heart_beat_;
};

} // namespace rm_auto_aim::tracker
