#include "rm_tracker/tracker_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <tuple>

#include <rm_tools/math.hpp>


namespace rm_auto_aim::tracker
{

TrackerNode::TrackerNode(const rclcpp::NodeOptions & _options) : Node("rm_tracker", _options)
{
    RCLCPP_INFO(get_logger(), "Starting armor tracker node");

    this->max_armor_distance_ = declare_parameter<double>("max_armor_distance", 10.0);
    this->max_armor_z_ = declare_parameter<double>("max_armor_z", 1.3);

    // clang-format off
    // Init armor_tracker
    try {
    ArmorTrackerParmas_s params{
        .noise = NoiseParams_s {
            .s2qxyz = declare_parameter<double>("ekf.sigma2_q_xyz", 0.05),
            .s2qyaw_min = declare_parameter<double>("ekf.sigma2_q_yaw_min", 5.0),
            .s2qyaw_max = declare_parameter<double>("ekf.sigma2_q_yaw_max", 10.0),
            .r_py = declare_parameter<double>("ekf.r_py", 4e-3),
            .r_d = declare_parameter<double>("ekf.r_d", 0.05),
            .r_yaw = declare_parameter<double>("ekf.r_yaw", 0.02),
        },
        .state_machine_params = StateMachineParams_s {
            .tracking_thres = static_cast<int>(declare_parameter<int>("tracker.tracking_thres", 5)),
            .change_thres = static_cast<int>(declare_parameter<int>("tracker.change_thres", 20)),
            .lost_time_thres = declare_parameter<double>("tracker.lost_time_thres", 0.3),
        },
        .max_iterations = static_cast<int>(declare_parameter<int>("ekf.max_iterations", 5)),
        .convergence_epsilon = declare_parameter<double>("ekf.convergence_epsilon", 1e-6),
        .mahalanobisSQ_gate = declare_parameter<double>("tracker.mahalanobis_gate", 9.49),
        .outpost_preset_vyaw = declare_parameter<bool>("outpost_preset_vyaw", false),
    };
    armor_tracker_ = std::make_unique<ArmorTracker>(params);
    } catch (std::exception & ex) {
        RCLCPP_ERROR(this->get_logger(), "Error creating tracker: %s", ex.what());
        throw ex;
    }
    // clang-format on

    // Reset tracker service
    reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "tracker/reset",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            armor_tracker_->resetTracker();
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Tracker reset!");
            return;
        });


    // Change target service
    change_target_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "tracker/change",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response)
        {
            armor_tracker_->changeTarget();
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
        armors_sub_, *tf2_buffer_, target_frame_, 50, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(1));
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&TrackerNode::armorsCallback, this);


    // Measurement publisher (for debug usage)
    info_pub_ = this->create_publisher<rm_interfaces::msg::TrackerInfo>("tracker/info", 10);

    // Publisher
    target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("tracker/target", rclcpp::SensorDataQoS());

    // heartbeat
    heart_beat_ = rm_tools::HeartBeatPublisher::create(this);
}

void TrackerNode::armorsCallback(const rm_interfaces::msg::Armors::SharedPtr & _armors_msg)
{
    static std::optional<rclcpp::Time> last_time;

    // Tranform armor position from image frame to world coordinate
    for (auto & armor : _armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = _armors_msg->header;
        ps.pose = armor.pose;
        try {
            armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
            return;
        }
    }

    // Filter abnormal armors
    _armors_msg->armors.erase(
        std::remove_if(
            _armors_msg->armors.begin(), _armors_msg->armors.end(),
            [this](const rm_interfaces::msg::Armor & _armor)
            {
                return std::abs(_armor.pose.position.z) > this->max_armor_z_ ||
                       Eigen::Vector2d(_armor.pose.position.x, _armor.pose.position.y).norm() >
                           this->max_armor_distance_;
            }),
        _armors_msg->armors.end());

    // Update tracker
    rclcpp::Time time = _armors_msg->header.stamp;
    double dt = last_time.has_value() ? (time - *last_time).seconds() : 0.0;
    if (last_time.has_value() && dt <= 0.0) {
        Target target_msg;
        target_msg.header.stamp = time;
        target_msg.header.frame_id = target_frame_;
        target_pub_->publish(target_msg);
        last_time = time;
        return;
    }

    auto [tracked_type, target_msg] = armor_tracker_->track(_armors_msg, _armors_msg->header.stamp);
    // (void)tracked_type;

    target_msg.header.stamp = time;
    target_msg.header.frame_id = target_frame_;
    target_pub_->publish(target_msg);
    
    last_time = time;

    if (!target_msg.tracking) {
        return;
    }

    const Eigen::VectorXd & state = armor_tracker_->getTrackingState(tracked_type);
    rm_interfaces::msg::TrackerInfo info_msg;

    info_msg.position.x = target_msg.position.x;
    info_msg.position.y = target_msg.position.y;
    info_msg.position.z = target_msg.position.z;
    info_msg.position_diff = 
        rm_tools::hypot(target_msg.position.x - state[kf::x], target_msg.position.y - state[kf::y]);
    info_msg.yaw_diff = std::abs(target_msg.yaw - state[kf::yaw]);
    
    info_pub_->publish(info_msg);
}

} // namespace rm_auto_aim::tracker

#include <rclcpp_components/register_node_macro.hpp>
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::tracker::TrackerNode)
