#include "rm_auto_aim_common/convert_msg.hpp"
#include "rm_tools/rotation_bridge.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rm_interfaces/msg/detail/target__struct.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <vector>

// #include "rm_auto_aim_common/common.hpp"
// #include "rm_auto_aim_common/armor.hpp"
// #include "rm_auto_aim_common/tools.hpp"

namespace rm_auto_aim
{

static inline double getAccordRadius(const rm_interfaces::msg::Target & _target, int _index)
{
    if (1 == _index) {
        return _target.radius + _target.dr1;
    } else if (2 == _index) {
        return _target.radius + _target.dr2;
    } else if (3 == _index) {
        return _target.radius + _target.dr3;
    } else {
        return _target.radius;
    }
}

static inline double getAccordZArmor(const rm_interfaces::msg::Target & _target, int _index)
{
    if (1 == _index) {
        return _target.position.z + _target.dz1;
    } else if (2 == _index) {
        return _target.position.z + _target.dz2;
    } else if (3 == _index) {
        return _target.position.z + _target.dz3;
    } else {
        return _target.position.z;
    }
}

std::vector<geometry_msgs::msg::Pose> Target2Armors(const rm_interfaces::msg::Target & _target)
{
    constexpr double PI = 3.1415926535897932384626433832795;
    constexpr double FIFTEEN_DEG_IN_RAD = PI / 6;

    std::vector<geometry_msgs::msg::Pose> armors_pose;

    armors_pose.reserve(_target.armors_num);

    for (int i = 0; i < _target.armors_num; ++i) {
        double sin_yaw = std::sin(_target.yaw + 2 * PI * i / _target.armors_num);
        double cos_yaw = std::cos(_target.yaw + 2 * PI * i / _target.armors_num);

        double radius = getAccordRadius(_target, i);

        armors_pose.emplace_back(geometry_msgs::msg::Pose(rosidl_runtime_cpp::MessageInitialization::SKIP));
        auto & pose = armors_pose.back();

        pose.position.x = -radius * cos_yaw + _target.position.x;
        pose.position.y = -radius * sin_yaw + _target.position.y;
        pose.position.z = getAccordZArmor(_target, i);

        using RotBridge = rm_tools::tf::rot_bridge;
        pose.orientation =
            RotBridge(
                Eigen::Vector3d((_target.yaw + 2 * PI * i / _target.armors_num), FIFTEEN_DEG_IN_RAD, 0.0), 2, 1, 0)
                .quaternion_ros();
    }

    return armors_pose;
}

} // namespace rm_auto_aim