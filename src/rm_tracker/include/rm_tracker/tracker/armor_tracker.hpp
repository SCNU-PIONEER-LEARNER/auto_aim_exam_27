#pragma once

static_assert(__cplusplus >= 201703L, "C++17 or later required!");

// STL
#include <array>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include <rclcpp/time.hpp>

#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"

#include "rm_tracker/target/irregular_target.hpp"
#include "rm_tracker/target/outpost_target.hpp"
#include "rm_tracker/target/robot_target.hpp"

#include "rm_auto_aim_common/common.hpp"
#include "rm_auto_aim_common/tracker_def.hpp"

namespace rm_auto_aim::tracker
{

using Armors = rm_interfaces::msg::Armors;
using ArmorGroup = std::vector<const Armor *>;
using ArmorGroups = std::vector<ArmorGroup>;
using Target = rm_interfaces::msg::Target;

struct ArmorTrackerParmas_s {
    NoiseParams_s noise;
    StateMachineParams_s state_machine_params;
    int max_iterations = 5;            // IESKF可选最大迭代次数参数
    double convergence_epsilon = 1e-6; // IESKF可选迭代精度
    double mahalanobisSQ_gate = 9.49;  // 马氏距离平方阈值，默认值chi-square, dof=4, 95%
    bool outpost_preset_vyaw = false;
};

class ArmorTracker {
public:
    ArmorTracker(ArmorTrackerParmas_s _params);

    ~ArmorTracker() = default;

    using SharedPtr = std::shared_ptr<ArmorTracker>;
    using UniquePtr = std::unique_ptr<ArmorTracker>;

    std::tuple<RobotType_i, Target> track(const Armors::SharedPtr & _armors_msg, rclcpp::Time _now);

    void resetTracker(); // 重置所有目标状态机和滤波器
    void changeTarget(); // 切换目标，重置新目标状态机和滤波器

    const Eigen::VectorXd & getTrackingState(RobotType_i _type) const // 获取正在跟踪的目标状态向量
    {
        return targets_[_type]->X_;
    }

    void resetPrevioustarget();

protected:
    RobotType_i tracking_armor_type_ = RobotType_i::NONE_0;

    std::array<std::unique_ptr<ArmorTarget>, TARGET_COUNT> targets_;

private:
    // RobotType_i last_tracking_armor_type = RobotType_i::NONE_0; // future expermental


}; // class ArmorTracker

} // namespace rm_auto_aim::tracker
