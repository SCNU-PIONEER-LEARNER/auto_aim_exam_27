#include <algorithm>
#include <cassert>
#include <cfloat>
#include <string>
#include <type_traits>

#include <rclcpp/logging.hpp>

#include "rm_auto_aim_common/tracker_def.hpp"
#include "rm_tools/math.hpp"
#include "rm_tools/rotation_bridge.hpp"
#include "rm_auto_aim_common/common.hpp"
#include "rm_tracker/target/outpost_target.hpp"
#include "rm_tracker/tracker/armor_tracker.hpp"

#include "rm_tools/math.hpp"
#include "rm_tools/rotation_bridge.hpp"

namespace rm_auto_aim::tracker
{

ArmorTracker::ArmorTracker(ArmorTrackerParmas_s _params)
{
    auto & pm = _params;
    pm.mahalanobisSQ_gate = pm.mahalanobisSQ_gate > 0.0 ? pm.mahalanobisSQ_gate : 9.49;

    // 空目标不为空指针，这个指针必须初始化不能漏
    targets_[NONE_0] = std::make_unique<RobotTarget>(
        pm.noise, pm.state_machine_params, pm.max_iterations, pm.convergence_epsilon, pm.mahalanobisSQ_gate);

    for (int i = HERO_1; i <= INFANTRY_4; ++i) {
        targets_[i] = std::make_unique<RobotTarget>(
            pm.noise, pm.state_machine_params, pm.max_iterations, pm.convergence_epsilon, pm.mahalanobisSQ_gate);
    }
    
    targets_[INFANTRY_5] = std::make_unique<RobotTarget>(
        pm.noise, pm.state_machine_params, pm.max_iterations, pm.convergence_epsilon, pm.mahalanobisSQ_gate);
    targets_[SENTRY_7] = std::make_unique<RobotTarget>(
        pm.noise, pm.state_machine_params, pm.max_iterations, pm.convergence_epsilon, pm.mahalanobisSQ_gate);
    targets_[BASE_8] = std::make_unique<RobotTarget>(
        pm.noise, pm.state_machine_params, pm.max_iterations, pm.convergence_epsilon, pm.mahalanobisSQ_gate);

    NoiseParams_s outpost_noise = pm.noise;
    outpost_noise.s2qxyz = outpost_noise.s2qxyz * 0.05;
    outpost_noise.s2qyaw_max = outpost_noise.s2qyaw_max * 0.01;
    outpost_noise.s2qyaw_min = outpost_noise.s2qyaw_min * 0.01;

    targets_[OUTPOST_11] = std::make_unique<OutpostTarget>(
        outpost_noise, pm.state_machine_params, pm.max_iterations, pm.convergence_epsilon,
        pm.outpost_preset_vyaw,
        pm.mahalanobisSQ_gate);

    for ([[maybe_unused]] const auto & pt : targets_) {
        assert(pt != nullptr && "所有目标都必须被初始化，但有目标指针没有被初始化");
    }
}

void ArmorTracker::resetTracker()
{
    for (auto & target : targets_) {
        if (target) {
            target->resetStateMachine();
        }
    }
}

void ArmorTracker::changeTarget()
{
    // 只做修改状态机这件事
    if (tracking_armor_type_ == RobotType_i::NONE_0) {
        return;
    }

    targets_[tracking_armor_type_]->state_ = State_e::CHANGE_TARGET;
}

} // namespace rm_auto_aim::tracker

namespace
{
// Detect if class has method "SpecialJudge"
template <typename, typename = void> struct HasSpecialJudge_s : std::false_type {};
template <typename T>
struct HasSpecialJudge_s<T, std::void_t<decltype(std::declval<T>().specialJudge(false))>> : std::true_type {};
} // namespace

namespace rm_auto_aim::tracker
{

std::tuple<RobotType_i, Target> ArmorTracker::track(const Armors::SharedPtr & _armors_msg, rclcpp::Time _now)
{
    //* 1. Predict
    for (auto & target : targets_) {
        if (!target || target->state_ == State_e::LOST)
            continue; // 空指针和未初始化目标跳过
        assert(target->isInitialized() && "Target must be initialized before prediction.");

        target->predict(*target, _now);
    }
    RCLCPP_DEBUG(rclcpp::get_logger("ArmorTracker"), "KF Predicted");


    //* 2. Outpost SpecialJudge
    static_assert(HasSpecialJudge_s<OutpostTarget>::value, "前哨站目标有未实现的函数！");
    OutpostTarget * op = static_cast<OutpostTarget *>(&(*(targets_[OUTPOST_11])));
    if (op->state_ != State_e::LOST)
        op->specialJudge(false);


    //* 3. Divide armors
    /// @note sort method depends on target state.
    ArmorGroups armors_ptrs(TARGET_COUNT, ArmorGroup());
    for (const auto & armor : _armors_msg->armors) {
        RobotType_i i = parseArmorType(armor);
        armors_ptrs[i].push_back(&armor);
    }
    armors_ptrs[NONE_0].clear(); // 无效装甲板清空，避免误用


    //* 4. Choose Tracking Armor
    bool change_target = targets_[tracking_armor_type_]->state_ == State_e::CHANGE_TARGET;
    bool reset_tracker = targets_[tracking_armor_type_]->state_ == State_e::LOST;
    if (reset_tracker || change_target) {
        double min_dist = DBL_MAX;
        RobotType_i new_type = NONE_0;

        for (const auto & armor : _armors_msg->armors) {
            RobotType_i parsed_type = parseArmorType(armor);

            if (armor.distance_to_image_center > min_dist || 
                parsed_type == RobotType_i::NONE_0) {
                continue;
            }

            if (change_target && parsed_type == tracking_armor_type_) {
                continue;
            }

            min_dist = armor.distance_to_image_center;
            new_type = parsed_type;
        }
        tracking_armor_type_ = new_type;
    }


    //* 5. Init or Update target
    for (int idx = HERO_1; idx < TARGET_COUNT; ++idx) {
        auto i = static_cast<RobotType_i>(idx);
        auto & target = targets_[i];
        auto & armors = armors_ptrs[i];

        if (armors.empty()) {
            if (target && target->state_ != State_e::LOST) {
                target->updateStateMachine(false, _now);
            }
            continue;
        }

        // Init
        if (target->state_ == State_e::LOST) {
            // Find the Armor has the smallest abs(armor_yaw)
            const Armor * init_armor = *std::min_element(
                armors.begin(), armors.end(), [](const Armor * _a, const Armor * _b) -> bool
                { return (std::abs(_a->yaw)) < (std::abs(_b->yaw)); });
            target->updateArmorsNum(*init_armor);
            target->initTarget(target->getStateFromArmor(*init_armor));            
            target->updateStateMachine(true, _now);
            
            // 这个本来要放在initTarget里的，但是所有target要改很难受，而且后面把ros换掉之后还要换一次，那下次丕定吧
            target->initKfPredictTime(_now);
        }
        // Update
        else {
            // The second member of pair is "armor_id" from state. The yaw of armor with id '0' is X_[yaw]
            std::vector<bool> used_armor_ids(static_cast<int>(target->armors_num_), false);
            bool matched = false;

            // flow update
            for (const auto & armor : armors) {
                const auto & xyz = armor->pose.position;
                Eigen::Vector3d ypd = rm_tools::xyz2ypd(Eigen::Vector3d(xyz.x, xyz.y, xyz.z));
                Eigen::Vector4d z_armor(
                    ypd[0], ypd[1], ypd[2], rm_tools::orientationToYaw(armor->pose.orientation));
                
                int armor_id = target->matchArmor(z_armor);
                if (armor_id < 0 || armor_id >= static_cast<int>(used_armor_ids.size()) ||
                    used_armor_ids[armor_id]) {
                    continue;
                }

                used_armor_ids[armor_id] = true;
                target->update(*target, z_armor, armor_id);
                matched = true;
            }
            target->updateStateMachine(matched, _now);
        }

    } // Init or Update

    auto tracking_target_state = [this, &armors_ptrs]() -> Target
    {
        if (RobotType_i::NONE_0 == this->tracking_armor_type_) {
            Target msg;
            msg.tracking = false;
            return msg;
        } else {
            Target msg = targets_[this->tracking_armor_type_]->getTargetMsg();
            msg.id = std::string(ARMOR_NAMES[static_cast<int>(this->tracking_armor_type_)]);
            msg.visiable_num = static_cast<int>(armors_ptrs[this->tracking_armor_type_].size());
            return msg;
        }
    }();

    return {tracking_armor_type_, tracking_target_state};
}



} // namespace rm_auto_aim::tracker
