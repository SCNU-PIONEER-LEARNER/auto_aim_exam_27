#include <Eigen/Dense>
#include <rclcpp/logging.hpp>

#include <vector>

#include "rm_tools/math.hpp"
#include "rm_tools/rotation_bridge.hpp"
#include "rm_tracker/target/armor_target.hpp"

namespace rm_auto_aim::tracker
{

std::vector<Eigen::Vector4d> ArmorTarget::getArmorsPositionFromState(const Eigen::VectorXd & _X) const
{
    std::vector<Eigen::Vector4d> armors_p;
    for (int i = 0; i < this->armors_num_; i++) {
        double armor_yaw = rm_tools::limit_rad(_X[kf::yaw] + i * 2 * CV_PI / this->armors_num_);
        Eigen::Vector3d xyz = getArmorPositionFromState(_X, i);
        Eigen::Vector3d ypd = rm_tools::xyz2ypd(xyz);
        armors_p.emplace_back(ypd[0], ypd[1], ypd[2], armor_yaw);
    }
    return armors_p;
}

void ArmorTarget::resetStateMachine()
{
    state_ = State_e::LOST;
    cnt_.detect_count = 0;
    cnt_.lost_count = 0;
    cnt_.track_count = 0;
    cnt_.change_count = 0;
}

void ArmorTarget::updateArmorsNum(const Armor & _armor)
{
    if (_armor.number == "outpost") {
        armors_num_ = ArmorsNum_i::OUTPOST_3;
    } else if (_armor.number == "base") {
        // armors_num_ = ArmorsNum_i::BASE_1;
        armors_num_ = ArmorsNum_i::NORMAL_4; // 先当成普通四装甲板处理，后续如果需要可以单独处理
    } else {
        armors_num_ = ArmorsNum_i::NORMAL_4;
    }
}

void ArmorTarget::updateStateMachine(bool _matched, rclcpp::Time _now)
{
    // Prevent radius from spreading but might cause divergence
    if (X_[kf::r] < 0.12 && armors_num_ != ArmorsNum_i::OUTPOST_3) {
        X_[kf::r] = 0.12;
    } else if (X_[kf::r] > 0.4 && armors_num_ != ArmorsNum_i::OUTPOST_3) {
        X_[kf::r] = 0.4;
    }

    const int diverge_check_thres = armors_num_ == ArmorsNum_i::OUTPOST_3 ? 10 : 3;

    // Check if EKF diverged after the target has had time to settle.
    if (state_ == State_e::TRACKING || state_ == State_e::TEMP_LOST) {
        if (cnt_.track_count > diverge_check_thres && diverged()) {
            RCLCPP_WARN(rclcpp::get_logger("rm_tracker"), "KF diverged!");
            state_ = State_e::LOST;
            cnt_.track_count = 0;
            cnt_.lost_count = 0;
            return;
        }
    }

    if (state_ == State_e::DETECTING) {
        if (_matched) {
            cnt_.detect_count++;
            if (cnt_.detect_count > state_params_.tracking_thres) {
                cnt_.detect_count = 0;
                state_ = State_e::TRACKING;
                cnt_.track_count = 0;
            }
        } else {
            cnt_.detect_count = 0;
            cnt_.track_count = 0;
            state_ = State_e::LOST;
        }
    } else if (state_ == State_e::TRACKING) {
        if (_matched) {
            cnt_.track_count++;
            cnt_.lost_count = 0;
        } else {
            state_ = State_e::TEMP_LOST;
            cnt_.lost_count++;
        }
    } else if (state_ == State_e::TEMP_LOST) {
        if (!_matched) {
            cnt_.lost_count++;
            double dt = (_now - last_time_).seconds();
            int lost_thresh =
                dt > 0.0 ? static_cast<int>(state_params_.lost_time_thres / dt) : 0;
            if (lost_thresh <= 0 || cnt_.lost_count > lost_thresh) {
                cnt_.lost_count = 0;
                cnt_.track_count = 0;
                state_ = State_e::LOST;
            }
        } else {
            state_ = State_e::TRACKING;
            cnt_.lost_count = 0;
            cnt_.track_count++;
        }
    } else if (state_ == State_e::CHANGE_TARGET) {
        if (cnt_.change_count > state_params_.change_thres) {
            state_ = State_e::TRACKING;
            cnt_.change_count = 0;
            cnt_.track_count = 0;
        } else {
            cnt_.change_count++;
        }
    } else if (state_ == State_e::LOST && _matched) {
        state_ = State_e::DETECTING;
        cnt_.detect_count = 0;
        cnt_.lost_count = 0;
        cnt_.track_count = 0;
        return;
    }

    // update time if matched
    if (_matched) {
        last_time_ = _now;  // This time only serve for LOST judge
    }
}

int ArmorTarget::matchArmor(const Eigen::VectorXd & _Z_meas) const
{
    assert(this->predicted_ && "matchArmor should be called after predict()");

    if (_Z_meas.size() != 4 || !_Z_meas.array().isFinite().all()) {
        RCLCPP_WARN(rclcpp::get_logger("rm_tracker"), "Invalid measurement in matchArmor");
        return -1;
    }

    const Eigen::MatrixXd R = this->R(_Z_meas);
    if (R.rows() != 4 || R.cols() != 4 || !R.array().isFinite().all()) {
        RCLCPP_WARN(rclcpp::get_logger("rm_tracker"), "Invalid measurement noise in matchArmor");
        return -1;
    }

    double mahalanobisSQ = DBL_MAX;
    int matched_id = -1;

    for (int id = 0; id < armors_num_; id++) {
        const Eigen::MatrixXd H = this->H_jacobian(id);
        if (H.rows() != 4 || H.cols() != X_.size() || !H.array().isFinite().all()) {
            continue;
        }

        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        S = (S + S.transpose()) * 0.5;
        S.diagonal().array() += 1e-12; // add a small value to the diagonal for numerical stability
        if (S.rows() != 4 || S.cols() != 4 || !S.array().isFinite().all()) {
            continue;
        }

        Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
        if (ldlt.info() != Eigen::Success) {
            RCLCPP_WARN(rclcpp::get_logger("rm_tracker"), "LDLT decomposition failed in matchArmor");
            continue;
        }

        const Eigen::VectorXd residual = z_substract(_Z_meas, this->h(X_, id));
        if (residual.size() != 4 || !residual.array().isFinite().all()) {
            continue;
        }
        const double tmp_mahalanobisSQ = residual.transpose() * ldlt.solve(residual);
        if (std::isfinite(tmp_mahalanobisSQ) && tmp_mahalanobisSQ < mahalanobisSQ &&
            tmp_mahalanobisSQ < mahalanobisSQ_gate_) {
            mahalanobisSQ = tmp_mahalanobisSQ;
            matched_id = id;
        }
    }

    if (!std::isfinite(mahalanobisSQ)) {
        matched_id = -1;
    }

    return matched_id;
}

Eigen::Vector4d ArmorTarget::getStateFromArmor(const Armor & _armor) const
{
    const double & xa = _armor.pose.position.x;
    const double & ya = _armor.pose.position.y;
    const double & za = _armor.pose.position.z;
    const double armor_yaw = rm_tools::orientationToYaw(_armor.pose.orientation);
    return Eigen::Vector4d(xa, ya, za, armor_yaw);
}

} // namespace rm_auto_aim::tracker
