#pragma once

#include <Eigen/Dense>

#include "rm_tracker/target/armor_target.hpp"
// #include "rm_tracker/kalman_filters/ekf.hpp"

namespace rm_auto_aim::tracker
{

class IrregularTarget : public ArmorTarget {
public:
    //* ========== 构造函数和析构函数 ==========

    explicit IrregularTarget(
        NoiseParams_s _N, StateMachineParams_s _state_params, int _max_iter, double _convergence_epsilon,
        double _mahalanobisSQ_gate = 9.49)
        : ArmorTarget(
              _N, _state_params, 15,
              _mahalanobisSQ_gate)
    {
        armors_num_ = ArmorsNum_i::NORMAL_4;
    }

    virtual ~IrregularTarget() = default;

    //* ========== 必须实现的虚函数 ==========

    void initTarget(const Eigen::VectorXd & _Z) override;

    Eigen::MatrixXd F(double _dt) const override; // 状态转移矩阵
    Eigen::MatrixXd Q(double _dt) const override; // 过程噪声协方差矩阵

    Eigen::MatrixXd H_jacobian(int _armor_id) const override;     // 观测矩阵
    Eigen::MatrixXd R(const Eigen::VectorXd & _Z) const override; // 观测噪声协方差矩阵

    Eigen::VectorXd h(const Eigen::VectorXd & _X, int _armor_id) const override;

    TargetMsg getTargetMsg() const override;

    //         === 装甲板特化接口 ===
    using ArmorTarget::getArmorPositionFromState;
    Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & _X, int _id) const override;

    using ArmorTarget::get_Z_ArmorFromState;
    double get_Z_ArmorFromState(const Eigen::VectorXd & _X, int _id) const override;

    using ArmorTarget::getRadiusFromState;
    double getRadiusFromState(const Eigen::VectorXd & _X, int _id) const override;

    //* ========== 含非线性量需要重载 ==========

    /// @brief 原`x_add`函数部分，理论上应该在_kf里调用
    void normalizeState() override;

    Eigen::VectorXd z_substract(Eigen::VectorXd _Z_meas, Eigen::VectorXd _Z_pred) const override;

    bool diverged() const override;

private:
}; // class IrregularTarget

} // namespace rm_auto_aim::tracker
