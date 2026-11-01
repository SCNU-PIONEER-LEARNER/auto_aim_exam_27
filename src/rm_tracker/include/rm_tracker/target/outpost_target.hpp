#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "rm_tracker/target/armor_target.hpp"

namespace rm_auto_aim::tracker
{

class OutpostTarget : public ArmorTarget {
public:
    //* ========== 构造函数和析构函数 ==========

    explicit OutpostTarget(
        NoiseParams_s _N, StateMachineParams_s _state_params, int _max_iterations, double _converge_epsilon,
        bool _use_preset_vyaw = false, double _mahalanobisSQ_gate = 9.49)
        : ArmorTarget(
              _N, _state_params, 11,
              _mahalanobisSQ_gate),
          use_preset_vyaw_(_use_preset_vyaw)
    {
        armors_num_ = ArmorsNum_i::OUTPOST_3;
    }

    virtual ~OutpostTarget() = default;

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

    /// @brief 前哨站独有函数
    /// @param _is_stop_rotating 前哨装甲板是否停止旋转，默认不停止
    /// @note 装甲板停转后逻辑需要修改
    void specialJudge(bool _is_stop_rotating = false);

    //* ============== 成员变量 ===============

private:
    bool use_preset_vyaw_ = false;
    static constexpr double OUTPOST_R = 0.2765;
    static constexpr double OUTPOST_V_YAW = 0.8 * M_PI;
    static constexpr double OUTPOST_DZ = 0.102;

}; // class OutpostTarget

} // namespace rm_auto_aim::tracker
