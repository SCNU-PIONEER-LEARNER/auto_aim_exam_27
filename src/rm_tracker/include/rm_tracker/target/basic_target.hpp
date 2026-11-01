#pragma once

static_assert(__cplusplus >= 201402L, "C++14 or later required!");

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

#include "rm_auto_aim_common/tracker_def.hpp"
#include "rm_interfaces/msg/detail/target__struct.hpp"


namespace rm_auto_aim::tracker
{

using TargetMsg = rm_interfaces::msg::Target;

// 状态机参数
struct StateMachineParams_s {
    int tracking_thres = INT32_MAX; // 从DETECTING转TRACKING的连续检测帧数阈值
    int change_thres = INT32_MAX;   // 从CHANGE_TARGET转TRACKING的连续切换帧数阈值
    double lost_time_thres = 0;     // 从TRACKING转LOST的连续丢失时间阈值
};

// 各种计数器
struct Counter_s {
    int lost_count = 0;   // 连续丢失计数器
    int detect_count = 0; // 连续检测计数器
    int track_count = 0;  // 连续跟踪计数器
    int change_count = 0; // 连续切换计数器
};

class BasicTarget {
public:
    // ========== 构造函数和析构函数 ==========

    explicit BasicTarget(int _dim, StateMachineParams_s _state_params)
        : X_(Eigen::VectorXd::Zero(_dim)), P_(Eigen::MatrixXd::Identity(_dim, _dim)),
          I_(Eigen::MatrixXd::Identity(_dim, _dim)), state_params_(_state_params)
    {
    }

    virtual ~BasicTarget() = default;

    // ========== 必须实现的虚函数 ==========

    virtual void initTarget(const Eigen::VectorXd & _Z) = 0;

    virtual Eigen::MatrixXd F(double _dt) const = 0; // 状态转移矩阵
    virtual Eigen::MatrixXd Q(double _dt) const = 0; // 白噪声矩阵

    virtual Eigen::MatrixXd H_jacobian(int _id) const = 0;           // 获取 状态向量➜ 观测向量 的雅可比矩阵
    virtual Eigen::MatrixXd R(const Eigen::VectorXd & _Z) const = 0; // 获取测量噪声矩阵

    virtual Eigen::VectorXd h(const Eigen::VectorXd & _X, int _id) const = 0; // 状态→观测的非线性映射

    virtual TargetMsg getTargetMsg() const = 0; // 获取目标消息

    // ========== 可重写的通用方法 ==========

    /// @brief 角度/四元数 等非线性量归一化，可选实现
    virtual void normalizeState() {};

    /// @brief 观测残差函数,非线性量必须重载
    virtual Eigen::VectorXd z_substract(Eigen::VectorXd _Z_meas, Eigen::VectorXd _Z_pred) const
    {
        return _Z_meas - _Z_pred;
    }

    /// @brief 发散检查，检查范围、角速度范围等
    virtual bool diverged() const { return true; }

    // ============ 可复用通用函数 =========

    /// @brief 判断是否完成初始化，_kf的predict函数里会调用这个函数检查
    bool isInitialized() const { return initialized_; }

    inline const Eigen::VectorXd &getState() const { return X_; }

    void initKfPredictTime(rclcpp::Time _now) {
        this->last_predict_time_ = _now;
    }

    void predict(BasicTarget &_target, rclcpp::Time _now);

    void update(BasicTarget &_target, Eigen::VectorXd _Z_meas, int _armor_id);

    // ============ 滤波器 ================
    Eigen::VectorXd X_;   // 状态
    Eigen::MatrixXd P_;       // 协方差矩阵
    const Eigen::MatrixXd I_; // 单位矩阵，计算卡尔曼增益时用到，根据状态维度初始化，缓存只读
    rclcpp::Time last_predict_time_;

    // ========== 成员变量 ==========
    bool predicted_ = false; // 是否已经预测过，预测后置true，更新后置false
    State_e state_;          // 状态机
    Counter_s cnt_;          // 各种给状态机用的计数器

    rclcpp::Time last_time_;   // 上次更新时间
    bool initialized_ = false; // 是否完成初始化，init函数调用后置true

    // 参数部分
    StateMachineParams_s state_params_; // 状态机参数
};

} // namespace rm_auto_aim::tracker
