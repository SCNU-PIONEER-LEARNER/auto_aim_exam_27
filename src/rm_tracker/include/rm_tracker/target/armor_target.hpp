#pragma once

#include <Eigen/Dense>
#include <vector>

#include "rm_auto_aim_common/common.hpp"
#include "rm_tracker/target/basic_target.hpp"

namespace rm_auto_aim::tracker
{

using Armor = rm_interfaces::msg::Armor;



// 噪声方差参数
struct NoiseParams_s {
    double s2qxyz = -1; // 位置状态(x,y,z)过程噪声方差的最小值 (平方后的Q矩阵位置分量)

    double s2qyaw_min = -1; // 偏航角状态过程噪声方差的最小值 (平方后的Q矩阵偏航角分量)
    double s2qyaw_max = -1; // 偏航角状态过程噪声方差的最大值 (平方后的Q矩阵偏航角分量)

    double r_py = -1;  // 观测噪声方差：俯仰角和偏航角 (Pitch and Yaw)
    double r_d = -1;   // 观测噪声方差：距离 (Distance)
    double r_yaw = -1; // 观测噪声方差：偏航角 (Yaw)
};

/**
 * @brief 具有装甲板信息的目标基类，包含状态机和装甲板相关的接口
 * @note 该类是一个抽象基类，不能直接实例化，必须通过继承实现纯虚函数，构造方法：
 * 
 * - 继承这个类，至少实现纯虚函数，并按需重载好函数
 * 
 * - 在派生类构造函数中调用基类init延迟初始化函数初始化状态向量和协方差矩阵
 */
class ArmorTarget : public BasicTarget {
public:
    //* ========== 构造函数和析构函数 ==========

    explicit ArmorTarget(
        NoiseParams_s _N, StateMachineParams_s _state_params, int _dim,
        double _mahalanobisSQ_gate = 9.49)
        : BasicTarget(_dim, _state_params), N_(_N), mahalanobisSQ_gate_(_mahalanobisSQ_gate)
    {
        resetStateMachine();
    }

    virtual ~ArmorTarget() = default;

    //* ========== 必须实现的虚函数 ==========

    /// @param _Z (xc, yc, zc, armor_yaw)^T
    virtual void initTarget(const Eigen::VectorXd & _Z) override = 0;

    virtual Eigen::MatrixXd F(double _dt) const override = 0; // 状态转移矩阵
    virtual Eigen::MatrixXd Q(double _dt) const override = 0; // 白噪声矩阵

    virtual Eigen::MatrixXd H_jacobian(int _armor_id) const override = 0; // 获取 状态向量➜观测向量 的雅可比矩阵

    /** @brief 获取测量噪声矩阵
     *  @param _Z 观测向量(yaw, pitch, distance, armor_yaw)^T
     *  @return 测量噪声矩阵 */
    virtual Eigen::MatrixXd R(const Eigen::VectorXd & _Z) const override = 0;

    virtual Eigen::VectorXd h(const Eigen::VectorXd & _X, int _id) const override = 0; // 状态➜观测的非线性映射

    using TargetMsg = rm_interfaces::msg::Target;
    virtual TargetMsg getTargetMsg() const override = 0; // 获取目标消息

    //        == 装甲板特化方法 ==

    // 从状态向量中提取装甲板位置，_id表示第几个装甲板
    // 非虚的公有接口，调用纯虚函数
    Eigen::Vector3d getArmorPositionFromState(int _id) const { return getArmorPositionFromState(X_, _id); }
    double get_Z_ArmorFromState(int _id) const { return get_Z_ArmorFromState(X_, _id); }
    double getRadiusFromState(int _id) const { return getRadiusFromState(X_, _id); }

    // 从任意状态向量中提取装甲板位置，_id表示第几个装甲板
    virtual Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & _X, int _id) const = 0;

    virtual double get_Z_ArmorFromState(const Eigen::VectorXd & _X, int _id) const = 0;

    virtual double getRadiusFromState(const Eigen::VectorXd & _X, int _id) const = 0;

    //* ==========      可选重载    ===========
    /** 
     * @brief 从状态向量中提取所有装甲板位置，
     * @param _X 状态向量，如果不提供则默认使用当前状态向量`X_`
     * @return 返回一个vector，元素个数等于装甲板数量，每个元素是一个四维向量(yaw, pitch, distance, angle)，其中angle表示装甲板朝向角（如果有的话）
    **/
    std::vector<Eigen::Vector4d> getArmorsPositionFromState() const { return getArmorsPositionFromState(X_); }

    /** 
     * @brief 从状态向量中提取所有装甲板位置，
     * @param _X 状态向量，如果不提供则默认使用当前状态向量`X_`
     * @return 返回一个vector，元素个数等于装甲板数量，每个元素是一个四维向量(yaw, pitch, distance, angle)，其中angle表示装甲板朝向（如果有的话）
    **/
    virtual std::vector<Eigen::Vector4d> getArmorsPositionFromState(const Eigen::VectorXd & _X) const;

    /**
     * @brief 重置状态机，设置初始状态和计数器等
     */
    virtual void resetStateMachine();

    //* ========== 含非线性量需要重载 ==========

    /// @brief 角度/四元数 等非线性量归一化，可选实现
    virtual void normalizeState() override {};

    /// @brief 观测残差函数,非线性量必须重载
    virtual Eigen::VectorXd z_substract(Eigen::VectorXd _Z_meas, Eigen::VectorXd _Z_pred) const override
    {
        return _Z_meas - _Z_pred;
    }

    /// @brief 检查半径范围、角速度范围等
    virtual bool diverged() const override { return true; }

    //* ============ 可复用通用函数 =========

    void updateArmorsNum(const Armor & _armor); // 根据装甲板信息更新装甲板数量（类型）

    void updateStateMachine([[maybe_unused]] bool _matched, rclcpp::Time _now);

    [[nodiscard]] Eigen::Vector4d getStateFromArmor(const Armor & _armor) const;

    /// @brief 匹配获取装甲板id
    /// @return 返回匹配的装甲板id，若没有匹配则返回-1。状态里yaw=0那块板id为0
    [[nodiscard]] int matchArmor(const Eigen::VectorXd & _Z_meas) const;

    //* ============ 成员变量 ==============

    // 状态部分

    // 这几个变量已经在BasicTarget基类中定义，当作下面两个变量没被注释就行
    // Eigen::VectorXd X_;      // 状态
    // Eigen::MatrixXd P_;      // 协方差矩阵

    // bool predicted_ = false; // 是否已经预测过，预测后置true，更新后置false
    // State_e state_;               // 状态机
    // Counter_s cnt_;               // 各种给状态机用的计数器
    ArmorsNum_i armors_num_; // 装甲板数量(类型)
    ArmorSize_i armor_size_; // 大小装甲板

    // 参数部分

    NoiseParams_s N_;                  // 噪声方差参数
    double mahalanobisSQ_gate_ = 9.49; // 马氏距离平方门限
    // StateMachineParams_s state_params_; // 状态机参数

}; // struct targetWithArmors

} // namespace rm_auto_aim::tracker
