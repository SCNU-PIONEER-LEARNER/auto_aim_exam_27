#include <cmath>
#include <type_traits>

#include "rm_tools/math.hpp"
#include "rm_tools/rotation_bridge.hpp"
#include "rm_tracker/target/robot_target.hpp"

#include "rm_auto_aim_common/tracker_def.hpp"

namespace rm_auto_aim::tracker
{

static_assert(
    !std::is_abstract_v<RobotTarget>,
    "RobotTarget is abstract, please implement all pure virtual functions.");

void RobotTarget::initTarget(const Eigen::VectorXd & _Z)
{
    const double & xa = _Z[0];
    const double & ya = _Z[1];
    const double & za = _Z[2];
    const double armor_yaw = rm_tools::limit_rad(_Z[3]);

    const double radius = 0.2;
    // 求中心点
    double xc = xa + radius * std::cos(armor_yaw);
    double yc = ya + radius * std::sin(armor_yaw);

    Eigen::VectorXd P0_dig(11);
    Eigen::VectorXd Xstate(11);

    // clang-format off
    //        xc  vx yc  vy zc  vz    yaw   v_yaw    r    dr     dz
    Xstate << xc, 0, yc, 0, za, 0, armor_yaw, 0,  radius,  0,    0;
    P0_dig << 1, 64, 1, 64, 1, 64,    0.4,   100,  1e-3,  1e-3, 1e-3;
    // clang-format on

    X_ = Xstate;
    P_ = P0_dig.asDiagonal();

    initialized_ = true;
    normalizeState();
}

Eigen::MatrixXd RobotTarget::F(double _dt) const
{
    // clang-format off
    return Eigen::MatrixXd {
    /*    x   vx    y   vy    z   vz  yaw v_yaw   r  dr  dz */
        { 1, _dt,   0,   0,   0,   0,   0,   0,    0,  0,  0}, // x
        { 0,   1,   0,   0,   0,   0,   0,   0,    0,  0,  0}, // vx
        { 0,   0,   1, _dt,   0,   0,   0,   0,    0,  0,  0}, // y
        { 0,   0,   0,   1,   0,   0,   0,   0,    0,  0,  0}, // vy
        { 0,   0,   0,   0,   1, _dt,   0,   0,    0,  0,  0}, // z
        { 0,   0,   0,   0,   0,   1,   0,   0,    0,  0,  0}, // vz
        { 0,   0,   0,   0,   0,   0,   1, _dt,    0,  0,  0}, // yaw
        { 0,   0,   0,   0,   0,   0,   0,   1,    0,  0,  0}, // v_yaw
        { 0,   0,   0,   0,   0,   0,   0,   0,    1,  0,  0}, // r
        { 0,   0,   0,   0,   0,   0,   0,   0,    0,  1,  0}, // dr
        { 0,   0,   0,   0,   0,   0,   0,   0,    0,  0,  1}  // dz
    };
    // clang-format on
}

Eigen::MatrixXd RobotTarget::Q(double _dt) const
{
    // Piecewise White Noise Model
    // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb

    // 仅定性
    [[maybe_unused]] const double vLinear = rm_tools::hypot(X_[kf::vx], X_[kf::vy]);
    const double vAngular = std::abs(X_[kf::v_yaw]);

    static const double SIGMA2_Q_XYZ_MIN = N_.s2qxyz / 2.0;

    const double sig2_xy = std::exp(-vAngular) * SIGMA2_Q_XYZ_MIN + SIGMA2_Q_XYZ_MIN; // 加速度方差
    // const double sig2_yaw =
    //     std::exp(-vLinear) * (N_.s2qyaw_max - N_.s2qyaw_min) + N_.s2qyaw_min; // 角加速度方差
    const double sig2_yaw = N_.s2qyaw_max; // 角加速度方差，暂不考虑线速度对其的影响
    const double sig2_z = 0.01 * sig2_xy;

    const double a = _dt * _dt * _dt * _dt / 4;
    const double b = _dt * _dt * _dt / 2;
    const double c = _dt * _dt;

    const double & sx = sig2_xy;
    const double & sy = sig2_yaw;
    const double & sz = sig2_z;

    // clang-format off
    return Eigen::MatrixXd {
    /*      x    vx     y    vy     z    vz   yaw  v_yaw   r   dr   dz */
        { a*sx, b*sx,    0,    0,    0,    0,    0,    0,   0,   0,   0}, // x
        { b*sx, c*sx,    0,    0,    0,    0,    0,    0,   0,   0,   0}, // vx
        {    0,    0, a*sx, b*sx,    0,    0,    0,    0,   0,   0,   0}, // y
        {    0,    0, b*sx, c*sx,    0,    0,    0,    0,   0,   0,   0}, // vy
        {    0,    0,    0,    0, a*sz, b*sz,    0,    0,   0,   0,   0}, // z
        {    0,    0,    0,    0, b*sz, c*sz,    0,    0,   0,   0,   0}, // vz
        {    0,    0,    0,    0,    0,    0, a*sy, b*sy,   0,   0,   0}, // yaw
        {    0,    0,    0,    0,    0,    0, b*sy, c*sy,   0,   0,   0}, // v_yaw
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,   0}, // r
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,   0}, // dr
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,   0,   0}  // dz
    };
    // clang-format on
}

Eigen::MatrixXd RobotTarget::H_jacobian(int _armor_id) const
{
    const int id = _armor_id; // _armor_id表示当前yaw落在四象限哪个范围之间，也就是第几个装甲板
    bool use_drdz = (id == 1 || id == 3); // 只有当装甲板在侧面时，dr和dz才有意义

    double armor_yaw = rm_tools::limit_rad(X_[kf::yaw] + id * 2 * CV_PI / armors_num_);
    double sin_armor_yaw = std::sin(armor_yaw);
    double cos_armor_yaw = std::cos(armor_yaw);
    double radius = getRadiusFromState(id);

    double dx_da = radius * sin_armor_yaw;
    double dy_da = -radius * cos_armor_yaw;
    double dx_dr = -cos_armor_yaw;
    double dy_dr = -sin_armor_yaw;

    double dx_ddr = use_drdz ? dx_dr : 0.0;
    double dy_ddr = use_drdz ? dy_dr : 0.0;
    double dz_ddz = use_drdz ? 1.0 : 0.0;

    // clang-format off
    Eigen::MatrixXd H_armor_xyza {
    /*   x  vx  y  vy  z  vz    yaw  v_yaw      r       dr      dz */
        {1,  0, 0,  0, 0,  0, dx_da,     0, dx_dr, dx_ddr,       0},
        {0,  0, 1,  0, 0,  0, dy_da,     0, dy_dr, dy_ddr,       0},
        {0,  0, 0,  0, 1,  0,     0,     0,     0,      0, dz_ddz},
        {0,  0, 0,  0, 0,  0,     1,     0,     0,      0,       0}
    };
    // clang-format on

    Eigen::Vector3d armor_xyz = getArmorPositionFromState(X_, id);
    Eigen::MatrixXd H_armor_ypd = rm_tools::xyz2ypd_jacobian(armor_xyz);

    // clang-format off
    Eigen::MatrixXd H_armor_ypda{
        {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
        {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
        {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
        {                0,                 0,                 0, 1}
    };
    // clang-format on

    return H_armor_ypda * H_armor_xyza;
}

Eigen::MatrixXd RobotTarget::R(const Eigen::VectorXd & _Z) const
{
    const double & distance = _Z[2];
    const double & center_yaw = _Z[0];
    const double & armor_yaw = _Z[3];
    double delta_angle = rm_tools::limit_rad(armor_yaw - center_yaw);

    // 不严谨数据，仅定性
    double r_distance_gain = 1.5 / N_.r_d;
    double r_yaw_gain = 10 / N_.r_yaw;

    Eigen::Vector4d R_dig(
        N_.r_py,                                                        // yaw
        N_.r_py,                                                        // pitch
        std::log(std::abs(delta_angle) + 1) / r_distance_gain + N_.r_d, // distance
        std::log(std::sqrt(distance) + 1) / r_yaw_gain + N_.r_yaw       // armor_yaw
    );
    return R_dig.asDiagonal();
}

Eigen::VectorXd RobotTarget::h(const Eigen::VectorXd & _X, int _armor_id) const
{
    Eigen::Vector3d xyz = getArmorPositionFromState(_X, _armor_id);
    Eigen::Vector3d ypd = rm_tools::xyz2ypd(xyz);
    double angle = rm_tools::limit_rad(_X[kf::yaw] + _armor_id * 2 * CV_PI / armors_num_);
    return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], angle);
}

TargetMsg RobotTarget::getTargetMsg() const
{
    TargetMsg msg;

    const bool target_available = state_ == State_e::TRACKING || state_ == State_e::TEMP_LOST;
    if (!target_available) {
        msg.tracking = false;
        return msg;
    }

    msg.tracking = true;
    msg.armors_num = static_cast<int>(armors_num_);

    msg.position.x = X_[kf::x];
    msg.position.y = X_[kf::y];
    msg.position.z = X_[kf::z];

    msg.velocity.x = X_[kf::vx];
    msg.velocity.y = X_[kf::vy];
    msg.velocity.z = X_[kf::vz];

    msg.yaw = X_[kf::yaw];
    msg.v_yaw = X_[kf::v_yaw];
    msg.radius = X_[kf::r];

    msg.dr1 = X_[kf::dr];
    msg.dr2 = 0.0;
    msg.dr3 = X_[kf::dr];
    msg.dz1 = X_[kf::dz];
    msg.dz2 = 0.0;
    msg.dz3 = X_[kf::dz];

    return msg;
}

Eigen::Vector3d RobotTarget::getArmorPositionFromState(const Eigen::VectorXd & _X, int _armor_id) const
{
    double armor_yaw = rm_tools::limit_rad(_X[kf::yaw] + _armor_id * 2 * CV_PI / armors_num_);
    double radius = getRadiusFromState(_X, _armor_id);

    double xa = _X[kf::x] - radius * std::cos(armor_yaw);
    double ya = _X[kf::y] - radius * std::sin(armor_yaw);
    double za = get_Z_ArmorFromState(_X, _armor_id);

    return Eigen::Vector3d(xa, ya, za);
}

void RobotTarget::normalizeState() { X_[kf::yaw] = rm_tools::limit_rad(X_[kf::yaw]); }

Eigen::VectorXd RobotTarget::z_substract(Eigen::VectorXd _Z_meas, Eigen::VectorXd _Z_pred) const
{
    Eigen::VectorXd residual = _Z_meas - _Z_pred;
    residual[0] = rm_tools::limit_rad(residual[0]); // yaw
    residual[1] = rm_tools::limit_rad(residual[1]); // pitch
    residual[3] = rm_tools::limit_rad(residual[3]); // armor_yaw
    return residual;
}

bool RobotTarget::diverged() const
{
    bool r_ok = X_[kf::r] > 0.05 && X_[kf::r] < 0.5;
    // bool l_ok = X_[kf::r] + X_[kf::dr] > 0.05 && X_[kf::r] + X_[kf::dr] < 0.5;
    bool l_ok = true;
    // bool v_yaw_ok = std::abs(X_[kf::v_yaw]) < 15;
    bool v_yaw_ok = true;
    return !(r_ok && l_ok && v_yaw_ok);
}

double RobotTarget::getRadiusFromState(const Eigen::VectorXd & _X, int _armor_id) const
{
    return (_armor_id == 1 || _armor_id == 3) ? _X[kf::r] + _X[kf::dr] : _X[kf::r];
}

double RobotTarget::get_Z_ArmorFromState(const Eigen::VectorXd & _X, int _armor_id) const
{
    if (_armor_id == 1 || _armor_id == 3) {
        return _X[kf::z] + _X[kf::dz];
    }
    return _X[kf::z];
}

} // namespace rm_auto_aim::tracker
