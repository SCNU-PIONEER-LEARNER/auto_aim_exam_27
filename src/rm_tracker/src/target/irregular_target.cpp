#include <cmath>
#include <type_traits>

#include "rm_tools/math.hpp"
#include "rm_tools/rotation_bridge.hpp"
#include "rm_tracker/target/irregular_target.hpp"

namespace rm_auto_aim::tracker
{

static_assert(
    !std::is_abstract_v<IrregularTarget>,
    "IrregularTarget is abstract, please implement all pure virtual functions.");

void IrregularTarget::initTarget(const Eigen::VectorXd & _Z)
{
    const double & xa = _Z[0];
    const double & ya = _Z[1];
    const double & za = _Z[2];
    const double armor_yaw = rm_tools::limit_rad(_Z[3]);

    const double radius = 0.2;
    // 求中心点
    double xc = xa + radius * std::cos(armor_yaw);
    double yc = ya + radius * std::sin(armor_yaw);

    Eigen::VectorXd P0_dig(15);
    Eigen::VectorXd Xstate(15);

    // clang-format off
    //        xc  vx yc  vy zc  vz    yaw   v_yaw    r    dr1     dr2    dr3   dz1 dz2 dz3
    Xstate << xc, 0, yc, 0, za, 0, armor_yaw, 0,  radius,  0,      0,    0,    0,  0,  0;
    P0_dig << 1, 64, 1, 64, 1, 64,    0.4,   100,  1e-3,  1e-3,  1e-3,  1e-3,  1e-3,  1e-3,  1e-3;
    // clang-format on

    X_ = Xstate;
    P_ = P0_dig.asDiagonal();

    initialized_ = true;
    normalizeState();
}

Eigen::MatrixXd IrregularTarget::F(double _dt) const
{
    // clang-format off
    return Eigen::MatrixXd {
    /*    x   vx    y   vy    z   vz  yaw v_yaw   r dr1 dr2 dr3 dz1 dz2 dz3 */
        { 1, _dt,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0}, // x
        { 0,   1,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0}, // vx
        { 0,   0,   1, _dt,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0}, // y
        { 0,   0,   0,   1,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  0}, // vy
        { 0,   0,   0,   0,   1, _dt,   0,   0,   0,  0,  0,  0,  0,  0,  0}, // z
        { 0,   0,   0,   0,   0,   1,   0,   0,   0,  0,  0,  0,  0,  0,  0}, // vz
        { 0,   0,   0,   0,   0,   0,   1, _dt,   0,  0,  0,  0,  0,  0,  0}, // yaw
        { 0,   0,   0,   0,   0,   0,   0,   1,   0,  0,  0,  0,  0,  0,  0}, // v_yaw
        { 0,   0,   0,   0,   0,   0,   0,   0,   1,  0,  0,  0,  0,  0,  0}, // r
        { 0,   0,   0,   0,   0,   0,   0,   0,   0,  1,  0,  0,  0,  0,  0}, // dr1
        { 0,   0,   0,   0,   0,   0,   0,   0,   0,  0,  1,  0,  0,  0,  0}, // dr2
        { 0,   0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  1,  0,  0,  0}, // dr3
        { 0,   0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  1,  0,  0}, // dz1
        { 0,   0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0,  1,  0}, // dz2
        { 0,   0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0,  0,  1}  // dz3
    };
    // clang-format on
}

Eigen::MatrixXd IrregularTarget::Q(double _dt) const
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
    /*      x    vx     y    vy     z    vz   yaw  v_yaw   r dr1 dr2 dr3 dz1 dz2 dz3 */
        { a*sx, b*sx,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // x
        { b*sx, c*sx,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // vx
        {    0,    0, a*sx, b*sx,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // y
        {    0,    0, b*sx, c*sx,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // vy
        {    0,    0,    0,    0, a*sz, b*sz,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // z
        {    0,    0,    0,    0, b*sz, c*sz,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // vz
        {    0,    0,    0,    0,    0,    0, a*sy, b*sy,   0,  0,  0,  0,  0,  0,  0}, // yaw
        {    0,    0,    0,    0,    0,    0, b*sy, c*sy,   0,  0,  0,  0,  0,  0,  0}, // v_yaw
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // r
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // dr1
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // dr2
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // dr3
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // dz1
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}, // dz2
        {    0,    0,    0,    0,    0,    0,    0,    0,   0,  0,  0,  0,  0,  0,  0}  // dz3
    };
    // clang-format on
}

Eigen::MatrixXd IrregularTarget::H_jacobian(int _armor_id) const
{
    const int id = _armor_id; // _armor_id表示当前yaw落在四象限哪个范围之间，也就是第几个装甲板

    double armor_yaw = rm_tools::limit_rad(X_[kf::yaw] + id * 2 * CV_PI / armors_num_);
    double sin_armor_yaw = std::sin(armor_yaw);
    double cos_armor_yaw = std::cos(armor_yaw);
    double radius = getRadiusFromState(id);

    double dx_da = radius * sin_armor_yaw;
    double dy_da = -radius * cos_armor_yaw;
    double dx_dr = -cos_armor_yaw;
    double dy_dr = -sin_armor_yaw;

    // clang-format off
    Eigen::MatrixXd H_armor_xyza{
    /*   x  vx  y  vy  z  vz    yaw  v_yaw      r dr1 dr2 dr3 dz1 dz2 dz3 */
        {1,  0, 0,  0, 0,  0, dx_da,     0, dx_dr,  0,  0,  0,  0,  0,  0}, // yaw
        {0,  0, 1,  0, 0,  0, dy_da,     0, dy_dr,  0,  0,  0,  0,  0,  0}, // pitch
        {0,  0, 0,  0, 1,  0,     0,     0,     0,  0,  0,  0,  0,  0,  0}, // distance
        {0,  0, 0,  0, 0,  0,     1,     0,     0,  0,  0,  0,  0,  0,  0}  // armor_yaw
    };
    // clang-format on

    if (id > 0) {
        H_armor_xyza(0, kf::dr1 - 1 + id) = dx_dr;
        H_armor_xyza(1, kf::dr1 - 1 + id) = dy_dr;
        H_armor_xyza(2, kf::dz1 - 1 + id) = 1.0;
    }

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

Eigen::MatrixXd IrregularTarget::R(const Eigen::VectorXd & _Z) const
{
    double distance = _Z[2];
    double center_yaw = _Z[0];
    double armor_yaw = _Z[3];
    double delta_angle = rm_tools::limit_rad(armor_yaw - center_yaw);

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

Eigen::VectorXd IrregularTarget::h(const Eigen::VectorXd & _X, int _armor_id) const
{
    Eigen::Vector3d xyz = getArmorPositionFromState(_X, _armor_id);
    Eigen::Vector3d ypd = rm_tools::xyz2ypd(xyz);
    double angle = rm_tools::limit_rad(_X[kf::yaw] + _armor_id * 2 * CV_PI / armors_num_);
    return Eigen::Vector4d(ypd[0], ypd[1], ypd[2], angle);
}

TargetMsg IrregularTarget::getTargetMsg() const
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

    msg.dr1 = X_[kf::dr1];
    msg.dr2 = X_[kf::dr2];
    msg.dr3 = X_[kf::dr3];
    msg.dz1 = X_[kf::dz1];
    msg.dz2 = X_[kf::dz2];
    msg.dz3 = X_[kf::dz3];

    return msg;
}

Eigen::Vector3d IrregularTarget::getArmorPositionFromState(const Eigen::VectorXd & _X, int _armor_id) const
{
    double armor_yaw = rm_tools::limit_rad(_X[kf::yaw] + _armor_id * 2 * CV_PI / armors_num_);
    double radius = getRadiusFromState(_X, _armor_id);

    double xa = _X[kf::x] - radius * std::cos(armor_yaw);
    double ya = _X[kf::y] - radius * std::sin(armor_yaw);
    double za = get_Z_ArmorFromState(_X, _armor_id);

    return Eigen::Vector3d(xa, ya, za);
}

void IrregularTarget::normalizeState() { X_[kf::yaw] = rm_tools::limit_rad(X_[kf::yaw]); }

Eigen::VectorXd IrregularTarget::z_substract(Eigen::VectorXd _Z_meas, Eigen::VectorXd _Z_pred) const
{
    Eigen::VectorXd residual = _Z_meas - _Z_pred;
    residual[0] = rm_tools::limit_rad(residual[0]);
    residual[1] = rm_tools::limit_rad(residual[1]);
    residual[3] = rm_tools::limit_rad(residual[3]);
    return residual;
}

bool IrregularTarget::diverged() const
{
    bool radius_ok = true;
    for (int id = 0; id < static_cast<int>(armors_num_); ++id) {
        double armor_radius = getRadiusFromState(id);
        radius_ok = radius_ok && armor_radius > 0.05 && armor_radius < 0.5;
    }
    bool v_yaw_ok = std::abs(X_[kf::v_yaw]) < 15;
    return !(radius_ok && v_yaw_ok);
}

double IrregularTarget::getRadiusFromState(const Eigen::VectorXd & _X, int _armor_id) const
{
    return _armor_id > 0 ? _X[kf::r] + _X[kf::dr1 - 1 + _armor_id] : _X[kf::r];
}

double IrregularTarget::get_Z_ArmorFromState(const Eigen::VectorXd & _X, int _armor_id) const
{
    return _armor_id > 0 ? _X[kf::z] + _X[kf::dz1 - 1 + _armor_id] : _X[kf::z];
}

} // namespace rm_auto_aim::tracker
