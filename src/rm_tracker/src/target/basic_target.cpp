#include "rm_tracker/target/basic_target.hpp"

namespace rm_auto_aim::tracker {

void BasicTarget::predict(BasicTarget &_target, rclcpp::Time _now) {
  assert(_target.isInitialized() &&
         "Target must be initialized before prediction.");

  double dt = (_now - this->last_predict_time_).seconds();

  Eigen::MatrixXd F = _target.F(dt);
  Eigen::MatrixXd Q = _target.Q(dt);

  // 引用绑定
  Eigen::MatrixXd &P = _target.P_;

  P = F * P * F.transpose() + Q;
  _target.normalizeState();

  this->last_predict_time_ = _now;
  _target.predicted_ = true;
}

void BasicTarget::update(BasicTarget &_target, Eigen::VectorXd _Z_meas,
                                  int _armor_id) {
  // 引用绑定
  Eigen::VectorXd &X = _target.X_;
  Eigen::MatrixXd &P = _target.P_;
  const Eigen::MatrixXd &I = _target.I_; // 单位矩阵

  Eigen::VectorXd X_prior = X;
  Eigen::MatrixXd H = _target.H_jacobian(_armor_id);
  Eigen::MatrixXd R = _target.R(_Z_meas);

  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  X = X_prior + K * _target.z_substract(_Z_meas, _target.h(X_prior, _armor_id));
  _target.normalizeState();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  _target.predicted_ = false;
}

} // namespace rm_auto_aim::tracker
