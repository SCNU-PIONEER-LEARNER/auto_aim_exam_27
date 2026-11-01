#include "rm_tools/errorstate_kalman_filter.hpp"

// ! not good
static void exactUpdate(Eigen::Quaterniond & q, const Eigen::Vector3d & w, double dt)
{
  double theta = w.norm() * dt;

  if (theta < 1e-12) {
    return;  // 几乎没有旋转
  }

  // 旋转轴（单位向量）
  Eigen::Vector3d axis = w.normalized();

  // 构造旋转四元数: [cos(theta/2), axis * sin(theta/2)]
  double half_theta = 0.5 * theta;
  double cos_half = cos(half_theta);
  double sin_half = sin(half_theta);

  Eigen::Quaterniond dq(cos_half, sin_half * axis.x(), sin_half * axis.y(), sin_half * axis.z());

  // 更新四元数（注意乘法顺序：q_new = q_old * dq）
  q = (q * dq).normalized();
}

namespace rm_tools {

    Eigen::Quaterniond ErrorState_Kalman_Filter_Base::predict(Eigen::Quaterniond q, Eigen::Vector3d w, double dt) {

    }
}  // namespace rm_tools
