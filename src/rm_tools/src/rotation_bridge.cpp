#include "rm_tools/rotation_bridge.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <opencv2/calib3d.hpp>  // Rodrigues公式函数
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>  // 放在Eigen后面
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// clang-format off`

namespace rm_tools::tf
{

// class rot_bridge begin

double rot_bridge::limit_rad(double angle)
{
    while (angle > CV_PI) angle -= 2 * CV_PI;
    while (angle <= -CV_PI) angle += 2 * CV_PI;
    return angle;
}

rot_bridge::rot_bridge(const geometry_msgs::msg::Quaternion q)
{
    q_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    q_.normalize();
}

rot_bridge::rot_bridge(const tf2::Quaternion q)
{
    q_ = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    q_.normalize();
}

rot_bridge::rot_bridge(const cv::Mat & R)
{
    // 空矩阵检查
    if (R.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("rm_tools"), "输入的旋转矩阵为空！");
        q_ = Eigen::Quaterniond::Identity();
        return;
    }

    // 确保矩阵是双精度
    cv::Mat R_double;
    if (R.type() != CV_64F) {
        R.convertTo(R_double, CV_64F);
    } else {
        R_double = R;
    }

    
    if (R.rows == 3 && R.cols == 1) {
        // 旋转向量分支
        double angle = cv::norm(R);
        if (angle < 1e-8) {
            // 太小写成单位四元数，防止算轴时发生除0问题
            q_ = Eigen::Quaterniond::Identity();
            return;
        }

        // 从旋转向量提取轴
        // 使用 cv::norm 已经计算了角度，但提取数据时需要类型安全
        Eigen::Vector3d axis;
        axis << R.at<double>(0), R.at<double>(1), R.at<double>(2);
        axis /= angle;

        double half = angle / 2.0;
        double sin_half = std::sin(half);
        double cos_half = std::cos(half);

        q_ = Eigen::Quaterniond(cos_half, sin_half * axis(0), sin_half * axis(1), sin_half * axis(2));
        return;
    } else if (R.rows == 3 && R.cols == 3) {
        Eigen::Matrix3d rmat_eigen;
        cv::cv2eigen(R, rmat_eigen);
        q_ = Eigen::Quaterniond(rmat_eigen);
    } else {
        RCLCPP_ERROR(
            rclcpp::get_logger("rm_tools"), "用于转换成四元数的 R 格式不对！R既不是旋转向量也不是旋转矩阵");
        // assert(false && "用于转换成四元数的 R 格式不对！R既不是旋转向量也不是旋转矩阵");
        q_ = Eigen::Quaterniond::Identity();  // 单位四元数，表示不发生旋转
    }
}

rot_bridge::rot_bridge(const Eigen::Vector3d eulers, int axis0, int axis1, int axis2, bool extrinsic)
{
    // 验证轴索引有效性
    if (axis0 < 0 || axis0 > 2 || axis1 < 0 || axis1 > 2 || axis2 < 0 || axis2 > 2) {
        throw std::invalid_argument("Axis indices must be 0 (x), 1 (y), or 2 (z)");
    }

    // 根据内外旋决定实际旋转顺序和角度映射
    int axes[3];
    double angles[3];

    if (extrinsic) {
        // 外旋：等价于反向轴顺序的内旋（Z-Y-X 外旋 = X-Y-Z 内旋）
        axes[0] = axis2;
        angles[0] = eulers[2];
        axes[1] = axis1;
        angles[1] = eulers[1];
        axes[2] = axis0;
        angles[2] = eulers[0];
    } else {
        // 内旋：直接使用给定顺序
        axes[0] = axis0;
        angles[0] = eulers[0];
        axes[1] = axis1;
        angles[1] = eulers[1];
        axes[2] = axis2;
        angles[2] = eulers[2];
    }

    // 构造三个基本旋转（内旋顺序：先 axes[0]，再 axes[1]，最后 axes[2]）
    Eigen::AngleAxisd rot0(angles[0], getAxisVector(axes[0]));
    Eigen::AngleAxisd rot1(angles[1], getAxisVector(axes[1]));
    Eigen::AngleAxisd rot2(angles[2], getAxisVector(axes[2]));

    // 内旋复合：q = rot2 * rot1 * rot0 （先应用 rot0，最后应用 rot2）
    q_ = Eigen::Quaterniond(rot2 * rot1 * rot0);

    // 确保四元数归一化（消除浮点累积误差）
    q_.normalize();
}

geometry_msgs::msg::Quaternion rot_bridge::quaternion_ros() const noexcept
{
    geometry_msgs::msg::Quaternion q;
    q.set__w(q_.w());
    q.set__x(q_.x());
    q.set__y(q_.y());
    q.set__z(q_.z());
    return q;
}

tf2::Quaternion rot_bridge::quaternion_tf2() const noexcept
{
    return tf2::Quaternion(q_.x(), q_.y(), q_.z(), q_.w());
}

cv::Mat rot_bridge::rmat_cv() const noexcept
{
    Eigen::Matrix3d rmat_eigen(q_);
    cv::Mat rmat;
    cv::eigen2cv(rmat_eigen, rmat);
    return rmat;
}

cv::Mat rot_bridge::rvec() const noexcept
{
    Eigen::AngleAxisd angle_axis(q_);
    Eigen::Vector3d rvec_eigen = angle_axis.angle() * angle_axis.axis();

    cv::Mat rvec(3, 1, CV_64F);
    cv::eigen2cv(rvec_eigen, rvec);
    return rvec;
}

Eigen::Vector3d rot_bridge::eulers(int axis0, int axis1, int axis2, bool extrinsic)
{
    if (!extrinsic) std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper) k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d & xyzw = q_.coeffs();
    if (is_proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;
    auto n2 = a * a + b * b + c * c + d * d;
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    auto half_sum = std::atan2(b, a);
    auto half_diff = std::atan2(-d, c);

    auto eps = 1e-7;
    auto safe1 = std::abs(eulers[1]) >= eps;
    auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
    auto safe = safe1 && safe2;
    if (safe) {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    } else {
        // todo: pitch == 90degree 的特殊情况没有处理
        if (!extrinsic) {
            eulers[0] = 0;
            if (!safe1) eulers[2] = 2 * half_sum;
            if (!safe2) eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1) eulers[0] = 2 * half_sum;
            if (!safe2) eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

    if (!is_proper) {
        eulers[2] *= sign;
        eulers[1] -= CV_PI / 2;
    }

    if (!extrinsic) std::swap(eulers[0], eulers[2]);

    return eulers;
}

// class rot_bridge end

}  // namespace rm_tools::tf

namespace rm_tools
{
bool is_valid_rotation_matrix(const cv::Mat & R)
{
    if (R.empty()) return true;
    if (R.rows != 3 || R.cols != 3) return true;
    if (R.type() != CV_64F) return true;

    // 检查是否是旋转矩阵（正交且行列式≈1）
    cv::Mat R_transpose = R.t();
    cv::Mat should_be_identity = R * R_transpose;

    cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    double norm = cv::norm(should_be_identity - identity);

    if (norm > 1e-6) {
        RCLCPP_WARN(rclcpp::get_logger("rm_tools"), "警告：旋转矩阵不是正交的，误差：%lf", norm);
    }

    return norm > 0.01;  // 允许一定误差
}


}  // namespace rm_tool