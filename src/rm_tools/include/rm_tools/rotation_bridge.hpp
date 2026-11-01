#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>  // 放在Eigen后面

#ifndef CV_PI
#define CV_PI 3.1415926535897932384626433832795
#endif

// using namespace rm_tools::tf; 是安全的
// 当然更推荐 using rm_tools::tf::rot_bridge;


// clang-format off

namespace rm_tools::tf
{

/**
 * @brief 为了转换各种旋转量诞生的旋转转换类
 *  
 * 这个类支持用各种不同类型的旋转量构造，并支持不同输出类型，包括Eigen、OpenCV、ROS2、tf四种不同库的旋转  
 * 示例：
 * 
 * ```cpp
 * using rm_tools::tf::rot_bridge; // 这个命名空间是有点长，但这样写是安全的
 * cv::Mat q = rot_bridge(tf_msg.transform.rotation).rmat_cv(); // 一行搞定
 * ```
 * 
 * @note 支持输出的类型：
 * 
 * - 四元数(对应方法：quaternion)
 * 
 * - 旋转矩阵(对应方法：rmat)
 * 
 * - 欧拉角(对应方法：eulers)
 */
class rot_bridge {
    public:
    // ----------------------构造函数--------------
    //! 添加构造函数时务必确保得到的四元数必须是归一化的

        /**
         * @brief 支持很多种构造方法，比如四元数，旋转矩阵，旋转向量，欧拉角等
         */
        rot_bridge(const Eigen::Quaterniond q) { q_ = q; q_.normalize();};
        rot_bridge(const geometry_msgs::msg::Quaternion q);
        rot_bridge(const tf2::Quaternion q);
        rot_bridge(const Eigen::Matrix3d R) { q_ = Eigen::Quaterniond(R); }

        /**
         * @brief 支持很多种构造方法，比如四元数，旋转矩阵，旋转向量，欧拉角等
         * 
         * @param R 同时支持旋转向量与旋转矩阵作为输入
         */
        rot_bridge(const cv::Mat &R);

        /**
         * @brief 支持很多种构造方法，比如四元数，旋转矩阵，旋转向量，欧拉角等
         * 
         * @param eulers  欧拉角（以3维向量形式）
         * @param axis  x=0, y=1, z=2
         * @param extrinsic  是否启用外旋组合
         */
        rot_bridge(const Eigen::Vector3d eulers, int axis0, int axis1, int axis2, bool extrinsic = false);

        // ------------------输出结果-------------------------
        
        /**
         * @brief 输出对应四元数
         * @return 输出四元数类型：函数后不加后缀为Eigen，加_ros为ros消息类型，加_tf2为tf2类型
         */
        Eigen::Quaterniond quaternion() const noexcept { return q_; };

        /**
         * @brief 输出对应四元数
         * @return 输出四元数类型：函数后不加后缀为Eigen，加_ros为ros消息类型，加_tf2为tf2类型
         */
        geometry_msgs::msg::Quaternion quaternion_ros() const noexcept;

        /**
         * @brief 输出对应四元数
         * @return 输出四元数类型：函数后不加后缀为Eigen，加_ros为ros消息类型，加_tf2为tf2类型
         */
        tf2::Quaternion quaternion_tf2() const noexcept;

        /**
         * @brief 输出旋转矩阵
         * @return 不加后缀输出Eigen矩阵，加_cv后缀输出cv的矩阵，目前不支持tf的3x3矩阵
         */
        cv::Mat rmat_cv() const noexcept;

        /**
         * @brief 输出旋转矩阵
         * @return 不加后缀输出Eigen矩阵，加_cv后缀输出cv的矩阵，目前不支持tf的3x3矩阵
         */
        Eigen::Matrix3d rmat() const noexcept { return Eigen::Matrix3d(q_); }
        
        /**
         * @brief 输出旋转向量，opencv的
         */
        cv::Mat rvec() const noexcept;

        /**
         * @brief 输出欧拉角
         * @param axis  x=0, y=1, z=2
         * @param extrinsic  是否启用外旋组合
         * 
         * 参考：rm_tools/math.hpp
         */
        Eigen::Vector3d eulers(int axis0, int axis1, int axis2, bool extrinsic = false);

        /**
         * @brief 输出欧拉角(角度版)
         * @param axis  x=0, y=1, z=2
         * @param extrinsic  是否启用外旋组合
         * 
         * 参考：rm_tools/math.hpp
         */
        Eigen::Vector3d eulers_degree(int axis0, int axis1, int axis2, bool extrinsic = false)
            { return (this->eulers(axis0, axis1, axis2, extrinsic) * (180.0 / CV_PI)); };
        
    // 工具函数
        void inv() { q_.conjugate(); };
        double limit_rad(double angle);

        
    private:
        Eigen::Quaterniond q_;
        
        Eigen::Vector3d getAxisVector(int axis)
        {
          switch (axis) {
            case 0:
              return Eigen::Vector3d::UnitX();  // x-axis
            case 1:
              return Eigen::Vector3d::UnitY();  // y-axis
            case 2:
              return Eigen::Vector3d::UnitZ();  // z-axis
            default:
              throw std::invalid_argument("Invalid axis index: must be 0 (x), 1 (y), or 2 (z)");
          }
        }
};


}  // namespace rm_tools

namespace rm_tools
{
    bool is_valid_rotation_matrix(const cv::Mat & R);
} // namespace rm_tools

