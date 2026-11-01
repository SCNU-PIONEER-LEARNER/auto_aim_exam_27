#ifndef ERRORSTATE_KALMAN_FILTER_HPP
#define ERRORSTATE_KALMAN_FILTER_HPP


// Eigen
#include <Eigen/Geometry>

struct full_state {
    Eigen::Vector3d p;        // posision
    Eigen::Vector3d v;        // velocity
    Eigen::Quaterniond q;
    Eigen::Vector3d w;        // angle velocity
};

namespace rm_tools {

class ErrorState_Kalman_Filter_Base {
    public:
    protected:
    
    /**
     * @brief 预测四元数
     * @param q 上一刻位姿
     * @param w 上一刻角速度向量，规定使用世界坐标系，模长为速度大小
     * @param dt Δt
     * @return 本刻预测位姿
     */
    Eigen::Quaterniond predict(Eigen::Quaterniond q, Eigen::Vector3d w, double dt);
    
    private:
};

class ErrorState_Kalman_Filter : public ErrorState_Kalman_Filter_Base {
    public:
    protected:
    
    private:
    
};

} // namespace rm_tools
    
    #endif