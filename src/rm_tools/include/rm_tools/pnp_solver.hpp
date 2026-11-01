// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef RM_TOOLS__PNP_SOLVER_HPP_
#define RM_TOOLS__PNP_SOLVER_HPP_

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>
#include <vector>
// STD
#include <array>
#include <string>
#include <vector>

#include "iostream"

namespace rm_tools
{
class PnPSolver
{
  public:
    struct YawObjectPointCorrection
    {
        double width_gain_m = 0.0;
        double height_loss_m = 0.0;
    };

    PnPSolver(
        const std::array<double, 9> & camera_matrix, const std::vector<double> & distortion_coefficients,
        cv::SolvePnPMethod method = cv::SOLVEPNP_IPPE);

    // Set an object coord system
    void setObjectPoints(
        const std::string & coord_frame_name, const std::vector<cv::Point3f> & object_points) noexcept;

    // Get 3d position of the object coord system using PnP algorithm
    template <class InputArray>
    bool solvePnPGeneric(
        const InputArray & image_points, std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs,
        const std::string & coord_frame_name)
    {
        rvecs.clear();
        tvecs.clear();
        if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
            const auto & object_points = object_points_map_[coord_frame_name];
            int solutions = cv::solvePnPGeneric(
                object_points, image_points, camera_matrix_, distortion_coefficients_, rvecs, tvecs, false,
                method_);
            return solutions > 0;
        } else {
            return false;
        }
    }

    template <class InputArray>
    bool solvePnPGenericWithYawCorrection(
        const InputArray & image_points, std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs,
        const std::string & coord_frame_name, double armor_yaw_rad)
    {
        rvecs.clear();
        tvecs.clear();
        if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
            const auto & object_points = object_points_map_[coord_frame_name];
            std::vector<cv::Point3f> modified_object_points = object_points;
            get_yaw_corrected_object_points(modified_object_points, armor_yaw_rad);
            int solutions = cv::solvePnPGeneric(
                modified_object_points, image_points, camera_matrix_, distortion_coefficients_, rvecs, tvecs,
                false, method_);
            return solutions > 0;
        } else {
            return false;
        }
    }

    static YawObjectPointCorrection calculate_yaw_object_point_correction(double armor_yaw_rad) noexcept;

    static void get_yaw_corrected_object_points(
        std::vector<cv::Point3f> & world_points, double armor_yaw_rad) noexcept;

    bool get_world_points(const std::string & coord_frame_name, std::vector<cv::Point3f> & world_points)
    {
        if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
            const auto & object_points = object_points_map_[coord_frame_name];
            world_points = object_points;
            return true;
        } else {
            return false;
        }
    }

    bool get_yaw_corrected_world_points(
        const std::string & coord_frame_name, std::vector<cv::Point3f> & world_points, double armor_yaw_rad)
    {
        if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
            const auto & object_points = object_points_map_[coord_frame_name];
            world_points = object_points;
            get_yaw_corrected_object_points(world_points, armor_yaw_rad);
            return true;
        } else {
            return false;
        }
    }
    // Get 3d position of the object coord system using PnP algorithm
    template <class InputArray>
    bool solvePnP(
        const InputArray & image_points, cv::Mat & rvec, cv::Mat & tvec, const std::string & coord_frame_name)
    {
        if (object_points_map_.find(coord_frame_name) != object_points_map_.end()) {
            const auto & object_points = object_points_map_[coord_frame_name];
            return cv::solvePnP(
                object_points, image_points, camera_matrix_, distortion_coefficients_, rvec, tvec, false,
                method_);
        } else {
            return false;
        }
    }

    // Calculate the distance between armor center and image center
    float calculateDistanceToCenter(const cv::Point2f & image_point) const noexcept;

    double calculateReprojectionError(
        const std::vector<cv::Point2f> & image_points, const cv::Mat & rvec, const cv::Mat & tvec,
        const std::string & coord_frame_name);

    double calculateYawCorrectedReprojectionError(
        const std::vector<cv::Point2f> & image_points, const cv::Mat & rvec, const cv::Mat & tvec,
        const std::string & coord_frame_name, double armor_yaw_rad);

    double calculateReprojectionError(
        const std::vector<cv::Point2f> & image_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
        const std::string & coord_frame_name);

    double calculateYawCorrectedReprojectionError(
        const std::vector<cv::Point2f> & image_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
        const std::string & coord_frame_name, double armor_yaw_rad);

    std::vector<cv::Point2f> getReprojectionPoint(
        const cv::Vec3d & rvec, const cv::Vec3d & tvec, const std::string & coord_frame_name);

    std::vector<cv::Point2f> getYawCorrectedReprojectionPoint(
        const cv::Vec3d & rvec, const cv::Vec3d & tvec, const std::string & coord_frame_name,
        double armor_yaw_rad);

    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;

  private:
    using YawShapeSamples = std::array<double, 5>;

    static constexpr YawShapeSamples YAW_DEG_SAMPLES = {0.0, 20.0, 40.0, 60.0, 80.0};
    static constexpr YawShapeSamples RAW_WIDTH_SHAPE_MM = {0.0, 0.659, 4.702, 9.902, 9.902};
    static constexpr YawShapeSamples RAW_HEIGHT_SHAPE_MM = {0.0, 0.142, 3.958, 6.065, 6.065};
    static constexpr double RAW_WIDTH_SHAPE_MAX_MM = 9.902;
    static constexpr double RAW_HEIGHT_SHAPE_MAX_MM = 6.065;
    static constexpr double MAX_WIDTH_GAIN_M = 1.56 / 1000.0;
    static constexpr double MAX_HEIGHT_LOSS_M = 7.24 / 1000.0;

    static double interpolateYawShape(
        double abs_yaw_deg, const YawShapeSamples & yaw_samples,
        const YawShapeSamples & shape_samples) noexcept;

    std::unordered_map<std::string, std::vector<cv::Point3f>> object_points_map_;
    cv::SolvePnPMethod method_;
};

}  // namespace rm_tools

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
