#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_


#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <tuple>

#include "armor_detector/armor.hpp"
#include "rm_tools/pnp_solver.hpp"

namespace rm_auto_aim
{

class Detector
{
public:
  Detector();

  bool traditional_correct(Armor & armor, const cv::Mat & img);
  cv::Mat draw_result(const std::vector<Armor> armors, cv::Mat img, double latency);
  cv::Mat get_all_binary_img(std::vector<Armor> armors);
  bool check_geometry(const Lightbar & lightbar) const;
  void pca_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;
  void zy_corrector(Lightbar & lightbar) const;

  cv::Mat result_img;
  int binary_thres;
  bool use_traditional_;
  double max_angle_error_;
  double min_lightbar_ratio_, max_lightbar_ratio_;
  double min_l2l_ratio_;
  double min_lightbar_length_;
  double min_armor_ratio_, max_armor_ratio_;
  double max_side_ratio_;
  double max_rectangular_error_;
  bool use_pca;
  double tolerate;
};

}  // namespace rm_auto_aim


#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
