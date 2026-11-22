// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__CV_DETECTOR_HPP_
#define ARMOR_DETECTOR__CV_DETECTOR_HPP_



// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "rm_interfaces/msg/debug_armors.hpp"
#include "rm_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{


class cv_Detector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    // area condition
    double min_fill_ratio;
    size_t min_contour_size;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  cv_Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
    : binary_thres(bin_thres), 
      detect_color(color), 
      l(l), a(a) {};

  std::vector<Armor> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input) const;
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // For debug usage
  cv::Mat draw_result(const std::vector<Armor> armors, cv::Mat img, double latency) const;
  cv::Mat get_all_binary_img(std::vector<Armor> armors) const;
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);

  int binary_thres;
  int detect_color; // 0-RED , 1-BLUE
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

  // Debug msgs
  cv::Mat binary_img;
  rm_interfaces::msg::DebugLights debug_lights;
  rm_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light & light);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  std::tuple<ArmorType, bool> isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim


#endif  // ARMOR_DETECTOR__CV_DETECTOR_HPP_
