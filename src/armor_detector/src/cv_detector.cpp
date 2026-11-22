// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/cv_detector.hpp"
#include "rm_interfaces/msg/debug_armor.hpp"
#include "rm_interfaces/msg/debug_light.hpp"

#include <rclcpp/logging.hpp>


namespace rm_auto_aim
{

std::vector<Armor> cv_Detector::detect(const cv::Mat & input)
{
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  armors_ = matchLights(lights_);

  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

cv::Mat cv_Detector::preprocessImage(const cv::Mat & rgb_img) const
{
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

std::vector<Light> cv_Detector::findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < l.min_contour_size) continue;

    auto b_rect = cv::boundingRect(contour);
    auto r_rect = cv::minAreaRect(contour);
    cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
    std::vector<cv::Point> mask_contour;
    for (const auto & p : contour) {
      mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
    }
    cv::fillPoly(mask, {mask_contour}, 255);
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    // points / rotated rect area
    bool is_fill_rotated_rect =
      points.size() / (r_rect.size.width * r_rect.size.height) > l.min_fill_ratio;
    cv::Vec4f return_param;
    cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2f top, bottom;
    double angle_k;
    if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
      top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
      bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
      angle_k = 0;
    } else {
      auto k = return_param[1] / return_param[0];
      auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
      top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
      bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
      angle_k = std::atan(k) / CV_PI * 180 - 90;
      if (angle_k > 90) {
        angle_k = 180 - angle_k;
      }
    }
    auto light = Light(b_rect, top, bottom, points.size(), angle_k);

    if (isLight(light) && is_fill_rotated_rect) {
      auto rect = light;
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rgb_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? Color::red : Color::blue;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

// 轻度魔改版
std::vector<Armor> cv_Detector::matchLights(const std::vector<Light> & lights)
{
  using std::vector;
  vector<Light> lights_sorted(lights);
  vector<Armor> armors;

  this->debug_armors.data.clear();

  std::sort(
    lights_sorted.begin(), lights_sorted.end(),
    [](const Light & a, const Light & b) -> bool {
       return a.center.x < b.center.x; 
    });

  // Loop all the pairing of lights
  for (auto light_1 = lights_sorted.begin(); light_1 != lights_sorted.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights_sorted.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      if (containLight(*light_1, *light_2, lights_sorted)) {
        continue;
      }

      auto [type, valid] = isArmor(*light_1, *light_2);

      // 装甲板不合法就跳过
      if (!valid) continue;

      vector<cv::Point2f> armor_key_points{
        (*light_1).top, (*light_2).top, (*light_2).bottom, (*light_1).bottom};
      float min_x = armor_key_points[0].x;
      float max_x = armor_key_points[0].x;
      float min_y = armor_key_points[0].y;
      float max_y = armor_key_points[0].y;
      for (size_t i = 1; i < armor_key_points.size(); i++) {
        if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
        if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
        if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
        if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
      }
      cv::Rect box(min_x, min_y, max_x - min_x, max_y - min_y);

      auto armor = Armor(detect_color, box, armor_key_points);
      armor.type = type;
      armors.emplace_back(armor);
    }
  }
  return armors;
}

bool cv_Detector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  rm_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool cv_Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

std::tuple<ArmorType,bool> cv_Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type = ArmorType::small;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::big : ArmorType::small;
  } else {
    type = ArmorType::small;
    RCLCPP_ERROR(rclcpp::get_logger("armor_detector"),"识别出不合法的装甲板！！ok: %d%d%d", 
    light_ratio_ok ? 1:0, center_distance_ok ? 1 : 0, angle_ok ? 1 : 0);
  }

  // * 被优化掉了
  // Fill in debug information
  rm_interfaces::msg::DebugArmor armor_data;
  armor_data.type = 1 == static_cast<int>(type) ? "big" : "small";
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return {type, is_armor};
}


// *************************** 以下为工具函数 ***************************


cv::Mat cv_Detector::draw_result(const std::vector<Armor> armors, cv::Mat img, double latency) const
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == Color::red ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw Armors
  for (const auto & armor : armors) {
    cv::line(img, armor.points[0], armor.points[2], cv::Scalar(0, 255, 0), 1);
    cv::line(img, armor.points[1], armor.points[3], cv::Scalar(0, 255, 0), 1);
    cv::putText(
      img, armor.rfs, armor.points[0], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    }
  for (const auto & armor : armors)
    for (const auto & pt : armor.net_points) {
      cv::circle(img, pt, 1, cv::Scalar(125, 125, 125), -1);
  }
  std::stringstream latency_ss;
  latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
  auto latency_s = latency_ss.str();
  cv::putText(
    img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
  return img;
}

cv::Mat cv_Detector::get_all_binary_img(std::vector<Armor> armors) const
{
  if (armors.empty()) {
    cv::Mat black_image = cv::Mat::zeros(cv::Size(250, 60), CV_8UC1);
    cv::putText(
      black_image, "NO armor", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255), 2,
      8);
    return black_image;
  } else {
    int max_width = 0;
    int max_height = 0;
    std::vector<cv::Mat> binary_imgs;
    std::vector<cv::Mat> padd_imgs;
    binary_imgs.reserve(armors.size());
    padd_imgs.reserve(armors.size());
    for (auto & armor : armors) {
      if (!armor.pattern.empty()) binary_imgs.emplace_back(armor.pattern);
    }
    for (const auto & img : binary_imgs) {
      max_width = std::max(max_width, img.cols);
      max_height = std::max(max_height, img.rows);
    }
    if (max_width == 0 || binary_imgs.size() == 0 || max_height == 0)
      return cv::Mat(cv::Size(20, 28), CV_8UC1);
    for (auto & img : binary_imgs) {
      cv::Mat padded_img(max_height, max_width, img.type(), cv::Scalar(0, 0, 0));
      auto x_offset = (max_width - img.cols) / 2;
      auto y_offset = (max_height - img.rows) / 2;
      img.copyTo(padded_img(cv::Rect(x_offset, y_offset, img.cols, img.rows)));

      img = padded_img;
      padd_imgs.emplace_back(img);
    }
    cv::Mat all_binary_img;
    cv::vconcat(padd_imgs, all_binary_img);
    return all_binary_img;
  }
}

} // namespace rm_auto_aim

