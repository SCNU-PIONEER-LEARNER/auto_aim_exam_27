// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.


// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"


namespace rm_auto_aim
{

ArmorName labelId2ArmorName(const int label)
{
  if (label == 0) return ArmorName::one;
  if (label == 1) return ArmorName::two;
  if (label == 2) return ArmorName::three;
  if (label == 3) return ArmorName::four;
  if (label == 4) return ArmorName::five;
  if (label == 5) return ArmorName::outpost;
  if (label == 6) return ArmorName::sentry;
  if (label == 7) return ArmorName::base;
  return ArmorName::not_armor;
}

NumberClassifier::NumberClassifier(
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)
{
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors) const
{
  enum corner { left_top = 0, right_top = 1, right_bottom = 2, left_bottom = 3 };
  // Light length in image
  constexpr int light_length = 12;
  // Image size after warp
  constexpr int warp_height = 28;
  constexpr int small_armor_width = 32;
  constexpr int large_armor_width = 54;
  // Number ROI size, fixed by the MLP input shape
  const cv::Size roi_size(20, 28);

  for (auto & armor : armors) {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {
      armor.points[left_bottom], armor.points[left_top], 
      armor.points[right_top],armor.points[right_bottom]
    };

    constexpr int top_light_y = (warp_height - light_length) / 2 - 1;
    constexpr int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::small ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // armor.debug_img_ = number_image;   // 这里出来图像全黑，源图像没有问题?

    // Get ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    // armor.pattern = number_image;
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.pattern = number_image.clone();    // 这个图像全白
  }
}

void NumberClassifier::classify(std::vector<Armor> & armors)
{
  for (auto & armor : armors) {
    cv::Mat image = armor.pattern.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    const ArmorName armor_name = labelId2ArmorName(label_id);

    armor.confidence = confidence;
    armor.refill_numberid(static_cast<int>(armor_name), confidence);
  }

  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) -> bool {
        const auto armor_name_id = static_cast<int>(armor.name);
        const auto armor_type_id = static_cast<int>(armor.type);
        if (
          armor_name_id < 0 || armor_name_id >= static_cast<int>(ARMOR_NAMES.size()) ||
          armor_type_id < 0 || armor_type_id >= static_cast<int>(ARMOR_TYPES.size())) {
          return true;
        }

        if (armor.confidence < threshold) {
          return true;
        }

        for (const auto & ignore_class : ignore_classes_) {
          if (ignore_class == ARMOR_NAMES[armor_name_id]) {
            return true;
          }
        }

        bool mismatch_armor_type = false;

        if (armor.type == ArmorType::big) {
          mismatch_armor_type =
            armor.name == ArmorName::outpost || armor.name == ArmorName::two ||
            armor.name == ArmorName::sentry || armor.name == ArmorName::base;
        } else if (armor.type == ArmorType::small) {
          mismatch_armor_type = armor.name == ArmorName::one;
        }
        return mismatch_armor_type;
      }),
    armors.end());
}

}  // namespace rm_auto_aim
