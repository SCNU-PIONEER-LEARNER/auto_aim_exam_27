#ifndef RM_TOOLS__IMAGE_UTILS_HPP_
#define RM_TOOLS__IMAGE_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_tools
{

enum class ColorDiffMode
{
  RED_MINUS_BLUE,
  BLUE_MINUS_RED,
};

inline bool isImageSizeValid(const cv::Size & image_size) noexcept
{
  return image_size.width > 0 && image_size.height > 0;
}

inline cv::Rect clampRectToImage(const cv::Rect & rect, const cv::Size & image_size)
{
  if (!isImageSizeValid(image_size)) {
    return {};
  }
  return rect & cv::Rect(0, 0, image_size.width, image_size.height);
}

inline bool isPointInsideImage(const cv::Point2f & point, const cv::Size & image_size) noexcept
{
  return isImageSizeValid(image_size) && std::isfinite(point.x) && std::isfinite(point.y) &&
         point.x >= 0.0F && point.y >= 0.0F && point.x < static_cast<float>(image_size.width) &&
         point.y < static_cast<float>(image_size.height);
}

inline bool isPointInsideImage(const cv::Point & point, const cv::Size & image_size) noexcept
{
  return isPointInsideImage(cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)),
                            image_size);
}

inline bool isRectInsideImage(const cv::Rect & rect, const cv::Size & image_size) noexcept
{
  if (!isImageSizeValid(image_size) || rect.empty() || rect.x < 0 || rect.y < 0) {
    return false;
  }
  const int64_t right = static_cast<int64_t>(rect.x) + rect.width;
  const int64_t bottom = static_cast<int64_t>(rect.y) + rect.height;
  return right <= image_size.width && bottom <= image_size.height;
}

inline ColorDiffMode colorDiffModeFromDetectColor(int detect_color) noexcept
{
  return detect_color == 0 ? ColorDiffMode::RED_MINUS_BLUE : ColorDiffMode::BLUE_MINUS_RED;
}

inline cv::Mat colorDiffBinary(
  const cv::Mat & rgb_img, ColorDiffMode mode, int threshold)
{
  std::vector<cv::Mat> channels;
  cv::split(rgb_img, channels);
  if (channels.size() < 3) {
    return cv::Mat::zeros(rgb_img.size(), CV_8UC1);
  }

  cv::Mat diff;
  if (mode == ColorDiffMode::RED_MINUS_BLUE) {
    cv::subtract(channels[0], channels[2], diff);
  } else {
    cv::subtract(channels[2], channels[0], diff);
  }

  cv::Mat binary;
  cv::threshold(diff, binary, threshold, 255, cv::THRESH_BINARY);
  return binary;
}

inline cv::Mat colorDiffBinary(const cv::Mat & rgb_img, int detect_color, int threshold)
{
  return colorDiffBinary(rgb_img, colorDiffModeFromDetectColor(detect_color), threshold);
}

}  // namespace rm_tools

#endif  // RM_TOOLS__IMAGE_UTILS_HPP_
