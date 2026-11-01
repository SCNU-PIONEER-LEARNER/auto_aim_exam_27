#include "rm_tools/image_utils.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

TEST(ImageUtils, ClampsRectToImageBounds)
{
  const cv::Rect clipped = rm_tools::clampRectToImage(cv::Rect{-5, 10, 20, 30}, cv::Size{100, 80});

  EXPECT_EQ(clipped, cv::Rect(0, 10, 15, 30));
}

TEST(ImageUtils, ReturnsEmptyRectForInvalidImageBounds)
{
  EXPECT_TRUE(rm_tools::clampRectToImage(cv::Rect{0, 0, 10, 10}, cv::Size{}).empty());
  EXPECT_TRUE(rm_tools::clampRectToImage(cv::Rect{120, 0, 10, 10}, cv::Size{100, 80}).empty());
}

TEST(ImageUtils, ChecksPointAndRectInsideImage)
{
  const cv::Size image_size{100, 80};

  EXPECT_TRUE(rm_tools::isPointInsideImage(cv::Point2f{99.0F, 79.0F}, image_size));
  EXPECT_FALSE(rm_tools::isPointInsideImage(cv::Point2f{100.0F, 40.0F}, image_size));
  EXPECT_FALSE(rm_tools::isPointInsideImage(cv::Point2f{-1.0F, 40.0F}, image_size));

  EXPECT_TRUE(rm_tools::isRectInsideImage(cv::Rect{0, 0, 100, 80}, image_size));
  EXPECT_FALSE(rm_tools::isRectInsideImage(cv::Rect{-1, 0, 100, 80}, image_size));
  EXPECT_FALSE(rm_tools::isRectInsideImage(cv::Rect{0, 0, 101, 80}, image_size));
}

TEST(ImageUtils, BuildsColorDifferenceBinaryMasks)
{
  cv::Mat rgb_image(1, 2, CV_8UC3);
  rgb_image.at<cv::Vec3b>(0, 0) = cv::Vec3b(220, 0, 20);
  rgb_image.at<cv::Vec3b>(0, 1) = cv::Vec3b(20, 0, 220);

  const cv::Mat red_binary = rm_tools::colorDiffBinary(
    rgb_image, rm_tools::ColorDiffMode::RED_MINUS_BLUE, 100);
  const cv::Mat blue_binary = rm_tools::colorDiffBinary(
    rgb_image, rm_tools::ColorDiffMode::BLUE_MINUS_RED, 100);

  EXPECT_EQ(red_binary.at<uchar>(0, 0), 255);
  EXPECT_EQ(red_binary.at<uchar>(0, 1), 0);
  EXPECT_EQ(blue_binary.at<uchar>(0, 0), 0);
  EXPECT_EQ(blue_binary.at<uchar>(0, 1), 255);
}

TEST(ImageUtils, BuildsZeroMaskForNonRgbInput)
{
  const cv::Mat gray_image = cv::Mat::ones(3, 4, CV_8UC1) * 255;

  const cv::Mat binary = rm_tools::colorDiffBinary(
    gray_image, rm_tools::ColorDiffMode::RED_MINUS_BLUE, 100);

  EXPECT_EQ(binary.size(), gray_image.size());
  EXPECT_EQ(binary.type(), CV_8UC1);
  EXPECT_EQ(cv::countNonZero(binary), 0);
}
