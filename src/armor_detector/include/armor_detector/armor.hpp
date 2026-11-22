#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace rm_auto_aim
{

// Armor size, Unit: m
constexpr double SMALL_ARMOR_WIDTH = 132.0 / 1000.0;
constexpr double SMALL_ARMOR_HEIGHT = 57.0 / 1000.0;  // 55
constexpr double LARGE_ARMOR_WIDTH = 223.0 / 1000.0;
constexpr double LARGE_ARMOR_HEIGHT = 57.0 / 1000.0;  // 55

// 15 degree in rad
constexpr double FIFTTEN_DEGREE_RAD = 15 * CV_PI / 180;

enum Color { red, blue, extinguish, purple };
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType { big, small };
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority { first = 1, second, third, forth, fifth };

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

struct Lightbar
{
  //std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  std::vector<cv::Point> contour;
  cv::RotatedRect rotated_rect;
  Lightbar(const cv::RotatedRect & rotated_rect, const std::vector<cv::Point> & contour);
  Lightbar(){};
};

struct Armor
{
  Color color;
  Lightbar left, right;  //used to be const
  double lightbar_ratio;
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;  // 归一化坐标
  double confidence;
  cv::Rect box;
  std::vector<cv::Point2f> points;      //最终角点
  std::vector<cv::Point2f> net_points;  //神经网络角点

  double ratio;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio;         // 左边灯条长度与右边灯条长度之比
  double rectangular_error;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type;
  ArmorName name;
  ArmorPriority priority;
  int class_id;
  std::string rfs;  // result for show
  cv::Mat pattern;

  bool duplicated;

  double yaw_raw;  // rad

  Eigen::Vector4d pose;
  std::vector<Eigen::Vector4d> P_world;

  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);

  /**
   * @brief 传统视觉用的构造函数
   */
  Armor(int color_id, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);

  // 传统视觉后填数字接口用
  void refill_numberid(int num_id, float confidence);

  // Build the points in the object coordinate system, start from top left in
  // clockwise order
  template <typename PointType>
  static inline std::vector<PointType> buildObjectPoints(
    const double & w, const double & h) noexcept
  {
    return {
      PointType(0, w / 2, h / 2), PointType(0, -w / 2, h / 2), PointType(0, -w / 2, -h / 2),
      PointType(0, w / 2, -h / 2)};
  }
};

struct Light : public cv::Rect
{
  Light() = default;
  explicit Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
  : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle)
  {
    length = cv::norm(top - bottom);
    width = area / length;
    center = (top + bottom) / 2;
  }

  int color;
  cv::Point2f top, bottom;
  cv::Point2f center;
  double length;
  double width;
  float tilt_angle;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
