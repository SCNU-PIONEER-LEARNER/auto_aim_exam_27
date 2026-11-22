#ifndef ARMOR_DETECTOR_YOLO_HPP
#define ARMOR_DETECTOR_YOLO_HPP


#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace rm_auto_aim
{
class YOLOBase
{
public:
  virtual std::vector<Armor> detect(const cv::Mat & img) = 0;

  virtual std::vector<Armor> postprocess(
    double scale, const cv::Mat & output, const cv::Mat & bgr_img) = 0;
};

class YOLO
{
public:
  YOLO(
    const std::string & model_path, double conf_threshold, const std::string & device,
    int inference_mode);

  std::vector<Armor> detect(const cv::Mat & img);

  std::vector<Armor> postprocess(double scale, const cv::Mat & output, const cv::Mat & bgr_img);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace rm_auto_aim


#endif  // AUTO_AIM__YOLO_HPP