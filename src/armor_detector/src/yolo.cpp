
#include "armor_detector/yolo.hpp"

#include "armor_detector/yolov5.hpp"

namespace rm_auto_aim
{

YOLO::YOLO(
  const std::string & model_path, double conf_threshold, const std::string & device,
  int inference_mode)
{
  yolo_ = std::make_unique<YOLOV5>(model_path, conf_threshold, device, inference_mode);
}
std::vector<Armor> YOLO::detect(const cv::Mat & img) { return yolo_->detect(img); }

std::vector<Armor> YOLO::postprocess(double scale, const cv::Mat & output, const cv::Mat & bgr_img)
{
  return yolo_->postprocess(scale, output, bgr_img);
}

}  // namespace rm_auto_aim
