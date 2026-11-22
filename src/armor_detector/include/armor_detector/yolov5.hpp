#ifndef ARMOR_CETECTOR_YOLOV5_HPP
#define ARMOR_CETECTOR_YOLOV5_HPP



#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "armor.hpp"
#include "detector.hpp"
#include "rm_tools/thread_safe_queue.hpp"
#include "yolo.hpp"

namespace rm_auto_aim
{
class YOLOV5 : public YOLOBase
{
public:
  YOLOV5(
    const std::string & model_path, double conf_threshold, const std::string & device,
    int inference_mode);

  std::vector<Armor> detect(const cv::Mat & bgr_img) override;

  std::vector<Armor> postprocess(
    double scale, const cv::Mat & output, const cv::Mat & bgr_img) override;

  void push(cv::Mat img, int64_t timestamp_nanosec);
  std::tuple<std::vector<Armor>, int64_t> pop();
  std::tuple<cv::Mat, std::vector<Armor>, int64_t> debug_pop();
  std::optional<std::tuple<cv::Mat, std::vector<Armor>, int64_t>> try_debug_pop();

  rm_tools::ThreadSafeQueue<std::tuple<cv::Mat, int64_t, ov::InferRequest>> queue_{
    16, [] { }};

private:
  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_traditional_;
  bool use_roi_ = false;
  const int class_num_ = 13;
  const float nms_threshold_ = 0.3;
  float score_threshold_ = 0.7;
  double min_confidence_ = 0.5;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::mutex mtx_;
  cv::Rect roi_;
  cv::Point2f offset_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  std::vector<Armor> parse(double scale, const cv::Mat & output, const cv::Mat & bgr_img);

  double sigmoid(double x);
};

}  // namespace rm_auto_aim


#endif  //AUTO_AIM__YOLOV5_HPP
