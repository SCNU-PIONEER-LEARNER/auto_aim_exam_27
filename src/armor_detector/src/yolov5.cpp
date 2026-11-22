
#include "armor_detector/yolov5.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rm_auto_aim
{
YOLOV5::YOLOV5(
  const std::string & model_path, double conf_threshold, const std::string & device,
  int inference_mode)
{
  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})  // TODO
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::RGB);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::BGR)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    .scale(255.0);
  device_ = device;
  model = ppp.build();
  if (inference_mode == 0) {
    compiled_model_ = core_.compile_model(
      model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  } else {
    compiled_model_ = core_.compile_model(
      model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));
  }
  score_threshold_ = conf_threshold;
  RCLCPP_INFO(rclcpp::get_logger("armor_detector"), "yolo initialized !");
}

std::vector<Armor> YOLOV5::detect(const cv::Mat & raw_img)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto total_start = std::chrono::high_resolution_clock::now();
  auto preprocess_start = total_start;
  auto inference_start = total_start;
  auto postprocess_start = total_start;

  if (raw_img.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "Empty img!, camera drop!");
    return std::vector<Armor>();
  }

  cv::Mat bgr_img;

  // 预处理阶段计时开始
  preprocess_start = std::chrono::high_resolution_clock::now();

  if (use_roi_) {
    if (roi_.width == -1) {
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preprocess
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  // 推理阶段计时开始
  inference_start = std::chrono::high_resolution_clock::now();

  // infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // 后处理阶段计时开始
  postprocess_start = std::chrono::high_resolution_clock::now();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto result = parse(scale, output, raw_img);

  // 计算各阶段时间
  auto total_end = std::chrono::high_resolution_clock::now();

  auto preprocess_time =
    std::chrono::duration_cast<std::chrono::microseconds>(inference_start - preprocess_start);
  auto inference_time =
    std::chrono::duration_cast<std::chrono::microseconds>(postprocess_start - inference_start);
  auto postprocess_time =
    std::chrono::duration_cast<std::chrono::microseconds>(total_end - postprocess_start);
  auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start);

  RCLCPP_DEBUG(
    rclcpp::get_logger("armor_detector"),
    " Preprocess: %.2fms, Inference: %.2fms, Postprocess: %.2fms, Total: %.2fms, Armors: %zu",
    preprocess_time.count() / 1000.0, inference_time.count() / 1000.0,
    postprocess_time.count() / 1000.0, total_time.count() / 1000.0, result.size());

  if (total_time > std::chrono::milliseconds(30)) {
    RCLCPP_WARN(
      rclcpp::get_logger("yolo_latency"), "Too SLOW! Total time %.2fms > 30ms",
      total_time.count() / 1000.0);
  }

  return result;
}

void YOLOV5::push(cv::Mat img, int64_t t)
{
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  auto infer_request = compiled_model_.create_infer_request();
  auto input_tensor = infer_request.get_input_tensor();

  cv::Mat wrapper(640, 640, CV_8UC3, input_tensor.data<uint8_t>());

  wrapper = cv::Scalar(0, 0, 0);

  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, wrapper(roi), {w, h});
  infer_request.start_async();
  queue_.push({img.clone(), t, std::move(infer_request)});
}

std::tuple<std::vector<Armor>, int64_t> YOLOV5::pop()
{
  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();
  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = parse(scale, output, img);

  return {armors, t};
}

std::tuple<cv::Mat, std::vector<Armor>, int64_t> YOLOV5::debug_pop()
{
  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();
  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = parse(scale, output, img);
  return {img, std::move(armors), t};
}

std::optional<std::tuple<cv::Mat, std::vector<Armor>, int64_t>> YOLOV5::try_debug_pop()
{
  auto queued = queue_.try_pop();
  if (!queued.has_value()) {
    return std::nullopt;
  }

  auto [img, t, infer_request] = std::move(*queued);
  infer_request.wait();
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = parse(scale, output, img);
  return std::make_tuple(img, std::move(armors), t);
}

std::vector<Armor> YOLOV5::parse(double scale, const cv::Mat & output, const cv::Mat & bgr_img)
{
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;
    //从左上角开始，顺时针
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

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

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::vector<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    ++it;
  }
  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  return name_ok;
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::vector<Armor> YOLOV5::postprocess(
  double scale, const cv::Mat & output, const cv::Mat & bgr_img)
{
  return parse(scale, output, bgr_img);
}

}  // namespace rm_auto_aim
