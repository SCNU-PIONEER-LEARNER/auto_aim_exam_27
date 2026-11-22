#include "armor_detector/detector.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>



namespace rm_auto_aim
{

Detector::Detector()
{
  RCLCPP_INFO(rclcpp::get_logger("armor_detector"), "Detector initialized !");
}

cv::Mat Detector::draw_result(const std::vector<Armor> armors, cv::Mat img, double latency)
{
  for (auto & armor : armors) {
    cv::line(img, armor.points[0], armor.points[2], cv::Scalar(0, 255, 0), 1);
    cv::line(img, armor.points[1], armor.points[3], cv::Scalar(0, 255, 0), 1);
    cv::putText(
      img, armor.rfs, armor.points[0], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    for (const auto & armor : armors)
      for (const auto & pt : armor.net_points) {
        cv::circle(img, pt, 1, cv::Scalar(125, 125, 125), -1);
      }
  }
  std::stringstream latency_ss;
  latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
  auto latency_s = latency_ss.str();
  cv::putText(
    img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
  return img;
}

bool Detector::traditional_correct(Armor & armor, const cv::Mat & img)
{
  // 取得四个角点
  auto tl = armor.points[0];
  auto tr = armor.points[1];
  auto br = armor.points[2];
  auto bl = armor.points[3];
  // 计算向量和调整后的点
  auto lt2b = bl - tl;
  auto rt2b = br - tr;
  //高扩大1.5倍
  auto tl1 = (tl + bl) / 2 - lt2b;
  auto bl1 = (tl + bl) / 2 + lt2b;
  auto br1 = (tr + br) / 2 + rt2b;
  auto tr1 = (tr + br) / 2 - rt2b;
  //计算宽度
  auto tl2tr = tr1 - tl1;
  auto bl2br = br1 - bl1;

  //宽扩大
  auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
  auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
  auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
  auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;
  // 构造新的四个角点
  std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
  auto armor_rotaterect = cv::minAreaRect(points);
  cv::Rect boundingBox = armor_rotaterect.boundingRect();
  // 检查boundingBox是否超出图像边界
  if (
    boundingBox.x < 0 || boundingBox.y < 0 || boundingBox.x + boundingBox.width > img.cols ||
    boundingBox.y + boundingBox.height > img.rows) {
    return false;
  }

  // 在图像上裁剪出这个矩形区域（ROI）
  cv::Mat armor_roi = img(boundingBox);
  if (armor_roi.empty()) {
    return false;
  }

  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY);
  // 进行二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);
  armor.pattern = binary_img;
  // 获取轮廓点
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  // 获取灯条
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, contour);

    if (!check_geometry(lightbar)) continue;

    if (use_pca && lightbar.length > 3) {
      pca_corrector(lightbar, gray_img);
    } else {
      zy_corrector(lightbar);
    }
    lightbars.emplace_back(lightbar);
  }

  if (lightbars.size() < 2) return false;

  // 将灯条从左到右排序
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  // 计算与 tl_roi, bl_roi 和 br_roi, tr_roi 距离最近的灯条
  Lightbar * closest_left_lightbar = nullptr;
  Lightbar * closest_right_lightbar = nullptr;
  float min_distance_tl_bl = std::numeric_limits<float>::max();
  float min_distance_br_tr = std::numeric_limits<float>::max();
  for (auto & lightbar : lightbars) {
    float distance_tl_bl =
      cv::norm(tl - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(bl - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_tl_bl < min_distance_tl_bl) {
      min_distance_tl_bl = distance_tl_bl;
      closest_left_lightbar = &lightbar;
    }
    float distance_br_tr =
      cv::norm(br - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(tr - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_br_tr < min_distance_br_tr) {
      min_distance_br_tr = distance_br_tr;
      closest_right_lightbar = &lightbar;
    }
  }

  if (closest_left_lightbar && closest_right_lightbar) {
    // 计算网络框偏移后的传统角点
    const cv::Point2f offset(boundingBox.x, boundingBox.y);
    auto avg_length = 0.5 * (closest_left_lightbar->length + closest_right_lightbar->length);
    const cv::Point2f lt_trad = closest_left_lightbar->top + offset;
    const cv::Point2f lb_trad = closest_left_lightbar->bottom + offset;
    const cv::Point2f rt_trad = closest_right_lightbar->top + offset;
    const cv::Point2f rb_trad = closest_right_lightbar->bottom + offset;

    // 点误差向量
    const cv::Point2f lt_error = lt_trad - armor.points[0];
    const cv::Point2f lb_error = lb_trad - armor.points[3];
    const cv::Point2f rt_error = rt_trad - armor.points[1];
    const cv::Point2f rb_error = rb_trad - armor.points[2];

    // 点距离
    const float lt_dist = cv::norm(lt_error);
    const float lb_dist = cv::norm(lb_error);
    const float rt_dist = cv::norm(rt_error);
    const float rb_dist = cv::norm(rb_error);

    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "lt_dist: %f", lt_dist);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "lb_dist: %f", lb_dist);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "rt_dist: %f", rt_dist);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "rb_dist: %f", rb_dist);

    bool updated = false;

    if (lt_dist / avg_length < tolerate) {
      armor.points[0] = lt_trad;
      updated = true;
    }
    if (lb_dist / avg_length < tolerate) {
      armor.points[3] = lb_trad;
      updated = true;
    }
    if (rt_dist / avg_length < tolerate) {
      armor.points[1] = rt_trad;
      updated = true;
    }
    if (rb_dist / avg_length < tolerate) {
      armor.points[2] = rb_trad;
      updated = true;
    }

    if (updated) {  //update side_ratio
      double left_width = cv::norm(armor.points[0] - armor.points[3]);
      double right_width = cv::norm(armor.points[1] - armor.points[2]);
      armor.side_ratio = left_width / right_width;
      return true;
    }
  }

  RCLCPP_WARN(
    rclcpp::get_logger("armor_detector"), "diff between network and traditon is too large!");
  return false;
}


bool Detector::check_geometry(const Lightbar & lightbar) const
{
  auto angle_ok = lightbar.angle_error < max_angle_error_;
  auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
  auto length_ok = lightbar.length > min_lightbar_length_;
  return angle_ok && ratio_ok && length_ok;
}

void Detector::pca_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const
{
  // 配置参数
  constexpr float MAX_BRIGHTNESS = 25;  // 归一化最大亮度值
  constexpr float ROI_SCALE = 0.07;     // ROI扩展比例
  constexpr float SEARCH_START = 0.4;   // 搜索起始位置比例（原0.8/2）
  constexpr float SEARCH_END = 0.6;     // 搜索结束位置比例（原1.2/2）

  // 扩展并裁剪ROI
  cv::Rect roi_box = lightbar.rotated_rect.boundingRect();
  roi_box.x -= roi_box.width * ROI_SCALE;
  roi_box.y -= roi_box.height * ROI_SCALE;
  roi_box.width += 2 * roi_box.width * ROI_SCALE;
  roi_box.height += 2 * roi_box.height * ROI_SCALE;

  // 边界约束
  roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);

  // 归一化ROI
  cv::Mat roi = gray_img(roi_box);
  const float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // 计算质心
  const cv::Moments moments = cv::moments(roi);
  const cv::Point2f centroid(
    moments.m10 / moments.m00 + roi_box.x, moments.m01 / moments.m00 + roi_box.y);

  // 生成稀疏点云（优化性能）
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; ++i) {
    for (int j = 0; j < roi.cols; ++j) {
      const float weight = roi.at<float>(i, j);
      if (weight > 1e-3) {          // 忽略极小值提升性能
        points.emplace_back(j, i);  // 坐标相对于ROI区域
      }
    }
  }

  // PCA计算对称轴方向
  cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
  axis /= cv::norm(axis);
  if (axis.y > 0) axis = -axis;  // 统一方向

  const auto find_corner = [&](int direction) -> cv::Point2f {
    const float dx = axis.x * direction;
    const float dy = axis.y * direction;
    const float search_length = lightbar.length * (SEARCH_END - SEARCH_START);

    std::vector<cv::Point2f> candidates;

    // 横向采样多个候选线
    const int half_width = (lightbar.width - 2) / 2;
    for (int i_offset = -half_width; i_offset <= half_width; ++i_offset) {
      // 计算搜索起点
      cv::Point2f start_point(
        centroid.x + lightbar.length * SEARCH_START * dx + i_offset,
        centroid.y + lightbar.length * SEARCH_START * dy);

      // 沿轴搜索亮度跳变点
      cv::Point2f corner = start_point;
      float max_diff = 0;
      bool found = false;

      for (float step = 0; step < search_length; ++step) {
        const cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);

        // 边界检查
        if (
          cur_point.x < 0 || cur_point.x >= gray_img.cols || cur_point.y < 0 ||
          cur_point.y >= gray_img.rows) {
          break;
        }

        // 计算亮度差（使用双线性插值提升精度）
        const auto prev_val = gray_img.at<uchar>(cv::Point2i(cur_point - cv::Point2f(dx, dy)));
        const auto cur_val = gray_img.at<uchar>(cv::Point2i(cur_point));
        const float diff = prev_val - cur_val;

        if (diff > max_diff && prev_val > mean_val) {
          max_diff = diff;
          corner = cur_point - cv::Point2f(dx, dy);  // 跳变发生在上一位置
          found = true;
        }
      }

      if (found) {
        candidates.push_back(corner);
      }
    }

    // 返回候选点均值
    return candidates.empty()
             ? cv::Point2f(-1, -1)
             : std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0)) /
                 static_cast<float>(candidates.size());
  };

  // 并行检测顶部和底部
  lightbar.top = find_corner(1);
  lightbar.bottom = find_corner(-1);
}

void Detector::zy_corrector(Lightbar & lightbar) const
{
  auto b_rect = cv::boundingRect(lightbar.contour);
  cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
  std::vector<cv::Point> mask_contour;
  for (const auto & p : lightbar.contour) {
    mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
  }
  cv::fillPoly(mask, {mask_contour}, 255);
  std::vector<cv::Point> points;
  cv::findNonZero(mask, points);
  cv::Vec4f return_param;
  cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
  cv::Point2f top, bottom;
  if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
    top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
    bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
  } else {
    auto k = return_param[1] / return_param[0];
    auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
    lightbar.top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
    lightbar.bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
  }
}
cv::Mat Detector::get_all_binary_img(std::vector<Armor> armors)
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

}  // namespace rm_auto_aim

