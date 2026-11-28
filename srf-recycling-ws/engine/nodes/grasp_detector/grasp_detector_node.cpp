#include "grasp_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "srf/msgs.hpp"
#include "srf/qos.hpp"
#include "srf/tf/math.hpp"

#ifdef SRF_RECYCLING_WS_HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace srf_recycling_ws {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kColorFxScale = 0.961;  // D435 color 640x480 fx≈615 -> 320x240 fx≈307.5

[[maybe_unused]] std::string json_quoted(const std::string& value) {  // OpenCV 없이 빌드하면 안 쓰인다
  std::string out = "\"";
  for (const char character : value) {
    if (character == '"' || character == '\\') {
      out.push_back('\\');
    }
    out.push_back(character);
  }
  out.push_back('"');
  return out;
}

}  // namespace

#ifdef SRF_RECYCLING_WS_HAS_OPENCV

namespace {

struct Candidate {
  cv::Rect box;
  cv::Point2f center;
  double area = 0.0;
  std::string color;
  std::vector<cv::Point> contour;
  cv::Mat mask;
};

cv::Mat red_mask(const cv::Mat& hsv, const cv::Mat& bgr) {
  // 어두운 빨강(원기둥 앞면/그늘)은 HSV V 임계값에서 끊기고 가장자리만 남는다.
  // R이 G/B보다 확실히 큰 픽셀과 HSV 빨강을 합쳐 앞면을 살린다.
  std::vector<cv::Mat> channels;
  cv::split(bgr, channels);
  cv::Mat bright;
  cv::Mat red_green;
  cv::Mat red_blue;
  cv::compare(channels[2], 25, bright, cv::CMP_GT);
  cv::subtract(channels[2], channels[1], red_green);
  cv::subtract(channels[2], channels[0], red_blue);
  cv::compare(red_green, 8, red_green, cv::CMP_GT);
  cv::compare(red_blue, 8, red_blue, cv::CMP_GT);
  cv::bitwise_and(bright, red_green, bright);
  cv::bitwise_and(bright, red_blue, bright);

  cv::Mat low;
  cv::Mat high;
  cv::inRange(hsv, cv::Scalar(0, 60, 15), cv::Scalar(15, 255, 255), low);
  cv::inRange(hsv, cv::Scalar(165, 60, 15), cv::Scalar(180, 255, 255), high);
  cv::bitwise_or(low, high, low);
  cv::bitwise_or(bright, low, bright);
  return bright;
}

cv::Mat make_mask(const cv::Mat& hsv, const cv::Mat& bgr, const std::string& color) {
  cv::Mat mask;
  if (color == "red") {
    mask = red_mask(hsv, bgr);
  } else if (color == "yellow") {
    cv::inRange(hsv, cv::Scalar(15, 80, 60), cv::Scalar(40, 255, 255), mask);
  } else if (color == "blue") {
    cv::inRange(hsv, cv::Scalar(90, 80, 60), cv::Scalar(130, 255, 255), mask);
  }
  if (!mask.empty()) {
    const cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);
  }
  return mask;
}

std::optional<Candidate> find_candidate(const cv::Mat& hsv, const cv::Mat& bgr) {
  Candidate best;
  bool found = false;
  for (const char* color : {"red", "yellow", "blue"}) {
    const cv::Mat mask = make_mask(hsv, bgr, color);
    if (mask.empty()) {
      continue;
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (std::size_t index = 0; index < contours.size(); ++index) {
      const double area = cv::contourArea(contours[index]);
      if (area < 120.0 || area <= best.area) {
        continue;
      }
      Candidate candidate;
      candidate.box = cv::boundingRect(contours[index]);
      candidate.center = cv::Point2f(0.0F, 0.0F);
      const cv::Moments moments = cv::moments(contours[index]);
      if (moments.m00 > 0.0) {
        candidate.center = cv::Point2f(static_cast<float>(moments.m10 / moments.m00),
                                       static_cast<float>(moments.m01 / moments.m00));
      } else {
        candidate.center = cv::Point2f(candidate.box.x + candidate.box.width * 0.5F,
                                       candidate.box.y + candidate.box.height * 0.5F);
      }
      candidate.area = area;
      candidate.color = color;
      candidate.contour = contours[index];
      candidate.mask = mask;
      best = std::move(candidate);
      found = true;
    }
  }
  if (!found) {
    return std::nullopt;
  }
  // best.mask에 선택한 컨투어만 남긴다. (depth 샘플링을 물체 픽셀로 제한)
  cv::Mat selected = cv::Mat::zeros(best.mask.size(), CV_8UC1);
  const std::vector<std::vector<cv::Point>> selected_contours{best.contour};
  cv::drawContours(selected, selected_contours, 0, cv::Scalar(255), cv::FILLED);
  best.mask = std::move(selected);
  return best;
}

}  // namespace

GraspDetectorNode::GraspDetectorNode(srf::Context& context, std::string color_topic,
                                     std::string depth_topic, std::string objects_topic,
                                     std::string info_topic)
    : Node(context, "grasp_detector"),
      color_topic_(std::move(color_topic)),
      depth_topic_(std::move(depth_topic)),
      objects_topic_(std::move(objects_topic)),
      info_topic_(std::move(info_topic)) {
  tf_listener_ = std::make_unique<srf::TransformListener>(tf_buffer_, *this);
  color_subscription_ = subscribe<srf::msgs::Image>(
      color_topic_, [this](const srf::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        color_ = message;
        color_seen_ = true;
      },
      srf::QOS_SENSOR_DATA);
  depth_subscription_ = subscribe<srf::msgs::Image>(
      depth_topic_, [this](const srf::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        depth_ = message;
        depth_seen_ = true;
      },
      srf::QOS_SENSOR_DATA);
  timer_ = create_timer(0.25, [this] { detect(); });
  logger().info("GraspDetector: ready color=%s depth=%s objects=%s",
                color_topic_.c_str(), depth_topic_.c_str(),
                objects_topic_.c_str());
}

GraspDetectorNode::~GraspDetectorNode() = default;

void GraspDetectorNode::detect() {
  srf::msgs::Image color;
  srf::msgs::Image depth;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!color_seen_ || !depth_seen_) {
      return;
    }
    color = color_;
    depth = depth_;
  }
  const double current = now();
  if (color.data.empty() || depth.data.empty() ||
      current - color.header.stamp > 1.0 || current - depth.header.stamp > 1.0) {
    return;
  }

  cv::Mat bgr;
  if (color.encoding == "jpeg" || color.encoding == "jpg") {
    bgr = cv::imdecode(color.data, cv::IMREAD_COLOR);
  } else if (color.encoding == "rgb8" || color.encoding == "bgr8") {
    const int type = CV_8UC3;
    const int step = color.step > 0 ? static_cast<int>(color.step)
                                    : static_cast<int>(color.width) * 3;
    bgr = cv::Mat(static_cast<int>(color.height), static_cast<int>(color.width), type,
                  const_cast<std::uint8_t*>(color.data.data()), step);
    if (color.encoding == "rgb8") {
      cv::cvtColor(bgr, bgr, cv::COLOR_RGB2BGR);
    }
    bgr = bgr.clone();
  }
  if (bgr.empty()) {
    return;
  }

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  const auto candidate = find_candidate(hsv, bgr);
  if (!candidate) {
    return;
  }

  const int depth_width = static_cast<int>(depth.width);
  const int depth_height = static_cast<int>(depth.height);
  if (depth.encoding != "16UC1" || depth_width <= 0 || depth_height <= 0) {
    return;
  }
  const int depth_step = depth.step > 0 ? static_cast<int>(depth.step) : depth_width * 2;
  const cv::Mat depth_mat(depth_height, depth_width, CV_16UC1,
                          const_cast<std::uint8_t*>(depth.data.data()), depth_step);

  const double c_fx = static_cast<double>(color.width) * kColorFxScale;
  const double c_fy = c_fx;
  const double c_cx = static_cast<double>(color.width) * 0.5;
  const double c_cy = static_cast<double>(color.height) * 0.5;
  // D435 depth FOV는 color보다 넓다. color 픽셀 광선을 depth 픽셀로 옮겨 샘플한다.
  const double d_fx = (depth_width * 0.5) / std::tan(87.0 * kPi / 360.0);
  const double d_fy = (depth_height * 0.5) / std::tan(58.0 * kPi / 360.0);

  std::vector<double> depths;
  depths.reserve(static_cast<std::size_t>(candidate->mask.total() / 4));
  for (int y = candidate->box.y; y < candidate->box.y + candidate->box.height; y += 2) {
    for (int x = candidate->box.x; x < candidate->box.x + candidate->box.width; x += 2) {
      if (candidate->mask.at<std::uint8_t>(y, x) == 0) {
        continue;
      }
      const double tx = std::atan((static_cast<double>(x) - c_cx) / c_fx);
      const double ty = std::atan((static_cast<double>(y) - c_cy) / c_fy);
      const int du = static_cast<int>(depth_width * 0.5 + d_fx * std::tan(tx));
      const int dv = static_cast<int>(depth_height * 0.5 + d_fy * std::tan(ty));
      if (du < 0 || du >= depth_width || dv < 0 || dv >= depth_height) {
        continue;
      }
      const std::uint16_t raw = depth_mat.at<std::uint16_t>(dv, du);
      if (raw == 0) {
        continue;
      }
      const double meters = static_cast<double>(raw) / 1000.0;
      if (meters > 0.10 && meters < 1.20) {
        depths.push_back(meters);
      }
    }
  }
  if (depths.size() < 20) {
    return;
  }
  std::sort(depths.begin(), depths.end());
  const double near_limit = depths[depths.size() * 5 / 100] + 0.08;
  std::vector<double> near;
  near.reserve(depths.size());
  for (const double value : depths) {
    if (value <= near_limit) {
      near.push_back(value);
    }
  }
  if (near.empty()) {
    near = depths;
  }
  const double surface_depth = near[near.size() / 2];
  // bbox 폭은 물체 경계를 조금 넓게 잡는다. 0.95는 그리퍼 목표값 여유.
  const double width =
      std::clamp(static_cast<double>(candidate->box.width) * surface_depth / c_fx * 0.95,
                 0.012, 0.200);
  const double height =
      std::clamp(static_cast<double>(candidate->box.height) * surface_depth / c_fy,
                 0.012, 0.300);
  const double center_depth = surface_depth + width * 0.5;
  // 배경의 빨간 물체/케이블을 잡는 false positive를 막는다.
  // 그리퍼 최대 개방(약 6.2cm)에 가까운 후보는 pick이 거부하므로 아예
  // 발행하지 않고 마지막 valid 물체를 유지한다.
  if (width > 0.072 || height > 0.300 || center_depth > 0.60) {
    return;
  }
  const double camera_x = (static_cast<double>(candidate->center.x) - c_cx) *
                          center_depth / c_fx;
  const double camera_y = (static_cast<double>(candidate->center.y) - c_cy) *
                          center_depth / c_fy;

  srf::msgs::Vector3 point{camera_x, camera_y, center_depth};
  try {
    // 현재 시각은 odom TF 최신 시각보다 몇 ms 앞설 수 있다. 검출에 쓴
    // color 프레임 stamp로 조회해 "history ... requested" 경고를 피한다.
    const double tf_time =
        (color.header.stamp > 0.0 ? color.header.stamp : current) - 0.05;
    const srf::msgs::TransformStamped transform =
        tf_buffer_.lookup_transform("odom", "camera_color_optical_frame", tf_time);
    point = srf::tfmath::apply(transform.transform, point);
  } catch (const std::exception& exception) {
    if (!warned_tf_) {
      logger().warn("GraspDetector: odom TF 대기: %s", exception.what());
      warned_tf_ = true;
    }
    return;
  }
  warned_tf_ = false;

  // 바닥에 선 물체로 보고 중심 높이를 height/2로 쓴다. 색 중심의 optical y는
  // 원기둥 앞면/바닥 반사에 끌려 z가 0에 가깝게 나왔다.
  point.z = height * 0.5;

  srf::msgs::Path path;
  path.header = srf::msgs::Header{current, "odom"};
  srf::msgs::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position = srf::msgs::Point{point.x, point.y, point.z};
  pose.pose.orientation = srf::tfmath::from_yaw(0.0);
  path.poses.push_back(pose);
  publish<srf::msgs::Path>(objects_topic_, std::move(path));

  char info[256];
  std::snprintf(info, sizeof(info),
                "{\"color\":%s,\"width\":%.4f,\"height\":%.4f,\"depth\":%.4f,"
                "\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}",
                json_quoted(candidate->color).c_str(), width, height, center_depth,
                point.x, point.y, point.z);
  publish<srf::msgs::String>(info_topic_, srf::msgs::String{info});

  if (current - last_log_ >= 2.0) {
    last_log_ = current;
    logger().info("GraspDetector: %s width=%.3f height=%.3f odom=(%.3f, %.3f, %.3f) depth=%.3f",
                  candidate->color.c_str(), width, height, point.x, point.y, point.z,
                  center_depth);
  }
}

#else

GraspDetectorNode::GraspDetectorNode(srf::Context& context, std::string color_topic,
                                     std::string depth_topic, std::string objects_topic,
                                     std::string info_topic)
    : Node(context, "grasp_detector"),
      color_topic_(std::move(color_topic)),
      depth_topic_(std::move(depth_topic)),
      objects_topic_(std::move(objects_topic)),
      info_topic_(std::move(info_topic)) {
  logger().warn("GraspDetector: OpenCV 없음 — 실기 물체 검출 비활성");
}

GraspDetectorNode::~GraspDetectorNode() = default;

void GraspDetectorNode::detect() {}

#endif

}  // namespace srf_recycling_ws
