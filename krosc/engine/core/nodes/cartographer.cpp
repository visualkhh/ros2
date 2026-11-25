#include "krosc/nodes/cartographer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include "krosc/tf/math.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf/tf_ros.hpp"

namespace krosc::nodes {

namespace {

constexpr double kDegreesToRadians = std::numbers::pi / 180.0;
constexpr double kMatchMinLinear = 0.01;  // m: 이만큼도 안 움직였으면 스캔 매칭을 건너뛴다
constexpr double kMatchMinAngular = 0.5 * kDegreesToRadians;
constexpr double kRelocalizeScale = 5.0;  // 지도 load/initialpose 뒤 첫 매칭은 탐색 범위를 이만큼 넓힌다

}  // namespace

Cartographer::Cartographer(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)), broadcaster_(*this) {
  const auto positive = [](const double& value) { return value > 0.0; };
  map_frame_ = declare_parameter<std::string>("map_frame", std::string("map"));
  odom_frame_ = declare_parameter<std::string>("odom_frame", std::string("odom"));
  tracking_frame_ = declare_parameter<std::string>("tracking_frame", std::string("base_footprint"));
  scan_topic_ = declare_parameter<std::string>("scan_topic", std::string("scan"));
  resolution_ = declare_parameter<double>("resolution", 0.05, positive);
  map_publish_period_sec_ = declare_parameter<double>("map_publish_period_sec", 1.0, positive);
  tf_publish_period_sec_ = declare_parameter<double>("tf_publish_period_sec", 0.05, positive);
  linear_search_window_ = declare_parameter<double>("linear_search_window", 0.1, positive);
  angular_search_window_ = declare_parameter<double>("angular_search_window", 6.0 * kDegreesToRadians, positive);
  min_score_ = declare_parameter<double>("min_score", 0.55,
                                         [](const double& value) { return value >= 0.0 && value <= 1.0; });
  motion_filter_max_distance_ = declare_parameter<double>("motion_filter_max_distance", 0.05, positive);
  motion_filter_max_angle_ = declare_parameter<double>("motion_filter_max_angle", 3.0 * kDegreesToRadians, positive);
  num_matching_points_ = declare_parameter("num_matching_points", 60,
                                           [](const std::int64_t& value) { return value > 0; });
  match_every_ = declare_parameter("match_every", 1, [](const std::int64_t& value) { return value >= 1; });
  tf_smoothing_ = declare_parameter<double>("tf_smoothing", 0.0,
                                            [](const double& value) { return value >= 0.0 && value < 1.0; });

  grid_ = std::make_unique<cartographer::ProbabilityGrid>(resolution_.value());
  listener_ = std::make_unique<TransformListener>(buffer_, *this);
  subscribe<msgs::LaserScan>(scan_topic_.value(), [this](const msgs::LaserScan& scan) { on_scan(scan); });
  subscribe<msgs::OccupancyGrid>(
      "map", [this](const msgs::OccupancyGrid& map) { on_map(map); }, QOS_LATCHED);
  subscribe<msgs::PoseWithCovarianceStamped>(
      "initialpose", [this](const msgs::PoseWithCovarianceStamped& message) { on_initial_pose(message); });
  create_timer(map_publish_period_sec_.value(), [this] { publish_map(); });
  create_timer(tf_publish_period_sec_.value(), [this] { publish_tf(); });
  mode_ = declare_parameter<std::string>(
      "mode", std::string("mapping"),
      [](const std::string& value) { return value == "idle" || value == "mapping" || value == "localization"; },
      [this](const std::string& value) { set_mode(value); });
  set_mode(mode_.value());
  subscribe<msgs::String>(
      "slam_mode",
      [this](const msgs::String& message) {
        if (message.data == "idle" || message.data == "mapping" || message.data == "localization") {
          mode_.set(message.data);
        }
      },
      QOS_LATCHED);
  logger().info("Cartographer: map=%s odom=%s scan=%s resolution=%.2f",
                map_frame_.value().c_str(), odom_frame_.value().c_str(), scan_topic_.value().c_str(),
                resolution_.value());
}

std::string Cartographer::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_str_;
}

int Cartographer::scans() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return scans_;
}

int Cartographer::rejected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rejected_;
}

bool Cartographer::localized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return localized_;
}

std::size_t Cartographer::map_cells() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return grid_ ? grid_->size() : 0;
}

cartographer::Pose2 Cartographer::map_to_odom() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_to_odom_;
}

std::string Cartographer::slam_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_str_;
}

void Cartographer::set_mode(const std::string& mode) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode == mode_str_) {
      return;
    }
    mode_str_ = mode;
    has_last_inserted_ = false;
    tf_published_valid_ = false;
    dirty_ = false;
    if (mode == "mapping") {
      grid_ = std::make_unique<cartographer::ProbabilityGrid>(resolution_.value());
      // 새 지도는 출발점 기준으로 다시 만든다.
      map_to_odom_ = {0.0, 0.0, 0.0};
      localized_ = false;
      has_last_match_ = false;
      relocalize_ = true;
    } else if (mode == "localization") {
      grid_ = external_map_ ? std::make_unique<cartographer::ProbabilityGrid>(
                                  cartographer::ProbabilityGrid::from_message(*external_map_))
                            : std::make_unique<cartographer::ProbabilityGrid>(resolution_.value());
      // 지도 원점 = 저장 당시 출발점이라는 가정으로 재출발하고, 첫 매칭은 넓게 찾는다.
      // (2D Pose를 주지 않아도 몇 스캔 안에 붙게 하기 위함)
      map_to_odom_ = {0.0, 0.0, 0.0};
      localized_ = false;
      has_last_match_ = false;
      relocalize_ = true;
    }
  }
  logger().info("Cartographer mode: %s", mode.c_str());
  publish("slam_status", msgs::String{mode}, QOS_LATCHED);
}

void Cartographer::on_scan(const msgs::LaserScan& scan) {
  std::string mode;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_str_ == "idle") {
      return;
    }
    ++scan_index_;
    const std::int64_t every = std::max<std::int64_t>(1, match_every_.value());
    if (scan_index_ % every != 0) {
      return;
    }
    mode = mode_str_;
  }

  msgs::TransformStamped transform;
  try {
    transform = buffer_.lookup_transform(odom_frame_.value(), scan.header.frame_id, scan.header.stamp);
  } catch (const TransformException&) {
    try {  // 스캔이 TF보다 먼저 왔다: 최신 TF로 (버리면 바퀴가 헛돌 때 스캔이 연달아 빠져 위치를 놓친다)
      transform = buffer_.lookup_transform(odom_frame_.value(), scan.header.frame_id);
    } catch (const TransformException&) {
      return;
    }
  }
  const cartographer::Pose2 sensor_in_odom{transform.transform.translation.x,
                                           transform.transform.translation.y,
                                           tfmath::yaw_of(transform.transform.rotation)};

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((mode == "localization" || mode == "mapping") && localized_ && has_last_match_ && !relocalize_ &&
        std::hypot(sensor_in_odom[0] - last_match_odom_[0], sensor_in_odom[1] - last_match_odom_[1]) <
            kMatchMinLinear &&
        std::abs(cartographer::wrap_angle(sensor_in_odom[2] - last_match_odom_[2])) < kMatchMinAngular) {
      return;  // 서 있는 동안은 지난 보정을 그대로 쓴다 (매핑도 지도가 안 변한다: CPU/거절 방지)
    }
    last_match_odom_ = sensor_in_odom;
    has_last_match_ = true;
  }

  cartographer::ScanPoints points;
  points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    points.emplace_back(range * std::cos(angle), range * std::sin(angle));
  }
  if (points.size() < 10) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!grid_) {
      return;
    }
    cartographer::Pose2 pose = cartographer::compose(map_to_odom_, sensor_in_odom);
    if (!grid_->empty()) {
      const std::size_t wanted = static_cast<std::size_t>(std::max<std::int64_t>(1, num_matching_points_.value()));
      const std::size_t step = std::max<std::size_t>(1, points.size() / wanted);
      cartographer::ScanPoints sampled;
      sampled.reserve(points.size() / step + 1);
      for (std::size_t index = 0; index < points.size(); index += step) {
        sampled.push_back(points[index]);
      }
      const double scale = relocalize_ ? kRelocalizeScale : 1.0;
      relocalize_ = false;
      const auto [found, value] = cartographer::match(
          *grid_, sampled, pose, linear_search_window_.value() * scale, angular_search_window_.value() * scale);
      if (value >= min_score_.value()) {
        pose = found;
      } else {
        ++rejected_;
      }
    } else if (mode == "localization") {
      return;  // 지도가 아직 없다
    }

    map_to_odom_ = cartographer::compose(pose, cartographer::inverse(sensor_in_odom));
    ++scans_;
    localized_ = true;
    if (mode == "mapping" && moved(pose)) {
      const double c = std::cos(pose[2]);
      const double s = std::sin(pose[2]);
      std::vector<std::pair<double, double>> hits;
      hits.reserve(points.size());
      for (const auto& [px, py] : points) {
        hits.emplace_back(pose[0] + c * px - s * py, pose[1] + s * px + c * py);
      }
      grid_->insert({pose[0], pose[1]}, hits);
      last_inserted_ = pose;
      has_last_inserted_ = true;
      dirty_ = true;
    }
  }
}

void Cartographer::on_map(const msgs::OccupancyGrid& map) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (std::abs(map.header.stamp - own_map_stamp_) < 1e-9 && map.info.width == own_map_width_ &&
      map.info.height == own_map_height_) {
    return;  // 내가 낸 지도가 map 토픽으로 되돌아온 것
  }
  external_map_ = map;
  if (mode_str_ == "localization") {
    grid_ = std::make_unique<cartographer::ProbabilityGrid>(cartographer::ProbabilityGrid::from_message(map));
    has_last_inserted_ = false;
  }
}

void Cartographer::on_initial_pose(const msgs::PoseWithCovarianceStamped& message) {
  if (message.header.frame_id != map_frame_.value()) {
    logger().warn("/initialpose는 %s 좌표계여야 한다: %s", map_frame_.value().c_str(),
                  message.header.frame_id.c_str());
    return;
  }
  msgs::TransformStamped transform;
  try {
    transform = buffer_.lookup_transform(odom_frame_.value(), tracking_frame_.value());
  } catch (const TransformException&) {
    logger().warn("%s -> %s TF가 아직 없어 /initialpose를 쓸 수 없다", odom_frame_.value().c_str(),
                  tracking_frame_.value().c_str());
    return;
  }
  const cartographer::Pose2 base_in_odom{transform.transform.translation.x, transform.transform.translation.y,
                                         tfmath::yaw_of(transform.transform.rotation)};
  const cartographer::Pose2 base_in_map{message.pose.pose.position.x, message.pose.pose.position.y,
                                        tfmath::yaw_of(message.pose.pose.orientation)};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_to_odom_ = cartographer::compose(base_in_map, cartographer::inverse(base_in_odom));
    relocalize_ = true;
    localized_ = true;
    has_last_inserted_ = false;
    tf_published_valid_ = false;
  }
  logger().info("initialpose: (%.2f, %.2f, %.0fdeg)", base_in_map[0], base_in_map[1],
                base_in_map[2] / kDegreesToRadians);
}

void Cartographer::publish_map() {
  msgs::OccupancyGrid message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!dirty_ || mode_str_ != "mapping" || !grid_) {
      return;
    }
    dirty_ = false;
    message = grid_->to_message(msgs::Header{now(), map_frame_.value()});
    own_map_stamp_ = message.header.stamp;
    own_map_width_ = message.info.width;
    own_map_height_ = message.info.height;
  }
  const std::uint32_t width = message.info.width;
  const std::uint32_t height = message.info.height;
  publish("map", std::move(message), QOS_LATCHED);
  logger().info("지도 발행: %ux%u", width, height);
}

void Cartographer::publish_tf() {
  cartographer::Pose2 pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_str_ == "idle" || !localized_) {
      return;
    }
    const cartographer::Pose2 target = map_to_odom_;
    const double blend = tf_smoothing_.value();
    if (!tf_published_valid_ || blend <= 0.0) {
      pose = target;
    } else {
      const double delta = cartographer::wrap_angle(target[2] - tf_published_[2]);
      pose = {tf_published_[0] + blend * (target[0] - tf_published_[0]),
              tf_published_[1] + blend * (target[1] - tf_published_[1]),
              cartographer::wrap_angle(tf_published_[2] + blend * delta)};
    }
    tf_published_ = pose;
    tf_published_valid_ = true;
  }
  msgs::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = map_frame_.value();
  transform.child_frame_id = odom_frame_.value();
  transform.transform.translation = {pose[0], pose[1], 0.0};
  transform.transform.rotation = tfmath::from_yaw(pose[2]);
  broadcaster_.send_transform(transform);
}

bool Cartographer::moved(const cartographer::Pose2& pose) const {
  if (!has_last_inserted_) {
    return true;
  }
  const double turn = std::abs(cartographer::wrap_angle(pose[2] - last_inserted_[2]));
  return std::hypot(pose[0] - last_inserted_[0], pose[1] - last_inserted_[1]) >=
             motion_filter_max_distance_.value() ||
         turn >= motion_filter_max_angle_.value();
}

}  // namespace krosc::nodes
