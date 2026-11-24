#include "nodes/rl_nav_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <utility>

#include "krosc/math.hpp"
#include "krosc/qos.hpp"

#ifndef KROSC_WS_MODELS_DIR
#define KROSC_WS_MODELS_DIR "models"
#endif

namespace krosc_ws {

namespace {
constexpr double kPi = 3.14159265358979323846;

double now_seconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

RlNavController::RlNavController(krosc::Context& context, std::string name,
                                 std::map<std::string, krosc::ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      model_path_(declare_parameter<std::string>(
          "model_path", std::string(KROSC_WS_MODELS_DIR) + "/rl_maze/policy.onnx")),
      enable_default_(declare_parameter<bool>("enable", false)),
      fallback_default_(declare_parameter<std::string>("fallback", "pursuit")),
      lookahead_(declare_parameter<std::int64_t>("lookahead", 5, [](std::int64_t value) { return value > 0 && value <= 16; })),
      wp_step_(declare_parameter<double>("wp_step", 0.30, [](double value) { return value > 0.0; })),
      wp_scale_(declare_parameter<double>("wp_scale", 2.0, [](double value) { return value > 0.0; })),
      lidar_rays_(declare_parameter<std::int64_t>("lidar_rays", 36, [](std::int64_t value) { return value > 0 && value <= 720; })),
      lidar_max_(declare_parameter<double>("lidar_max", 3.5, [](double value) { return value > 0.0; })),
      lidar_inflation_(declare_parameter<double>("lidar_inflation", 0.22,
                                                 [](double value) { return value >= 0.0; })),
      inflation_scale_(declare_parameter<double>("inflation_scale", 0.50, [](double value) { return value > 0.0; })),
      max_lin_(declare_parameter<double>("max_lin", 0.26, [](double value) { return value > 0.0; })),
      max_ang_(declare_parameter<double>("max_ang", 1.82, [](double value) { return value > 0.0; })),
      v_min_ratio_(declare_parameter<double>("v_min_ratio", -0.2)),
      frequency_(declare_parameter<double>("frequency", 20.0, [](double value) { return value > 0.0; })),
      scan_timeout_(declare_parameter<double>("scan_timeout", 0.35, [](double value) { return value > 0.0; })),
      odom_timeout_(declare_parameter<double>("odom_timeout", 0.35, [](double value) { return value > 0.0; })),
      map_frame_(declare_parameter<std::string>("map_frame", "map")),
      robot_frame_(declare_parameter<std::string>("robot_frame", "base_footprint")) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enable_ = enable_default_.value();
    fallback_ = fallback_default_.value();
    if (fallback_ != "stop") {
      fallback_ = "pursuit";
    }
    std::string error;
    if (!load_policy(error)) {
      logger().warn("RL policy 없음: %s", error.c_str());
    }
    if (enable_ && !policy_.loaded()) {
      enable_ = false;
    }
  }

  scan_subscription_ = subscribe<krosc::msgs::LaserScan>(
      "scan", [this](const auto& message) { on_scan(message); }, krosc::QOS_SENSOR_DATA);
  imu_subscription_ = subscribe<krosc::msgs::Imu>(
      "imu", [this](const auto& message) { on_imu(message); }, krosc::QOS_SENSOR_DATA);
  odom_subscription_ = subscribe<krosc::msgs::Odometry>(
      "odom", [this](const auto& message) { on_odom(message); }, krosc::QOS_SENSOR_DATA);
  plan_subscription_ = subscribe<krosc::msgs::Path>(
      "plan", [this](const auto& message) { on_plan(message); }, krosc::QOS_LATCHED);
  enable_subscription_ = subscribe<krosc::msgs::Bool>(
      "rl_nav_enable", [this](const auto& message) { on_enable(message); }, krosc::QOS_DEFAULT);
  fallback_subscription_ = subscribe<krosc::msgs::String>(
      "rl_nav_fallback", [this](const auto& message) { on_fallback(message); }, krosc::QOS_DEFAULT);
  reload_subscription_ = subscribe<krosc::msgs::Empty>(
      "rl_nav_reload", [this](const auto& message) { on_reload(message); }, krosc::QOS_DEFAULT);
  tf_listener_ = std::make_unique<krosc::TransformListener>(tf_buffer_, *this);
  timer_ = create_timer(1.0 / frequency_.value(), [this] { tick(); });

  {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_status_locked(enable_ ? 0 : 2, false, 0.0, 0.0, 0.0, 0.0, 0.0);
  }
  logger().info("RlNavController: model=%s inputs=%lld outputs=%lld enable=%d fallback=%s",
                policy_.loaded() ? policy_.path().c_str() : "(none)",
                static_cast<long long>(policy_.inputs()), static_cast<long long>(policy_.outputs()),
                enable_ ? 1 : 0, fallback_.c_str());
}

bool RlNavController::load_policy(std::string& error) {
  policy_.unload();
  policy_error_.clear();
  const std::string path = model_path_.value();
  try {
    policy_.load(path);
    const std::int64_t expected = 2LL * lookahead_.value() + lidar_rays_.value() + 5;
    if ((policy_.inputs() != expected && policy_.inputs() != expected + 1) || policy_.outputs() != 2) {
      std::ostringstream stream;
      stream << "정책 입출력이 맞지 않는다: input=" << policy_.inputs() << " output=" << policy_.outputs()
             << " (기대 " << expected << " 또는 " << expected + 1 << " → 2)";
      policy_.unload();
      error = stream.str();
      policy_error_ = error;
      return false;
    }
    logger().info("RL ONNX loaded: %s (obs %lld → %lld)", path.c_str(),
                  static_cast<long long>(policy_.inputs()), static_cast<long long>(policy_.outputs()));
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    policy_error_ = error;
    return false;
  }
}

void RlNavController::on_scan(const krosc::msgs::LaserScan& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  scan_ = std::make_shared<const krosc::msgs::LaserScan>(message);
  scan_time_ = now();
}

void RlNavController::on_imu(const krosc::msgs::Imu& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_ = std::make_shared<const krosc::msgs::Imu>(message);
  imu_time_ = now();
}

void RlNavController::on_odom(const krosc::msgs::Odometry& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  odom_ = std::make_shared<const krosc::msgs::Odometry>(message);
  odom_time_ = now();
}

void RlNavController::on_plan(const krosc::msgs::Path& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  plan_ = std::make_shared<const krosc::msgs::Path>(message);
  plan_time_ = now();
  prev_action_ = {0.0F, 0.0F};
}

void RlNavController::on_enable(const krosc::msgs::Bool& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (message.data && !policy_.loaded()) {
    enable_ = false;
    publish_status_locked(1, false, 0.0, 0.0, 0.0, 0.0, 0.0);
    return;
  }
  enable_ = message.data;
  prev_action_ = {0.0F, 0.0F};
  active_last_ = false;
  publish_status_locked(enable_ ? 0 : 2, false, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void RlNavController::on_fallback(const krosc::msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string mode = message.data;
  if (mode == "nav") {
    mode = "pursuit";
  }
  if (mode != "pursuit" && mode != "stop") {
    return;
  }
  fallback_ = mode;
  active_last_ = false;
  publish_status_locked(enable_ ? 0 : 2, false, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void RlNavController::on_reload(const krosc::msgs::Empty&) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  const bool ok = load_policy(error);
  if (!ok && enable_) {
    enable_ = false;
  }
  prev_action_ = {0.0F, 0.0F};
  active_last_ = false;
  publish_status_locked(ok ? (enable_ ? 0 : 2) : 1, false, 0.0, 0.0, 0.0, 0.0, 0.0);
}

std::optional<RlNavController::Pose2D> RlNavController::robot_pose() const {
  try {
    const auto transform =
        tf_buffer_.lookup_transform(map_frame_.value(), robot_frame_.value());
    return Pose2D{transform.transform.translation.x, transform.transform.translation.y,
                  krosc::tfmath::yaw_of(transform.transform.rotation)};
  } catch (const krosc::TransformException&) {
    return std::nullopt;
  }
}

double RlNavController::laser_yaw(const std::string& scan_frame) const {
  if (scan_frame.empty() || scan_frame == robot_frame_.value()) {
    return 0.0;
  }
  try {
    const auto transform = tf_buffer_.lookup_transform(robot_frame_.value(), scan_frame);
    return krosc::tfmath::yaw_of(transform.transform.rotation);
  } catch (const krosc::TransformException&) {
    return 0.0;
  }
}

std::vector<float> RlNavController::waypoints(const Pose2D& pose,
                                              const krosc::msgs::Path& plan) const {
  const int lookahead = static_cast<int>(lookahead_.value());
  std::vector<float> out(2 * static_cast<std::size_t>(lookahead), 0.0F);
  if (plan.poses.empty()) {
    return out;
  }
  std::optional<krosc::msgs::Transform> to_map;
  if (!plan.header.frame_id.empty() && plan.header.frame_id != map_frame_.value()) {
    try {
      to_map = tf_buffer_.lookup_transform(map_frame_.value(), plan.header.frame_id).transform;
    } catch (const krosc::TransformException&) {
      return out;
    }
  }
  std::vector<std::pair<double, double>> points;
  points.reserve(plan.poses.size());
  for (const auto& pose_stamped : plan.poses) {
    if (to_map) {
      const krosc::msgs::Vector3 moved = krosc::tfmath::apply(
          *to_map, krosc::msgs::Vector3{pose_stamped.pose.position.x, pose_stamped.pose.position.y, 0.0});
      points.emplace_back(moved.x, moved.y);
    } else {
      points.emplace_back(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
    }
  }
  if (points.empty()) {
    return out;
  }
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double distance = std::hypot(points[i].first - pose.x, points[i].second - pose.y);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  int k = 0;
  double accumulated = 0.0;
  double wanted = wp_step_.value();
  std::size_t i = nearest;
  const auto emit = [&](double x, double y) {
    const double dx = x - pose.x;
    const double dy = y - pose.y;
    out[2 * static_cast<std::size_t>(k)] = static_cast<float>((c * dx + s * dy) / wp_scale_.value());
    out[2 * static_cast<std::size_t>(k) + 1] = static_cast<float>((-s * dx + c * dy) / wp_scale_.value());
    ++k;
  };
  while (k < lookahead && i + 1 < points.size()) {
    const auto& a = points[i];
    const auto& b = points[i + 1];
    const double segment = std::hypot(b.first - a.first, b.second - a.second);
    while (k < lookahead && accumulated + segment >= wanted) {
      const double u = segment > 1e-9 ? (wanted - accumulated) / segment : 0.0;
      emit(a.first + u * (b.first - a.first), a.second + u * (b.second - a.second));
      wanted += wp_step_.value();
    }
    accumulated += segment;
    ++i;
  }
  while (k < lookahead) {
    emit(points.back().first, points.back().second);
  }
  return out;
}

std::vector<float> RlNavController::scan_rays(const krosc::msgs::LaserScan& scan, double yaw,
                                              double inflation) const {
  const int rays = static_cast<int>(lidar_rays_.value());
  std::vector<float> out(static_cast<std::size_t>(rays), 1.0F);
  const int count = static_cast<int>(scan.ranges.size());
  if (count == 0 || scan.angle_increment == 0.0) {
    return out;
  }
  for (int ray = 0; ray < rays; ++ray) {
    double angle = 2.0 * kPi * static_cast<double>(ray) / static_cast<double>(rays) -
                   yaw - scan.angle_min;
    angle = std::fmod(std::fmod(angle, 2.0 * kPi) + 2.0 * kPi, 2.0 * kPi);
    const int index = static_cast<int>(std::lround(angle / scan.angle_increment));
    double nearest = lidar_max_.value();
    for (int offset = -1; offset <= 1; ++offset) {
      const float range = scan.ranges[static_cast<std::size_t>(((index + offset) % count + count) % count)];
      if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
        nearest = std::min(nearest, static_cast<double>(range));
      }
    }
    nearest = std::max(0.0, nearest - std::max(0.0, inflation));
    out[static_cast<std::size_t>(ray)] =
        static_cast<float>(std::clamp(nearest, 0.0, lidar_max_.value()) / lidar_max_.value());
  }
  return out;
}

void RlNavController::publish_zero_locked() {
  publish("cmd_vel_rl", krosc::msgs::Twist{}, krosc::QOS_COMMAND);
}

void RlNavController::publish_status_locked(int reason, bool active, double v, double w, double a0,
                                            double a1, double inference_ms) {
  const double current = now();
  bool scan_ok = scan_ != nullptr;
  double scan_age = -1.0;
  double scan_min = lidar_max_.value();
  if (scan_ok) {
    scan_age = current - scan_time_;
    for (float range : scan_->ranges) {
      if (std::isfinite(range) && range >= scan_->range_min && range <= scan_->range_max) {
        scan_min = std::min(scan_min, static_cast<double>(range));
      }
    }
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << "enable=" << (enable_ ? 1 : 0) << ";fallback=" << fallback_
         << ";policy=" << (policy_.loaded() ? 1 : 0) << ";scan=" << (scan_ok ? 1 : 0)
         << ";scan_age=" << scan_age << ";scan_min=" << scan_min
         << ";odom=" << (odom_ != nullptr ? 1 : 0) << ";plan="
         << (plan_ != nullptr && !plan_->poses.empty() ? 1 : 0)
         << ";v=" << v << ";w=" << w << ";a0=" << a0 << ";a1=" << a1
         << ";infer_ms=" << inference_ms << ";reason=" << reason
         << ";active=" << (active ? 1 : 0);
  if (!policy_error_.empty()) {
    stream << ";error=" << policy_error_;
  }
  publish("rl_nav_status", krosc::msgs::String{stream.str()}, krosc::QOS_DEFAULT);
}

void RlNavController::tick() {
  std::lock_guard<std::mutex> lock(mutex_);
  const double current = now();
  const bool scan_ok = scan_ != nullptr && current - scan_time_ <= scan_timeout_.value();
  const bool odom_ok = odom_ != nullptr && current - odom_time_ <= odom_timeout_.value();
  const bool plan_ok = plan_ != nullptr && !plan_->poses.empty();
  const bool policy_ok = policy_.loaded();

  int reason = 0;
  if (!enable_) {
    reason = 2;
  } else if (!policy_ok) {
    reason = 1;
  } else if (!scan_ok) {
    reason = 3;
  } else if (!odom_ok) {
    reason = 5;
  } else if (!plan_ok) {
    reason = 6;
  }

  bool active = reason == 0;
  std::optional<Pose2D> pose;
  if (active) {
    pose = robot_pose();
    if (!pose) {
      reason = 7;
      active = false;
    }
  }

  if (!active) {
    if (fallback_ == "stop") {
      publish_zero_locked();
    } else if (active_last_) {
      publish_zero_locked();
    }
    active_last_ = false;
    publish_status_locked(reason, false, 0.0, 0.0, 0.0, 0.0, 0.0);
    return;
  }

  const double v_measured = odom_->twist.twist.linear.x;
  const double w_measured = odom_->twist.twist.angular.z;
  const double gyro = (imu_ != nullptr && current - imu_time_ <= scan_timeout_.value())
                          ? imu_->angular_velocity.z
                          : w_measured;
  std::vector<float> observation = waypoints(*pose, *plan_);
  const std::vector<float> rays = scan_rays(*scan_, laser_yaw(scan_->header.frame_id),
                                            lidar_inflation_.value());
  observation.insert(observation.end(), rays.begin(), rays.end());
  observation.push_back(static_cast<float>(v_measured / max_lin_.value()));
  observation.push_back(static_cast<float>(w_measured / max_ang_.value()));
  observation.push_back(static_cast<float>(gyro / max_ang_.value()));
  observation.push_back(prev_action_[0]);
  observation.push_back(prev_action_[1]);
  if (policy_.inputs() >= static_cast<std::int64_t>(observation.size()) + 1) {
    observation.push_back(static_cast<float>(lidar_inflation_.value() / inflation_scale_.value()));
  }
  if (static_cast<std::int64_t>(observation.size()) != policy_.inputs()) {
    if (active_last_) {
      publish_zero_locked();
    }
    active_last_ = false;
    publish_status_locked(4, false, 0.0, 0.0, 0.0, 0.0, 0.0);
    return;
  }

  double inference_ms = 0.0;
  std::vector<float> action;
  try {
    const double started = now_seconds();
    action = policy_.run(observation);
    inference_ms = (now_seconds() - started) * 1000.0;
  } catch (const std::exception& exception) {
    policy_error_ = exception.what();
    if (active_last_) {
      publish_zero_locked();
    }
    active_last_ = false;
    publish_status_locked(4, false, 0.0, 0.0, 0.0, 0.0, inference_ms);
    return;
  }
  if (action.size() < 2) {
    if (active_last_) {
      publish_zero_locked();
    }
    active_last_ = false;
    publish_status_locked(4, false, 0.0, 0.0, 0.0, 0.0, inference_ms);
    return;
  }

  const float a0 = std::clamp(action[0], -1.0F, 1.0F);
  const float a1 = std::clamp(action[1], -1.0F, 1.0F);
  prev_action_ = {a0, a1};
  krosc::msgs::Twist command;
  command.linear.x = std::max(static_cast<double>(a0) * max_lin_.value(),
                              v_min_ratio_.value() * max_lin_.value());
  command.angular.z = static_cast<double>(a1) * max_ang_.value();
  publish("cmd_vel_rl", command, krosc::QOS_COMMAND);
  active_last_ = true;
  publish_status_locked(0, true, command.linear.x, command.angular.z, a0, a1, inference_ms);
}

}  // namespace krosc_ws
