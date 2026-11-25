#include "nodes/rl_nav_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <sstream>
#include <utility>

#include "krosc/math.hpp"
#include "krosc/nodes/nav2/controllers/dwb_local_planner.hpp"
#include "krosc/nodes/nav2/controllers/mppi_controller.hpp"
#include "krosc/nodes/nav2/controllers/regulated_pure_pursuit_controller.hpp"
#include "ml/model_catalog.hpp"

#ifndef KROSC_WS_MODELS_DIR
#define KROSC_WS_MODELS_DIR "artifacts/models"
#endif

namespace krosc_ws {

namespace {
constexpr double kPi = 3.14159265358979323846;

std::string number(double value) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return buffer;
}

std::string join(const std::vector<std::string>& values) {
  std::string text;
  for (const auto& value : values) {
    if (!text.empty()) {
      text += ",";
    }
    text += value;
  }
  return text.empty() ? "-" : text;
}

void add_value(krosc::msgs::DiagnosticStatus& status, const std::string& key,
               const std::string& value) {
  status.values.push_back(krosc::msgs::KeyValue{key, value});
}
}  // namespace

RlNavController::RlNavController() = default;

RlNavController::~RlNavController() {
  status_timer_.cancel();
  cancel_subscriptions();
}

void RlNavController::cancel_subscriptions() {
  scan_subscription_.cancel();
  imu_subscription_.cancel();
  enable_subscription_.cancel();
  fallback_subscription_.cancel();
  reload_subscription_.cancel();
  model_subscription_.cancel();
}

void RlNavController::configure(krosc::Node& node, const std::string& name) {
  NavControllerPlugin::configure(node, name);
  const auto positive = [](double value) { return value > 0.0; };
  const auto non_negative = [](double value) { return value >= 0.0; };
  const auto prefix = [&name](const char* suffix) { return name + "." + suffix; };

  enable_ = node_->declare_parameter<bool>(prefix("enable"), false);
  model_path_ = node_->declare_parameter<std::string>(prefix("model_path"), "");
  models_directory_ = node_->declare_parameter<std::string>(
      prefix("models_directory"), std::string(KROSC_WS_MODELS_DIR) + "/rl_nav_controller");
  model_ = node_->declare_parameter<std::string>(prefix("model"), "rl_maze");
  fallback_controller_ = node_->declare_parameter<std::string>(
      prefix("fallback_controller"), "rpp",
      [](const std::string& value) { return value == "mppi" || value == "dwb" || value == "rpp"; });
  max_lin_ = node_->declare_parameter<double>(prefix("max_lin"), 0.26, positive);
  max_ang_ = node_->declare_parameter<double>(prefix("max_ang"), 1.82, positive);
  v_min_ratio_ = node_->declare_parameter<double>(prefix("v_min_ratio"), -0.2,
                                                  [](double value) { return value <= 0.0; });
  lookahead_ = node_->declare_parameter<std::int64_t>(prefix("lookahead"), 5,
                                                      [](std::int64_t value) { return value > 0; });
  wp_step_ = node_->declare_parameter<double>(prefix("wp_step"), 0.30, positive);
  wp_scale_ = node_->declare_parameter<double>(prefix("wp_scale"), 2.0, positive);
  lidar_rays_ = node_->declare_parameter<std::int64_t>(prefix("lidar_rays"), 36,
                                                       [](std::int64_t value) { return value > 0; });
  lidar_max_ = node_->declare_parameter<double>(prefix("lidar_max"), 3.5, positive);
  lidar_inflation_ = node_->declare_parameter<double>(prefix("lidar_inflation"), 0.22, non_negative);
  inflation_scale_ = node_->declare_parameter<double>(prefix("inflation_scale"), 0.5, positive);
  scan_timeout_ = node_->declare_parameter<double>(prefix("scan_timeout"), 0.35, positive);
  base_frame_ = node_->declare_parameter<std::string>(prefix("base_frame"), "base_footprint");
  scan_topic_ = node_->declare_parameter<std::string>(prefix("scan_topic"), "scan");
  imu_topic_ = node_->declare_parameter<std::string>(prefix("imu_topic"), "imu");
  diagnostics_period_ = node_->declare_parameter<double>(prefix("diagnostics_period"), 1.0, positive);

  listener_ = std::make_unique<krosc::TransformListener>(buffer_, *node_);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!set_fallback_locked(fallback_controller_.value())) {
    node_->logger().warn("RL fallback '%s'를 만들지 못했다", fallback_controller_.value().c_str());
    set_fallback_locked("rpp");
  }
  refresh_models_locked();
  publish_model_list_locked();

  std::string requested = model_.value();
  if (!model_path_.value().empty()) {
    requested = std::filesystem::path(model_path_.value()).parent_path().filename().string();
  } else if (std::find(model_names_.begin(), model_names_.end(), requested) == model_names_.end() &&
             !model_names_.empty()) {
    requested = model_names_.front();
  }
  std::string error;
  if (!requested.empty() && !load_policy_locked(requested, error)) {
    if (enable_.value()) {
      node_->logger().warn("RL policy 없음: %s", error.c_str());
    } else {
      node_->logger().info("RL policy 없음 (RL OFF): %s", error.c_str());
    }
  }

  scan_subscription_ = node_->subscribe<krosc::msgs::LaserScan>(
      scan_topic_.value(), [this](const auto& message) { on_scan(message); },
      krosc::QOS_SENSOR_DATA);
  imu_subscription_ = node_->subscribe<krosc::msgs::Imu>(
      imu_topic_.value(), [this](const auto& message) { on_imu(message); }, krosc::QOS_SENSOR_DATA);
  enable_subscription_ = node_->subscribe<krosc::msgs::Bool>(
      "rl_nav_enable", [this](const auto& message) { on_enable(message); }, krosc::QOS_DEFAULT);
  fallback_subscription_ = node_->subscribe<krosc::msgs::String>(
      "rl_nav_fallback", [this](const auto& message) { on_fallback(message); }, krosc::QOS_DEFAULT);
  reload_subscription_ = node_->subscribe<krosc::msgs::Empty>(
      "rl_nav_reload", [this](const auto& message) { on_reload(message); }, krosc::QOS_DEFAULT);
  model_subscription_ = node_->subscribe<krosc::msgs::String>(
      "rl_nav_model", [this](const auto& message) { on_model(message); }, krosc::QOS_DEFAULT);
  last_model_list_ = node_->now();
  publish_status_locked(Velocity{}, 2, false, 0.0);
  // Viz가 늦게 붙어도(특히 시작 직후) 상태/모델 목록을 받도록 주기적으로 다시 낸다.
  // 주행 중에는 compute가 0.2s마다 상태를 내므로 이 타이머는 쉰다.
  status_timer_ = node_->create_timer(0.5, [this] {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = node_->now();
    if (now - last_status_ >= 1.0) {
      const int reason = !enable_.value() ? 2 : (policy_.loaded() ? 0 : 1);
      publish_status_locked(Velocity{}, reason, false, 0.0);
    }
    if (now - last_model_list_ >= 2.0) {
      last_model_list_ = now;
      refresh_models_locked();  // 폴더를 새로 추가해도 재시작 없이 목록에 뜨게
      publish_model_list_locked();
    }
  });
  node_->logger().info(
      "RlNavController plugin: enable=%d fallback=%s model=%s inputs=%lld onnxruntime=%d provider=%s",
      enable_.value(), fallback_controller_.value().c_str(), current_model_.c_str(),
      static_cast<long long>(policy_.inputs()), ml::OnnxPolicy::runtime_available() ? 1 : 0,
      policy_.provider().c_str());
}

bool RlNavController::set_fallback_locked(const std::string& kind) {
  std::unique_ptr<NavControllerPlugin> inner;
  if (kind == "mppi") {
    inner = std::make_unique<krosc::nodes::nav2::MPPIController>();
  } else if (kind == "dwb") {
    inner = std::make_unique<krosc::nodes::nav2::DWBLocalPlanner>();
  } else if (kind == "rpp") {
    inner = std::make_unique<krosc::nodes::nav2::RegulatedPurePursuitController>();
  }
  if (!inner) {
    return false;
  }
  // 같은 종류로 다시 전환해도 파라미터 재선언 예외가 나지 않게 인스턴스별 이름을 쓴다.
  inner->configure(*node_, name_ + ".inner_" + kind + "_" + std::to_string(++fallback_instance_));
  fallback_ = std::move(inner);
  return true;
}

void RlNavController::on_scan(const krosc::msgs::LaserScan& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  scan_ = std::make_shared<const krosc::msgs::LaserScan>(message);
  scan_time_ = node_->now();
}

void RlNavController::on_imu(const krosc::msgs::Imu& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  gyro_ = message.angular_velocity.z;
}

void RlNavController::on_enable(const krosc::msgs::Bool& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  enable_.set(message.data);
  prev_action_ = {0.0F, 0.0F};
  active_last_ = false;
  publish_status_locked(Velocity{}, enable_.value() ? 0 : 2, false, 0.0);
}

void RlNavController::on_fallback(const krosc::msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!set_fallback_locked(message.data)) {
    node_->logger().warn("RL fallback 이름이 잘못됐다: %s", message.data.c_str());
    return;
  }
  fallback_controller_.set(message.data);
  active_last_ = false;
  publish_status_locked(Velocity{}, enable_.value() ? 0 : 2, false, 0.0);
}

void RlNavController::on_reload(const krosc::msgs::Empty&) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string target = current_model_.empty() ? model_.value() : current_model_;
  std::string error;
  if (!load_policy_locked(target, error)) {
    node_->logger().warn("RL policy reload 실패: %s", error.c_str());
  }
  prev_action_ = {0.0F, 0.0F};
  active_last_ = false;
  publish_status_locked(Velocity{}, policy_.loaded() ? (enable_.value() ? 0 : 2) : 1, false, 0.0);
}

void RlNavController::on_model(const krosc::msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!model_path_.value().empty()) {
    node_->logger().warn("RL model 선택 무시: model_path=%s가 우선한다", model_path_.value().c_str());
    return;
  }
  if (!ml::safe_model_name(message.data)) {
    node_->logger().warn("RL model 이름이 잘못됐다: %s", message.data.c_str());
    return;
  }
  std::string error;
  if (!load_policy_locked(message.data, error)) {
    node_->logger().warn("RL model 전환 실패 (%s): %s", message.data.c_str(), error.c_str());
  }
  prev_action_ = {0.0F, 0.0F};
  active_last_ = false;
  publish_status_locked(Velocity{}, policy_.loaded() ? (enable_.value() ? 0 : 2) : 1, false, 0.0);
  refresh_models_locked();
}

void RlNavController::refresh_models_locked() {
  std::vector<std::string> found = ml::list_models(models_directory_.value());
  if (found != model_names_) {
    model_names_ = std::move(found);
    publish_model_list_locked();
  }
}

void RlNavController::publish_model_list_locked() {
  std::ostringstream text;
  for (std::size_t index = 0; index < model_names_.size(); ++index) {
    if (index != 0) {
      text << '\n';
    }
    text << model_names_[index];
  }
  node_->publish("rl_nav_model_list", krosc::msgs::String{text.str()}, krosc::QOS_LATCHED);
}

bool RlNavController::load_policy_locked(const std::string& model, std::string& error) {
  policy_error_.clear();
  const std::string path = model_path_.value().empty()
                               ? ml::model_file(models_directory_.value(), model)
                               : model_path_.value();
  if (path.empty()) {
    error = "model not found: " + model + " (" + models_directory_.value() + ")";
    policy_error_ = error;
    return false;
  }
  try {
    ml::OnnxPolicy candidate;
    candidate.load(path);
    // 관측 = waypoint(2*lookahead) + ray(모델마다 다름) + [v, w, gyro, a0, a1] + (선택) inflation.
    // 정책 입력 크기에서 ray 수를 역산해 Isaac(360 rays)과 기존 rl_maze(36 rays)를 모두 지원한다.
    const std::int64_t look = lookahead_.value();
    const std::int64_t configured = lidar_rays_.value();
    const std::int64_t inputs = candidate.inputs();
    std::int64_t rays = 0;
    bool inflation = false;
    if (inputs == 2 * look + configured + 5) {
      rays = configured;
    } else if (inputs == 2 * look + configured + 6) {
      rays = configured;
      inflation = true;
    } else {
      rays = inputs - 2 * look - 5;
    }
    if (rays < 8 || rays > 2048 || candidate.outputs() != 2) {
      std::ostringstream stream;
      stream << "정책 입출력이 맞지 않는다: input=" << inputs << " output=" << candidate.outputs()
             << " (ray " << rays << " → 2)";
      error = stream.str();
      policy_error_ = error;
      return false;
    }
    policy_ = std::move(candidate);
    active_rays_ = rays;
    inflation_feature_ = inflation;
    current_model_ = model;
    node_->logger().info("RL ONNX loaded: %s (obs %lld = %lld rays%s → %lld)", path.c_str(),
                  static_cast<long long>(policy_.inputs()), static_cast<long long>(active_rays_),
                  inflation_feature_ ? " + inflation" : "",
                  static_cast<long long>(policy_.outputs()));
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    policy_error_ = error;
    return false;
  }
}

double RlNavController::laser_yaw() {
  if (laser_yaw_) {
    return *laser_yaw_;
  }
  if (!scan_) {
    return 0.0;
  }
  try {
    const auto transform = buffer_.lookup_transform(base_frame_.value(), scan_->header.frame_id);
    laser_yaw_ = krosc::tfmath::yaw_of(transform.transform.rotation);
    return *laser_yaw_;
  } catch (const krosc::TransformException&) {
    return 0.0;
  }
}

double RlNavController::scan_min() const {
  double best = lidar_max_.value();
  if (!scan_) {
    return -1.0;
  }
  for (float range : scan_->ranges) {
    if (std::isfinite(range) && range >= scan_->range_min && range <= scan_->range_max) {
      best = std::min(best, static_cast<double>(range));
    }
  }
  return best;
}

std::vector<float> RlNavController::waypoints(const Pose2& pose,
                                              const std::vector<Pose2>& path) const {
  std::vector<float> out;
  const std::int64_t count = lookahead_.value();
  const double step = wp_step_.value();
  const double scale = wp_scale_.value();
  if (path.empty()) {
    return out;
  }
  const auto emit = [&](double x, double y) {
    const double dx = x - pose.x;
    const double dy = y - pose.y;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    out.push_back(static_cast<float>((c * dx + s * dy) / scale));
    out.push_back(static_cast<float>((-s * dx + c * dy) / scale));
  };
  double accumulated = 0.0;
  double wanted = step;
  std::size_t index = krosc::nodes::nav2::closest_index(pose, path);
  while (out.size() < static_cast<std::size_t>(2 * count) && index + 1 < path.size()) {
    const auto& a = path[index];
    const auto& b = path[index + 1];
    const double segment = std::hypot(b.x - a.x, b.y - a.y);
    while (out.size() < static_cast<std::size_t>(2 * count) && accumulated + segment >= wanted) {
      const double u = segment > 1e-9 ? (wanted - accumulated) / segment : 0.0;
      emit(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y));
      wanted += step;
    }
    accumulated += segment;
    ++index;
  }
  while (out.size() < static_cast<std::size_t>(2 * count)) {
    emit(path.back().x, path.back().y);
  }
  return out;
}

std::vector<float> RlNavController::scan_rays(const krosc::msgs::LaserScan& scan, double laser_yaw,
                                               double inflation) const {
  // 모델 입력 크기에 맞춰 ray 수를 정한다. 파라미터(기본 36)와 다르면 로드 시 계산된 값을 쓴다.
  const std::int64_t rays = active_rays_ > 0 ? active_rays_ : lidar_rays_.value();
  const double max_range = lidar_max_.value();
  std::vector<float> out;
  out.reserve(static_cast<std::size_t>(rays));
  const std::size_t n = scan.ranges.size();
  if (n == 0 || scan.angle_increment == 0.0) {
    out.assign(static_cast<std::size_t>(rays), 1.0F);
    return out;
  }
  for (std::int64_t r = 0; r < rays; ++r) {
    const double angle = (2.0 * kPi * static_cast<double>(r) / static_cast<double>(rays) - laser_yaw -
                          scan.angle_min) /
                         scan.angle_increment;
    const long long index = std::llround(angle);
    double best = max_range;
    for (long long offset = -1; offset <= 1; ++offset) {
      const long long slot = ((index + offset) % static_cast<long long>(n) + static_cast<long long>(n)) %
                             static_cast<long long>(n);
      const double value = scan.ranges[static_cast<std::size_t>(slot)];
      if (std::isfinite(value) && value >= scan.range_min && value <= scan.range_max) {
        best = std::min(best, value);
      }
    }
    best = std::min(std::max(best - std::max(0.0, inflation), 0.0), max_range);
    out.push_back(static_cast<float>(best / max_range));
  }
  return out;
}

void RlNavController::publish_status_locked(const Velocity& command, int reason, bool active,
                                            double inference_ms) {
  if (node_ == nullptr) {
    return;
  }
  const double current = node_->now();
  const bool scan_ok = scan_ != nullptr;
  const double scan_age = scan_ok ? current - scan_time_ : -1.0;
  const double minimum = scan_ok ? scan_min() : -1.0;
  const std::string version = ml::OnnxPolicy::runtime_version();

  if (current - last_status_ >= 0.2 || (reason == 2 && active_last_)) {
    last_status_ = current;
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << "enable=" << (enable_.value() ? 1 : 0)
           << ";fallback=" << fallback_controller_.value()
           << ";model=" << (current_model_.empty() ? "-" : current_model_)
           << ";policy=" << (policy_.loaded() ? 1 : 0)
           << ";rays=" << (active_rays_ > 0 ? active_rays_ : lidar_rays_.value())
           << ";scan=" << (scan_ok ? 1 : 0)
           << ";scan_age=" << scan_age << ";scan_min=" << minimum
           << ";odom=1;plan=1"
           << ";v=" << command.linear << ";w=" << command.angular << ";a0=" << prev_action_[0]
           << ";a1=" << prev_action_[1] << ";infer_ms=" << inference_ms << ";reason=" << reason
           << ";active=" << (active ? 1 : 0)
           << ";onnxruntime=" << (ml::OnnxPolicy::runtime_available() ? 1 : 0)
           << ";provider=" << (policy_.provider().empty() ? "-" : policy_.provider())
           << ";providers=" << join(ml::OnnxPolicy::available_providers())
           << ";ort_version=" << (version.empty() ? "-" : version);
    if (!policy_.provider_error().empty()) {
      stream << ";provider_error=" << policy_.provider_error();
    }
    if (!policy_error_.empty()) {
      stream << ";error=" << policy_error_;
    }
    node_->publish("rl_nav_status", krosc::msgs::String{stream.str()}, krosc::QOS_LATCHED);
  }

  if (current - last_diagnostics_ >= diagnostics_period_.value()) {
    last_diagnostics_ = current;
    publish_diagnostics_locked(command, reason, active, inference_ms);
  }
  active_last_ = active;
}

void RlNavController::publish_diagnostics_locked(const Velocity& command, int reason, bool active,
                                                 double inference_ms) {
  if (node_ == nullptr) {
    return;
  }
  krosc::msgs::DiagnosticArray array;
  array.header.stamp = node_->now();
  krosc::msgs::DiagnosticStatus status;
  status.name = node_->name() + ": RL Navigation";
  status.hardware_id = "rl_nav_controller";

  const bool runtime = ml::OnnxPolicy::runtime_available();
  const bool enabled = enable_.value();
  const bool loaded = policy_.loaded();
  if (!runtime) {
    status.level = enabled ? 1 : 0;
    status.message = enabled ? "ONNX Runtime 없이 빌드됨 — fallback " + fallback_controller_.value()
                             : "ONNX Runtime 없음 (RL OFF)";
  } else if (!enabled) {
    status.level = 0;
    status.message = "disabled";
  } else if (!loaded) {
    status.level = 1;
    status.message = "모델/정책 없음 — fallback " + fallback_controller_.value();
  } else if (!active && reason != 0) {
    status.level = 1;
    status.message = "fallback " + fallback_controller_.value() + " (reason " +
                     std::to_string(reason) + ")";
  } else {
    status.level = 0;
    status.message = active ? "RL active" : "ready (no command yet)";
  }

  const bool scan_ok = scan_ != nullptr;
  add_value(status, "onnxruntime", runtime ? "true" : "false");
  add_value(status, "onnxruntime_version",
            ml::OnnxPolicy::runtime_version().empty() ? "-" : ml::OnnxPolicy::runtime_version());
  add_value(status, "provider", policy_.provider().empty() ? "-" : policy_.provider());
  add_value(status, "providers", join(ml::OnnxPolicy::available_providers()));
  if (!policy_.provider_error().empty()) {
    add_value(status, "provider_error", policy_.provider_error());
  }
  add_value(status, "model", current_model_.empty() ? "-" : current_model_);
  add_value(status, "policy", loaded ? "loaded" : "none");
  add_value(status, "enable", enabled ? "true" : "false");
  add_value(status, "fallback", fallback_controller_.value());
  add_value(status, "active", active ? "true" : "false");
  add_value(status, "reason", std::to_string(reason));
  add_value(status, "infer_ms", number(inference_ms));
  add_value(status, "scan", scan_ok ? "true" : "false");
  add_value(status, "scan_age", number(scan_ok ? node_->now() - scan_time_ : -1.0));
  add_value(status, "v", number(command.linear));
  add_value(status, "w", number(command.angular));
  if (!policy_error_.empty()) {
    add_value(status, "error", policy_error_);
  }
  array.status.push_back(std::move(status));
  node_->publish("diagnostics", std::move(array), krosc::QOS_DEFAULT);
}

RlNavController::Velocity RlNavController::compute_velocity_commands(
    const Pose2& pose, const Velocity& velocity, const std::vector<Pose2>& path,
    const Costmap& costmap, double dt) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double current = node_->now();
  const bool scan_ok = scan_ != nullptr && current - scan_time_ <= scan_timeout_.value();
  if (!enable_.value() || !policy_.loaded() || !scan_ok || path.empty()) {
    int reason = 2;
    if (enable_.value()) {
      if (!policy_.loaded()) {
        reason = 1;
      } else if (!scan_ok) {
        reason = 3;
      } else {
        reason = 6;
      }
    }
    if (!fallback_) {
      throw krosc::nodes::nav2::ControllerError(
          krosc::msgs::nav2::FollowPathResult::NO_VALID_CONTROL, "RL fallback이 없다");
    }
    const Velocity command = fallback_->compute_velocity_commands(pose, velocity, path, costmap, dt);
    local_plan_ = fallback_->local_plan();
    publish_status_locked(command, reason, false, 0.0);
    return command;
  }

  const double inflation = lidar_inflation_.value();
  std::vector<float> observation = waypoints(pose, path);
  const auto rays = scan_rays(*scan_, laser_yaw(), inflation);
  observation.insert(observation.end(), rays.begin(), rays.end());
  const double max_linear = max_lin_.value();
  const double max_angular = max_ang_.value();
  const double gyro = gyro_.value_or(velocity.angular);
  observation.push_back(static_cast<float>(velocity.linear / max_linear));
  observation.push_back(static_cast<float>(velocity.angular / max_angular));
  observation.push_back(static_cast<float>(gyro / max_angular));
  observation.push_back(prev_action_[0]);
  observation.push_back(prev_action_[1]);
  if (inflation_feature_) {
    observation.push_back(static_cast<float>(inflation / inflation_scale_.value()));
  }
  if (static_cast<std::int64_t>(observation.size()) != policy_.inputs()) {
    policy_error_ = "관측 크기가 정책과 다르다: " + std::to_string(observation.size()) + " != " +
                    std::to_string(policy_.inputs());
    const Velocity command = fallback_->compute_velocity_commands(pose, velocity, path, costmap, dt);
    local_plan_ = fallback_->local_plan();
    publish_status_locked(command, 4, false, 0.0);
    return command;
  }
  const auto started = std::chrono::steady_clock::now();
  std::vector<float> action;
  try {
    action = policy_.run(observation);
  } catch (const std::exception& exception) {
    policy_error_ = exception.what();
    const Velocity command = fallback_->compute_velocity_commands(pose, velocity, path, costmap, dt);
    local_plan_ = fallback_->local_plan();
    publish_status_locked(command, 4, false, 0.0);
    return command;
  }
  const double inference_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  if (action.size() < 2) {
    policy_error_ = "정책 출력이 2개보다 작다";
    const Velocity command = fallback_->compute_velocity_commands(pose, velocity, path, costmap, dt);
    local_plan_ = fallback_->local_plan();
    publish_status_locked(command, 4, false, inference_ms);
    return command;
  }
  const float a0 = std::clamp(action[0], -1.0F, 1.0F);
  const float a1 = std::clamp(action[1], -1.0F, 1.0F);
  prev_action_ = {a0, a1};
  const double linear = std::max(static_cast<double>(a0) * max_linear,
                                 v_min_ratio_.value() * max_linear);
  const double angular = static_cast<double>(a1) * max_angular;
  local_plan_ = krosc::nodes::nav2::simulate(pose, linear, angular, 1.0, 0.1);
  const Velocity command{linear, angular};
  publish_status_locked(command, 0, true, inference_ms);
  return command;
}

}  // namespace krosc_ws
