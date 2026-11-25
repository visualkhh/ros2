#include "krosc/nodes/collision_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "krosc/qos.hpp"

namespace krosc::nodes {

namespace {

std::pair<std::vector<double>, std::vector<double>> default_polygon(const std::string& name) {
  if (name == "PolygonStop") {
    return {{0.3, 0.15, 0.3, -0.15, 0.0, -0.15, 0.0, 0.15}, {}};
  }
  if (name == "PolygonSlow") {
    return {{0.5, 0.2, 0.5, -0.2, 0.0, -0.2, 0.0, 0.2}, {}};
  }
  return {{}, {}};
}

bool valid_polygon_points(const std::vector<double>& points) {
  return points.size() >= 6 && points.size() % 2 == 0;
}

CollisionMonitor::Action parse_action(const std::string& action) {
  if (action == "slowdown") {
    return CollisionMonitor::Action::Slowdown;
  }
  if (action == "limit") {
    return CollisionMonitor::Action::Limit;
  }
  if (action == "approach") {
    return CollisionMonitor::Action::Approach;
  }
  return CollisionMonitor::Action::Stop;
}

}  // namespace

bool CollisionMonitor::Polygon::contains(double x, double y) const {
  if (type == "circle") {
    return std::hypot(x, y) <= radius;
  }
  bool hit = false;
  if (points.size() < 3) {
    return false;
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& [x1, y1] = points[i];
    const auto& [x2, y2] = points[(i + 1) % points.size()];
    if ((y1 > y) != (y2 > y) && x < x1 + (y - y1) * (x2 - x1) / (y2 - y1)) {
      hit = !hit;
    }
  }
  return hit;
}

bool CollisionMonitor::Polygon::hits(const std::vector<std::pair<double, double>>& points) const {
  std::int64_t count = 0;
  for (const auto& [x, y] : points) {
    if (contains(x, y)) {
      ++count;
    }
  }
  return count >= min_points;
}

std::pair<double, double> CollisionMonitor::Polygon::apply(
    double linear, double angular, const std::vector<std::pair<double, double>>& points) const {
  if (!hits(points)) {
    return {linear, angular};
  }
  if (action == Action::Stop) {
    return {0.0, 0.0};
  }
  if (action == Action::Slowdown) {
    return {linear * slowdown_ratio, angular * slowdown_ratio};
  }
  if (action == Action::Limit) {
    return {std::clamp(linear, -linear_limit, linear_limit),
            std::clamp(angular, -angular_limit, angular_limit)};
  }
  return {linear * slowdown_ratio, angular * slowdown_ratio};  // approach: 최소 감속으로 처리
}

CollisionMonitor::CollisionMonitor(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      base_frame_id_(declare_parameter<std::string>("base_frame_id", "base_footprint")),
      cmd_vel_in_topic_(declare_parameter<std::string>("cmd_vel_in_topic", "cmd_vel_mux")),
      cmd_vel_out_topic_(declare_parameter<std::string>("cmd_vel_out_topic", "cmd_vel")),
      teleop_topic_(declare_parameter<std::string>("teleop_topic", "cmd_vel_teleop")),
      status_topic_(declare_parameter<std::string>("status_topic", "collision_status")),
      source_timeout_(declare_parameter<double>("source_timeout", 2.0, [](double value) { return value > 0.0; })),
      teleop_timeout_(declare_parameter<double>("teleop_timeout", 0.5, [](double value) { return value > 0.0; })),
      max_observation_range_(
          declare_parameter<double>("max_observation_range", 1.5, [](double value) { return value > 0.0; })),
      reverse_escape_ratio_(declare_parameter<double>("reverse_escape_ratio", 0.3,
                                                     [](double value) { return value >= 0.0 && value <= 1.0; })),
      rotation_clearance_(
          declare_parameter<double>("rotation_clearance", 0.22, [](double value) { return value >= 0.0; })),
      rotation_escape_ratio_(declare_parameter<double>("rotation_escape_ratio", 0.5,
                                                      [](double value) { return value >= 0.0 && value <= 1.0; })),
      enabled_(declare_parameter<bool>(
          "enabled", true, {},
          [this](const bool& value) { on_enabled(value); })),
      ignore_collision_during_teleop_(declare_parameter<bool>("ignore_collision_during_teleop", false)),
      polygon_names_(declare_parameter<std::vector<std::string>>("polygons",
                                                                 std::vector<std::string>{"PolygonStop", "PolygonSlow"})),
      observation_sources_(declare_parameter<std::vector<std::string>>("observation_sources",
                                                                       std::vector<std::string>{"scan"})) {
  for (const auto& polygon_name : polygon_names_.value()) {
    const auto [default_points, unused] = default_polygon(polygon_name);
    (void)unused;
    const bool polygon_type = polygon_name == "PolygonStop" || polygon_name == "PolygonSlow";
    Polygon polygon;
    polygon.name = polygon_name;
    polygon.type = declare_parameter<std::string>(
        polygon_name + ".type", polygon_type ? "polygon" : "circle",
        [](const std::string& value) { return value == "polygon" || value == "circle"; });
    const std::vector<double> fallback = polygon_type ? default_points : std::vector<double>{};
    const auto points = declare_parameter<std::vector<double>>(
        polygon_name + ".points", fallback,
        [](const std::vector<double>& value) { return value.empty() || valid_polygon_points(value); });
    for (std::size_t i = 0; i + 1 < points.value().size(); i += 2) {
      polygon.points.emplace_back(points.value()[i], points.value()[i + 1]);
    }
    polygon.radius = declare_parameter<double>(polygon_name + ".radius", 0.3,
                                               [](double value) { return value > 0.0; });
    const std::string default_action = polygon_type ? (polygon_name == "PolygonStop" ? "stop" : "slowdown")
                                                    : "stop";
    const auto action = declare_parameter<std::string>(
        polygon_name + ".action_type", default_action,
        [](const std::string& value) {
          return value == "stop" || value == "slowdown" || value == "limit" || value == "approach";
        });
    polygon.action = parse_action(action.value());
    polygon.min_points = declare_parameter<std::int64_t>(
        polygon_name + ".min_points", 4, [](std::int64_t value) { return value > 0; });
    polygon.slowdown_ratio = declare_parameter<double>(
        polygon_name + ".slowdown_ratio", 0.3, [](double value) { return value >= 0.0 && value <= 1.0; });
    polygon.linear_limit = declare_parameter<double>(
        polygon_name + ".linear_limit", 0.1, [](double value) { return value >= 0.0; });
    polygon.angular_limit = declare_parameter<double>(
        polygon_name + ".angular_limit", 0.5, [](double value) { return value >= 0.0; });
    polygon.enabled = declare_parameter<bool>(polygon_name + ".enabled", true);
    polygons_.push_back(std::move(polygon));
  }

  listener_ = std::make_unique<TransformListener>(buffer_, *this);
  for (const auto& source : observation_sources_.value()) {
    const auto topic = declare_parameter<std::string>(source + ".topic", source);
    scan_subscriptions_.push_back(
        subscribe<msgs::LaserScan>(topic.value(),
                                   [this, source](const msgs::LaserScan& scan) { on_scan(source, scan); },
                                   QOS_SENSOR_DATA));
  }
  command_subscription_ = subscribe<msgs::Twist>(
      cmd_vel_in_topic_.value(), [this](const msgs::Twist& twist) { on_cmd_vel(twist); }, QOS_COMMAND);
  teleop_subscription_ = subscribe<msgs::Twist>(
      teleop_topic_.value(), [this](const msgs::Twist& twist) { on_teleop(twist); }, QOS_COMMAND);
  publish_status();
  logger().info("CollisionMonitor: in=%s out=%s base=%s polygons=%zu enabled=%s",
                cmd_vel_in_topic_.value().c_str(), cmd_vel_out_topic_.value().c_str(),
                base_frame_id_.value().c_str(), polygons_.size(), enabled_.value() ? "true" : "false");
}

void CollisionMonitor::on_scan(const std::string& source, const msgs::LaserScan& scan) {
  if (!enabled_.value()) {
    return;
  }
  msgs::TransformStamped transform;
  try {
    transform = buffer_.lookup_transform(base_frame_id_.value(), scan.header.frame_id);
  } catch (const TransformException&) {
    return;
  }
  const double yaw = tfmath::yaw_of(transform.transform.rotation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double min_range = std::max<double>(scan.range_min, 0.0);
  const double max_range = std::min<double>(scan.range_max, max_observation_range_.value());
  std::vector<std::pair<double, double>> points;
  points.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(range) || range < min_range || range > max_range) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double px = range * std::cos(angle);
    const double py = range * std::sin(angle);
    points.emplace_back(transform.transform.translation.x + c * px - s * py,
                        transform.transform.translation.y + s * px + c * py);
  }
  const double stamp = scan.header.stamp > 0.0 ? scan.header.stamp : now();
  std::lock_guard<std::mutex> lock(mutex_);
  observations_[source] = Observation{stamp, std::move(points)};
}

std::pair<double, double> CollisionMonitor::check(double linear, double angular) {
  const double current = now();
  std::map<std::string, Observation> observations;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    observations = observations_;
  }
  if (observations.size() < observation_sources_.value().size()) {
    state_ = "no fresh observation: stop";
    return {0.0, 0.0};
  }
  for (const auto& [source, observation] : observations) {
    (void)source;
    if (current - observation.stamp > source_timeout_.value()) {
      state_ = "no fresh observation: stop";
      return {0.0, 0.0};
    }
  }
  std::vector<std::pair<double, double>> points;
  for (const auto& [source, observation] : observations) {
    (void)source;
    points.insert(points.end(), observation.points.begin(), observation.points.end());
  }

  std::pair<double, double> best{linear, angular};
  state_.clear();
  for (const auto& polygon : polygons_) {
    if (!polygon.enabled) {
      continue;
    }
    std::pair<double, double> result = polygon.apply(linear, angular, points);
    bool front = false;
    for (const auto& [x, y] : polygon.points) {
      (void)y;
      if (x > 0.0) {
        front = true;
        break;
      }
    }
    if (polygon.action == Action::Stop && result.first == 0.0 && result.second == 0.0) {
      if (linear < 0.0 && front) {
        result = {linear * reverse_escape_ratio_.value(), angular * reverse_escape_ratio_.value()};
      } else if (std::abs(linear) < 1e-6 && angular != 0.0) {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto& [x, y] : points) {
          nearest = std::min(nearest, std::hypot(x, y));
        }
        if (nearest >= rotation_clearance_.value()) {
          result = {0.0, angular * rotation_escape_ratio_.value()};
        }
      }
    }
    if (std::abs(result.first) < std::abs(best.first) ||
        (std::abs(result.first) == std::abs(best.first) && std::abs(result.second) < std::abs(best.second))) {
      best = result;
      state_ = polygon.name + ": " + (polygon.action == Action::Stop ? "stop" : "slowdown");
    }
  }
  return best;
}

void CollisionMonitor::on_cmd_vel(const msgs::Twist& twist) {
  if (!enabled_.value()) {
    state_ = "disabled";
    publish(cmd_vel_out_topic_.value(), twist, QOS_COMMAND);
    publish_status();
    return;
  }
  if (ignore_collision_during_teleop_.value() && now() - last_teleop_ <= teleop_timeout_.value()) {
    state_ = "teleop: collision check off";
    publish(cmd_vel_out_topic_.value(), twist, QOS_COMMAND);
    publish_status();
    return;
  }
  const auto [linear, angular] = check(twist.linear.x, twist.angular.z);
  msgs::Twist output;
  output.linear.x = linear;
  output.angular.z = angular;
  publish(cmd_vel_out_topic_.value(), output, QOS_COMMAND);
  publish_status();
}

void CollisionMonitor::on_teleop(const msgs::Twist& twist) {
  (void)twist;
  last_teleop_ = now();
}

void CollisionMonitor::on_enabled(bool enabled) {
  state_ = enabled ? "" : "disabled";
  publish_status();
  logger().info("collision monitor: %s", enabled ? "enabled" : "disabled");
}

void CollisionMonitor::publish_status() {
  std::string text = enabled_.value() ? "enabled" : "disabled";
  if (enabled_.value() && !state_.empty()) {
    text += " | " + state_;
  }
  if (text == last_status_) {
    return;
  }
  last_status_ = text;
  publish(status_topic_.value(), msgs::String{text}, QOS_LATCHED);
}

}  // namespace krosc::nodes
