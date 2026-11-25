#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include "krosc/tf/math.hpp"
#include "krosc/qos.hpp"

namespace krosc::nodes::nav2 {

namespace {

bool valid_layer(const std::string& name) {
  return name == "static_layer" || name == "obstacle_layer" || name == "inflation_layer";
}

void line_cell(int x, int y, int x1, int y1, std::vector<std::pair<int, int>>& out) {
  const int dx = std::abs(x1 - x);
  const int dy = -std::abs(y1 - y);
  const int sx = x < x1 ? 1 : -1;
  const int sy = y < y1 ? 1 : -1;
  int error = dx + dy;
  while (x != x1 || y != y1) {
    out.emplace_back(x, y);
    const int e2 = 2 * error;
    if (e2 >= dy) {
      error += dy;
      x += sx;
    }
    if (e2 <= dx) {
      error += dx;
      y += sy;
    }
  }
}

}  // namespace

Costmap2DROS::Costmap2DROS(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      global_frame_(declare_parameter<std::string>("global_frame", "map")),
      robot_base_frame_(declare_parameter<std::string>("robot_base_frame", "base_footprint")),
      plugins_(declare_parameter<std::vector<std::string>>(
          "plugins", {"static_layer", "inflation_layer"},
          [](const std::vector<std::string>& values) {
            return std::all_of(values.begin(), values.end(), valid_layer);
          })),
      robot_radius_(declare_parameter<double>("robot_radius", 0.1, [](double value) { return value > 0.0; })),
      inflation_radius_(declare_parameter<double>("inflation_radius", 0.55,
                                                  [](double value) { return value > 0.0; })),
      cost_scaling_factor_(declare_parameter<double>("cost_scaling_factor", 3.0,
                                                     [](double value) { return value > 0.0; })),
      track_unknown_space_(declare_parameter<bool>("track_unknown_space", true)),
      map_topic_(declare_parameter<std::string>("map_topic", "map")),
      rolling_window_(declare_parameter<bool>("rolling_window", false)),
      width_(declare_parameter<double>("width", 3.0, [](double value) { return value > 0.0; })),
      height_(declare_parameter<double>("height", 3.0, [](double value) { return value > 0.0; })),
      resolution_(declare_parameter<double>("resolution", 0.05, [](double value) { return value > 0.0; })),
      update_frequency_(declare_parameter<double>("update_frequency", 5.0,
                                                  [](double value) { return value > 0.0; })),
      scan_period_(declare_parameter<double>("scan_period", 0.0, [](double value) { return value >= 0.0; })),
      observation_sources_(declare_parameter<std::vector<std::string>>("observation_sources", {"scan"})),
      obstacle_max_range_(declare_parameter<double>("obstacle_max_range", 2.5,
                                                    [](double value) { return value > 0.0; })),
      raytrace_max_range_(declare_parameter<double>("raytrace_max_range", 3.0,
                                                   [](double value) { return value > 0.0; })) {
  listener_ = std::make_unique<TransformListener>(buffer_, *this);
  if (has_layer("static_layer")) {
    map_subscription_ = subscribe<msgs::OccupancyGrid>(
        map_topic_.value(), [this](const msgs::OccupancyGrid& map) { on_map(map); }, QOS_LATCHED);
  }
  if (has_layer("obstacle_layer")) {
    for (const auto& source : observation_sources_.value()) {
      const auto topic = declare_parameter<std::string>(source + ".topic", source).value();
      scan_subscriptions_.push_back(subscribe<msgs::LaserScan>(
          topic, [this, source](const msgs::LaserScan& scan) { on_scan(source, scan); }, QOS_SENSOR_DATA));
    }
  }
  if (rolling_window_.value() || has_layer("obstacle_layer")) {
    update_timer_ = create_timer(1.0 / update_frequency_.value(), [this] { rebuild(); });
  }
  logger().info("costmap '%s': frame=%s plugins=%zu rolling=%s", this->name().c_str(),
                global_frame_.value().c_str(), plugins_.value().size(),
                rolling_window_.value() ? "true" : "false");
}

bool Costmap2DROS::has_layer(const std::string& layer) const {
  const auto values = plugins_.value();
  return std::find(values.begin(), values.end(), layer) != values.end();
}

std::shared_ptr<const Costmap> Costmap2DROS::costmap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return costmap_;
}

double Costmap2DROS::costmap_resolution() const {
  if (rolling_window_.value()) {
    return resolution_.value();
  }
  return map_ ? map_->info.resolution : resolution_.value();
}

std::optional<msgs::Pose2D> Costmap2DROS::get_robot_pose() const {
  try {
    const msgs::TransformStamped transform =
        buffer_.lookup_transform(global_frame_.value(), robot_base_frame_.value());
    return msgs::Pose2D{transform.transform.translation.x, transform.transform.translation.y,
                        tfmath::yaw_of(transform.transform.rotation)};
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

void Costmap2DROS::on_map(const msgs::OccupancyGrid& map) {
  const bool empty = map.info.width == 0 || map.info.height == 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (empty) {
      map_.reset();
      if (!rolling_window_.value()) {
        costmap_.reset();
      }
    } else {
      map_ = map;
    }
  }
  if (empty && !rolling_window_.value()) {
    publish_costmap();
    return;
  }
  rebuild();
}

void Costmap2DROS::on_scan(const std::string& source, const msgs::LaserScan& scan) {
  (void)source;
  if (scan_period_.value() > 0.0) {
    const double current = now();
    if (current - last_scan_time_ < scan_period_.value()) {
      return;
    }
    last_scan_time_ = current;
  }
  const double resolution = costmap_resolution();
  if (!(resolution > 0.0)) {
    return;
  }
  msgs::TransformStamped transform;
  try {
    transform = buffer_.lookup_transform(global_frame_.value(), scan.header.frame_id,
                                         scan.header.stamp == 0.0 ? std::nullopt
                                                                  : std::optional<double>(scan.header.stamp));
  } catch (const TransformException&) {
    try {
      transform = buffer_.lookup_transform(global_frame_.value(), scan.header.frame_id);
    } catch (const TransformException&) {
      return;
    }
  }
  const double yaw = tfmath::yaw_of(transform.transform.rotation);
  const double ox = transform.transform.translation.x;
  const double oy = transform.transform.translation.y;
  const int origin_i = static_cast<int>(std::floor(ox / resolution));
  const int origin_j = static_cast<int>(std::floor(oy / resolution));
  std::set<std::pair<int, int>> marks;
  std::set<std::pair<int, int>> clears;
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const float range = scan.ranges[index];
    const bool hit = std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
    const double reach = std::min(hit ? static_cast<double>(range) : scan.range_max, raytrace_max_range_.value());
    const double angle = yaw + scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const int end_i = static_cast<int>(std::floor((ox + reach * std::cos(angle)) / resolution));
    const int end_j = static_cast<int>(std::floor((oy + reach * std::sin(angle)) / resolution));
    std::vector<std::pair<int, int>> line;
    line_cell(origin_i, origin_j, end_i, end_j, line);
    clears.insert(line.begin(), line.end());
    if (hit && range <= obstacle_max_range_.value()) {
      marks.emplace(static_cast<int>(std::floor((ox + range * std::cos(angle)) / resolution)),
                    static_cast<int>(std::floor((oy + range * std::sin(angle)) / resolution)));
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& cell : clears) {
    if (marks.count(cell) == 0) {
      obstacles_.erase(cell);
    }
  }
  obstacles_.insert(marks.begin(), marks.end());
}

void Costmap2DROS::rebuild() {
  msgs::OccupancyGrid map_copy;
  bool has_map = false;
  std::set<std::pair<int, int>> obstacles;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_) {
      map_copy = *map_;
      has_map = true;
    }
    obstacles = obstacles_;
  }
  std::shared_ptr<Costmap> costmap;
  if (rolling_window_.value()) {
    const auto pose = get_robot_pose();
    if (!pose) {
      return;
    }
    const double resolution = resolution_.value();
    const int width = std::max(1, static_cast<int>(std::lround(width_.value() / resolution)));
    const int height = std::max(1, static_cast<int>(std::lround(height_.value() / resolution)));
    const double origin_x =
        std::floor((pose->x - width_.value() / 2.0) / resolution) * resolution;
    const double origin_y =
        std::floor((pose->y - height_.value() / 2.0) / resolution) * resolution;
    costmap = std::make_shared<Costmap>(resolution, width, height, origin_x, origin_y);
    if (has_layer("static_layer") && has_map && map_copy.header.frame_id == global_frame_.value()) {
      const Costmap static_map = Costmap::from_occupancy_grid(map_copy, track_unknown_space_.value());
      for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
          const auto world = costmap->map_to_world(i, j);
          const auto cell = static_map.world_to_map(world.first, world.second);
          costmap->set_cost(i, j, cell ? static_map.cost(cell->first, cell->second) : FREE_SPACE);
        }
      }
    }
  } else if (has_layer("static_layer") && has_map) {
    costmap = std::make_shared<Costmap>(Costmap::from_occupancy_grid(map_copy, track_unknown_space_.value()));
  } else {
    return;
  }
  if (!costmap || !costmap->valid()) {
    return;
  }
  if (has_layer("obstacle_layer")) {
    // obstacles_는 global_frame의 (0,0) 기준 resolution 격자다. 지도 origin이 격자에 맞지 않아도
    // 되도록 칸 중심을 world로 바꾼 뒤 이 costmap의 칸으로 다시 찾는다.
    const double stored_resolution = costmap_resolution();
    for (const auto& [ci, cj] : obstacles) {
      const double x = (static_cast<double>(ci) + 0.5) * stored_resolution;
      const double y = (static_cast<double>(cj) + 0.5) * stored_resolution;
      const auto cell = costmap->world_to_map(x, y);
      if (cell) {
        costmap->set_cost(cell->first, cell->second, LETHAL_OBSTACLE);
      }
    }
  }
  if (has_layer("inflation_layer")) {
    costmap->inflate(robot_radius_.value(), inflation_radius_.value(), cost_scaling_factor_.value());
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    costmap_ = std::move(costmap);
  }
  publish_costmap();
}

void Costmap2DROS::publish_costmap() {
  const auto current = costmap();
  msgs::OccupancyGrid message;
  if (current) {
    message = current->to_message(msgs::Header{now(), global_frame_.value()});
  } else {
    message.header = msgs::Header{now(), global_frame_.value()};
  }
  publish(name() + "/costmap", std::move(message), QOS_LATCHED);
}

}  // namespace krosc::nodes::nav2
