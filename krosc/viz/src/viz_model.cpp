#include "krosc/viz/viz_model.hpp"

#include <chrono>
#include <utility>

namespace krosc::viz {

namespace {
constexpr const char* kRosoutTopic = "rosout";
}

VizModel::VizModel(Context& context) : node_(context, "Viz"), listener_(buffer_, node_) {
  subscribe_latest<msgs::LaserScan>("scan", scan_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::Imu>("imu", imu_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::Odometry>("odom", odom_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::Odometry>("odometry/filtered", filtered_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::OccupancyGrid>("map", map_, QOS_LATCHED);
  subscribe_latest<msgs::OccupancyGrid>("global_costmap/costmap", global_costmap_, QOS_LATCHED);
  subscribe_latest<msgs::OccupancyGrid>("local_costmap/costmap", local_costmap_, QOS_LATCHED);
  subscribe_latest<msgs::Path>("plan", plan_, QOS_LATCHED);
  subscribe_latest<msgs::Path>("local_plan", local_plan_, QOS_SENSOR_DATA);
  subscribe_latest<msgs::DiagnosticArray>("diagnostics", diagnostics_, QOS_DEFAULT);
  subscribe_latest<msgs::String>(ROBOT_DESCRIPTION_TOPIC_NAME, robot_description_, QOS_LATCHED);
  subscribe_latest<msgs::String>("map_list", map_list_, QOS_LATCHED);
  subscribe_latest<msgs::String>("map_status", map_status_, QOS_LATCHED);
  subscribe_latest<msgs::String>("slam_status", slam_status_, QOS_LATCHED);
  subscribe_stats_only(TF_TOPIC_NAME, "tf2_msgs/TFMessage", QOS_TF);
  subscribe_stats_only(TF_STATIC_TOPIC_NAME, "tf2_msgs/TFMessage", QOS_TF_STATIC);
  subscribe_stats_only(kRosoutTopic, "rcl_interfaces/Log", QOS_DEFAULT);
  node_.subscribe<msgs::Log>(kRosoutTopic, [this](const msgs::Log& log) { store_log(log); }, QOS_DEFAULT);
}

VizModel::~VizModel() {
  node_.shutdown();
}

void VizModel::subscribe_stats_only(const std::string& topic, const std::string& type, QoS qos) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_[topic] = Stat{type, 0, 0.0, 0.0};
  }
  if (topic == TF_TOPIC_NAME) {
    node_.subscribe<msgs::TFMessage>(
        topic,
        [this, topic](const msgs::TFMessage&) {
          const double now = monotonic_now();
          std::lock_guard<std::mutex> lock(mutex_);
          auto& stat = stats_[topic];
          ++stat.count;
          if (stat.count == 1) {
            stat.first = now;
          }
          stat.last = now;
        },
        qos);
  } else if (topic == TF_STATIC_TOPIC_NAME) {
    node_.subscribe<msgs::TFMessage>(
        topic,
        [this, topic](const msgs::TFMessage&) {
          const double now = monotonic_now();
          std::lock_guard<std::mutex> lock(mutex_);
          auto& stat = stats_[topic];
          ++stat.count;
          if (stat.count == 1) {
            stat.first = now;
          }
          stat.last = now;
        },
        qos);
  } else if (topic == kRosoutTopic) {
    node_.subscribe<msgs::Log>(
        topic,
        [this, topic](const msgs::Log&) {
          const double now = monotonic_now();
          std::lock_guard<std::mutex> lock(mutex_);
          auto& stat = stats_[topic];
          ++stat.count;
          if (stat.count == 1) {
            stat.first = now;
          }
          stat.last = now;
        },
        qos);
  }
}

void VizModel::store_log(const msgs::Log& log) {
  std::lock_guard<std::mutex> lock(mutex_);
  logs_.push_back(log);
  while (logs_.size() > 50) {
    logs_.pop_front();
  }
}

double VizModel::monotonic_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void VizModel::set_fixed_frame(std::string frame) {
  fixed_frame_ = std::move(frame);
}

std::shared_ptr<const msgs::LaserScan> VizModel::scan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return scan_;
}
std::shared_ptr<const msgs::Imu> VizModel::imu() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_;
}
std::shared_ptr<const msgs::Odometry> VizModel::odom() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_;
}
std::shared_ptr<const msgs::Odometry> VizModel::filtered() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return filtered_;
}
std::shared_ptr<const msgs::OccupancyGrid> VizModel::map() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_;
}
std::shared_ptr<const msgs::OccupancyGrid> VizModel::global_costmap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return global_costmap_;
}
std::shared_ptr<const msgs::OccupancyGrid> VizModel::local_costmap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return local_costmap_;
}
std::shared_ptr<const msgs::Path> VizModel::plan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_;
}
std::shared_ptr<const msgs::Path> VizModel::local_plan() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return local_plan_;
}
std::shared_ptr<const msgs::DiagnosticArray> VizModel::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return diagnostics_;
}
std::shared_ptr<const msgs::String> VizModel::robot_description() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_description_;
}

std::shared_ptr<const msgs::String> VizModel::map_list() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_list_;
}

std::shared_ptr<const msgs::String> VizModel::map_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return map_status_;
}

std::shared_ptr<const msgs::String> VizModel::slam_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return slam_status_;
}


std::vector<msgs::Log> VizModel::logs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {logs_.begin(), logs_.end()};
}

std::vector<VizModel::TopicRow> VizModel::topic_rows() const {
  const double now = monotonic_now();
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TopicRow> rows;
  rows.reserve(stats_.size());
  for (const auto& [topic, stat] : stats_) {
    const double span = stat.last - stat.first;
    const double hz = stat.count > 1 && span > 0.0 ? static_cast<double>(stat.count - 1) / span : 0.0;
    rows.push_back(TopicRow{topic, stat.type, stat.count, hz, stat.count > 0 && now - stat.last < 2.0});
  }
  return rows;
}

std::optional<msgs::PoseStamped> VizModel::pose_in_fixed(const std::string& frame, std::optional<double> stamp) const {
  if (frame.empty() || frame == fixed_frame_) {
    msgs::PoseStamped pose;
    pose.header.frame_id = fixed_frame_;
    pose.pose.orientation.w = 1.0;
    return pose;
  }
  try {
    const msgs::TransformStamped transform = buffer_.lookup_transform(fixed_frame_, frame, stamp);
    msgs::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = fixed_frame_;
    pose.pose.position = {transform.transform.translation.x, transform.transform.translation.y,
                          transform.transform.translation.z};
    pose.pose.orientation = transform.transform.rotation;
    return pose;
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

std::optional<msgs::TransformStamped> VizModel::transform_in_fixed(const std::string& frame) const {
  if (frame.empty()) {
    return std::nullopt;
  }
  try {
    return buffer_.lookup_transform(fixed_frame_, frame, std::nullopt);
  } catch (const TransformException&) {
    return std::nullopt;
  }
}

void VizModel::publish_cmd_vel(double linear, double angular) {
  msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  node_.publish("cmd_vel", twist, QOS_COMMAND);
}

void VizModel::publish_goal(double x, double y, double theta) {
  msgs::PoseStamped goal;
  goal.header.stamp = node_.now();
  goal.header.frame_id = fixed_frame_;
  goal.pose.position = {x, y, 0.0};
  goal.pose.orientation = tfmath::from_yaw(theta);
  node_.publish("goal_pose", goal, QOS_SENSOR_DATA);
}

}  // namespace krosc::viz
