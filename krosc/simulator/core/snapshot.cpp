#include "krosc/simulator/snapshot.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace krosc::simulator {

void SnapshotStore::set(std::shared_ptr<const SimSnapshot> snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(snapshot);
}

std::shared_ptr<const SimSnapshot> SnapshotStore::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

SnapshotRecorder::SnapshotRecorder(const World& world, const TurtleBot3& robot) : world_(world), robot_(robot) {}

void SnapshotRecorder::record() {
  auto snapshot = std::make_shared<SimSnapshot>();
  snapshot->time = world_.time();
  snapshot->robot_pose = robot_.pose;
  snapshot->command = robot_.command();
  const auto joints = robot_.joint_positions();
  if (auto it = joints.find("wheel_left_joint"); it != joints.end()) {
    snapshot->left_wheel = it->second;
  }
  if (auto it = joints.find("wheel_right_joint"); it != joints.end()) {
    snapshot->right_wheel = it->second;
  }
  for (const auto& object : world_.objects()) {
    if (object->kind() != "puck") {
      continue;
    }
    const auto* puck = dynamic_cast<const Puck*>(object.get());
    if (puck == nullptr) {
      continue;
    }
    SnapshotPuck state;
    state.pose = puck->pose;
    state.radius = puck->radius;
    state.height = puck->height;
    state.shape = static_cast<int>(puck->shape);
    state.color = puck->color;
    snapshot->pucks.push_back(state);
  }

  if (last_scan_time_ < 0.0 || world_.time() - last_scan_time_ >= kScanPeriod) {
    const msgs::Pose2D sensor = robot_.scan_pose();
    const double increment = 2.0 * 3.14159265358979323846 / kScanBeams;
    std::vector<double> angles(static_cast<std::size_t>(kScanBeams));
    for (int i = 0; i < kScanBeams; ++i) {
      angles[static_cast<std::size_t>(i)] = sensor.theta + static_cast<double>(i) * increment;
    }
    std::vector<float> ranges(static_cast<std::size_t>(kScanBeams));
    world_.ray_distances(sensor.x, sensor.y, angles, TurtleBot3::kLidarMountHeight, ranges);
    for (auto& range : ranges) {
      if (!(static_cast<double>(range) >= kScanRangeMin && static_cast<double>(range) <= kScanRangeMax)) {
        range = std::numeric_limits<float>::infinity();
      }
    }
    last_scan_ranges_ = std::move(ranges);
    last_scan_pose_ = sensor;
    last_scan_time_ = world_.time();
  }
  snapshot->scan_ranges = last_scan_ranges_;
  snapshot->scan_pose = last_scan_pose_;
  store_.set(std::move(snapshot));
}

}  // namespace krosc::simulator
