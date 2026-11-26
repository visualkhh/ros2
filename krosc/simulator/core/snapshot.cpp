#include "krosc/simulator/snapshot.hpp"

#include <utility>

namespace krosc::simulator {

void SnapshotStore::set(std::shared_ptr<const SimulatorSnapshot> snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(snapshot);
}

std::shared_ptr<const SimulatorSnapshot> SnapshotStore::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

SnapshotRecorder::SnapshotRecorder(const World& world, const SimulatorRobot& robot)
    : world_(world), robot_(robot) {}

void SnapshotRecorder::record() {
  auto snapshot = std::make_shared<SimulatorSnapshot>();
  snapshot->time = world_.time();
  snapshot->robot_pose = robot_.pose;
  const RobotState state = robot_.state();
  snapshot->command = state.command;
  if (auto it = state.joints.find("wheel_left_joint"); it != state.joints.end()) {
    snapshot->left_wheel = it->second;
  }
  if (auto it = state.joints.find("wheel_right_joint"); it != state.joints.end()) {
    snapshot->right_wheel = it->second;
  }
  snapshot->scan_ranges = state.scan_ranges;
  snapshot->scan_pose = state.scan_pose;
  snapshot->visuals = state.visuals;

  for (std::size_t index = 0; index < world_.objects().size(); ++index) {
    const auto& object = world_.objects()[index];
    if (!object || object.get() == &robot_ || object->sides <= 0 || object->radius <= 0.0 ||
        object->height <= 0.0) {
      continue;
    }
    SnapshotObject body;
    body.id = static_cast<int>(index);
    body.pose = object->pose;
    body.radius = object->radius;
    body.height = object->height;
    body.z = object->z;
    body.sides = object->sides;
    body.held = object->held;
    body.color = object->color;
    snapshot->objects.push_back(body);
  }
  store_.set(std::move(snapshot));
}

}  // namespace krosc::simulator
