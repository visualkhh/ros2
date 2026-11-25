#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/simulator/maze_world.hpp"
#include "krosc/simulator/turtlebot3.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// 시뮬레이션 스레드가 만드는 불변 스냅샷. 화면(Qt 스레드)은 이 값만 읽는다.
struct SnapshotPuck {
  msgs::Pose2D pose;
  double radius = 0.06;
  double height = 0.04;
  int shape = 32;
  std::array<double, 3> color{1.0, 1.0, 1.0};
};

struct SimSnapshot {
  double time = 0.0;
  msgs::Pose2D robot_pose;
  msgs::Twist command;
  double left_wheel = 0.0;
  double right_wheel = 0.0;
  std::vector<SnapshotPuck> pucks;
  std::vector<float> scan_ranges;  // 진실 라이다 (잡음 없음)
  msgs::Pose2D scan_pose;
};

class SnapshotStore {
 public:
  void set(std::shared_ptr<const SimSnapshot> snapshot);
  [[nodiscard]] std::shared_ptr<const SimSnapshot> latest() const;

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const SimSnapshot> latest_;
};

// SimulationRunner의 listener에서 매 스텝 부른다. 월드는 이 스레드만 만지므로 여기서 읽어도 안전하다.
class SnapshotRecorder {
 public:
  SnapshotRecorder(const World& world, const TurtleBot3& robot);

  void record();
  [[nodiscard]] SnapshotStore& store() { return store_; }
  [[nodiscard]] const SnapshotStore& store() const { return store_; }

  static constexpr int kScanBeams = 360;
  static constexpr double kScanPeriod = 0.1;
  static constexpr double kScanRangeMin = 0.12;
  static constexpr double kScanRangeMax = 3.5;

 private:
  const World& world_;
  const TurtleBot3& robot_;
  SnapshotStore store_;
  double last_scan_time_ = -1.0;
  std::vector<float> last_scan_ranges_;
  msgs::Pose2D last_scan_pose_;
};

}  // namespace krosc::simulator
