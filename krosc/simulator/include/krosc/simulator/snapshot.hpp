#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/simulator/robot.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// 시뮬레이션 스레드가 만드는 불변 스냅샷. 화면(Qt 스레드)은 이 값만 읽는다.
struct SnapshotObject {
  int id = -1;  // world.objects() 인덱스. 화면에서 퍽을 집어 옮길 때 쓴다.
  msgs::Pose2D pose;
  double radius = 0.06;
  double height = 0.04;
  double z = 0.0;
  int sides = 32;
  bool held = false;  // 그리퍼가 들고 있으면 드래그로 옮기지 않는다.
  std::array<double, 3> color{1.0, 1.0, 1.0};
};

struct SimulatorSnapshot {
  double time = 0.0;
  msgs::Pose2D robot_pose;
  msgs::Twist command;
  double left_wheel = 0.0;
  double right_wheel = 0.0;
  std::vector<SnapshotObject> objects;
  std::vector<float> scan_ranges;  // 진실 라이다 (잡음 없음)
  msgs::Pose2D scan_pose;
  std::vector<RobotVisual> visuals;
};

class SnapshotStore {
 public:
  void set(std::shared_ptr<const SimulatorSnapshot> snapshot);
  [[nodiscard]] std::shared_ptr<const SimulatorSnapshot> latest() const;

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const SimulatorSnapshot> latest_;
};

// SimulationRunner의 listener에서 매 스텝 부른다. 월드/로봇은 이 스레드만 만지므로
// 여기서 읽어도 안전하다. 구체 로봇은 SimulatorRobot::state()로 자신의 센서 값을 준다.
class SnapshotRecorder {
 public:
  SnapshotRecorder(const World& world, const SimulatorRobot& robot);

  void record();
  [[nodiscard]] SnapshotStore& store() { return store_; }
  [[nodiscard]] const SnapshotStore& store() const { return store_; }

 private:
  const World& world_;
  const SimulatorRobot& robot_;
  SnapshotStore store_;
};

}  // namespace krosc::simulator
