#pragma once

// 시뮬레이터 코어: 월드/로봇/ROS 브리지/러너를 Context에 붙여 조립한다.
// Viz/VizWeb과 같이 Context를 밖에서 받고, 통신은 Context의 PeerNetwork가 담당한다.

#include <cstdint>
#include <memory>
#include <string>

namespace krosc {
class Context;
class Node;
}

namespace krosc::simulator {

class SimulationRunner;
class MazeWorld;
class SnapshotStore;
class TurtleBot3;
class World;

struct SimulatorOptions {
  bool noisy = true;
  int pucks = 10;
  std::uint32_t seed = 42;
  double step_hz = 100.0;
  std::string urdf_path;
  bool bridge = true;    // scan/imu/joint_states/cmd_vel 토픽 브리지
  bool odometry = true;  // DiffDriveOdometry 노드
  bool ekf = true;       // EKF 노드
};

class Simulator {
 public:
  Simulator(krosc::Context& context, SimulatorOptions options = {});
  ~Simulator();

  Simulator(const Simulator&) = delete;
  Simulator& operator=(const Simulator&) = delete;

  void start();
  void stop();

  [[nodiscard]] krosc::Context& context() { return context_; }
  [[nodiscard]] World& world();
  [[nodiscard]] MazeWorld& maze_world();
  [[nodiscard]] TurtleBot3& robot();
  [[nodiscard]] SimulationRunner& runner();
  [[nodiscard]] SnapshotStore& snapshots();
  [[nodiscard]] const SnapshotStore& snapshots() const;
  [[nodiscard]] krosc::Node& bridge_node();
  [[nodiscard]] const std::string& urdf() const;

  // 시작 시 로봇 명령 (데모 기본값: 천천히 전진 + 회전).
  void set_demo_command();
  void set_stop_command();

 private:
  struct Impl;
  krosc::Context& context_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace krosc::simulator
