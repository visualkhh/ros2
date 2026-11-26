#pragma once

// 시뮬레이터 코어 런타임: 호출자가 만든 월드와 로봇을 고정 주기로 돌리고,
// 화면이 읽을 스냅샷만 만든다. ROS 브리지/지도/URDF는 호출자(앱)가 소유한다.

#include <memory>

#include "krosc/simulator/robot.hpp"
#include "krosc/simulator/snapshot.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc {
class Context;
}

namespace krosc::simulator {

class SimulationRunner;

struct SimulatorOptions {
  double step_hz = 100.0;
};

class Simulator {
 public:
  Simulator(krosc::Context& context, World& world, SimRobot& robot,
            SimulatorOptions options = {});
  ~Simulator();

  Simulator(const Simulator&) = delete;
  Simulator& operator=(const Simulator&) = delete;

  void start();
  void stop();

  [[nodiscard]] World& world();
  [[nodiscard]] SimRobot& robot();
  [[nodiscard]] SimulationRunner& runner();
  [[nodiscard]] SnapshotStore& snapshots();
  [[nodiscard]] const SnapshotStore& snapshots() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace krosc::simulator
