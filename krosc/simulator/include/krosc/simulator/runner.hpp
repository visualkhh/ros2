#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "krosc/simulator/world.hpp"

namespace krosc::simulator {

// 월드를 고정 간격(step_hz)으로 실시간에 맞춰 돌린다. 월드는 이 스레드만 만진다.
// 스텝이 끝날 때마다 listener(world)를 부른다 (시뮬레이션 스레드).
class SimulationRunner {
 public:
  explicit SimulationRunner(World& world, double step_hz = 60.0);
  ~SimulationRunner();

  SimulationRunner(const SimulationRunner&) = delete;
  SimulationRunner& operator=(const SimulationRunner&) = delete;

  [[nodiscard]] double dt() const { return dt_; }
  void add_listener(std::function<void(World&)> listener);
  void step_once();
  void start();
  void stop();

 private:
  void loop();

  World& world_;
  double dt_;
  std::vector<std::function<void(World&)>> listeners_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_ = false;
  bool running_ = false;
};

}  // namespace krosc::simulator
