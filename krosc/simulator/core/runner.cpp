#include "krosc/simulator/runner.hpp"

#include <stdexcept>
#include <utility>

namespace krosc::simulator {

SimulationRunner::SimulationRunner(World& world, double step_hz) : world_(world), dt_(1.0 / step_hz) {
  if (!(step_hz > 0.0)) {
    throw std::invalid_argument("step_hz는 0보다 커야 한다");
  }
}

SimulationRunner::~SimulationRunner() { stop(); }

void SimulationRunner::add_listener(std::function<void(World&)> listener) {
  if (listener) {
    listeners_.push_back(std::move(listener));
  }
}

void SimulationRunner::step_once() {
  world_.step(dt_);
  for (auto& listener : listeners_) {
    listener(world_);
  }
}

void SimulationRunner::start() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      return;
    }
    stop_ = false;
    running_ = true;
  }
  thread_ = std::thread([this] { loop(); });
}

void SimulationRunner::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return;
    }
    stop_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
    thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = false;
}

void SimulationRunner::loop() {
  auto next = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop_) {
    lock.unlock();
    step_once();
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt_));
    lock.lock();
    if (stop_) {
      break;
    }
    if (wake_.wait_until(lock, next, [this] { return stop_; })) {
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - next > std::chrono::milliseconds(250)) {
      next = now;  // 너무 밀렸다: 따라잡지 않고 버린다
    }
  }
}

}  // namespace krosc::simulator
