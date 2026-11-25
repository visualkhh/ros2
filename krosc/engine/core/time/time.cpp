#include "krosc/time/time.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace krosc {

double SystemTimeSource::now() const {
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(since_epoch).count();
}

double SystemTimeSource::monotonic() const {
  const auto since_start = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(since_start).count();
}

SimulatedTimeSource::SimulatedTimeSource(double start) : time_(start) {}

double SimulatedTimeSource::now() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return time_;
}

void SimulatedTimeSource::add_listener(std::function<void()> callback) {
  if (!callback) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  listeners_.push_back(std::move(callback));
}

void SimulatedTimeSource::set(double time) {
  std::vector<std::function<void()>> listeners;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (time < time_) {
      throw std::invalid_argument("시뮬레이션 시간은 되돌릴 수 없다");
    }
    time_ = time;
    listeners = listeners_;
  }
  // 시간을 민 스레드에서 바로 타이머를 판정한다 (스케줄 스레드를 기다리지 않는다).
  for (auto& listener : listeners) {
    listener();
  }
}

void SimulatedTimeSource::advance(double dt) { set(now() + dt); }

}  // namespace krosc
