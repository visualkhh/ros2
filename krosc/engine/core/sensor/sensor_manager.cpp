#include "krosc/sensor/sensor_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace krosc::sensor {

namespace {

double monotonic_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

SensorManager::~SensorManager() { stop(); }

bool SensorManager::add(Sensor& sensor, std::string name, double period) {
  if (started_ || name.empty() || !(period > 0.0)) {
    return false;
  }
  for (const auto& entry : entries_) {
    if (entry.name == name) {
      return false;
    }
  }
  Entry entry;
  entry.sensor = &sensor;
  entry.name = std::move(name);
  entry.period = period;
  entries_.push_back(std::move(entry));
  return true;
}

bool SensorManager::start(bool run_loop) {
  if (started_) {
    return false;
  }
  for (auto& entry : entries_) {
    entry.sensor->start();
    entry.next_read = 0.0;
  }
  started_ = true;
  if (run_loop) {
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }
  return true;
}

void SensorManager::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  for (auto it = entries_.rbegin(); it != entries_.rend(); ++it) {
    it->sensor->stop();
  }
  started_ = false;
}

std::size_t SensorManager::poll(double time) {
  std::size_t failures = 0;
  for (auto& entry : entries_) {
    if (time + 1e-9 < entry.next_read) {
      continue;
    }
    try {
      if (!entry.sensor->read(time, entry.period)) {
        ++failures_;
        ++failures;
      }
    } catch (...) {
      ++failures_;
      ++failures;
    }
    advance(entry, time);
  }
  return failures;
}

void SensorManager::advance(Entry& entry, double time) {
  entry.next_read += entry.period;
  if (entry.next_read <= time + 1e-9) {
    ++missed_;
    entry.next_read = time + entry.period;
  }
}

void SensorManager::loop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (running_) {
    lock.unlock();
    poll(monotonic_seconds());
    lock.lock();
    if (!running_) {
      break;
    }
    const double due = next_due();
    if (!std::isfinite(due)) {
      cv_.wait_for(lock, std::chrono::milliseconds(100),
                   [this] { return !running_; });
      continue;
    }
    const double wait = due - monotonic_seconds();
    if (wait > 0.0) {
      cv_.wait_for(lock, std::chrono::duration<double>(wait),
                   [this] { return !running_; });
    }
  }
}

double SensorManager::next_due() const {
  double due = std::numeric_limits<double>::infinity();
  for (const auto& entry : entries_) {
    due = std::min(due, entry.next_read);
  }
  return due;
}

}  // namespace krosc::sensor
