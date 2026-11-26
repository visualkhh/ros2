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

SensorManager::~SensorManager() { finish(); }

bool SensorManager::add(std::string name, double period, std::unique_ptr<Sensor> sensor) {
  if (started_ || finished_ || name.empty() || !(period > 0.0) || !sensor ||
      find_entry(name) != nullptr) {
    return false;
  }
  Entry entry;
  entry.name = std::move(name);
  entry.period = period;
  entry.sensor = std::move(sensor);
  entries_.push_back(std::move(entry));
  return true;
}

bool SensorManager::start(bool run_loop) {
  if (started_ || finished_) {
    return false;
  }
  for (auto& entry : entries_) {
    if (!entry.initialized) {
      if (!entry.sensor->on_init()) {
        report_error(entry, SensorError::Stage::Init, 0.0);
        stop();
        return false;
      }
      entry.initialized = true;
    }
    if (!entry.sensor->on_activate()) {
      report_error(entry, SensorError::Stage::Activate, 0.0);
      stop();
      return false;
    }
    entry.active = true;
    entry.next_due = 0.0;  // 첫 poll에서 바로 한 번 읽는다
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
  // 켠 역순으로 내린다.
  for (auto it = entries_.rbegin(); it != entries_.rend(); ++it) {
    if (it->active) {
      if (!it->sensor->on_deactivate()) {
        ++deactivate_failures_;
        report_error(*it, SensorError::Stage::Deactivate, 0.0);
      }
      it->active = false;
    }
  }
  started_ = false;
}

void SensorManager::finish() {
  if (finished_) {
    return;
  }
  stop();
  for (auto& entry : entries_) {
    if (!entry.finished) {
      entry.sensor->on_finish();
      entry.finished = true;
    }
  }
  finished_ = true;
}

std::size_t SensorManager::poll(double time) {
  std::size_t failures = 0;
  for (auto& entry : entries_) {
    if (!entry.active || time + 1e-9 < entry.next_due) {
      continue;
    }
    if (!entry.sensor->on_update(time, entry.period)) {
      ++failures;
      ++read_failures_;
      ++entry.consecutive_failures;
      report_error(entry, SensorError::Stage::Read, time);
    } else {
      entry.consecutive_failures = 0;
    }
    entry.next_due += entry.period;
    if (entry.next_due <= time + 1e-9) {
      // 지연으로 밀린 주기는 몰아서 호출하지 않고 다음 주기부터 다시 맞춘다.
      ++missed_;
      entry.next_due = time + entry.period;
    }
  }
  return failures;
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
    if (entry.active) {
      due = std::min(due, entry.next_due);
    }
  }
  return due;
}

void SensorManager::report_error(Entry& entry, SensorError::Stage stage, double time) {
  entry.sensor->on_error(SensorError{stage, entry.consecutive_failures, time});
}

Sensor* SensorManager::find(const std::string& name) {
  Entry* entry = find_entry(name);
  return entry == nullptr ? nullptr : entry->sensor.get();
}

const Sensor* SensorManager::find(const std::string& name) const {
  const Entry* entry = find_entry(name);
  return entry == nullptr ? nullptr : entry->sensor.get();
}

std::size_t SensorManager::active_count() const {
  return static_cast<std::size_t>(
      std::count_if(entries_.begin(), entries_.end(), [](const Entry& entry) { return entry.active; }));
}

SensorManager::Entry* SensorManager::find_entry(const std::string& name) {
  const auto found = std::find_if(entries_.begin(), entries_.end(),
                                  [&name](const Entry& entry) { return entry.name == name; });
  return found == entries_.end() ? nullptr : &*found;
}

const SensorManager::Entry* SensorManager::find_entry(const std::string& name) const {
  const auto found = std::find_if(entries_.begin(), entries_.end(),
                                  [&name](const Entry& entry) { return entry.name == name; });
  return found == entries_.end() ? nullptr : &*found;
}

}  // namespace krosc::sensor
