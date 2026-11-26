#include "krosc/hardware/hardware_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace krosc::hardware {

namespace {

double monotonic_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

HardwareManager::~HardwareManager() { stop(); }

bool HardwareManager::name_used(const std::string& name) const {
  for (const auto& config : hardware_configs_) {
    if (config.name == name) {
      return true;
    }
  }
  for (const auto& entry : sensors_) {
    if (entry.config.name == name) {
      return true;
    }
  }
  for (const auto& entry : controls_) {
    if (entry.config.name == name) {
      return true;
    }
  }
  return false;
}

bool HardwareManager::add_hardware(Hardware& hardware, HardwareConfig config) {
  if (started_ || config.name.empty() || name_used(config.name)) {
    return false;
  }
  hardware_.push_back(&hardware);
  hardware_configs_.push_back(std::move(config));
  return true;
}

bool HardwareManager::add_sensor(sensor::Sensor& sensor, SensorConfig config) {
  if (started_ || config.name.empty() || !(config.period > 0.0) || name_used(config.name)) {
    return false;
  }
  SensorEntry entry;
  entry.config = std::move(config);
  entry.sensor = &sensor;
  sensors_.push_back(entry);
  return true;
}

bool HardwareManager::add_control(control::Control& control, ControlConfig config) {
  if (started_ || config.name.empty() || !(config.read_period > 0.0) ||
      config.write_period < 0.0 || name_used(config.name)) {
    return false;
  }
  ControlEntry entry;
  entry.config = std::move(config);
  entry.control = &control;
  controls_.push_back(entry);
  return true;
}

void HardwareManager::set_control_hook(ControlHook hook, double period) {
  if (started_ || !(period > 0.0)) {
    return;
  }
  control_hook_ = std::move(hook);
  control_period_ = period;
}

bool HardwareManager::start(bool run_loop) {
  if (started_) {
    return false;
  }
  for (Hardware* hardware : hardware_) {
    hardware->start();
  }
  for (auto& entry : controls_) {
    const control::ControlInfo info{entry.config.name, "control", "krosc"};
    if (!entry.control->on_init(info) || !entry.control->on_activate()) {
      ++control_failures_;
      stop();
      return false;
    }
    entry.next_read = 0.0;
    entry.next_write = 0.0;
  }
  for (auto& entry : sensors_) {
    entry.sensor->start();
    entry.next_read = 0.0;
  }
  next_control_ = 0.0;
  started_ = true;
  if (run_loop) {
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }
  return true;
}

void HardwareManager::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  for (auto it = sensors_.rbegin(); it != sensors_.rend(); ++it) {
    it->sensor->stop();
  }
  for (auto it = controls_.rbegin(); it != controls_.rend(); ++it) {
    it->control->on_deactivate();
  }
  for (auto it = hardware_.rbegin(); it != hardware_.rend(); ++it) {
    (*it)->stop();
  }
  started_ = false;
}

std::size_t HardwareManager::poll(double time) {
  std::size_t failures = 0;

  for (auto& entry : sensors_) {
    if (time + 1e-9 < entry.next_read) {
      continue;
    }
    try {
      if (!entry.sensor->read(time, entry.config.period)) {
        ++sensor_failures_;
        ++failures;
      }
    } catch (...) {
      ++sensor_failures_;
      ++failures;
    }
    advance(entry.next_read, entry.config.period, time);
  }

  for (auto& entry : controls_) {
    if (time + 1e-9 < entry.next_read) {
      continue;
    }
    try {
      if (!entry.control->read(time, entry.config.read_period)) {
        ++control_read_failures_;
        ++failures;
      }
    } catch (...) {
      ++control_read_failures_;
      ++failures;
    }
    advance(entry.next_read, entry.config.read_period, time);
  }

  if (control_hook_ && control_period_ > 0.0 && time + 1e-9 >= next_control_) {
    try {
      control_hook_(time, control_period_);
    } catch (...) {
      ++control_failures_;
      ++failures;
    }
    advance(next_control_, control_period_, time);
  }

  for (auto& entry : controls_) {
    if (entry.config.write_period <= 0.0 || time + 1e-9 < entry.next_write) {
      continue;
    }
    try {
      if (!entry.control->write(time, entry.config.write_period)) {
        ++control_write_failures_;
        ++failures;
      }
    } catch (...) {
      ++control_write_failures_;
      ++failures;
    }
    advance(entry.next_write, entry.config.write_period, time);
  }

  return failures;
}

void HardwareManager::advance(double& due, double period, double time) {
  due += period;
  if (due <= time + 1e-9) {
    ++missed_;
    due = time + period;
  }
}

void HardwareManager::loop() {
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

double HardwareManager::next_due() const {
  double due = std::numeric_limits<double>::infinity();
  for (const auto& entry : sensors_) {
    due = std::min(due, entry.next_read);
  }
  for (const auto& entry : controls_) {
    due = std::min(due, entry.next_read);
    if (entry.config.write_period > 0.0) {
      due = std::min(due, entry.next_write);
    }
  }
  if (control_hook_ && control_period_ > 0.0) {
    due = std::min(due, next_control_);
  }
  return due;
}

}  // namespace krosc::hardware
