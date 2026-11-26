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

bool HardwareManager::add(std::unique_ptr<Hardware> hardware, HardwareConfig config) {
  if (started_ || config.name.empty() || !hardware || !(config.read_period > 0.0) ||
      config.write_period < 0.0 || find(config.name) != nullptr) {
    return false;
  }
  Entry entry;
  entry.config = std::move(config);
  entry.hardware = std::move(hardware);
  entries_.push_back(std::move(entry));
  return true;
}

void HardwareManager::set_control_hook(ControlHook hook, double control_period) {
  if (started_ || !(control_period > 0.0)) {
    control_hook_ = {};
    control_period_ = 0.0;
    return;
  }
  control_hook_ = std::move(hook);
  control_period_ = control_period;
}

bool HardwareManager::start(bool run_loop) {
  if (started_) {
    return false;
  }
  for (auto& entry : entries_) {
    entry.hardware->start();
    entry.next_read = 0.0;
    entry.next_write = 0.0;
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
  for (auto it = entries_.rbegin(); it != entries_.rend(); ++it) {
    it->hardware->stop();
  }
  started_ = false;
}

std::size_t HardwareManager::poll(double time) {
  std::size_t failures = 0;

  for (auto& entry : entries_) {
    if (entry.config.read_period <= 0.0 || time + 1e-9 < entry.next_read) {
      continue;
    }
    try {
      if (!entry.hardware->read(time, entry.config.read_period)) {
        ++read_failures_;
        ++failures;
      }
    } catch (...) {
      ++read_failures_;
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

  for (auto& entry : entries_) {
    if (entry.config.write_period <= 0.0 || time + 1e-9 < entry.next_write) {
      continue;
    }
    try {
      if (!entry.hardware->write(time, entry.config.write_period)) {
        ++write_failures_;
        ++failures;
      }
    } catch (...) {
      ++write_failures_;
      ++failures;
    }
    advance(entry.next_write, entry.config.write_period, time);
  }

  return failures;
}

void HardwareManager::advance(double& due, double period, double time) {
  due += period;
  if (due <= time + 1e-9) {
    // 밀린 주기는 몰아서 부르지 않는다.
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
  for (const auto& entry : entries_) {
    if (entry.config.read_period > 0.0) {
      due = std::min(due, entry.next_read);
    }
    if (entry.config.write_period > 0.0) {
      due = std::min(due, entry.next_write);
    }
  }
  if (control_hook_ && control_period_ > 0.0) {
    due = std::min(due, next_control_);
  }
  return due;
}

Hardware* HardwareManager::find(const std::string& name) {
  for (auto& entry : entries_) {
    if (entry.config.name == name) {
      return entry.hardware.get();
    }
  }
  return nullptr;
}

}  // namespace krosc::hardware
