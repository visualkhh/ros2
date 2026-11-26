#include "krosc/control/control_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace krosc::control {

namespace {

double monotonic_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

ControlManager::~ControlManager() { stop(); }

bool ControlManager::add(Control& control, std::string name, double read_period,
                         double write_period) {
  if (started_ || name.empty() || !(read_period > 0.0) || write_period < 0.0) {
    return false;
  }
  for (const auto& entry : entries_) {
    if (entry.name == name) {
      return false;
    }
  }
  Entry entry;
  entry.control = &control;
  entry.name = std::move(name);
  entry.read_period = read_period;
  entry.write_period = write_period;
  entries_.push_back(std::move(entry));
  return true;
}

void ControlManager::set_control_hook(ControlHook hook, double period) {
  if (started_ || !(period > 0.0)) {
    return;
  }
  control_hook_ = std::move(hook);
  hook_period_ = period;
}

bool ControlManager::start(bool run_loop) {
  if (started_) {
    return false;
  }
  for (auto& entry : entries_) {
    const ControlInfo info{entry.name, "control", "krosc"};
    if (!entry.control->on_init(info) || !entry.control->on_activate()) {
      ++hook_failures_;
      stop();
      return false;
    }
    entry.next_read = 0.0;
    entry.next_write = 0.0;
  }
  next_hook_ = 0.0;
  started_ = true;
  if (run_loop) {
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }
  return true;
}

void ControlManager::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  for (auto it = entries_.rbegin(); it != entries_.rend(); ++it) {
    it->control->on_deactivate();
  }
  started_ = false;
}

std::size_t ControlManager::poll(double time) {
  std::size_t failures = 0;

  for (auto& entry : entries_) {
    if (time + 1e-9 < entry.next_read) {
      continue;
    }
    try {
      if (!entry.control->read(time, entry.read_period)) {
        ++read_failures_;
        ++failures;
      }
    } catch (...) {
      ++read_failures_;
      ++failures;
    }
    advance(entry.next_read, entry.read_period, time);
  }

  if (control_hook_ && hook_period_ > 0.0 && time + 1e-9 >= next_hook_) {
    try {
      control_hook_(time, hook_period_);
    } catch (...) {
      ++hook_failures_;
      ++failures;
    }
    advance(next_hook_, hook_period_, time);
  }

  for (auto& entry : entries_) {
    if (entry.write_period <= 0.0 || time + 1e-9 < entry.next_write) {
      continue;
    }
    try {
      if (!entry.control->write(time, entry.write_period)) {
        ++write_failures_;
        ++failures;
      }
    } catch (...) {
      ++write_failures_;
      ++failures;
    }
    advance(entry.next_write, entry.write_period, time);
  }

  return failures;
}

void ControlManager::advance(double& due, double period, double time) {
  due += period;
  if (due <= time + 1e-9) {
    ++missed_;
    due = time + period;
  }
}

void ControlManager::loop() {
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

double ControlManager::next_due() const {
  double due = std::numeric_limits<double>::infinity();
  for (const auto& entry : entries_) {
    due = std::min(due, entry.next_read);
    if (entry.write_period > 0.0) {
      due = std::min(due, entry.next_write);
    }
  }
  if (control_hook_ && hook_period_ > 0.0) {
    due = std::min(due, next_hook_);
  }
  return due;
}

}  // namespace krosc::control
