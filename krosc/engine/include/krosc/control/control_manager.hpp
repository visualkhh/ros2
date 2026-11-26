#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/control/control.hpp"

namespace krosc::control {

// 논리 Control들을 각자의 read/write period로 실행한다.
// read → control hook → write 순서를 한 루프에서 지킨다.
class ControlManager {
 public:
  using ControlHook = std::function<void(double time, double period)>;

  ControlManager() = default;
  ~ControlManager();

  ControlManager(const ControlManager&) = delete;
  ControlManager& operator=(const ControlManager&) = delete;

  [[nodiscard]] bool add(Control& control, std::string name, double read_period,
                         double write_period);
  void set_control_hook(ControlHook hook, double period);

  [[nodiscard]] bool start(bool run_loop = true);
  void stop();

  std::size_t poll(double time);

  [[nodiscard]] std::uint64_t read_failures() const { return read_failures_; }
  [[nodiscard]] std::uint64_t write_failures() const { return write_failures_; }
  [[nodiscard]] std::uint64_t hook_failures() const { return hook_failures_; }
  [[nodiscard]] std::uint64_t missed() const { return missed_; }

 private:
  struct Entry {
    Control* control = nullptr;
    std::string name;
    double read_period = 0.02;
    double write_period = 0.0;
    double next_read = 0.0;
    double next_write = 0.0;
  };

  void loop();
  [[nodiscard]] double next_due() const;
  void advance(double& due, double period, double time);

  std::vector<Entry> entries_;
  ControlHook control_hook_;
  double hook_period_ = 0.0;
  double next_hook_ = 0.0;

  std::uint64_t read_failures_ = 0;
  std::uint64_t write_failures_ = 0;
  std::uint64_t hook_failures_ = 0;
  std::uint64_t missed_ = 0;

  bool started_ = false;
  bool running_ = false;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace krosc::control
