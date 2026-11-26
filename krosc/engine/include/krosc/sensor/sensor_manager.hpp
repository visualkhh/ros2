#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/sensor/sensor.hpp"

namespace krosc::sensor {

// 논리 Sensor들을 각자의 period로 실행한다.
class SensorManager {
 public:
  SensorManager() = default;
  ~SensorManager();

  SensorManager(const SensorManager&) = delete;
  SensorManager& operator=(const SensorManager&) = delete;

  [[nodiscard]] bool add(Sensor& sensor, std::string name, double period);
  [[nodiscard]] bool start(bool run_loop = true);
  void stop();

  std::size_t poll(double time);

  [[nodiscard]] std::uint64_t failures() const { return failures_; }
  [[nodiscard]] std::uint64_t missed() const { return missed_; }

 private:
  struct Entry {
    Sensor* sensor = nullptr;
    std::string name;
    double period = 0.1;
    double next_read = 0.0;
  };

  void loop();
  [[nodiscard]] double next_due() const;
  void advance(Entry& entry, double time);

  std::vector<Entry> entries_;
  std::uint64_t failures_ = 0;
  std::uint64_t missed_ = 0;
  bool started_ = false;
  bool running_ = false;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace krosc::sensor
