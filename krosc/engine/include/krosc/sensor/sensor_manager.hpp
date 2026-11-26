#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/sensor/sensor.hpp"

namespace krosc::sensor {

// 여러 센서의 수명주기와 주기 실행을 관리한다.
// start()는 각 센서의 period에 맞춰 read()를 부르는 전용 스레드를 띄운다.
// 시간은 steady_clock 기준이며, 메시지 stamp는 각 센서가 Context 시각으로 찍는다.
class SensorManager {
 public:
  SensorManager() = default;
  ~SensorManager();

  SensorManager(const SensorManager&) = delete;
  SensorManager& operator=(const SensorManager&) = delete;

  // 이름은 중복될 수 없고, period는 0보다 커야 한다.
  // start() 전에만 추가할 수 있다.
  [[nodiscard]] bool add(SensorInfo info, std::unique_ptr<Sensor> sensor);

  // 모든 센서를 on_init -> on_activate 순서로 올린다.
  // 하나라도 실패하면 이미 활성화한 센서를 모두 내리고 false.
  // run_loop=false면 스레드를 띄우지 않는다 (테스트/수동 poll용).
  [[nodiscard]] bool start(bool run_loop = true);
  // 스트리밍만 멈춘다. on_finish는 부르지 않으므로 다시 start()할 수 있다.
  void stop();
  // 최종 정리: stop() 후 각 센서의 on_finish()를 한 번씩 부른다.
  // 이후에는 start()할 수 없다. 소멸자에서도 자동 호출된다.
  void finish();

  // 주기가 된 센서만 read한다. 이번 호출에서 실패한 센서 수를 돌려준다.
  // run_loop=true인 동안에는 내부 스레드만 호출해야 한다.
  std::size_t poll(double time);

  [[nodiscard]] Sensor* find(const std::string& name);
  [[nodiscard]] const Sensor* find(const std::string& name) const;
  [[nodiscard]] std::size_t size() const { return entries_.size(); }
  [[nodiscard]] std::size_t active_count() const;
  [[nodiscard]] std::uint64_t read_failures() const { return read_failures_; }
  [[nodiscard]] std::uint64_t deactivate_failures() const { return deactivate_failures_; }
  [[nodiscard]] std::uint64_t missed() const { return missed_; }
  [[nodiscard]] bool finished() const { return finished_; }

 private:
  struct Entry {
    SensorInfo info;
    std::unique_ptr<Sensor> sensor;
    bool initialized = false;
    bool active = false;
    bool finished = false;
    double next_due = 0.0;  // steady_clock epoch 기준 [s]
    std::uint64_t consecutive_failures = 0;
  };

  void loop();
  [[nodiscard]] double next_due() const;
  void report_error(Entry& entry, SensorError::Stage stage, double time);
  [[nodiscard]] Entry* find_entry(const std::string& name);
  [[nodiscard]] const Entry* find_entry(const std::string& name) const;

  std::vector<Entry> entries_;
  std::uint64_t read_failures_ = 0;
  std::uint64_t deactivate_failures_ = 0;
  std::uint64_t missed_ = 0;
  bool started_ = false;
  bool finished_ = false;
  bool running_ = false;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace krosc::sensor
