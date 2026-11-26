#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/executor.hpp"
#include "krosc/sensor/sensor.hpp"

namespace krosc::sensor {

// 여러 센서의 수명주기와 주기 실행을 관리한다.
// 스케줄러 스레드 1개가 센서별 period를 보고 due인 센서를 Executor 워커 풀로 post한다.
// 같은 센서의 on_update()가 끝나기 전에 다음 주기가 오면 그 주기는 건너뛰고 missed로 센다.
// 시간은 steady_clock 기준이며, 메시지 stamp는 각 센서가 Context 시각으로 찍는다.
class SensorManager {
 public:
  // workers: on_update()를 실행할 워커 수(기본 10). 0이면 inline(테스트/수동 poll용).
  explicit SensorManager(std::size_t workers = 10);
  ~SensorManager();

  SensorManager(const SensorManager&) = delete;
  SensorManager& operator=(const SensorManager&) = delete;

  // 이름은 중복될 수 없고, period는 0보다 커야 한다.
  // start() 전에만 추가할 수 있다.
  [[nodiscard]] bool add(std::string name, double period, std::unique_ptr<Sensor> sensor);

  // 모든 센서를 on_init -> on_activate 순서로 올린다.
  // 하나라도 실패하면 이미 활성화한 센서를 모두 내리고 false.
  // run_loop=false면 스레드를 띄우지 않는다 (테스트/수동 poll용).
  [[nodiscard]] bool start(bool run_loop = true);
  // 스트리밍만 멈춘다. on_finish는 부르지 않으므로 다시 start()할 수 있다.
  void stop();
  // 최종 정리: stop() 후 각 센서의 on_finish()를 한 번씩 부른다.
  // 이후에는 start()할 수 없다. 소멸자에서도 자동 호출된다.
  void finish();

  // 주기가 된 센서만 on_update()를 워커 풀로 보낸다.
  // inline 모드(workers=0)에서는 이번 호출에서 실패한 센서 수를 돌려주고,
  // 워커 풀 모드에서는 0을 돌려주며 실패는 read_failures()로 집계된다.
  // run_loop=true인 동안에는 내부 스케줄러만 호출해야 한다.
  std::size_t poll(double time);

  [[nodiscard]] Sensor* find(const std::string& name);
  [[nodiscard]] const Sensor* find(const std::string& name) const;
  [[nodiscard]] std::size_t size() const { return entries_.size(); }
  [[nodiscard]] std::size_t active_count() const;
  [[nodiscard]] std::uint64_t read_failures() const { return read_failures_.load(); }
  [[nodiscard]] std::uint64_t deactivate_failures() const { return deactivate_failures_.load(); }
  [[nodiscard]] std::uint64_t missed() const { return missed_.load(); }
  [[nodiscard]] bool finished() const { return finished_; }

 private:
  struct Entry {
    std::string name;
    double period = 0.1;
    std::unique_ptr<Sensor> sensor;
    bool initialized = false;
    bool active = false;
    bool finished = false;
    bool busy = false;  // 워커가 on_update()를 실행 중
    double next_due = 0.0;  // steady_clock epoch 기준 [s]
    std::uint64_t consecutive_failures = 0;
  };

  void loop();
  [[nodiscard]] double next_due() const;
  bool run_update(Entry& entry, double time, double period);
  void report_error(Entry& entry, SensorError::Stage stage, double time);
  [[nodiscard]] Entry* find_entry(const std::string& name);
  [[nodiscard]] const Entry* find_entry(const std::string& name) const;

  Executor executor_;
  std::vector<Entry> entries_;
  std::atomic<std::uint64_t> read_failures_{0};
  std::atomic<std::uint64_t> deactivate_failures_{0};
  std::atomic<std::uint64_t> missed_{0};
  bool started_ = false;
  bool finished_ = false;
  bool running_ = false;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace krosc::sensor
