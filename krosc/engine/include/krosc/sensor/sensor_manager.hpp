#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "krosc/sensor/sensor.hpp"

namespace krosc::sensor {

// 여러 센서의 수명주기와 read를 한 곳에서 관리한다.
// 제어 루프와 같은 스레드에서 호출하는 것을 전제로 한다 (내부 락 없음).
class SensorManager {
 public:
  SensorManager() = default;
  ~SensorManager();

  SensorManager(const SensorManager&) = delete;
  SensorManager& operator=(const SensorManager&) = delete;

  // 이름은 중복될 수 없다. 빈 이름이나 null 센서면 false.
  [[nodiscard]] bool add(std::string name, std::string type, std::unique_ptr<Sensor> sensor);

  // 모든 센서를 on_init -> on_activate 순서로 올린다.
  // 하나라도 실패하면 이미 활성화한 센서를 모두 내리고 false.
  [[nodiscard]] bool start();
  void stop();

  // 활성 센서만 read한다. 이번 주기에 실패한 센서 수를 돌려준다.
  std::size_t read(double time, double period);

  [[nodiscard]] Sensor* find(const std::string& name);
  [[nodiscard]] const Sensor* find(const std::string& name) const;
  [[nodiscard]] std::size_t size() const { return entries_.size(); }
  [[nodiscard]] std::size_t active_count() const;
  [[nodiscard]] std::uint64_t read_failures() const { return read_failures_; }

 private:
  struct Entry {
    std::string name;
    std::string type;
    std::unique_ptr<Sensor> sensor;
    bool initialized = false;
    bool active = false;
  };

  [[nodiscard]] Entry* find_entry(const std::string& name);
  [[nodiscard]] const Entry* find_entry(const std::string& name) const;

  std::vector<Entry> entries_;
  std::uint64_t read_failures_ = 0;
};

}  // namespace krosc::sensor
