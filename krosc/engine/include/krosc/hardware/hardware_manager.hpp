#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/control/control.hpp"
#include "krosc/hardware/hardware.hpp"
#include "krosc/sensor/sensor.hpp"

namespace krosc::hardware {

struct HardwareConfig {
  std::string name;
};

struct SensorConfig {
  std::string name;
  double period = 0.1;  // read 주기 [s]
};

struct ControlConfig {
  std::string name;
  double read_period = 0.02;   // state read 주기 [s]
  double write_period = 0.0;   // command write 주기 [s], <=0이면 안 씀
};

// Hardware(물리) + Sensor/Control(논리)을 등록받아 한 루프에서 돌린다.
//   sensor read -> control read -> control hook -> control write
// control hook은 ControllerManager::update() 같은 제어 사이클이다.
class HardwareManager {
 public:
  using ControlHook = std::function<void(double time, double period)>;

  HardwareManager() = default;
  ~HardwareManager();

  HardwareManager(const HardwareManager&) = delete;
  HardwareManager& operator=(const HardwareManager&) = delete;

  [[nodiscard]] bool add_hardware(Hardware& hardware, HardwareConfig config);
  [[nodiscard]] bool add_sensor(sensor::Sensor& sensor, SensorConfig config);
  [[nodiscard]] bool add_control(control::Control& control, ControlConfig config);

  void set_control_hook(ControlHook hook, double period);

  [[nodiscard]] bool start(bool run_loop = true);
  void stop();

  // due인 sensor/control을 한 번 실행한다. 실패한 동작 수를 돌려준다.
  std::size_t poll(double time);

  [[nodiscard]] std::uint64_t sensor_failures() const { return sensor_failures_; }
  [[nodiscard]] std::uint64_t control_read_failures() const { return control_read_failures_; }
  [[nodiscard]] std::uint64_t control_write_failures() const { return control_write_failures_; }
  [[nodiscard]] std::uint64_t control_failures() const { return control_failures_; }
  [[nodiscard]] std::uint64_t missed() const { return missed_; }

 private:
  struct SensorEntry {
    SensorConfig config;
    sensor::Sensor* sensor = nullptr;
    double next_read = 0.0;
  };
  struct ControlEntry {
    ControlConfig config;
    control::Control* control = nullptr;
    double next_read = 0.0;
    double next_write = 0.0;
  };

  void loop();
  [[nodiscard]] double next_due() const;
  void advance(double& due, double period, double time);
  [[nodiscard]] bool name_used(const std::string& name) const;

  std::vector<HardwareConfig> hardware_configs_;
  std::vector<Hardware*> hardware_;
  std::vector<SensorEntry> sensors_;
  std::vector<ControlEntry> controls_;
  ControlHook control_hook_;
  double control_period_ = 0.0;
  double next_control_ = 0.0;

  std::uint64_t sensor_failures_ = 0;
  std::uint64_t control_read_failures_ = 0;
  std::uint64_t control_write_failures_ = 0;
  std::uint64_t control_failures_ = 0;
  std::uint64_t missed_ = 0;

  bool started_ = false;
  bool running_ = false;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace krosc::hardware
