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

#include "krosc/hardware/hardware.hpp"

namespace krosc::hardware {

// HardwareManager 등록 정보. 주기는 Hardware 안이 아니라 여기서 정한다.
// period <= 0이면 해당 동작을 하지 않는다.
struct HardwareConfig {
  std::string name;
  double read_period = 0.1;   // read(=센서 역할) 주기 [s]
  double write_period = 0.0;  // write(=제어 역할) 주기 [s]
};

// Hardware들을 한 루프에서 순서대로 돌린다.
//   read() -> control hook -> write()
// control hook은 ControllerManager::update() 같은 제어 사이클을 read와 write 사이에 둔다.
class HardwareManager {
 public:
  using ControlHook = std::function<void(double time, double period)>;

  HardwareManager() = default;
  ~HardwareManager();

  HardwareManager(const HardwareManager&) = delete;
  HardwareManager& operator=(const HardwareManager&) = delete;

  [[nodiscard]] bool add(std::unique_ptr<Hardware> hardware, HardwareConfig config);
  // control_period <= 0이면 hook을 끈다.
  void set_control_hook(ControlHook hook, double control_period);

  [[nodiscard]] bool start(bool run_loop = true);
  void stop();

  // due인 read/write/control을 한 번 실행한다. 실패한 동작 수를 돌려준다.
  std::size_t poll(double time);

  [[nodiscard]] Hardware* find(const std::string& name);
  [[nodiscard]] std::size_t size() const { return entries_.size(); }
  [[nodiscard]] std::uint64_t read_failures() const { return read_failures_; }
  [[nodiscard]] std::uint64_t write_failures() const { return write_failures_; }
  [[nodiscard]] std::uint64_t control_failures() const { return control_failures_; }
  [[nodiscard]] std::uint64_t missed() const { return missed_; }

 private:
  struct Entry {
    HardwareConfig config;
    std::unique_ptr<Hardware> hardware;
    double next_read = 0.0;
    double next_write = 0.0;
  };

  void loop();
  [[nodiscard]] double next_due() const;
  void advance(double& due, double period, double time);

  std::vector<Entry> entries_;
  ControlHook control_hook_;
  double control_period_ = 0.0;
  double next_control_ = 0.0;

  std::uint64_t read_failures_ = 0;
  std::uint64_t write_failures_ = 0;
  std::uint64_t control_failures_ = 0;
  std::uint64_t missed_ = 0;

  bool started_ = false;
  bool running_ = false;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace krosc::hardware
