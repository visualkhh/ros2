#pragma once

#include "krosc/context.hpp"

namespace krosc::sensor {

// 논리 센서 레이어. 물리 Hardware를 참조해서 데이터를 읽고 토픽으로 발행한다.
// 주기/lifecycle 실행은 HardwareManager가 담당한다.
class Sensor {
 public:
  explicit Sensor(krosc::Context& context) : context_(context) {}
  virtual ~Sensor() = default;

  [[nodiscard]] krosc::Context& context() const { return context_; }
  [[nodiscard]] double now() const { return context_.now(); }

  virtual void start() {}
  virtual void stop() {}
  [[nodiscard]] virtual bool is_connected() const { return true; }
  virtual bool read(double time, double period) = 0;

 private:
  krosc::Context& context_;
};

}  // namespace krosc::sensor
