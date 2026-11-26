#pragma once

#include <cstdint>

#include "krosc/context.hpp"

namespace krosc::sensor {

// 센서 lifecycle 실패 알림. 구현체는 on_error()에서 로그/진단/복구를 할 수 있다.
struct SensorError {
  enum class Stage { Init, Activate, Read, Deactivate };
  Stage stage = Stage::Read;
  std::uint64_t consecutive_failures = 0;  // on_update 연속 실패 횟수
  double time = 0.0;                       // 실패 시각 (steady clock)
};

// read-only 센서의 수명주기:
//   on_init -> on_activate -> on_update 반복 -> on_deactivate -> ... -> on_finish
// on_deactivate는 스트리밍만 멈추고(재시작 가능), on_finish는 최종 정리다.
// 발행 토픽/프레임/설정은 구현체가 생성자에서 소유한다. 베이스는 lifecycle만 안다.
class Sensor {
 public:
  // 모든 센서는 Context를 가진다. 발행/시각/stamp는 이 Context를 쓴다.
  explicit Sensor(krosc::Context& context) : context_(context) {}
  virtual ~Sensor() = default;

  [[nodiscard]] krosc::Context& context() const { return context_; }
  [[nodiscard]] double now() const { return context_.now(); }

  virtual bool on_init() { return true; }
  virtual bool on_activate() { return true; }
  virtual bool on_deactivate() { return true; }
  virtual bool on_update(double time, double period) = 0;

  // 최종 정리. Context가 아직 살아 있을 때 한 번 호출된다.
  // 마지막 상태 publish나 서비스 호출을 해도 되지만, 오래 블록하지 않는다.
  virtual void on_finish() {}
  // 실패 알림. on_update 실패는 매번, init/activate/deactivate 실패는 전이 시점에 불린다.
  virtual void on_error(const SensorError&) {}

 private:
  krosc::Context& context_;
};

}  // namespace krosc::sensor
