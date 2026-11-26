#pragma once

namespace krosc::hardware {

// 물리 장치/전송 계층의 공통 lifecycle.
// Control/Sensor 같은 논리 레이어는 이 클래스를 상속하지 않고, Hardware를 참조해서 쓴다.
class Hardware {
 public:
  virtual ~Hardware() = default;

  virtual void start() = 0;
  virtual void stop() = 0;
  [[nodiscard]] virtual bool connected() const = 0;
};

}  // namespace krosc::hardware
