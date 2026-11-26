#pragma once

namespace krosc::hardware {

// 물리 장치/전송 계층의 공통 lifecycle.
// control::SystemInterface나 sensor::Sensor 같은 "역할 어댑터"와는 별개다.
//   예: OpenCrSerial(시리얼/파서/캐시)은 Hardware, OpenCrSystem은 control 어댑터.
class Hardware {
 public:
  virtual ~Hardware() = default;

  virtual void start() = 0;
  virtual void stop() = 0;
  [[nodiscard]] virtual bool connected() const = 0;
};

}  // namespace krosc::hardware
