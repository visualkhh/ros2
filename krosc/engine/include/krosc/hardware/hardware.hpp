#pragma once

namespace krosc::hardware {

// 물리 장치/전송 계층의 공통 lifecycle.
// control::ControlInterface 같은 "역할 어댑터"와는 별개다.
//   예: OpenCrSerial(시리얼/파서/캐시)은 Hardware.
//       read()는 센서 역할(데이터 발행), write()는 제어 역할(마지막 명령 유지 전송).
class Hardware {
 public:
  virtual ~Hardware() = default;

  virtual void start() = 0;
  virtual void stop() = 0;
  [[nodiscard]] virtual bool connected() const = 0;

  // read/write 주기는 Hardware가 아니라 HardwareManager 등록 시 정한다.
  // period는 해당 동작의 설정 주기[s]다.
  virtual bool read(double time, double period) {
    (void)time;
    (void)period;
    return true;
  }
  virtual bool write(double time, double period) {
    (void)time;
    (void)period;
    return true;
  }
};

}  // namespace krosc::hardware
