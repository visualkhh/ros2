#pragma once

namespace krosc::hardware {

// 물리 장치/전송 계층의 공통 lifecycle.
// Control/Sensor 같은 논리 레이어는 이 클래스를 상속하지 않고, Hardware를 참조해서 쓴다.
class Hardware {
 public:
  virtual ~Hardware() = default;

  virtual void start() = 0;
  virtual void stop() = 0;
  // 연결/해제는 링크만 다룬다. start()/stop()은 워커 스레드/파이프라인 수명주기다.
  // 자동 재연결 구현은 disconnect() 뒤에 reader 루프가 다시 열지 않아야 한다.
  virtual void connect() = 0;
  virtual void disconnect() = 0;
  [[nodiscard]] virtual bool is_connected() const = 0;
};

}  // namespace krosc::hardware
