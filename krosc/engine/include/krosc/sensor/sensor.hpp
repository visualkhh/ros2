#pragma once

#include <string>

namespace krosc::sensor {

// 센서 플러그인 정보. control의 HardwareInfo와 같은 역할이지만,
// 센서 계층이 control에 의존하지 않도록 별도로 둔다.
struct SensorInfo {
  std::string name = "sensor";
  std::string type;
};

// read-only 센서의 수명주기: on_init -> on_activate -> read 반복 -> on_deactivate.
// LaserScan 같은 배열 데이터는 드라이버가 토픽으로 내보내고, 이 인터페이스는 상태와 수명만 다룬다.
class Sensor {
 public:
  virtual ~Sensor() = default;

  virtual bool on_init(const SensorInfo& info) {
    info_ = info;
    return true;
  }
  virtual bool on_activate() { return true; }
  virtual bool on_deactivate() { return true; }
  virtual bool read(double time, double period) = 0;

  [[nodiscard]] const SensorInfo& info() const { return info_; }

 private:
  SensorInfo info_;
};

}  // namespace krosc::sensor
