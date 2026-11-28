#pragma once

#include <memory>

namespace srf_default_ws::sim {

// 시뮬레이터 OpenCR의 부저를 macOS/Linux 스피커로 낸다.
// 엔진 스레드에서 play()를 호출해도 Qt 메인 스레드로 넘겨 QAudioSink에서 재생한다.
class SimulatorBuzzer {
 public:
  SimulatorBuzzer();
  ~SimulatorBuzzer();

  SimulatorBuzzer(const SimulatorBuzzer&) = delete;
  SimulatorBuzzer& operator=(const SimulatorBuzzer&) = delete;

  void play(int frequency_hz, int duration_ms);

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace srf_default_ws::sim
