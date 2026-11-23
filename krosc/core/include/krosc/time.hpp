#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace krosc {

// 시간 원천: 실제 시간이거나 시뮬레이션이 밀어 주는 시간. ROS의 use_sim_time과 같은 역할.
class TimeSource {
 public:
  virtual ~TimeSource() = default;
  [[nodiscard]] virtual double now() const = 0;        // 메시지 시각 등에 쓰는 시각(초)
  [[nodiscard]] virtual double monotonic() const = 0;  // 타이머 스케줄용 단조 시각(초)
  [[nodiscard]] virtual bool realtime() const = 0;     // true면 시간이 스스로 흐른다
  virtual void add_listener(std::function<void()> /*callback*/) {}
};

class SystemTimeSource final : public TimeSource {
 public:
  [[nodiscard]] double now() const override;
  [[nodiscard]] double monotonic() const override;
  [[nodiscard]] bool realtime() const override { return true; }
};

class SimulatedTimeSource final : public TimeSource {
 public:
  explicit SimulatedTimeSource(double start = 0.0);
  [[nodiscard]] double now() const override;
  [[nodiscard]] double monotonic() const override { return now(); }
  [[nodiscard]] bool realtime() const override { return false; }
  void add_listener(std::function<void()> callback) override;
  void set(double time);
  void advance(double dt);

 private:
  mutable std::mutex mutex_;
  double time_;
  std::vector<std::function<void()>> listeners_;
};

}  // namespace krosc
