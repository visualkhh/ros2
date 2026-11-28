#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "krosc/executor.hpp"
#include "krosc/time/time.hpp"

namespace krosc {

namespace detail {
struct TimerState;
struct TimerManagerState;
}  // namespace detail

// 타이머 손잡이. cancel()은 여러 번 불러도 안전하다.
class Timer {
 public:
  Timer() = default;

  [[nodiscard]] double period() const;
  [[nodiscard]] std::string name() const;
  [[nodiscard]] std::uint64_t calls() const;
  [[nodiscard]] std::uint64_t missed() const;
  [[nodiscard]] bool canceled() const;
  void cancel();

 private:
  friend class TimerManager;
  Timer(std::weak_ptr<detail::TimerManagerState> manager, std::shared_ptr<detail::TimerState> state)
      : manager_(std::move(manager)), state_(std::move(state)) {}

  std::weak_ptr<detail::TimerManagerState> manager_;
  std::shared_ptr<detail::TimerState> state_;
};

// 타이머 레지스트리와 스케줄.
//
// - 실제 시간: 스케줄 스레드 하나가 가장 가까운 주기까지 기다렸다 깨운다 (타이머마다 스레드를 만들지 않는다).
// - 시뮬레이션 시간: 시간을 set하는 스레드가 그 자리에서 바로 판정한다 (결정적, 스케줄 스레드 없음).
// - 콜백은 Executor에서 돈다 (strand를 주면 그 줄에서). inline Executor면 set()/스케줄 스레드에서 바로 실행된다.
// - 시간이 한꺼번에 건너뛰면 한 번만 부르고 나머지는 missed로 센다 (박자는 유지).
class TimerManager {
 public:
  TimerManager(TimeSource& time_source, Executor& executor);
  ~TimerManager();

  TimerManager(const TimerManager&) = delete;
  TimerManager& operator=(const TimerManager&) = delete;

  // strand: 콜백을 올릴 직렬 줄 (Node가 자기 줄을 넘긴다). 없으면 Executor로 바로 올린다.
  Timer create(double period, std::function<void()> callback, std::string name = {},
               std::shared_ptr<Strand> strand = nullptr);
  [[nodiscard]] bool join(std::optional<double> timeout_seconds = std::nullopt);
  void shutdown();
  [[nodiscard]] std::size_t timer_count() const;

  struct TimerStats {
    std::string name;
    double period = 0.0;
    std::uint64_t calls = 0;
    std::uint64_t missed = 0;
  };
  [[nodiscard]] std::vector<TimerStats> stats() const;

 private:
  std::shared_ptr<detail::TimerManagerState> state_;
  TimeSource& time_source_;
  Executor& executor_;
};

}  // namespace krosc
