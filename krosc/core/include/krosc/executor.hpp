#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace krosc {

// 콜백 실행기.
//
// threads == 0: inline 모드. post()가 부른 스레드에서 바로 콜백을 실행한다. 시뮬레이션/테스트처럼
//   결정적이고 지연이 중요한 곳에 쓴다 (스레드 생성/락 없음).
// threads > 0: 공용 워커 스레드 풀. 구독마다 스레드를 만들지 않고, 구독 하나의 메시지는
//   한 번에 하나의 drain 작업으로만 큐에 올라가 순서가 유지된다.
//
// join()은 지금까지 post된 작업(작업 안에서 post한 것 포함)이 모두 끝날 때까지 기다린다.
class Executor {
 public:
  explicit Executor(std::size_t threads = 0);
  ~Executor();

  Executor(const Executor&) = delete;
  Executor& operator=(const Executor&) = delete;

  void post(std::function<void()> task);
  void join();
  void shutdown();

  [[nodiscard]] bool inline_mode() const { return threads_ == 0; }
  [[nodiscard]] std::size_t thread_count() const { return threads_; }
  [[nodiscard]] std::size_t pending() const;

 private:
  void worker_loop();

  const std::size_t threads_;
  mutable std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable idle_cv_;
  std::queue<std::function<void()>> tasks_;
  std::vector<std::thread> workers_;
  std::size_t running_ = 0;
  bool stopping_ = false;
};

}  // namespace krosc
