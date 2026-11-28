#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
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

// 직렬 실행 줄 (Boost.Asio strand, ROS 2 MutuallyExclusive callback group).
// 스레드가 아니라 큐 하나와 "실행 중" 플래그다. 이 줄의 작업은 Executor 공용 큐에 한 번에 하나만
// 올라가므로 서로 동시에 돌지 않는다. 다른 Strand의 작업과는 같은 워커 풀에서 병렬로 돈다.
// 기다리는 작업은 이 큐에 있고 워커 스레드를 차지하지 않는다.
//
// inline Executor에서도 같다: 비어 있으면 부른 스레드에서 바로 실행하고, 이미 실행 중이면
// (콜백 안에서 자기 노드로 발행했거나 다른 스레드가 실행 중) 큐에 넣어 그 실행이 이어서 처리한다.
class Strand : public std::enable_shared_from_this<Strand> {
 public:
  explicit Strand(Executor& executor) : executor_(executor) {}

  Strand(const Strand&) = delete;
  Strand& operator=(const Strand&) = delete;

  void post(std::function<void()> task);

 private:
  void drain();

  Executor& executor_;
  std::mutex mutex_;
  std::deque<std::function<void()>> queue_;
  bool running_ = false;
};

}  // namespace krosc
