#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "krosc/executor.hpp"
#include "krosc/parameter.hpp"
#include "krosc/service.hpp"
#include "krosc/time.hpp"
#include "krosc/timer.hpp"
#include "krosc/topic.hpp"

namespace krosc {

// 노드들이 공유하는 묶음: 실행기, 토픽/서비스, 타이머, 파라미터, 시간 원천.
class Context {
 public:
  struct Options {
    // 0이면 inline 실행기 (시뮬레이션/테스트에 결정적이고 빠르다), >0이면 워커 스레드 풀.
    std::size_t executor_threads = 0;
    std::shared_ptr<TimeSource> time_source;  // 없으면 SystemTimeSource
    ParameterManager::Overrides parameters;
    bool publish_parameter_events = true;
    std::string name = "krosc";
  };

  Context();
  explicit Context(Options options);
  ~Context();

  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;

  [[nodiscard]] Executor& executor() { return *executor_; }
  [[nodiscard]] TopicRegistry& topics() { return *topics_; }
  [[nodiscard]] ServiceRegistry& services() { return *services_; }
  [[nodiscard]] TimerManager& timers() { return *timers_; }
  [[nodiscard]] ParameterManager& parameters() { return *parameters_; }
  [[nodiscard]] TimeSource& time_source() { return *time_source_; }
  [[nodiscard]] double now() const { return time_source_->now(); }
  [[nodiscard]] const std::string& name() const { return name_; }

  void register_node(const std::string& name);
  [[nodiscard]] std::vector<std::string> node_names() const;

  void shutdown();

 private:
  std::string name_;
  // 선언 역순으로 소멸한다: parameters -> timers -> services -> topics -> executor -> time_source.
  std::shared_ptr<TimeSource> time_source_;
  std::unique_ptr<Executor> executor_;
  std::unique_ptr<TopicRegistry> topics_;
  std::unique_ptr<ServiceRegistry> services_;
  std::unique_ptr<TimerManager> timers_;
  std::unique_ptr<ParameterManager> parameters_;
  mutable std::mutex node_mutex_;
  std::set<std::string> node_names_;
  bool shutdown_ = false;
};

}  // namespace krosc
