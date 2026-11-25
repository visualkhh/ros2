#pragma once

// krosc 액션: ROS 2 action처럼 Goal / Feedback / Result를 가진 장기 실행 작업.
// 로컬에서는 서비스+이벤트 토픽으로, 원격에서는 PeerNetwork RPC가 같은 API를 포워딩한다.
//
// ActionServer: goal을 큐에 넣고 전용 워커 스레드가 하나씩 실행한다 (ROS action server처럼).
// ActionClient: goal을 보내고 결과를 future로 기다린다. 피드백은 콜백으로 받는다.

#include <any>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "krosc/action_types.hpp"
#include "krosc/context.hpp"
#include "krosc/service.hpp"

namespace krosc {

struct ActionResult {
  GoalStatus status = GoalStatus::Accepted;
  std::any result;
};

inline std::string make_goal_id() {
  static std::atomic<std::uint64_t> counter{1};
  return "goal_" + std::to_string(counter.fetch_add(1));
}

template <class Goal, class Feedback>
class ServerGoalHandle {
 public:
  using FeedbackPublisher = std::function<void(const std::string&, const Feedback&)>;

  ServerGoalHandle(std::string goal_id, Goal goal, FeedbackPublisher publish)
      : goal_id_(std::move(goal_id)), goal_(std::move(goal)), publish_(std::move(publish)) {}

  [[nodiscard]] const std::string& goal_id() const { return goal_id_; }
  [[nodiscard]] const Goal& goal() const { return goal_; }
  [[nodiscard]] bool is_cancel_requested() const { return cancel_.load(std::memory_order_acquire); }
  void request_cancel() { cancel_.store(true, std::memory_order_release); }
  void publish_feedback(const Feedback& feedback) {
    if (publish_) {
      publish_(goal_id_, feedback);
    }
  }

 private:
  std::string goal_id_;
  Goal goal_;
  FeedbackPublisher publish_;
  std::atomic<bool> cancel_{false};
};

template <class Goal, class Feedback, class Result>
class ActionServer {
 public:
  using Execute = std::function<Result(ServerGoalHandle<Goal, Feedback>&)>;
  using GoalCallback = std::function<bool(const Goal&)>;

  ActionServer(Context& context, std::string name, Execute execute, GoalCallback goal_callback = {})
      : context_(context),
        name_(std::move(name)),
        execute_(std::move(execute)),
        goal_callback_(std::move(goal_callback)) {
    send_service_ = context_.services().template create_service<SendGoalRequest, SendGoalResponse>(
        name_ + "/send_goal", [this](const SendGoalRequest& request) { return on_send_goal(request); });
    cancel_service_ = context_.services().template create_service<CancelGoalRequest, CancelGoalResponse>(
        name_ + "/cancel_goal", [this](const CancelGoalRequest& request) { return on_cancel_goal(request); });
    worker_ = std::thread([this] { run(); });
  }

  ~ActionServer() { shutdown(); }

  ActionServer(const ActionServer&) = delete;
  ActionServer& operator=(const ActionServer&) = delete;

  [[nodiscard]] const std::string& name() const { return name_; }

  void shutdown() {
    if (stopping_.exchange(true)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto& [id, handle] : handles_) {
        (void)id;
        handle->request_cancel();
      }
      queue_.clear();
    }
    cv_.notify_all();
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
      worker_.join();
    }
    send_service_.destroy();
    cancel_service_.destroy();
  }

 private:
  struct Pending {
    std::string goal_id;
    Goal goal;
  };
  using Handle = ServerGoalHandle<Goal, Feedback>;

  SendGoalResponse on_send_goal(const SendGoalRequest& request) {
    Goal goal;
    try {
      goal = std::any_cast<Goal>(request.goal);
    } catch (const std::bad_any_cast&) {
      return SendGoalResponse{false};
    }
    if (goal_callback_ && !goal_callback_(goal)) {
      return SendGoalResponse{false};
    }
    auto handle = std::make_shared<Handle>(
        request.goal_id, goal, [this](const std::string& id, const Feedback& feedback) {
          ActionEvent event;
          event.goal_id = id;
          event.has_feedback = true;
          event.feedback = feedback;
          publish(event);
        });
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handles_[request.goal_id] = handle;
      queue_.push_back(Pending{request.goal_id, goal});
    }
    cv_.notify_one();
    return SendGoalResponse{true};
  }

  CancelGoalResponse on_cancel_goal(const CancelGoalRequest& request) {
    std::shared_ptr<Handle> handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = handles_.find(request.goal_id);
      if (it != handles_.end()) {
        handle = it->second;
      }
    }
    if (!handle) {
      return CancelGoalResponse{false};
    }
    handle->request_cancel();
    return CancelGoalResponse{true};
  }

  void run() {
    for (;;) {
      Pending pending;
      std::shared_ptr<Handle> handle;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stopping_.load() || !queue_.empty(); });
        if (stopping_.load()) {
          return;
        }
        pending = std::move(queue_.front());
        queue_.pop_front();
        auto it = handles_.find(pending.goal_id);
        if (it == handles_.end()) {
          continue;
        }
        handle = it->second;
      }
      if (!handle->is_cancel_requested()) {
        try {
          Result result = execute_(*handle);
          {
            std::lock_guard<std::mutex> lock(mutex_);
            handles_.erase(pending.goal_id);
          }
          ActionEvent event;
          event.goal_id = pending.goal_id;
          event.has_result = true;
          event.result = std::any(std::move(result));
          event.status = handle->is_cancel_requested() ? GoalStatus::Canceled : GoalStatus::Succeeded;
          publish(event);
          continue;
        } catch (const std::exception& error) {
          std::fprintf(stderr, "[krosc/action] %s goal %s aborted: %s\n", name_.c_str(),
                       pending.goal_id.c_str(), error.what());
        } catch (...) {
          std::fprintf(stderr, "[krosc/action] %s goal %s aborted\n", name_.c_str(), pending.goal_id.c_str());
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        handles_.erase(pending.goal_id);
      }
      ActionEvent event;
      event.goal_id = pending.goal_id;
      event.has_result = true;
      event.status = handle->is_cancel_requested() ? GoalStatus::Canceled : GoalStatus::Aborted;
      publish(event);
    }
  }

  void publish(const ActionEvent& event) {
    context_.topics().publish(name_ + "/_action/event", event);
  }

  Context& context_;
  std::string name_;
  Execute execute_;
  GoalCallback goal_callback_;
  ServiceServer send_service_;
  ServiceServer cancel_service_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Pending> queue_;
  std::unordered_map<std::string, std::shared_ptr<Handle>> handles_;
  std::atomic<bool> stopping_{false};
};

template <class Goal, class Feedback, class Result>
class ClientGoalHandle;

template <class Goal, class Feedback, class Result>
class ActionClient {
 public:
  using FeedbackCallback = std::function<void(const Feedback&)>;

  ActionClient(Context& context, std::string name) : context_(context), name_(std::move(name)) {
    event_subscription_ = context_.topics().template subscribe<ActionEvent>(
        name_ + "/_action/event", [this](const ActionEvent& event) { on_event(event); });
  }

  ~ActionClient() { shutdown(); }

  ActionClient(const ActionClient&) = delete;
  ActionClient& operator=(const ActionClient&) = delete;

  [[nodiscard]] const std::string& name() const { return name_; }

  std::shared_ptr<ClientGoalHandle<Goal, Feedback, Result>> send_goal(const Goal& goal,
                                                                      FeedbackCallback feedback = {}) {
    auto handle = std::make_shared<ClientGoalHandle<Goal, Feedback, Result>>(make_goal_id(), *this,
                                                                            std::move(feedback));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handles_[handle->goal_id()] = handle;
    }
    try {
      const SendGoalResponse response =
          context_.services().template call<SendGoalRequest, SendGoalResponse>(name_ + "/send_goal",
                                                                     SendGoalRequest{handle->goal_id(), std::any(goal)});
      if (!response.accepted) {
        forget(handle->goal_id());
        handle->finish(ActionResult{GoalStatus::Rejected, std::any{}});
      }
    } catch (...) {
      forget(handle->goal_id());
      throw;
    }
    return handle;
  }

  bool cancel(const std::string& goal_id) {
    try {
      const CancelGoalResponse response =
          context_.services().template call<CancelGoalRequest, CancelGoalResponse>(name_ + "/cancel_goal",
                                                                        CancelGoalRequest{goal_id});
      return response.canceled;
    } catch (const ServiceNotFoundError&) {
      return false;
    }
  }

  void shutdown() {
    if (stopping_.exchange(true)) {
      return;
    }
    event_subscription_.cancel();
    std::deque<std::shared_ptr<ClientGoalHandle<Goal, Feedback, Result>>> pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto& [id, handle] : handles_) {
        (void)id;
        pending.push_back(handle);
      }
      handles_.clear();
    }
    for (auto& handle : pending) {
      handle->finish(ActionResult{GoalStatus::Aborted, std::any{}});
    }
  }

 private:
  void forget(const std::string& goal_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.erase(goal_id);
  }

  void on_event(const ActionEvent& event) {
    std::shared_ptr<ClientGoalHandle<Goal, Feedback, Result>> handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = handles_.find(event.goal_id);
      if (it == handles_.end()) {
        return;
      }
      handle = it->second;
    }
    if (event.has_result) {
      ActionResult result;
      result.status = event.status;
      if (event.result.has_value()) {
        try {
          result.result = std::any_cast<Result>(event.result);
        } catch (const std::bad_any_cast&) {
          result.status = GoalStatus::Aborted;
        }
      }
      forget(event.goal_id);
      handle->finish(std::move(result));
    } else if (event.has_feedback && handle->feedback_callback()) {
      try {
        handle->feedback_callback()(std::any_cast<Feedback>(event.feedback));
      } catch (const std::bad_any_cast&) {
      }
    }
  }

  Context& context_;
  std::string name_;
  Subscription<ActionEvent> event_subscription_;
  std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<ClientGoalHandle<Goal, Feedback, Result>>> handles_;
  std::atomic<bool> stopping_{false};
};

template <class Goal, class Feedback, class Result>
class ClientGoalHandle : public std::enable_shared_from_this<ClientGoalHandle<Goal, Feedback, Result>> {
 public:
  using FeedbackCallback = std::function<void(const Feedback&)>;

  ClientGoalHandle(std::string goal_id, ActionClient<Goal, Feedback, Result>& client,
                   FeedbackCallback feedback)
      : goal_id_(std::move(goal_id)), client_(client), feedback_(std::move(feedback)) {}

  [[nodiscard]] const std::string& goal_id() const { return goal_id_; }
  [[nodiscard]] GoalStatus status() const { return status_.load(); }
  [[nodiscard]] bool accepted() const { return status() != GoalStatus::Rejected; }
  [[nodiscard]] const FeedbackCallback& feedback_callback() const { return feedback_; }

  ActionResult result(std::optional<double> timeout = std::nullopt) {
    if (timeout) {
      return future_.wait_for(std::chrono::duration<double>(*timeout)) == std::future_status::ready
                 ? future_.get()
                 : ActionResult{GoalStatus::Accepted, std::any{}};
    }
    return future_.get();
  }

  bool cancel() { return client_.cancel(goal_id_); }

  void finish(ActionResult result) {
    status_.store(result.status);
    if (!promise_set_.exchange(true)) {
      promise_.set_value(std::move(result));
    }
  }

 private:
  std::string goal_id_;
  ActionClient<Goal, Feedback, Result>& client_;
  FeedbackCallback feedback_;
  std::atomic<GoalStatus> status_{GoalStatus::Accepted};
  std::promise<ActionResult> promise_;
  std::shared_future<ActionResult> future_ = promise_.get_future();
  std::atomic<bool> promise_set_{false};
};

}  // namespace krosc
