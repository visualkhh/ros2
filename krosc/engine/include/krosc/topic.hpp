#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

#include "krosc/executor.hpp"
#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"

namespace krosc {

namespace detail {

struct TopicState;

// 구독 하나. 큐와 콜백은 이 객체가 들고, 스레드/스케줄은 Executor가 맡는다.
// 같은 구독의 메시지는 drain 작업 하나로만 처리되어 순서가 유지된다 (구독마다 스레드를 만들지 않는다).
class SubscriptionBase : public std::enable_shared_from_this<SubscriptionBase> {
 public:
  SubscriptionBase(std::string topic, QoS qos, Executor& executor, std::weak_ptr<TopicState> state);
  virtual ~SubscriptionBase() = default;
  SubscriptionBase(const SubscriptionBase&) = delete;
  SubscriptionBase& operator=(const SubscriptionBase&) = delete;

  [[nodiscard]] const std::string& topic() const { return topic_; }
  [[nodiscard]] std::uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  [[nodiscard]] std::uint64_t delivered() const { return delivered_.load(std::memory_order_relaxed); }
  [[nodiscard]] bool closed() const { return closed_.load(std::memory_order_acquire); }

  virtual void push_void(const std::shared_ptr<const void>& message) = 0;
  virtual void drain() = 0;  // Executor가 부르는 큐 비우기 (같은 구독 안에서 순서 보장)

  void start();
  void cancel();
  void close();

 protected:
  virtual void enqueue_void(const std::shared_ptr<const void>& message) = 0;
  virtual void clear_queue_locked() = 0;

  void schedule();

  const std::string topic_;
  const QoS qos_;
  Executor& executor_;
  std::weak_ptr<TopicState> state_;
  mutable std::mutex mutex_;
  bool scheduled_ = false;
  bool paused_ = true;
  std::atomic<bool> closed_{false};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> delivered_{0};
};

template <class T>
class SubscriptionImpl final : public SubscriptionBase {
 public:
  SubscriptionImpl(std::string topic, QoS qos, Executor& executor, std::weak_ptr<TopicState> state,
                   std::function<void(const T&)> callback)
      : SubscriptionBase(std::move(topic), qos, executor, std::move(state)), callback_(std::move(callback)) {}

  void push_void(const std::shared_ptr<const void>& message) override {
    auto typed = std::shared_ptr<const T>(message, static_cast<const T*>(message.get()));
    enqueue_void(typed);
  }

 protected:
  void enqueue_void(const std::shared_ptr<const void>& message) override {
    auto typed = std::shared_ptr<const T>(message, static_cast<const T*>(message.get()));
    bool post = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_.load(std::memory_order_relaxed)) {
        return;
      }
      queue_.push_back(typed);
      trim_locked();
      if (!paused_ && !scheduled_) {
        scheduled_ = true;
        post = true;
      }
    }
    if (post) {
      auto self = shared_from_this();
      executor_.post([self = std::move(self)] { self->drain(); });
    }
  }

  void clear_queue_locked() override { queue_.clear(); }

  void drain() override {
    for (;;) {
      std::shared_ptr<const T> message;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_.load(std::memory_order_relaxed) || queue_.empty()) {
          scheduled_ = false;
          return;
        }
        message = std::move(queue_.front());
        queue_.pop_front();
      }
      try {
        callback_(*message);
      } catch (const std::exception& error) {
        std::fprintf(stderr, "[krosc] subscriber of '%s' failed: %s\n", topic_.c_str(), error.what());
      } catch (...) {
        std::fprintf(stderr, "[krosc] subscriber of '%s' failed: unknown exception\n", topic_.c_str());
      }
      delivered_.fetch_add(1, std::memory_order_relaxed);
    }
  }

 private:
  void trim_locked() {
    if (!qos_.depth.has_value()) {
      return;
    }
    const std::size_t depth = *qos_.depth;
    while (queue_.size() > depth) {
      queue_.pop_front();
      dropped_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  std::function<void(const T&)> callback_;
  std::deque<std::shared_ptr<const T>> queue_;
};

using SubscriberList = std::vector<std::shared_ptr<SubscriptionBase>>;

struct ChannelBase {
  std::type_index type{typeid(void)};
  std::string type_name;
  // 발행은 이 스냅샷의 shared_ptr만 복사한다 (구독 목록을 매 메시지마다 복사하지 않는다).
  std::shared_ptr<const SubscriberList> subscribers = std::make_shared<const SubscriberList>();
  std::atomic<std::uint64_t> messages{0};
  virtual ~ChannelBase() = default;
};

template <class T>
struct Channel final : ChannelBase {
  std::deque<std::shared_ptr<const T>> history;
};

struct TopicState {
  mutable std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<ChannelBase>> channels;
  bool stopping = false;
};

}  // namespace detail

// 구독 손잡이. cancel()로 명시적으로 끊는다. Node가 만든 구독은 Node가 소멸할 때 정리한다.
template <class T>
class Subscription {
 public:
  Subscription() = default;
  explicit Subscription(std::shared_ptr<detail::SubscriptionImpl<T>> impl) : impl_(std::move(impl)) {}

  void cancel() {
    if (impl_) {
      impl_->cancel();
      impl_.reset();
    }
  }

  [[nodiscard]] bool valid() const { return impl_ && !impl_->closed(); }
  [[nodiscard]] std::uint64_t dropped() const { return impl_ ? impl_->dropped() : 0; }
  [[nodiscard]] std::uint64_t delivered() const { return impl_ ? impl_->delivered() : 0; }
  [[nodiscard]] std::string topic() const { return impl_ ? impl_->topic() : std::string{}; }
  [[nodiscard]] std::shared_ptr<detail::SubscriptionBase> base() const { return impl_; }

 private:
  std::shared_ptr<detail::SubscriptionImpl<T>> impl_;
};

// 토픽 이름 -> 타입이 고정된 채널. 발행은 shared_ptr<const T>를 구독자 큐에 넣기만 하므로
// 구독자 수와 무관하게 메시지 복사가 한 번도 없다.
class TopicRegistry {
 public:
  explicit TopicRegistry(Executor& executor);
  ~TopicRegistry();

  TopicRegistry(const TopicRegistry&) = delete;
  TopicRegistry& operator=(const TopicRegistry&) = delete;

  template <class T>
  Subscription<T> subscribe(const std::string& topic, std::function<void(const T&)> callback, QoS qos = QOS_DEFAULT) {
    auto state = state_;
    std::shared_ptr<detail::SubscriptionImpl<T>> impl;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->stopping) {
        throw std::runtime_error("TopicRegistry가 종료됐다");
      }
      auto channel = get_or_create_channel_locked<T>(state, topic);
      impl = std::make_shared<detail::SubscriptionImpl<T>>(topic, qos, executor_, state, std::move(callback));
      {
        auto subscribers = std::make_shared<detail::SubscriberList>(*channel->subscribers);
        subscribers->push_back(impl);
        channel->subscribers = std::move(subscribers);
      }
      if (qos.latched()) {
        const std::size_t depth = qos.depth.value_or(1);
        const std::size_t begin = channel->history.size() > depth ? channel->history.size() - depth : 0;
        for (std::size_t i = begin; i < channel->history.size(); ++i) {
          impl->push_void(channel->history[i]);  // paused 상태라 큐에만 쌓인다
        }
      }
    }
    impl->start();
    return Subscription<T>(std::move(impl));
  }

  template <class T>
  void publish(const std::string& topic, T message, QoS qos = QOS_DEFAULT) {
    publish<T>(topic, std::make_shared<const T>(std::move(message)), qos);
  }

  template <class T>
  void publish(const std::string& topic, std::shared_ptr<const T> message, QoS qos = QOS_DEFAULT) {
    if (!message) {
      return;
    }
    auto state = state_;
    std::shared_ptr<const detail::SubscriberList> subscribers;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->stopping) {
        return;
      }
      auto channel = get_or_create_channel_locked<T>(state, topic);
      channel->messages.fetch_add(1, std::memory_order_relaxed);
      if (qos.latched()) {
        const std::size_t depth = qos.depth.value_or(1);
        channel->history.push_back(message);
        while (channel->history.size() > depth) {
          channel->history.pop_front();
        }
      }
      subscribers = channel->subscribers;
    }
    for (const auto& subscriber : *subscribers) {
      subscriber->push_void(message);
    }
  }

  void shutdown();

  struct TopicInfo {
    std::string topic;
    std::string type;
    std::size_t subscribers = 0;
    std::uint64_t messages = 0;
  };
  [[nodiscard]] std::vector<TopicInfo> topic_info() const;

 private:
  template <class T>
  static std::shared_ptr<detail::Channel<T>> get_or_create_channel_locked(
      const std::shared_ptr<detail::TopicState>& state, const std::string& topic) {
    auto it = state->channels.find(topic);
    if (it != state->channels.end()) {
      if (it->second->type != std::type_index(typeid(T))) {
        throw std::runtime_error("topic '" + topic + "' is " + it->second->type_name + ", got " +
                                 std::string(msgs::message_type_name<T>()));
      }
      return std::static_pointer_cast<detail::Channel<T>>(it->second);
    }
    auto channel = std::make_shared<detail::Channel<T>>();
    channel->type = std::type_index(typeid(T));
    channel->type_name = std::string(msgs::message_type_name<T>());
    state->channels.emplace(topic, channel);
    return channel;
  }

  Executor& executor_;
  std::shared_ptr<detail::TopicState> state_;
};

}  // namespace krosc
