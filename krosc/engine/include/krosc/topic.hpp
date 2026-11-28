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
struct ChannelBase;

// 구독 하나. 큐와 콜백은 이 객체가 들고, 스레드/스케줄은 Executor가 맡는다.
// 같은 구독의 메시지는 drain 작업 하나로만 처리되어 순서가 유지된다 (구독마다 스레드를 만들지 않는다).
class SubscriptionBase : public std::enable_shared_from_this<SubscriptionBase> {
 public:
  SubscriptionBase(std::string topic, QoS qos, Executor& executor, std::shared_ptr<Strand> strand,
                   std::weak_ptr<TopicState> state, std::weak_ptr<ChannelBase> channel);
  virtual ~SubscriptionBase() = default;
  SubscriptionBase(const SubscriptionBase&) = delete;
  SubscriptionBase& operator=(const SubscriptionBase&) = delete;

  [[nodiscard]] const std::string& topic() const { return topic_; }
  [[nodiscard]] std::uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  [[nodiscard]] std::uint64_t delivered() const { return delivered_.load(std::memory_order_relaxed); }
  [[nodiscard]] bool closed() const { return closed_.load(std::memory_order_acquire); }

  virtual void push(const std::shared_ptr<const void>& message) = 0;
  virtual void drain() = 0;  // Executor가 부르는 큐 비우기 (같은 구독 안에서 순서 보장)

  void start();
  void cancel();
  void close();

 protected:
  virtual void clear_queue_locked() = 0;

  void schedule();
  void post_drain();  // strand가 있으면 그 줄로, 없으면 Executor로 drain을 올린다

  const std::string topic_;
  const QoS qos_;
  Executor& executor_;
  std::shared_ptr<Strand> strand_;  // Node의 직렬 줄 (없으면 구독 단위로만 직렬)
  std::weak_ptr<TopicState> state_;
  std::weak_ptr<ChannelBase> channel_;  // cancel()이 이 채널의 구독 목록에서만 자신을 뺀다
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
  SubscriptionImpl(std::string topic, QoS qos, Executor& executor, std::shared_ptr<Strand> strand,
                   std::weak_ptr<TopicState> state, std::weak_ptr<ChannelBase> channel,
                   std::function<void(const T&)> callback)
      : SubscriptionBase(std::move(topic), qos, executor, std::move(strand), std::move(state), std::move(channel)),
        callback_(std::move(callback)) {}

  void push(const std::shared_ptr<const void>& message) override {
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
      post_drain();
    }
  }

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

 protected:
  void clear_queue_locked() override { queue_.clear(); }

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

using PublishHook = std::function<void(const std::string&, std::type_index,
                                      const std::shared_ptr<const void>&, QoS)>;
using SubscriptionHook = std::function<void()>;

struct TopicState {
  mutable std::mutex mutex;
  std::unordered_map<std::string, std::shared_ptr<ChannelBase>> channels;
  PublishHook publish_hook;
  SubscriptionHook subscription_hook;
  bool stopping = false;
};

// 발행 본체. 채널을 이미 찾은 상태에서 부른다 (Publisher는 이름 조회 없이 바로 온다).
// lock은 잡힌 채로 들어오고, 구독자 큐에 넣기 전에 풀린다.
template <class T>
void publish_locked(TopicState& state, std::unique_lock<std::mutex>& lock, Channel<T>& channel,
                    const std::string& topic, const std::shared_ptr<const T>& message, QoS qos) {
  channel.messages.fetch_add(1, std::memory_order_relaxed);
  if (qos.latched()) {
    const std::size_t depth = qos.depth.value_or(1);
    channel.history.push_back(message);
    while (channel.history.size() > depth) {
      channel.history.pop_front();
    }
  }
  const std::shared_ptr<const SubscriberList> subscribers = channel.subscribers;
  PublishHook publish_hook = state.publish_hook;
  lock.unlock();
  for (const auto& subscriber : *subscribers) {
    subscriber->push(message);
  }
  if (publish_hook) {
    publish_hook(topic, std::type_index(typeid(T)), std::shared_ptr<const void>(message), qos);
  }
}

}  // namespace detail

// 발행 손잡이 (rclcpp::Publisher). 만들 때 채널을 한 번 찾아 두고, publish마다 이름 해시/맵 조회를
// 하지 않는다. 타입이 다른 채널이면 만들 때 예외가 난다. 복사해도 같은 채널을 가리킨다.
template <class T>
class Publisher {
 public:
  Publisher() = default;
  Publisher(std::shared_ptr<detail::TopicState> state, std::shared_ptr<detail::Channel<T>> channel, std::string topic,
            QoS qos)
      : state_(std::move(state)), channel_(std::move(channel)), topic_(std::move(topic)), qos_(qos) {}

  void publish(T message) const { publish(std::make_shared<const T>(std::move(message))); }

  void publish(std::shared_ptr<const T> message) const {
    if (!message || !state_) {
      return;
    }
    std::unique_lock<std::mutex> lock(state_->mutex);
    if (state_->stopping) {
      return;
    }
    detail::publish_locked<T>(*state_, lock, *channel_, topic_, message, qos_);
  }

  [[nodiscard]] bool valid() const { return state_ != nullptr; }
  [[nodiscard]] const std::string& topic() const { return topic_; }
  [[nodiscard]] QoS qos() const { return qos_; }

 private:
  std::shared_ptr<detail::TopicState> state_;
  std::shared_ptr<detail::Channel<T>> channel_;
  std::string topic_;
  QoS qos_{};
};

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

  // strand: 콜백을 올릴 직렬 줄. Node::subscribe가 자기 줄을 넘긴다. 없으면 구독 단위로만 순서를 지킨다.
  template <class T>
  Subscription<T> subscribe(const std::string& topic, std::function<void(const T&)> callback, QoS qos = QOS_DEFAULT,
                            std::shared_ptr<Strand> strand = nullptr) {
    auto state = state_;
    std::shared_ptr<detail::SubscriptionImpl<T>> impl;
    detail::SubscriptionHook subscription_hook;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->stopping) {
        throw std::runtime_error("TopicRegistry가 종료됐다");
      }
      auto channel = get_or_create_channel_locked<T>(state, topic);
      impl = std::make_shared<detail::SubscriptionImpl<T>>(topic, qos, executor_, std::move(strand), state, channel,
                                                           std::move(callback));
      {
        auto subscribers = std::make_shared<detail::SubscriberList>(*channel->subscribers);
        subscribers->push_back(impl);
        channel->subscribers = std::move(subscribers);
      }
      if (qos.latched()) {
        const std::size_t depth = qos.depth.value_or(1);
        const std::size_t begin = channel->history.size() > depth ? channel->history.size() - depth : 0;
        for (std::size_t i = begin; i < channel->history.size(); ++i) {
          impl->push(channel->history[i]);  // paused 상태라 큐에만 쌓인다
        }
      }
      subscription_hook = state->subscription_hook;
    }
    impl->start();
    if (subscription_hook) {
      subscription_hook();
    }
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
    std::unique_lock<std::mutex> lock(state_->mutex);
    if (state_->stopping) {
      return;
    }
    auto channel = get_or_create_channel_locked<T>(state_, topic);
    detail::publish_locked<T>(*state_, lock, *channel, topic, message, qos);
  }

  template <class T>
  Publisher<T> create_publisher(const std::string& topic, QoS qos = QOS_DEFAULT) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->stopping) {
      throw std::runtime_error("TopicRegistry가 종료됐다");
    }
    return Publisher<T>(state_, get_or_create_channel_locked<T>(state_, topic), topic, qos);
  }

  void set_publish_hook(detail::PublishHook hook) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->publish_hook = std::move(hook);
  }

  void set_subscription_hook(detail::SubscriptionHook hook) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->subscription_hook = std::move(hook);
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
