#include "krosc/topic.hpp"

#include <algorithm>
#include <utility>

namespace krosc {

namespace detail {

SubscriptionBase::SubscriptionBase(std::string topic, QoS qos, Executor& executor, std::weak_ptr<TopicState> state)
    : topic_(std::move(topic)), qos_(qos), executor_(executor), state_(std::move(state)) {}

void SubscriptionBase::schedule() {
  bool post = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!paused_ && !scheduled_ && !closed_.load(std::memory_order_relaxed)) {
      scheduled_ = true;
      post = true;
    }
  }
  if (post) {
    auto self = shared_from_this();
    executor_.post([self = std::move(self)] { self->drain(); });
  }
}

void SubscriptionBase::start() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    paused_ = false;
  }
  schedule();
}

void SubscriptionBase::close() {
  closed_.store(true, std::memory_order_release);
  std::lock_guard<std::mutex> lock(mutex_);
  clear_queue_locked();
}

void SubscriptionBase::cancel() {
  if (closed_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  auto state = state_.lock();
  if (state) {
    std::lock_guard<std::mutex> lock(state->mutex);
    for (auto& [name, channel] : state->channels) {
      (void)name;
      auto subscribers = std::make_shared<SubscriberList>(*channel->subscribers);
      subscribers->erase(std::remove_if(subscribers->begin(), subscribers->end(),
                                        [this](const std::shared_ptr<SubscriptionBase>& subscriber) {
                                          return subscriber.get() == this;
                                        }),
                         subscribers->end());
      channel->subscribers = std::move(subscribers);
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  clear_queue_locked();
}

}  // namespace detail

TopicRegistry::TopicRegistry(Executor& executor) : executor_(executor), state_(std::make_shared<detail::TopicState>()) {}

TopicRegistry::~TopicRegistry() { shutdown(); }

void TopicRegistry::shutdown() {
  std::vector<std::shared_ptr<detail::SubscriptionBase>> subscribers;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->stopping) {
      return;
    }
    state_->stopping = true;
    for (auto& [name, channel] : state_->channels) {
      (void)name;
      for (const auto& subscriber : *channel->subscribers) {
        subscribers.push_back(subscriber);
      }
      channel->subscribers = std::make_shared<const detail::SubscriberList>();
    }
    state_->channels.clear();
  }
  for (auto& subscriber : subscribers) {
    subscriber->close();
  }
}

std::vector<TopicRegistry::TopicInfo> TopicRegistry::topic_info() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  std::vector<TopicInfo> result;
  result.reserve(state_->channels.size());
  for (const auto& [name, channel] : state_->channels) {
    result.push_back(TopicInfo{name, channel->type_name, channel->subscribers->size(),
                               channel->messages.load(std::memory_order_relaxed)});
  }
  std::sort(result.begin(), result.end(), [](const TopicInfo& a, const TopicInfo& b) { return a.topic < b.topic; });
  return result;
}

}  // namespace krosc
