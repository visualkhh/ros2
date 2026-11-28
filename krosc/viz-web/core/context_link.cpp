#include "krosc/viz_web/context_link.hpp"

#include <chrono>
#include <map>
#include <utility>

namespace krosc::viz_web {

struct ContextLink::Impl {
  explicit Impl(Context& context_value) : context(context_value) {}
  ~Impl() { shutdown(); }

  void start() {
    if (started.exchange(true)) {
      return;
    }
    stopping.store(false);
    status_stop.store(false);
    status_thread = std::thread([this] { status_loop(); });
  }

  bool subscribe(const TopicSpec& topic) {
    const auto* codec = net::find_topic_type(topic.kind);
    if (codec == nullptr || topic.name.empty() || subscriptions.count(topic.name) != 0) {
      return false;
    }
    try {
      const std::string name = topic.name;
      const net::Kind kind = topic.kind;
      auto subscription = codec->subscribe(
          context.topics(), name,
          [this, name, kind, codec](const void* message) {
            if (stopping.load()) {
              return;
            }
            FrameHandler handler;
            {
              std::lock_guard<std::mutex> lock(handler_mutex);
              handler = frame_handler;
            }
            if (!handler) {
              return;
            }
            net::RpcWriter writer;
            codec->encode(writer, message);
            handler(net::Frame{net::kFrameTopic, name, kind, std::move(writer.data)});
          },
          topic.qos);
      subscriptions.emplace(name, std::move(subscription));
      return true;
    } catch (...) {
      return false;
    }
  }

  bool unsubscribe(const std::string& name) {
    auto found = subscriptions.find(name);
    if (found == subscriptions.end()) {
      return false;
    }
    found->second->cancel();
    subscriptions.erase(found);
    return true;
  }

  bool subscribed(const std::string& name) const {
    return subscriptions.count(name) != 0;
  }

  void shutdown() {
    status_stop.store(true);
    if (status_thread.joinable()) {
      status_thread.join();
    }
    for (auto& [name, subscription] : subscriptions) {
      (void)name;
      subscription->cancel();
    }
    subscriptions.clear();
    started.store(false);
  }

  bool connected() const {
    const auto* peer = context.peer_network();
    if (peer == nullptr) {
      return true;  // local-only Context
    }
    return !peer->peer_endpoints().empty();
  }

  std::string endpoint() const {
    const auto* peer = context.peer_network();
    if (peer == nullptr) {
      return "local";
    }
    const auto endpoints = peer->peer_endpoints();
    if (endpoints.empty()) {
      return "peer: waiting";
    }
    std::string text = "peer: ";
    for (std::size_t i = 0; i < endpoints.size(); ++i) {
      if (i != 0) {
        text += ",";
      }
      text += endpoints[i];
    }
    return text;
  }

  void status_loop() {
    bool last = connected();
    StatusHandler handler;
    {
      std::lock_guard<std::mutex> lock(handler_mutex);
      handler = status_handler;
    }
    if (handler) {
      handler(last);
    }
    while (!status_stop.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      const bool now_connected = connected();
      if (now_connected == last) {
        continue;
      }
      last = now_connected;
      {
        std::lock_guard<std::mutex> lock(handler_mutex);
        handler = status_handler;
      }
      if (handler) {
        handler(last);
      }
    }
  }

  bool send(const net::Frame& frame) {
    if (frame.type != net::kFramePublish) {
      return false;
    }
    const auto* codec = net::find_topic_type(frame.kind);
    if (codec == nullptr) {
      return false;
    }
    const auto message = codec->decode(frame.payload.data(), frame.payload.size());
    if (!message) {
      return false;
    }
    const QoS qos = frame.kind == net::Kind::Twist ? QOS_COMMAND : QOS_DEFAULT;
    codec->publish(context.topics(), frame.topic, message, qos);
    return true;
  }

  Context& context;
  std::map<std::string, std::shared_ptr<detail::SubscriptionBase>> subscriptions;
  std::mutex handler_mutex;
  FrameHandler frame_handler;
  StatusHandler status_handler;
  std::atomic<bool> started{false};
  std::atomic<bool> stopping{false};
  std::atomic<bool> status_stop{true};
  std::thread status_thread;
};

ContextLink::ContextLink(Context& context) : impl_(std::make_unique<Impl>(context)) {}
ContextLink::~ContextLink() { shutdown(); }

void ContextLink::set_frame_handler(FrameHandler handler) {
  std::lock_guard<std::mutex> lock(impl_->handler_mutex);
  impl_->frame_handler = std::move(handler);
}

void ContextLink::set_status_handler(StatusHandler handler) {
  std::lock_guard<std::mutex> lock(impl_->handler_mutex);
  impl_->status_handler = std::move(handler);
}

void ContextLink::start() { impl_->start(); }

bool ContextLink::subscribe(const TopicSpec& topic) { return impl_ && impl_->subscribe(topic); }

bool ContextLink::unsubscribe(const std::string& name) { return impl_ && impl_->unsubscribe(name); }

bool ContextLink::subscribed(const std::string& name) const {
  return impl_ && impl_->subscribed(name);
}

void ContextLink::shutdown() {
  if (impl_) {
    impl_->stopping.store(true);
    impl_->shutdown();
  }
}

bool ContextLink::connected() const { return impl_ && impl_->connected(); }

std::string ContextLink::endpoint() const { return impl_ ? impl_->endpoint() : std::string{}; }

bool ContextLink::send(const net::Frame& frame) { return impl_ && impl_->send(frame); }

}  // namespace krosc::viz_web
