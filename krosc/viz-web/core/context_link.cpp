#include "krosc/viz_web/context_link.hpp"

#include <chrono>
#include <typeindex>
#include <typeinfo>
#include <utility>

namespace krosc::viz_web {

struct ContextLink::Impl {
  explicit Impl(Context& context_value) : context(context_value) {}
  ~Impl() { shutdown(); }

  template <class T>
  void subscribe_topic(const std::string& name, net::Kind kind, QoS qos) {
    auto subscription = context.topics().subscribe<T>(
        name,
        [this, name, kind](const T& message) {
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
          net::Frame frame;
          frame.type = net::kFrameTopic;
          frame.topic = name;
          frame.kind = kind;
          frame.payload = net::encode_message(std::type_index(typeid(T)), &message);
          handler(frame);
        },
        qos);
    subscriptions.push_back(subscription.base());
  }

  void start() {
    if (started.exchange(true)) {
      return;
    }
    stopping.store(false);
    status_stop.store(false);
    status_thread = std::thread([this] { status_loop(); });
  }

  void subscribe(const TopicSpec& topic) {
    switch (topic.kind) {
      case net::Kind::Twist:
        subscribe_topic<msgs::Twist>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::String:
        subscribe_topic<msgs::String>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::TF:
        subscribe_topic<msgs::TFMessage>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Scan:
        subscribe_topic<msgs::LaserScan>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Imu:
        subscribe_topic<msgs::Imu>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Odometry:
        subscribe_topic<msgs::Odometry>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::JointState:
        subscribe_topic<msgs::JointState>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Grid:
        subscribe_topic<msgs::OccupancyGrid>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Path:
        subscribe_topic<msgs::Path>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Diagnostics:
        subscribe_topic<msgs::DiagnosticArray>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Log:
        subscribe_topic<msgs::Log>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Pose:
        subscribe_topic<msgs::PoseStamped>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::PoseCov:
        subscribe_topic<msgs::PoseWithCovarianceStamped>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Bool:
        subscribe_topic<msgs::Bool>(topic.name, topic.kind, topic.qos);
        break;
      case net::Kind::Empty:
        subscribe_topic<msgs::Empty>(topic.name, topic.kind, topic.qos);
        break;
    }
  }

  void shutdown() {
    status_stop.store(true);
    if (status_thread.joinable()) {
      status_thread.join();
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
    const auto message =
        net::decode_message(frame.kind, frame.payload.data(), frame.payload.size());
    if (!message) {
      return false;
    }
    const QoS qos = frame.kind == net::Kind::Twist ? QOS_COMMAND : QOS_DEFAULT;
    switch (frame.kind) {
      case net::Kind::Twist:
        context.topics().publish<msgs::Twist>(
            frame.topic, std::static_pointer_cast<const msgs::Twist>(message), qos);
        break;
      case net::Kind::String:
        context.topics().publish<msgs::String>(
            frame.topic, std::static_pointer_cast<const msgs::String>(message), qos);
        break;
      case net::Kind::TF:
        context.topics().publish<msgs::TFMessage>(
            frame.topic, std::static_pointer_cast<const msgs::TFMessage>(message), qos);
        break;
      case net::Kind::Scan:
        context.topics().publish<msgs::LaserScan>(
            frame.topic, std::static_pointer_cast<const msgs::LaserScan>(message), qos);
        break;
      case net::Kind::Imu:
        context.topics().publish<msgs::Imu>(
            frame.topic, std::static_pointer_cast<const msgs::Imu>(message), qos);
        break;
      case net::Kind::Odometry:
        context.topics().publish<msgs::Odometry>(
            frame.topic, std::static_pointer_cast<const msgs::Odometry>(message), qos);
        break;
      case net::Kind::JointState:
        context.topics().publish<msgs::JointState>(
            frame.topic, std::static_pointer_cast<const msgs::JointState>(message), qos);
        break;
      case net::Kind::Grid:
        context.topics().publish<msgs::OccupancyGrid>(
            frame.topic, std::static_pointer_cast<const msgs::OccupancyGrid>(message), qos);
        break;
      case net::Kind::Path:
        context.topics().publish<msgs::Path>(
            frame.topic, std::static_pointer_cast<const msgs::Path>(message), qos);
        break;
      case net::Kind::Diagnostics:
        context.topics().publish<msgs::DiagnosticArray>(
            frame.topic, std::static_pointer_cast<const msgs::DiagnosticArray>(message), qos);
        break;
      case net::Kind::Log:
        context.topics().publish<msgs::Log>(
            frame.topic, std::static_pointer_cast<const msgs::Log>(message), qos);
        break;
      case net::Kind::Pose:
        context.topics().publish<msgs::PoseStamped>(
            frame.topic, std::static_pointer_cast<const msgs::PoseStamped>(message), qos);
        break;
      case net::Kind::PoseCov:
        context.topics().publish<msgs::PoseWithCovarianceStamped>(
            frame.topic, std::static_pointer_cast<const msgs::PoseWithCovarianceStamped>(message), qos);
        break;
      case net::Kind::Bool:
        context.topics().publish<msgs::Bool>(
            frame.topic, std::static_pointer_cast<const msgs::Bool>(message), qos);
        break;
      case net::Kind::Empty:
        context.topics().publish<msgs::Empty>(
            frame.topic, std::static_pointer_cast<const msgs::Empty>(message), qos);
        break;
      default:
        return false;
    }
    return true;
  }

  Context& context;
  std::vector<std::shared_ptr<detail::SubscriptionBase>> subscriptions;
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

void ContextLink::subscribe(const TopicSpec& topic) { impl_->subscribe(topic); }

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
