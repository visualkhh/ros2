#include "krosc/net_peer.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/net.hpp"
#include "krosc/qos.hpp"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace krosc::net {

namespace {

constexpr std::uint8_t kPeerTopicLatched = 3;  // kFrameTopic(1)의 latched 변형
constexpr std::uint8_t kPeerControl = 4;
constexpr std::size_t kMaxQueue = 512;

thread_local bool t_receiving = false;

struct ReceiveGuard {
  ReceiveGuard() { t_receiving = true; }
  ~ReceiveGuard() { t_receiving = false; }
};

std::string endpoint_text(const std::string& host, std::uint16_t port) {
  return host + ":" + std::to_string(port);
}

bool parse_endpoint(const std::string& text, std::string& host, std::uint16_t& port) {
  const std::size_t colon = text.rfind(':');
  if (colon == std::string::npos) {
    host = text;
    port = 9000;
    return !host.empty();
  }
  host = text.substr(0, colon);
  try {
    const int value = std::stoi(text.substr(colon + 1));
    if (value <= 0 || value > 65535) {
      return false;
    }
    port = static_cast<std::uint16_t>(value);
  } catch (...) {
    return false;
  }
  return !host.empty();
}

int connect_to(const std::string& host, std::uint16_t port) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* result = nullptr;
  const std::string service = std::to_string(port);
  if (::getaddrinfo(host.c_str(), service.c_str(), &hints, &result) != 0) {
    return -1;
  }
  int fd = -1;
  for (addrinfo* entry = result; entry != nullptr; entry = entry->ai_next) {
    fd = ::socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
    if (fd < 0) {
      continue;
    }
    if (::connect(fd, entry->ai_addr, entry->ai_addrlen) == 0) {
      break;
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(result);
  return fd;
}

void configure_socket(int fd) {
  int enabled = 1;
  (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
  timeval timeout{};
  timeout.tv_sec = 1;
  (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

std::vector<std::string> split_lines(const std::string& text) {
  std::vector<std::string> lines;
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

std::string join_lines(const std::vector<std::string>& lines) {
  std::string out;
  for (std::size_t index = 0; index < lines.size(); ++index) {
    if (index != 0) {
      out.push_back('\n');
    }
    out += lines[index];
  }
  return out;
}

QoS qos_for_remote_topic(const std::string& topic, bool latched) {
  if (latched) {
    return QOS_LATCHED;
  }
  if (topic == "tf") {
    return QOS_TF;
  }
  if (topic == "scan" || topic == "imu" || topic == "odom" || topic == "odometry/filtered" ||
      topic == "local_plan") {
    return QOS_SENSOR_DATA;
  }
  return QOS_DEFAULT;
}

}  // namespace

struct Peer : public std::enable_shared_from_this<Peer> {
  Peer(PeerNetwork::Impl& owner_value, int fd_value, std::string endpoint_value)
      : owner(owner_value), fd(fd_value), endpoint(std::move(endpoint_value)) {}

  void start();
  void send(const Frame& frame);
  void close();
  void read_loop();
  void write_loop();

  PeerNetwork::Impl& owner;
  int fd = -1;
  std::string endpoint;
  std::atomic<bool> alive{true};
  std::atomic<bool> retired{false};
  std::thread reader;
  std::thread writer;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<Frame> queue;
  std::mutex subscription_mutex;
  std::set<std::string> subscriptions;
  bool discovered = false;
};

struct PeerNetwork::Impl {
  Impl(Context& context_value, PeerNetworkOptions options_value)
      : context(context_value), options(std::move(options_value)) {}

  Context& context;
  PeerNetworkOptions options;
  std::atomic<bool> stopping{false};
  int listen_fd = -1;
  std::uint16_t bound_port = 0;
  std::thread accept_thread;
  std::vector<std::thread> connect_threads;
  std::thread reaper_thread;
  mutable std::mutex peers_mutex;
  std::vector<std::shared_ptr<Peer>> peers;
  std::mutex retired_mutex;
  std::condition_variable retired_cv;
  std::deque<std::shared_ptr<Peer>> retired;
  std::mutex subscriptions_mutex;
  std::vector<std::string> subscriptions;
  std::mutex latched_mutex;
  std::map<std::string, std::pair<Kind, std::vector<std::uint8_t>>> latched;
  std::set<std::string> remote_topics;
  std::mutex active_mutex;
  std::condition_variable active_cv;
  int active_threads = 0;

  void thread_started() {
    std::lock_guard<std::mutex> lock(active_mutex);
    ++active_threads;
  }

  void thread_finished() {
    std::lock_guard<std::mutex> lock(active_mutex);
    --active_threads;
    if (active_threads == 0) {
      active_cv.notify_all();
    }
  }

  bool enabled() const { return options.enabled(); }
  bool started = false;

  void start() {
    if (started || !enabled()) {
      return;
    }
    started = true;
    context.topics().set_publish_hook([this](const std::string& topic, std::type_index type,
                                             const std::shared_ptr<const void>& message, QoS qos) {
      on_publish(topic, type, message, qos);
    });
    context.topics().set_subscription_hook([this] { update_subscriptions(); });
    update_subscriptions();

    if (options.listen_enabled()) {
      listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
      if (listen_fd < 0) {
        throw std::runtime_error("PeerNetwork: socket() failed");
      }
      int enabled = 1;
      (void)::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = htonl(INADDR_ANY);
      const std::string bind_address = options.effective_bind_address();
      if (!bind_address.empty() && bind_address != "0.0.0.0" && bind_address != "*") {
        if (::inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1) {
          ::close(listen_fd);
          listen_fd = -1;
          throw std::runtime_error("PeerNetwork: 잘못된 bind 주소: " + bind_address);
        }
      }
      address.sin_port = htons(options.port);
      if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(listen_fd);
        listen_fd = -1;
        throw std::runtime_error("PeerNetwork: bind failed (port " + std::to_string(options.port) + ")");
      }
      if (::listen(listen_fd, 16) != 0) {
        ::close(listen_fd);
        listen_fd = -1;
        throw std::runtime_error("PeerNetwork: listen failed");
      }
      socklen_t length = sizeof(address);
      if (::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&address), &length) == 0) {
        bound_port = ntohs(address.sin_port);
      } else {
        bound_port = options.port;
      }
      accept_thread = std::thread([this] { accept_loop(); });
    }

    for (const auto& peer : options.all_servers()) {
      connect_threads.emplace_back([this, peer] { connect_loop(peer); });
    }
    reaper_thread = std::thread([this] { reap_loop(); });
  }

  void shutdown() {
    if (stopping.exchange(true)) {
      return;
    }
    context.topics().set_publish_hook({});
    context.topics().set_subscription_hook({});
    if (listen_fd >= 0) {
      ::shutdown(listen_fd, SHUT_RDWR);
      ::close(listen_fd);
      listen_fd = -1;
    }
    if (accept_thread.joinable()) {
      accept_thread.join();
    }
    {
      std::lock_guard<std::mutex> lock(peers_mutex);
      for (const auto& peer : peers) {
        if (peer) {
          peer->close();
        }
      }
    }
    for (auto& thread : connect_threads) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    {
      std::unique_lock<std::mutex> lock(active_mutex);
      active_cv.wait(lock, [this] { return active_threads == 0; });
    }
    std::vector<std::shared_ptr<Peer>> all;
    {
      std::lock_guard<std::mutex> lock(peers_mutex);
      all.swap(peers);
    }
    for (const auto& peer : all) {
      retire(peer);
    }
    retired_cv.notify_all();
    if (reaper_thread.joinable()) {
      reaper_thread.join();
    }
    // reaper가 stopping을 보고 먼저 끝난 경우 남은 스레드를 여기서 정리한다.
    {
      std::lock_guard<std::mutex> lock(retired_mutex);
      for (const auto& peer : retired) {
        if (peer->reader.joinable()) {
          peer->reader.join();
        }
        if (peer->writer.joinable()) {
          peer->writer.join();
        }
      }
      retired.clear();
    }
  }

  bool running() const { return !stopping.load(); }

  std::vector<std::string> peer_endpoints() const {
    std::vector<std::string> result;
    std::lock_guard<std::mutex> lock(peers_mutex);
    for (const auto& peer : peers) {
      if (peer && peer->alive.load()) {
        result.push_back(peer->endpoint);
      }
    }
    return result;
  }

  void accept_loop() {
    while (!stopping.load()) {
      sockaddr_in address{};
      socklen_t length = sizeof(address);
      const int fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&address), &length);
      if (fd < 0) {
        if (stopping.load()) {
          return;
        }
        continue;
      }
      if (stopping.load()) {
        ::close(fd);
        return;
      }
      configure_socket(fd);
      auto peer = std::make_shared<Peer>(*this, fd, endpoint_text(::inet_ntoa(address.sin_addr), ntohs(address.sin_port)));
      add_peer(peer);
      peer->start();
      send_discover(*peer);
      send_subscribe(*peer);
    }
  }

  void connect_loop(std::string text) {
    std::string host;
    std::uint16_t port = 9000;
    if (!parse_endpoint(text, host, port)) {
      std::fprintf(stderr, "[krosc] PeerNetwork: 잘못된 peer 주소: %s\n", text.c_str());
      return;
    }
    while (!stopping.load()) {
      const int fd = connect_to(host, port);
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      configure_socket(fd);
      auto peer = std::make_shared<Peer>(*this, fd, endpoint_text(host, port));
      add_peer(peer);
      peer->start();
      send_discover(*peer);
      send_subscribe(*peer);
      while (!stopping.load() && peer->alive.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      peer->close();
      if (!stopping.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }

  void add_peer(const std::shared_ptr<Peer>& peer) {
    std::lock_guard<std::mutex> lock(peers_mutex);
    peers.push_back(peer);
  }

  void retire(const std::shared_ptr<Peer>& peer) {
    if (!peer || peer->retired.exchange(true)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(retired_mutex);
      retired.push_back(peer);
    }
    retired_cv.notify_all();
  }

  void reap_loop() {
    while (true) {
      std::shared_ptr<Peer> peer;
      {
        std::unique_lock<std::mutex> lock(retired_mutex);
        retired_cv.wait(lock, [this] { return stopping.load() || !retired.empty(); });
        if (retired.empty()) {
          if (stopping.load()) {
            return;
          }
          continue;
        }
        peer = retired.front();
        retired.pop_front();
      }
      if (peer->reader.joinable()) {
        peer->reader.join();
      }
      if (peer->writer.joinable()) {
        peer->writer.join();
      }
      std::lock_guard<std::mutex> lock(peers_mutex);
      peers.erase(std::remove(peers.begin(), peers.end(), peer), peers.end());
    }
  }

  void send_control(Peer& peer, const std::string& topic, const std::string& text) {
    Frame frame;
    frame.type = kPeerControl;
    frame.topic = topic;
    frame.kind = Kind::String;
    frame.payload.assign(text.begin(), text.end());
    peer.send(frame);
  }

  void send_discover(Peer& peer) {
    const auto topics = context.topics().topic_info();
    std::vector<std::string> lines;
    lines.reserve(topics.size());
    for (const auto& info : topics) {
      lines.push_back(info.topic + "\t" + info.type);
    }
    send_control(peer, "__discover", join_lines(lines));
  }

  void send_subscribe(Peer& peer) {
    std::vector<std::string> topics;
    {
      std::lock_guard<std::mutex> lock(subscriptions_mutex);
      topics = subscriptions;
    }
    send_control(peer, "__subscribe", join_lines(topics));
  }

  void update_subscriptions() {
    std::vector<std::string> topics;
    for (const auto& info : context.topics().topic_info()) {
      if (info.subscribers > 0) {
        topics.push_back(info.topic);
      }
    }
    std::sort(topics.begin(), topics.end());
    {
      std::lock_guard<std::mutex> lock(subscriptions_mutex);
      if (topics == subscriptions) {
        return;
      }
      subscriptions = topics;
    }
    std::vector<std::shared_ptr<Peer>> snapshot;
    {
      std::lock_guard<std::mutex> lock(peers_mutex);
      snapshot = peers;
    }
    for (const auto& peer : snapshot) {
      if (peer && peer->alive.load()) {
        send_subscribe(*peer);
      }
    }
  }

  void on_publish(const std::string& topic, std::type_index type,
                  const std::shared_ptr<const void>& message, QoS qos) {
    if (stopping.load() || t_receiving) {
      return;
    }
    const auto kind = message_kind(type);
    if (!kind) {
      return;
    }
    Frame frame;
    frame.type = qos.latched() ? kPeerTopicLatched : kFrameTopic;
    frame.topic = topic;
    frame.kind = *kind;
    frame.payload = encode_message(type, message.get());
    if (qos.latched()) {
      std::lock_guard<std::mutex> lock(latched_mutex);
      latched[topic] = {*kind, frame.payload};
    }
    std::vector<std::shared_ptr<Peer>> snapshot;
    {
      std::lock_guard<std::mutex> lock(peers_mutex);
      snapshot = peers;
    }
    for (const auto& peer : snapshot) {
      if (!peer || !peer->alive.load()) {
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(peer->subscription_mutex);
        if (peer->discovered && peer->subscriptions.count(topic) == 0) {
          continue;
        }
      }
      peer->send(frame);
    }
  }

  void on_frame(Peer& peer, const Frame& frame) {
    if (frame.type == kPeerControl) {
      if (frame.topic == "__discover") {
        const std::string text(frame.payload.begin(), frame.payload.end());
        {
          std::lock_guard<std::mutex> lock(peer.subscription_mutex);
          peer.discovered = true;
        }
        for (const auto& line : split_lines(text)) {
          const std::size_t tab = line.find('\t');
          const std::string name = line.substr(0, tab);
          if (!name.empty()) {
            std::lock_guard<std::mutex> lock(latched_mutex);
            remote_topics.insert(name);
          }
        }
      } else if (frame.topic == "__subscribe") {
        const std::string text(frame.payload.begin(), frame.payload.end());
        std::set<std::string> requested;
        for (const auto& line : split_lines(text)) {
          requested.insert(line);
        }
        std::set<std::string> added;
        {
          std::lock_guard<std::mutex> lock(peer.subscription_mutex);
          for (const auto& topic : requested) {
            if (peer.subscriptions.count(topic) == 0) {
              added.insert(topic);
            }
          }
          peer.subscriptions = std::move(requested);
        }
        std::lock_guard<std::mutex> lock(latched_mutex);
        for (const auto& topic : added) {
          const auto found = latched.find(topic);
          if (found == latched.end()) {
            continue;
          }
          Frame reply;
          reply.type = kPeerTopicLatched;
          reply.topic = topic;
          reply.kind = found->second.first;
          reply.payload = found->second.second;
          peer.send(reply);
        }
      }
      return;
    }
    if (frame.type != kFrameTopic && frame.type != kPeerTopicLatched) {
      return;
    }
    const auto message = decode_message(frame.kind, frame.payload.data(), frame.payload.size());
    if (!message) {
      return;
    }
    const QoS qos = qos_for_remote_topic(frame.topic, frame.type == kPeerTopicLatched);
    ReceiveGuard guard;
    switch (frame.kind) {
      case Kind::Twist:
        context.topics().publish<msgs::Twist>(
            frame.topic, std::static_pointer_cast<const msgs::Twist>(message), qos);
        break;
      case Kind::String:
        context.topics().publish<msgs::String>(
            frame.topic, std::static_pointer_cast<const msgs::String>(message), qos);
        break;
      case Kind::TF:
        context.topics().publish<msgs::TFMessage>(
            frame.topic, std::static_pointer_cast<const msgs::TFMessage>(message), qos);
        break;
      case Kind::Scan:
        context.topics().publish<msgs::LaserScan>(
            frame.topic, std::static_pointer_cast<const msgs::LaserScan>(message), qos);
        break;
      case Kind::Imu:
        context.topics().publish<msgs::Imu>(
            frame.topic, std::static_pointer_cast<const msgs::Imu>(message), qos);
        break;
      case Kind::Odometry:
        context.topics().publish<msgs::Odometry>(
            frame.topic, std::static_pointer_cast<const msgs::Odometry>(message), qos);
        break;
      case Kind::JointState:
        context.topics().publish<msgs::JointState>(
            frame.topic, std::static_pointer_cast<const msgs::JointState>(message), qos);
        break;
      case Kind::Grid:
        context.topics().publish<msgs::OccupancyGrid>(
            frame.topic, std::static_pointer_cast<const msgs::OccupancyGrid>(message), qos);
        break;
      case Kind::Path:
        context.topics().publish<msgs::Path>(
            frame.topic, std::static_pointer_cast<const msgs::Path>(message), qos);
        break;
      case Kind::Diagnostics:
        context.topics().publish<msgs::DiagnosticArray>(
            frame.topic, std::static_pointer_cast<const msgs::DiagnosticArray>(message), qos);
        break;
      case Kind::Log:
        context.topics().publish<msgs::Log>(
            frame.topic, std::static_pointer_cast<const msgs::Log>(message), qos);
        break;
      case Kind::Pose:
        context.topics().publish<msgs::PoseStamped>(
            frame.topic, std::static_pointer_cast<const msgs::PoseStamped>(message), qos);
        break;
      case Kind::PoseCov:
        context.topics().publish<msgs::PoseWithCovarianceStamped>(
            frame.topic, std::static_pointer_cast<const msgs::PoseWithCovarianceStamped>(message), qos);
        break;
      case Kind::Bool:
        context.topics().publish<msgs::Bool>(
            frame.topic, std::static_pointer_cast<const msgs::Bool>(message), qos);
        break;
      case Kind::Empty:
        context.topics().publish<msgs::Empty>(
            frame.topic, std::static_pointer_cast<const msgs::Empty>(message), qos);
        break;
      default:
        break;
    }
  }
};

void Peer::start() {
  owner.thread_started();
  owner.thread_started();
  reader = std::thread([self = shared_from_this()] {
    self->read_loop();
    self->owner.thread_finished();
  });
  writer = std::thread([self = shared_from_this()] {
    self->write_loop();
    self->owner.thread_finished();
  });
}

void Peer::send(const Frame& frame) {
  if (!alive.load()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (!alive.load()) {
      return;
    }
    if (queue.size() >= kMaxQueue) {
      queue.pop_front();
    }
    queue.push_back(frame);
  }
  queue_cv.notify_one();
}

void Peer::close() {
  if (!alive.exchange(false)) {
    return;
  }
  ::shutdown(fd, SHUT_RDWR);
  ::close(fd);
  fd = -1;
  queue_cv.notify_all();
}

void Peer::read_loop() {
  while (alive.load()) {
    Frame frame;
    if (!read_frame(fd, frame)) {
      break;
    }
    owner.on_frame(*this, frame);
  }
  close();
  owner.retire(shared_from_this());
}

void Peer::write_loop() {
  while (alive.load()) {
    Frame frame;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_cv.wait(lock, [this] { return !alive.load() || !queue.empty(); });
      if (!alive.load()) {
        return;
      }
      frame = std::move(queue.front());
      queue.pop_front();
    }
    if (!write_frame(fd, frame)) {
      close();
      return;
    }
  }
}

PeerNetwork::PeerNetwork(Context& context, PeerNetworkOptions options)
    : impl_(std::make_unique<Impl>(context, std::move(options))) {
  impl_->start();
}

PeerNetwork::~PeerNetwork() { shutdown(); }

void PeerNetwork::start() { impl_->start(); }

void PeerNetwork::shutdown() {
  if (impl_) {
    impl_->shutdown();
  }
}

bool PeerNetwork::running() const { return impl_ && impl_->running(); }

std::uint16_t PeerNetwork::port() const { return impl_ ? impl_->bound_port : 0; }

std::vector<std::string> PeerNetwork::peer_endpoints() const {
  return impl_ ? impl_->peer_endpoints() : std::vector<std::string>{};
}

bool PeerNetwork::receiving() { return t_receiving; }

}  // namespace krosc::net
