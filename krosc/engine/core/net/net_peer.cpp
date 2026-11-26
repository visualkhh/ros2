#include "krosc/net/net_peer.hpp"

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
#include <cstdlib>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/net/net.hpp"
#include "krosc/qos.hpp"
#include "krosc/net/rpc.hpp"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace krosc::net {

namespace {

constexpr std::uint8_t kPeerTopicLatched = 3;  // kFrameTopic(1)의 latched 변형
constexpr std::uint8_t kPeerControl = 4;
constexpr std::size_t kMaxQueue = 512;

bool peer_debug() {
  static const bool enabled = std::getenv("KROSC_PEER_DEBUG") != nullptr;
  return enabled;
}

// peer에서 받은 프레임을 로컬에 publish할 때, 그 publish에서 파생된 응답(상태/목록)이
// peer로 다시 나가지 못하게 막던 문제를 고친다. 이제 "원래 보낸 peer"에게만 되돌려 보내지 않는다.
thread_local const void* t_origin_peer = nullptr;
thread_local std::string t_origin_topic;

struct ReceiveGuard {
  ReceiveGuard(const void* peer, std::string topic)
      : previous_peer(t_origin_peer), previous_topic(t_origin_topic) {
    t_origin_peer = peer;
    t_origin_topic = std::move(topic);
  }
  ~ReceiveGuard() {
    t_origin_peer = previous_peer;
    t_origin_topic = previous_topic;
  }
  const void* previous_peer;
  std::string previous_topic;
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
  if (topic == "cmd_vel" || topic == "cmd_vel_teleop") {
    return QOS_COMMAND;
  }
  return QOS_DEFAULT;
}

// 표시용 고주기 토픽은 큐에 같은 토픽이 있으면 최신 프레임으로 교체한다.
// (이벤트/명령/제어 프레임은 절대 합치지 않는다.)
// tf_static은 발행자별로 별도 프레임이라 큐에서도 합치면 map→odom이 사라진다.
bool latest_only_topic(const std::string& topic) {
  return topic == "tf" || topic == "joint_states" || topic == "odom" ||
         topic == "odometry/filtered" || topic == "scan" || topic == "imu" ||
         topic == "diagnostics" || topic == "local_plan" || topic == "moveit_state" ||
         topic == "moveit_status" || topic == "arm_status" || topic == "slam_status" ||
         topic == "goal_status" || topic == "collision_status" || topic == "rl_nav_status";
}

}  // namespace

struct Peer : public std::enable_shared_from_this<Peer> {
  Peer(PeerNetwork::Impl& owner_value, int fd_value, std::string endpoint_value)
      : owner(owner_value), fd(fd_value), endpoint(std::move(endpoint_value)) {}

  void start();
  void send(const Frame& frame);
  void send(std::shared_ptr<const Frame> frame);
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
  std::deque<std::shared_ptr<const Frame>> queue;
  std::mutex subscription_mutex;
  std::set<std::string> subscriptions;
  std::mutex services_mutex;
  std::set<std::string> services;
  bool discovered = false;
};

struct PeerNetwork::Impl {
  Impl(Context& context_value, PeerNetworkOptions options_value)
      : context(context_value), options(std::move(options_value)) {}
  ~Impl() { shutdown(); }

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
  // tf_static은 발행자마다 별도 메시지라 하나만 저장하면 map→odom 또는 URDF 고정
  // 관절 중 하나가 사라진다. 다른 래치 토픽은 최신 하나만 유지한다.
  using LatchedEntry = std::pair<Kind, std::vector<std::uint8_t>>;
  std::map<std::string, std::vector<LatchedEntry>> latched;
  std::set<std::string> remote_topics;
  std::mutex active_mutex;
  std::condition_variable active_cv;
  int active_threads = 0;

  // ---- RPC ----
  std::atomic<std::uint32_t> next_rpc_id{1};
  struct PendingRpc {
    bool done = false;
    bool ok = false;
    std::string response_type;
    std::vector<std::uint8_t> payload;
    std::string error;
  };
  std::mutex rpc_mutex;
  std::condition_variable rpc_cv;
  std::unordered_map<std::uint32_t, PendingRpc> rpc_pending;
  struct RpcTask {
    std::shared_ptr<Peer> peer;
    std::uint32_t request_id = 0;
    std::string service;
    std::string request_type;
    std::vector<std::uint8_t> payload;
  };
  std::thread rpc_worker;
  std::mutex rpc_queue_mutex;
  std::condition_variable rpc_queue_cv;
  std::deque<RpcTask> rpc_queue;

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
    context.services().set_remote_call(
        [this](const std::string& name, std::type_index request_type, const void* request,
               std::type_index response_type) {
          return remote_call(name, request_type, request, response_type);
        });
    context.services().set_change_hook([this] { advertise_services(); });
    rpc_worker = std::thread([this] { rpc_loop(); });

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
    context.services().set_remote_call({});
    context.services().set_change_hook({});
    {
      std::lock_guard<std::mutex> lock(rpc_mutex);
      for (auto& [id, pending] : rpc_pending) {
        (void)id;
        pending.done = true;
        pending.ok = false;
        pending.error = "PeerNetwork 종료";
      }
    }
    rpc_cv.notify_all();
    {
      std::lock_guard<std::mutex> lock(rpc_queue_mutex);
      rpc_queue.clear();
    }
    rpc_queue_cv.notify_all();
    if (rpc_worker.joinable()) {
      rpc_worker.join();
    }
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

  std::vector<std::string> remote_services() const {
    std::set<std::string> names;
    std::lock_guard<std::mutex> lock(peers_mutex);
    for (const auto& peer : peers) {
      if (!peer || !peer->alive.load()) {
        continue;
      }
      std::lock_guard<std::mutex> service_lock(peer->services_mutex);
      names.insert(peer->services.begin(), peer->services.end());
    }
    return {names.begin(), names.end()};
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
      send_services(*peer);
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
      send_services(*peer);
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

  std::vector<std::string> service_names() const {
    return context.services().names();
  }

  void send_services(Peer& peer) {
    send_control(peer, "__services", join_lines(service_names()));
  }

  void advertise_services() {
    if (!started || stopping.load()) {
      return;
    }
    std::vector<std::shared_ptr<Peer>> snapshot;
    {
      std::lock_guard<std::mutex> lock(peers_mutex);
      snapshot = peers;
    }
    for (const auto& peer : snapshot) {
      if (peer && peer->alive.load()) {
        send_services(*peer);
      }
    }
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
        // receive_remap: 로컬 cmd_vel_teleop을 구독해도 peer에게는 cmd_vel을 달라고 해야 한다.
        for (const auto& [remote, local] : options.receive_remap) {
          if (local == info.topic && remote != info.topic) {
            topics.push_back(remote);
          }
        }
      }
    }
    std::sort(topics.begin(), topics.end());
    topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
    {
      std::lock_guard<std::mutex> lock(subscriptions_mutex);
      if (topics == subscriptions) {
        return;
      }
      subscriptions = topics;
    }
    if (peer_debug()) {
      std::fprintf(stderr, "[peer] subscribe %zu topics: %s\n", topics.size(),
                   join_lines(topics).c_str());
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

  void send_rpc_error(const std::shared_ptr<Peer>& peer, std::uint32_t request_id,
                      const std::string& request_type, const std::string& error) {
    if (!peer || !peer->alive.load()) {
      return;
    }
    RpcEnvelope response;
    response.request_id = request_id;
    response.status = kRpcError;
    response.request_type = request_type;
    response.error = error;
    Frame frame;
    frame.type = kFrameRpcResponse;
    frame.kind = Kind::Rpc;
    frame.payload = encode_rpc_envelope(response);
    peer->send(frame);
  }

  std::any remote_call(const std::string& service, std::type_index request_type,
                       const void* request, std::type_index response_type) {
    if (stopping.load()) {
      throw std::runtime_error("PeerNetwork가 종료됐다");
    }
    std::string request_name;
    if (request_type == std::type_index(typeid(SendGoalRequest))) {
      request_name = "krosc/action/SendGoalRequest";
    } else if (request_type == std::type_index(typeid(CancelGoalRequest))) {
      request_name = "krosc/action/CancelGoalRequest";
    } else {
      const auto name = rpc_type_name(request_type);
      if (!name) {
        throw std::runtime_error("원격 서비스 요청 타입이 등록되지 않았다: " + service);
      }
      request_name = *name;
    }
    const auto response_name = rpc_type_name(response_type);
    if (!response_name) {
      throw std::runtime_error("원격 서비스 응답 타입이 등록되지 않았다: " + service);
    }
    std::vector<std::uint8_t> payload = encode_rpc_request(service, request_type, request);
    if ((request_type == std::type_index(typeid(SendGoalRequest)) ||
         request_type == std::type_index(typeid(CancelGoalRequest))) &&
        payload.empty()) {
      throw std::runtime_error("액션 goal을 인코딩하지 못했다: " + service);
    }

    // 서비스 광고를 보고 대상 peer를 고른다. 아직 광고 전이면 연결된 모두에게 보낸다.
    std::vector<std::shared_ptr<Peer>> all;
    {
      std::lock_guard<std::mutex> lock(peers_mutex);
      all = peers;
    }
    std::vector<std::shared_ptr<Peer>> targets;
    std::vector<std::shared_ptr<Peer>> unknown;
    for (const auto& peer : all) {
      if (!peer || !peer->alive.load()) {
        continue;
      }
      bool advertised = false;
      bool has = false;
      {
        std::lock_guard<std::mutex> lock(peer->services_mutex);
        advertised = !peer->services.empty();
        has = peer->services.count(service) != 0;
      }
      if (has) {
        targets.push_back(peer);
      } else if (!advertised) {
        unknown.push_back(peer);
      }
    }
    if (targets.empty()) {
      targets = std::move(unknown);
    }
    if (targets.empty()) {
      throw ServiceNotFoundError("no peer serves '" + service + "'");
    }

    const std::uint32_t request_id = next_rpc_id.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(rpc_mutex);
      rpc_pending[request_id] = PendingRpc{};
    }
    RpcEnvelope envelope;
    envelope.request_id = request_id;
    envelope.status = kRpcRequest;
    envelope.request_type = request_name;
    envelope.response_type = *response_name;
    envelope.payload = std::move(payload);
    Frame frame;
    frame.type = kFrameRpcRequest;
    frame.topic = service;
    frame.kind = Kind::Rpc;
    frame.payload = encode_rpc_envelope(envelope);
    for (const auto& peer : targets) {
      peer->send(frame);
    }

    const double timeout = options.rpc_timeout_sec > 0.0 ? options.rpc_timeout_sec : 5.0;
    PendingRpc result;
    {
      std::unique_lock<std::mutex> lock(rpc_mutex);
      auto it = rpc_pending.find(request_id);
      if (!rpc_cv.wait_for(lock, std::chrono::duration<double>(timeout), [&] {
            return it == rpc_pending.end() || it->second.done;
          })) {
        rpc_pending.erase(request_id);
        throw std::runtime_error("service '" + service + "' timeout");
      }
      if (it == rpc_pending.end()) {
        throw std::runtime_error("service '" + service + "' 취소됨");
      }
      result = std::move(it->second);
      rpc_pending.erase(it);
    }
    if (!result.ok) {
      throw std::runtime_error(result.error.empty() ? "remote service failed" : result.error);
    }
    std::any response = decode_rpc_response(service, result.response_type, result.payload.data(),
                                            result.payload.size());
    if (!response.has_value()) {
      throw std::runtime_error("service '" + service + "' 응답을 디코드하지 못했다");
    }
    return response;
  }

  void rpc_loop() {
    while (true) {
      RpcTask task;
      {
        std::unique_lock<std::mutex> lock(rpc_queue_mutex);
        rpc_queue_cv.wait(lock, [this] { return stopping.load() || !rpc_queue.empty(); });
        if (rpc_queue.empty()) {
          if (stopping.load()) {
            return;
          }
          continue;
        }
        task = std::move(rpc_queue.front());
        rpc_queue.pop_front();
      }
      if (stopping.load()) {
        return;
      }
      const auto types = context.services().service_types(task.service);
      if (!types) {
        continue;  // 이 peer가 서비스 주인이 아니다 (broadcast 요청)
      }
      std::any request =
          decode_rpc_request(task.service, task.request_type, task.payload.data(), task.payload.size());
      if (!request.has_value()) {
        send_rpc_error(task.peer, task.request_id, task.request_type,
                       "service request type mismatch: " + task.service);
        continue;
      }
      const auto response_name = rpc_type_name(types->second);
      if (!response_name || !rpc_type_registered(types->second)) {
        send_rpc_error(task.peer, task.request_id, task.request_type,
                       "service response type is not registered: " + task.service);
        continue;
      }
      std::vector<std::uint8_t> payload;
      try {
        payload = context.services().invoke_encoded(
            task.service, request, [](std::type_index type, const void* value) {
              return encode_rpc_value(type, value);
            });
      } catch (const std::exception& error) {
        send_rpc_error(task.peer, task.request_id, task.request_type, error.what());
        continue;
      }
      if (payload.empty() && *response_name != "std_msgs/Empty") {
        send_rpc_error(task.peer, task.request_id, task.request_type,
                       "service response encoding failed: " + task.service);
        continue;
      }
      RpcEnvelope envelope;
      envelope.request_id = task.request_id;
      envelope.status = kRpcOk;
      envelope.request_type = task.request_type;
      envelope.response_type = *response_name;
      envelope.payload = payload;
      Frame frame;
      frame.type = kFrameRpcResponse;
      frame.kind = Kind::Rpc;
      frame.payload = encode_rpc_envelope(envelope);
      if (task.peer) {
        task.peer->send(frame);
      }
    }
  }

  void on_publish(const std::string& topic, std::type_index type,
                  const std::shared_ptr<const void>& message, QoS qos) {
    if (stopping.load()) {
      return;
    }
    if (type == std::type_index(typeid(ActionEvent))) {
      // 액션 이벤트는 피드백/결과 타입이 action 이름마다 다르다. 코덱이 없으면 로컬 전용.
      if (t_origin_peer != nullptr) {
        return;  // 네트워크에서 받은 이벤트를 다시 뿌리지 않는다
      }
      const auto* event = static_cast<const ActionEvent*>(message.get());
      if (event == nullptr) {
        return;
      }
      std::vector<std::shared_ptr<Peer>> snapshot;
      {
        std::lock_guard<std::mutex> lock(peers_mutex);
        snapshot = peers;
      }
      const std::vector<std::uint8_t> payload = encode_action_event(topic, *event);
      if (payload.empty()) {
        return;
      }
      auto frame = std::make_shared<const Frame>(
          Frame{kFrameActionEvent, topic, Kind::Rpc, payload});
      for (const auto& peer : snapshot) {
        if (peer && peer->alive.load()) {
          peer->send(frame);
        }
      }
      return;
    }
    // 피어가 없거나 아무도 이 토픽을 구독하지 않으면 직렬화/복사를 하지 않는다.
    // latched는 나중에 붙을 피어를 위해 캐시해야 하므로 항상 인코딩한다.
    const bool is_latched = qos.latched();
    std::vector<std::shared_ptr<Peer>> snapshot;
    {
      std::lock_guard<std::mutex> lock(peers_mutex);
      snapshot = peers;
    }
    std::vector<std::shared_ptr<Peer>> targets;
    targets.reserve(snapshot.size());
    for (const auto& peer : snapshot) {
      if (!peer || !peer->alive.load()) {
        continue;
      }
      if (t_origin_peer == static_cast<const void*>(peer.get()) && t_origin_topic == topic) {
        continue;  // 같은 토픽만 되돌려 보내지 않는다 (응답/상태는 보낸 peer에도 전달)
      }
      bool subscribed = true;
      {
        std::lock_guard<std::mutex> lock(peer->subscription_mutex);
        subscribed = !peer->discovered || peer->subscriptions.count(topic) != 0;
      }
      if (subscribed) {
        targets.push_back(peer);
      }
    }
    if (peer_debug() && topic.rfind("camera/", 0) == 0) {
      std::fprintf(stderr, "[peer] publish %s targets=%zu peers=%zu\n", topic.c_str(),
                   targets.size(), snapshot.size());
    }
    if (targets.empty() && !is_latched) {
      return;
    }
    const auto kind = message_kind(type);
    if (!kind) {
      return;
    }
    Frame frame;
    frame.type = is_latched ? kPeerTopicLatched : kFrameTopic;
    frame.topic = topic;
    frame.kind = *kind;
    frame.payload = encode_message(type, message.get());
    if (is_latched) {
      std::lock_guard<std::mutex> lock(latched_mutex);
      auto& entries = latched[topic];
      if (topic == "tf_static") {
        if (entries.size() >= 64) {
          entries.erase(entries.begin());
        }
        entries.push_back({*kind, frame.payload});
      } else {
        entries.assign(1, {*kind, frame.payload});
      }
    }
    if (targets.empty()) {
      return;
    }
    auto shared = std::make_shared<const Frame>(std::move(frame));
    for (const auto& peer : targets) {
      peer->send(shared);
    }
  }

  void on_frame(Peer& peer, const Frame& frame) {
    if (frame.type == kFrameRpcResponse) {
      const auto envelope = decode_rpc_envelope(frame.payload);
      if (!envelope) {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(rpc_mutex);
        const auto found = rpc_pending.find(envelope->request_id);
        if (found == rpc_pending.end()) {
          return;  // 늦게 온 응답
        }
        PendingRpc& pending = found->second;
        pending.done = true;
        pending.ok = envelope->status == kRpcOk;
        pending.response_type = envelope->response_type;
        pending.payload = envelope->payload;
        pending.error = envelope->error;
      }
      rpc_cv.notify_all();
      return;
    }
    if (frame.type == kFrameRpcRequest) {
      const auto envelope = decode_rpc_envelope(frame.payload);
      if (!envelope || envelope->status != kRpcRequest) {
        return;
      }
      RpcTask task;
      task.peer = peer.shared_from_this();
      task.request_id = envelope->request_id;
      task.service = frame.topic;
      task.request_type = envelope->request_type;
      task.payload = envelope->payload;
      {
        std::lock_guard<std::mutex> lock(rpc_queue_mutex);
        if (rpc_queue.size() >= 256) {
          rpc_queue.pop_front();  // RPC 폭주 시 오래된 요청을 버린다
        }
        rpc_queue.push_back(std::move(task));
      }
      rpc_queue_cv.notify_one();
      return;
    }
    if (frame.type == kFrameActionEvent) {
      const auto event = decode_action_event(frame.topic, frame.payload.data(), frame.payload.size());
      if (!event) {
        return;
      }
      ReceiveGuard guard(&peer, frame.topic);
      context.topics().publish<ActionEvent>(frame.topic, *event, QOS_DEFAULT);
      return;
    }
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
      } else if (frame.topic == "__services") {
        const std::string text(frame.payload.begin(), frame.payload.end());
        std::set<std::string> services;
        for (const auto& line : split_lines(text)) {
          if (!line.empty()) {
            services.insert(line);
          }
        }
        std::lock_guard<std::mutex> lock(peer.services_mutex);
        peer.services = std::move(services);
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
        if (peer_debug()) {
          std::fprintf(stderr, "[peer] got __subscribe: %s\n", text.c_str());
        }
        std::lock_guard<std::mutex> lock(latched_mutex);
        for (const auto& topic : added) {
          const auto found = latched.find(topic);
          if (found == latched.end()) {
            continue;
          }
          for (const auto& entry : found->second) {
            Frame reply;
            reply.type = kPeerTopicLatched;
            reply.topic = topic;
            reply.kind = entry.first;
            reply.payload = entry.second;
            peer.send(reply);
          }
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
    std::string topic = frame.topic;
    if (const auto remapped = options.receive_remap.find(topic);
        remapped != options.receive_remap.end()) {
      topic = remapped->second;
    }
    const QoS qos = qos_for_remote_topic(topic, frame.type == kPeerTopicLatched);
    ReceiveGuard guard(&peer, topic);
    switch (frame.kind) {
      case Kind::Twist:
        context.topics().publish<msgs::Twist>(
            topic, std::static_pointer_cast<const msgs::Twist>(message), qos);
        break;
      case Kind::String:
        context.topics().publish<msgs::String>(
            topic, std::static_pointer_cast<const msgs::String>(message), qos);
        break;
      case Kind::TF:
        context.topics().publish<msgs::TFMessage>(
            topic, std::static_pointer_cast<const msgs::TFMessage>(message), qos);
        break;
      case Kind::Scan:
        context.topics().publish<msgs::LaserScan>(
            topic, std::static_pointer_cast<const msgs::LaserScan>(message), qos);
        break;
      case Kind::Image:
        context.topics().publish<msgs::Image>(
            topic, std::static_pointer_cast<const msgs::Image>(message), qos);
        break;
      case Kind::Imu:
        context.topics().publish<msgs::Imu>(
            topic, std::static_pointer_cast<const msgs::Imu>(message), qos);
        break;
      case Kind::Odometry:
        context.topics().publish<msgs::Odometry>(
            topic, std::static_pointer_cast<const msgs::Odometry>(message), qos);
        break;
      case Kind::JointState:
        context.topics().publish<msgs::JointState>(
            topic, std::static_pointer_cast<const msgs::JointState>(message), qos);
        break;
      case Kind::Grid:
        context.topics().publish<msgs::OccupancyGrid>(
            topic, std::static_pointer_cast<const msgs::OccupancyGrid>(message), qos);
        break;
      case Kind::Path:
        context.topics().publish<msgs::Path>(
            topic, std::static_pointer_cast<const msgs::Path>(message), qos);
        break;
      case Kind::Diagnostics:
        context.topics().publish<msgs::DiagnosticArray>(
            topic, std::static_pointer_cast<const msgs::DiagnosticArray>(message), qos);
        break;
      case Kind::Log:
        context.topics().publish<msgs::Log>(
            topic, std::static_pointer_cast<const msgs::Log>(message), qos);
        break;
      case Kind::Pose:
        context.topics().publish<msgs::PoseStamped>(
            topic, std::static_pointer_cast<const msgs::PoseStamped>(message), qos);
        break;
      case Kind::PoseCov:
        context.topics().publish<msgs::PoseWithCovarianceStamped>(
            topic, std::static_pointer_cast<const msgs::PoseWithCovarianceStamped>(message), qos);
        break;
      case Kind::Bool:
        context.topics().publish<msgs::Bool>(
            topic, std::static_pointer_cast<const msgs::Bool>(message), qos);
        break;
      case Kind::Empty:
        context.topics().publish<msgs::Empty>(
            topic, std::static_pointer_cast<const msgs::Empty>(message), qos);
        break;
      default:
        break;
    }
  }
};

void Peer::start() {
  // 두 스레드가 실제로 시작되기 전에 카운트한다. shutdown이 먼저 0을 보고
  // Impl을 지우는 경쟁을 막는다.
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
  send(std::make_shared<const Frame>(frame));
}

void Peer::send(std::shared_ptr<const Frame> frame) {
  if (!frame || !alive.load()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (!alive.load()) {
      return;
    }
    const bool data_frame = frame->type == kFrameTopic || frame->type == kPeerTopicLatched;
    if (data_frame && latest_only_topic(frame->topic)) {
      for (auto it = queue.rbegin(); it != queue.rend(); ++it) {
        if ((*it)->topic == frame->topic &&
            ((*it)->type == kFrameTopic || (*it)->type == kPeerTopicLatched)) {
          *it = std::move(frame);  // 아직 안 나간 같은 토픽 프레임을 최신 값으로 교체
          queue_cv.notify_one();
          return;
        }
      }
    }
    if (queue.size() >= kMaxQueue) {
      queue.pop_front();
    }
    queue.push_back(std::move(frame));
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
    std::shared_ptr<const Frame> frame;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_cv.wait(lock, [this] { return !alive.load() || !queue.empty(); });
      if (!alive.load()) {
        return;
      }
      frame = std::move(queue.front());
      queue.pop_front();
    }
    if (!frame || !write_frame(fd, *frame)) {
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

std::vector<std::string> PeerNetwork::remote_services() const {
  return impl_ ? impl_->remote_services() : std::vector<std::string>{};
}

std::uint16_t PeerNetwork::port() const { return impl_ ? impl_->bound_port : 0; }

std::vector<std::string> PeerNetwork::peer_endpoints() const {
  return impl_ ? impl_->peer_endpoints() : std::vector<std::string>{};
}

bool PeerNetwork::receiving() { return t_origin_peer != nullptr; }

}  // namespace krosc::net
