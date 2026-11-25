#include "krosc/viz_web/http_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "krosc/net.hpp"
#include "krosc/viz_web/websocket.hpp"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace krosc::viz_web {

namespace {

constexpr std::size_t kMaxRequest = 16u * 1024u;
constexpr std::size_t kMaxQueue = 256;

bool send_all(int fd, const std::uint8_t* data, std::size_t size) {
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t sent = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(sent);
  }
  return true;
}

std::string lowercase(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return text;
}

std::string trim(std::string text) {
  while (!text.empty() && (text.back() == '\r' || text.back() == ' ' || text.back() == '\t')) {
    text.pop_back();
  }
  const auto begin = text.find_first_not_of(" \t");
  return begin == std::string::npos ? std::string{} : text.substr(begin);
}

std::optional<std::string> frame_topic(const std::vector<std::uint8_t>& body) {
  return net::frame_topic(body);
}

// 표시용 고주기 토픽: 브라우저 큐에 같은 토픽이 이미 있으면 최신 프레임으로 교체한다.
// (이벤트/명령 토픽은 절대 합치지 않는다.)
bool latest_only(const std::string& topic) {
  return topic == "tf" || topic == "tf_static" || topic == "joint_states" || topic == "odom" ||
         topic == "odometry/filtered" || topic == "scan" || topic == "imu" ||
         topic == "diagnostics" || topic == "local_plan" || topic == "moveit_state" ||
         topic == "moveit_status" || topic == "arm_status" || topic == "slam_status" ||
         topic == "goal_status" || topic == "collision_status" || topic == "rl_nav_status";
}

// 브라우저로 내보내는 최대 주기(ms). 0 = 제한 없음. 렌더가 60fps면 그 이상은 의미가 없다.
int throttle_period_ms(const std::string& topic) {
  if (topic == "tf") return 20;                                    // 50Hz
  if (topic == "joint_states") return 33;                          // 30Hz
  if (topic == "odom" || topic == "odometry/filtered") return 50;  // 20Hz
  if (topic == "local_plan") return 100;                           // 10Hz
  return 0;
}

bool cacheable(const std::string& topic) {
  return topic == "map" || topic == "global_costmap/costmap" || topic == "local_costmap/costmap" ||
         topic == "map_list" || topic == "map_status" || topic == "slam_status" ||
         topic == "goal_status" || topic == "collision_status" || topic == "robot_description" ||
         topic == "rl_nav_model_list" || topic == "rl_nav_status" || topic == "moveit_status" ||
         topic == "moveit_state" || topic == "moveit_target_pose" || topic == "moveit_arm" ||
         topic == "moveit_path" || topic == "arm_status" || topic == "tf_static";
}

std::string content_type(const std::string& path) {
  const auto dot = path.rfind('.');
  const std::string extension = dot == std::string::npos ? "" : path.substr(dot);
  if (extension == ".html" || extension == ".htm") {
    return "text/html; charset=utf-8";
  }
  if (extension == ".js" || extension == ".mjs") {
    return "text/javascript; charset=utf-8";
  }
  if (extension == ".css") {
    return "text/css; charset=utf-8";
  }
  if (extension == ".json") {
    return "application/json; charset=utf-8";
  }
  if (extension == ".svg") {
    return "image/svg+xml";
  }
  if (extension == ".png") {
    return "image/png";
  }
  if (extension == ".ico") {
    return "image/x-icon";
  }
  return "application/octet-stream";
}

bool send_response(int fd, int status, const std::string& reason, const std::string& body,
                   const std::string& type) {
  std::ostringstream header;
  header << "HTTP/1.1 " << status << ' ' << reason << "\r\n";
  header << "Content-Type: " << type << "\r\n";
  header << "Content-Length: " << body.size() << "\r\n";
  header << "Cache-Control: no-cache\r\n";
  header << "Connection: close\r\n\r\n";
  const std::string text = header.str();
  return send_all(fd, reinterpret_cast<const std::uint8_t*>(text.data()), text.size()) &&
         (body.empty() || send_all(fd, reinterpret_cast<const std::uint8_t*>(body.data()), body.size()));
}

bool read_request(int fd, std::string& request) {
  char buffer[4096];
  while (request.size() < kMaxRequest) {
    const ssize_t received = ::recv(fd, buffer, sizeof(buffer), 0);
    if (received <= 0) {
      return false;
    }
    request.append(buffer, static_cast<std::size_t>(received));
    if (request.find("\r\n\r\n") != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

struct HttpServer::Client {
  explicit Client(int socket_fd) : fd(socket_fd) {
    writer = std::thread([this] { writer_loop(); });
  }

  ~Client() { stop(); }

  struct Item {
    std::uint8_t opcode = 0x2;
    std::string topic;
    std::shared_ptr<const std::vector<std::uint8_t>> frame;
  };

  void enqueue(std::uint8_t frame_opcode, std::shared_ptr<const std::vector<std::uint8_t>> frame,
               std::string topic = {}) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      if (!alive.load()) {
        return;
      }
      // 고주기 표시 토픽은 큐에 이미 있으면 자리에서 최신 값으로 교체한다 (지연 누적 방지).
      if (frame_opcode == 0x2 && latest_only(topic)) {
        for (auto it = queue.rbegin(); it != queue.rend(); ++it) {
          if (it->opcode == 0x2 && it->topic == topic) {
            it->frame = std::move(frame);
            queue_cv.notify_one();
            return;
          }
        }
      }
      queue.push_back(Item{frame_opcode, std::move(topic), std::move(frame)});
      while (queue.size() > kMaxQueue) {
        queue.pop_front();
      }
    }
    queue_cv.notify_one();
  }

  void stop() {
    if (!alive.exchange(false)) {
      return;
    }
    queue_cv.notify_all();
    ::shutdown(fd, SHUT_RDWR);
    if (writer.joinable() && writer.get_id() != std::this_thread::get_id()) {
      writer.join();
    }
    ::close(fd);
  }

  void writer_loop() {
    while (alive.load()) {
      Item item;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [this] { return !alive.load() || !queue.empty(); });
        if (!alive.load() && queue.empty()) {
          return;
        }
        item = std::move(queue.front());
        queue.pop_front();
      }
      if (!websocket_send_frame(fd, item.opcode, *item.frame)) {
        alive.store(false);
        ::shutdown(fd, SHUT_RDWR);
        return;
      }
    }
  }

  int fd = -1;
  std::atomic<bool> alive{true};
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<Item> queue;
  std::thread writer;
};

HttpServer::HttpServer(std::string bind_address, std::uint16_t port,
                       std::vector<std::string> www_roots, SendHandler send_frame,
                       ManifestHandler manifest)
    : bind_address_(std::move(bind_address)),
      port_(port),
      www_roots_(std::move(www_roots)),
      send_frame_(std::move(send_frame)),
      manifest_(std::move(manifest)) {}

HttpServer::~HttpServer() {
  if (stopping_.exchange(true)) {
    return;
  }
  if (listen_fd_ >= 0) {
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (const auto& client : clients_) {
      if (client) {
        client->alive.store(false);
        ::shutdown(client->fd, SHUT_RDWR);
      }
    }
  }
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    for (auto& thread : threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    threads_.clear();
  }
  std::lock_guard<std::mutex> lock(clients_mutex_);
  clients_.clear();
}

void HttpServer::start() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    throw std::runtime_error("web: socket 생성 실패");
  }
  int enabled = 1;
  (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (bind_address_.empty() || bind_address_ == "0.0.0.0") {
    address.sin_addr.s_addr = INADDR_ANY;
  } else if (::inet_pton(AF_INET, bind_address_.c_str(), &address.sin_addr) != 1) {
    address.sin_addr.s_addr = INADDR_ANY;
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    ::close(fd);
    throw std::runtime_error("web: bind 실패");
  }
  if (::listen(fd, 16) != 0) {
    ::close(fd);
    throw std::runtime_error("web: listen 실패");
  }
  sockaddr_in bound{};
  socklen_t length = sizeof(bound);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &length) == 0) {
    bound_port_ = ntohs(bound.sin_port);
  } else {
    bound_port_ = port_;
  }
  listen_fd_ = fd;
  accept_thread_ = std::thread([this] { accept_loop(); });
}

void HttpServer::accept_loop() {
  while (!stopping_.load()) {
    sockaddr_in peer{};
    socklen_t length = sizeof(peer);
    const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &length);
    if (fd < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    int enabled = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    timeval timeout{};
    timeout.tv_sec = 5;
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    std::lock_guard<std::mutex> lock(threads_mutex_);
    threads_.emplace_back([this, fd] { handle_client(fd); });
  }
}

void HttpServer::handle_client(int fd) {
  std::string request;
  if (!read_request(fd, request)) {
    ::close(fd);
    return;
  }
  std::istringstream stream(request);
  std::string method;
  std::string target;
  std::string version;
  stream >> method >> target >> version;
  if (method != "GET") {
    send_response(fd, 405, "Method Not Allowed", "method not allowed", "text/plain");
    ::close(fd);
    return;
  }

  std::map<std::string, std::string> headers;
  std::string line;
  std::getline(stream, line);  // 요청 줄의 남은 부분(\r)을 버린다.
  while (std::getline(stream, line)) {
    if (line == "\r" || line.empty()) {
      break;
    }
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    headers[lowercase(trim(line.substr(0, colon)))] = trim(line.substr(colon + 1));
  }

  const bool websocket = headers.find("sec-websocket-key") != headers.end() &&
                         lowercase(headers["upgrade"]).find("websocket") != std::string::npos;
  if (websocket) {
    const std::string key = headers["sec-websocket-key"];
    const std::string accept = websocket_accept(key);
    const std::string response = "HTTP/1.1 101 Switching Protocols\r\n"
                                 "Upgrade: websocket\r\n"
                                 "Connection: Upgrade\r\n"
                                 "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
    if (!send_all(fd, reinterpret_cast<const std::uint8_t*>(response.data()), response.size())) {
      ::close(fd);
      return;
    }
    auto client = std::make_shared<Client>(fd);
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      for (const auto& [topic_name, frame] : cache_) {
        client->enqueue(0x2, std::make_shared<const std::vector<std::uint8_t>>(frame), topic_name);
      }
    }
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients_.push_back(client);
    }
    while (client->alive.load() && !stopping_.load()) {
      std::uint8_t opcode = 0;
      std::vector<std::uint8_t> payload;
      if (!websocket_read_frame(fd, opcode, payload)) {
        break;
      }
      if (opcode == 0x1) {
        continue;
      }
      if (opcode == 0x2) {
        if (const auto frame = net::decode_body(payload)) {
          if (send_frame_) {
            send_frame_(*frame);
          }
        }
      } else if (opcode == 0x8) {
        break;
      } else if (opcode == 0x9) {
        auto pong = std::make_shared<const std::vector<std::uint8_t>>(std::move(payload));
        client->enqueue(0xA, std::move(pong));
      }
    }
    client->stop();
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients_.erase(std::remove(clients_.begin(), clients_.end(), client), clients_.end());
    }
    return;
  }

  std::string path = target;
  const auto query = path.find('?');
  if (query != std::string::npos) {
    path.resize(query);
  }
  if (path.empty() || path == "/") {
    path = "/index.html";
  }
  if (path.find("..") != std::string::npos) {
    send_response(fd, 400, "Bad Request", "bad path", "text/plain");
    ::close(fd);
    return;
  }
  if (!serve_static(fd, path)) {
    send_response(fd, 404, "Not Found", "not found", "text/plain");
  }
  ::close(fd);
}

bool HttpServer::serve_static(int fd, const std::string& target) {
  if (target == "/plugins.json" && manifest_) {
    return send_response(fd, 200, "OK", manifest_(), "application/json; charset=utf-8");
  }
  for (const auto& root : www_roots_) {
    std::filesystem::path path = std::filesystem::path(root) / target.substr(1);
    std::ifstream file(path, std::ios::binary);
    if (!file) {
      continue;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    const std::string body = buffer.str();
    return send_response(fd, 200, "OK", body, content_type(path.string()));
  }
  return false;
}

void HttpServer::broadcast_binary(const std::vector<std::uint8_t>& body) {
  const auto topic_name = frame_topic(body);
  if (topic_name) {
    if (cacheable(*topic_name)) {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cache_[*topic_name] = body;
    }
    const int period = throttle_period_ms(*topic_name);
    if (period > 0) {
      std::lock_guard<std::mutex> lock(throttle_mutex_);
      const auto now = std::chrono::steady_clock::now();
      auto& last = last_forward_[*topic_name];
      if (last.time_since_epoch().count() != 0 &&
          now - last < std::chrono::milliseconds(period)) {
        return;  // 이번 프레임은 버린다 (렌더 주기보다 빠름)
      }
      last = now;
    }
  }
  std::vector<std::shared_ptr<Client>> clients;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients = clients_;
  }
  if (clients.empty()) {
    return;
  }
  auto frame = std::make_shared<const std::vector<std::uint8_t>>(body);
  const std::string name = topic_name ? *topic_name : std::string{};
  for (const auto& client : clients) {
    if (client && client->alive.load()) {
      client->enqueue(0x2, frame, name);
    }
  }
}

void HttpServer::broadcast_text(const std::string& text) {
  std::vector<std::shared_ptr<Client>> clients;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients = clients_;
  }
  if (clients.empty()) {
    return;
  }
  auto frame = std::make_shared<const std::vector<std::uint8_t>>(text.begin(), text.end());
  for (const auto& client : clients) {
    if (client && client->alive.load()) {
      client->enqueue(0x1, frame);
    }
  }
}

}  // namespace krosc::viz_web
