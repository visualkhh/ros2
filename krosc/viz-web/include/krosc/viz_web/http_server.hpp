#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace krosc::net {
struct Frame;
}

namespace krosc::viz_web {

// HTTP 정적 파일 + /ws WebSocket. 브라우저와 로컬 토픽(ContextLink) 사이에서 프레임만 중계한다.
class HttpServer {
 public:
  using ManifestHandler = std::function<std::string()>;
  using SendHandler = std::function<void(const krosc::net::Frame&)>;
  using TextReply = std::function<void(std::string)>;
  using TextHandler = std::function<void(const std::string&, TextReply)>;

  HttpServer(std::string bind_address, std::uint16_t port, std::vector<std::string> www_roots,
             SendHandler send_frame, ManifestHandler manifest = {}, TextHandler text_handler = {});
  ~HttpServer();

  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  void start();
  [[nodiscard]] std::uint16_t port() const { return bound_port_; }

  void broadcast_binary(std::vector<std::uint8_t> body);
  void broadcast_text(const std::string& text);
  // 새 WebSocket 클라이언트가 붙을 때 보낼 현재 상태(예: 로봇 연결 여부).
  void set_text_on_connect(std::function<std::string()> handler) {
    text_on_connect_ = std::move(handler);
  }

 private:
  struct Client;
  void accept_loop();
  void handle_client(int fd);
  bool serve_static(int fd, const std::string& target);
  void start_websocket(int fd, const std::string& request);

  std::string bind_address_;
  std::uint16_t port_ = 8080;
  std::uint16_t bound_port_ = 0;
  std::vector<std::string> www_roots_;
  SendHandler send_frame_;
  ManifestHandler manifest_;
  TextHandler text_handler_;
  std::function<std::string()> text_on_connect_;
  std::atomic<bool> stopping_{false};
  int listen_fd_ = -1;
  std::thread accept_thread_;
  std::mutex clients_mutex_;
  std::vector<std::shared_ptr<Client>> clients_;
  std::mutex threads_mutex_;
  std::vector<std::thread> threads_;
  // 늦게 붙은 브라우저에도 지도/상태/모델 목록을 주기 위해 최신 프레임을 토픽별로 보관한다.
  // 고주기 토픽(tf/scan/odom)은 다음 메시지가 금방 오므로 캐시하지 않는다.
  std::mutex cache_mutex_;
  std::map<std::string, std::shared_ptr<const std::vector<std::uint8_t>>> cache_;
  // 브라우저 전송 주기 제한: 렌더(60fps)보다 자주 보내봐야 큐만 쌓인다.
  std::mutex throttle_mutex_;
  std::map<std::string, std::chrono::steady_clock::time_point> last_forward_;
};

}  // namespace krosc::viz_web
