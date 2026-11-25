#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace krosc::viz_web {

// 로봇의 TCP(9000) 프로토콜에 그대로 붙는다. 4바이트 길이 + 본문을 읽어 콜백으로 넘기고,
// 브라우저가 보낸 본문에 길이를 붙여 보낸다. 메시지 해석은 하지 않는다 (프론트엔드 담당).
class RobotLink {
 public:
  using FrameHandler = std::function<void(const std::vector<std::uint8_t>&)>;
  using StatusHandler = std::function<void(bool)>;

  RobotLink(std::string host, std::uint16_t port);
  ~RobotLink();

  RobotLink(const RobotLink&) = delete;
  RobotLink& operator=(const RobotLink&) = delete;

  void set_frame_handler(FrameHandler handler) { frame_handler_ = std::move(handler); }
  void set_status_handler(StatusHandler handler) { status_handler_ = std::move(handler); }

  void start();
  void shutdown();
  [[nodiscard]] bool connected() const { return connected_.load(); }
  [[nodiscard]] const std::string& endpoint() const { return host_; }

  bool send(const std::vector<std::uint8_t>& body);

 private:
  void run();

  std::string host_;
  std::uint16_t port_ = 9000;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> connected_{false};
  std::atomic<int> fd_{-1};
  std::mutex send_mutex_;
  std::thread thread_;
  FrameHandler frame_handler_;
  StatusHandler status_handler_;
};

}  // namespace krosc::viz_web
