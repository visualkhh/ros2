#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace krosc::net {

// TCP 프레임 프로토콜. 로봇 서버/PC 클라이언트/웹 게이트웨이가 같은 정의를 쓴다.
//   [u32 length][body]
//   body = [u8 type][u16 topic_len][topic][u8 kind][payload]
constexpr std::uint8_t kFrameTopic = 1;    // 서버 -> 클라이언트 토픽
constexpr std::uint8_t kFramePublish = 2;  // 클라이언트 -> 서버 명령
constexpr std::uint32_t kMaxFrame = 4u * 1024u * 1024u;

enum class Kind : std::uint8_t {
  Twist = 1,
  String = 2,
  TF = 3,
  Scan = 4,
  Imu = 5,
  Odometry = 6,
  JointState = 7,
  Grid = 8,
  Path = 9,
  Diagnostics = 10,
  Log = 11,
  Pose = 12,
  PoseCov = 13,
  Empty = 14,
  Bool = 15,
};

struct Frame {
  std::uint8_t type = kFramePublish;
  std::string topic;
  Kind kind = Kind::String;
  std::vector<std::uint8_t> payload;
};

// length prefix 없는 body. 웹소켓으로 브라우저에 넘기는 단위와 같다.
[[nodiscard]] std::vector<std::uint8_t> encode_body(const Frame& frame);
[[nodiscard]] std::optional<Frame> decode_body(const std::vector<std::uint8_t>& body);

// length prefix 포함. TCP 소켓에 그대로 쓰거나 읽는다.
[[nodiscard]] std::vector<std::uint8_t> encode_frame(const Frame& frame);
[[nodiscard]] bool read_frame(int fd, Frame& frame);
[[nodiscard]] bool write_frame(int fd, const Frame& frame);

// 캐시 키 등에 쓰는 topic 이름만 빠르게 꺼낸다.
[[nodiscard]] std::optional<std::string> frame_topic(const std::vector<std::uint8_t>& body);

// 원시 프레임을 그대로 주고받는 TCP 클라이언트.
// Qt Viz의 net::Client(토픽 타입까지 해석)와 달리, 웹 게이트웨이처럼
// 프레임을 브라우저에 그대로 중계해야 하는 쪽이 쓴다. 프로토콜/소켓 루프는 여기 하나만 둔다.
class FrameClient {
 public:
  using FrameHandler = std::function<void(const Frame&)>;
  using StatusHandler = std::function<void(bool)>;

  FrameClient(std::string host, std::uint16_t port);
  ~FrameClient();

  FrameClient(const FrameClient&) = delete;
  FrameClient& operator=(const FrameClient&) = delete;

  void set_frame_handler(FrameHandler handler) { frame_handler_ = std::move(handler); }
  void set_status_handler(StatusHandler handler) { status_handler_ = std::move(handler); }

  void start();
  void shutdown();

  [[nodiscard]] bool connected() const;
  [[nodiscard]] std::string endpoint() const;
  [[nodiscard]] bool send(const Frame& frame);

 private:
  void run();

  std::string host_;
  std::uint16_t port_ = 9000;
  FrameHandler frame_handler_;
  StatusHandler status_handler_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> connected_{false};
  std::atomic<int> fd_{-1};
  std::mutex send_mutex_;
  std::thread thread_;
};

}  // namespace krosc::net
