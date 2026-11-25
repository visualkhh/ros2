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
  Rpc = 16,
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

}  // namespace krosc::net
