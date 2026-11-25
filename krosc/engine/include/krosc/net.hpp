#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <vector>

#include "krosc/net_protocol.hpp"

namespace krosc {
class Context;
}

namespace krosc::net {

// 로컬 토픽 메시지를 타입 소거 상태로 인코딩/디코딩한다. peer 네트워크가 임의 토픽 이름을
// 고정 인터페이스(type_index/Kind)로 주고받기 위해 쓴다. 등록되지 않은 타입이면 빈 값.
[[nodiscard]] std::optional<Kind> message_kind(std::type_index type);
[[nodiscard]] std::vector<std::uint8_t> encode_message(std::type_index type, const void* message);
[[nodiscard]] std::shared_ptr<const void> decode_message(Kind kind, const std::uint8_t* data,
                                                         std::size_t size);

// 실기(Pi) 쪽: 로컬 토픽을 TCP(9000)로 내보내고, 원격 viz의 cmd_vel을 로컬 토픽으로 넣는다.
// 한 번에 한 클라이언트만 붙는다 (Mac viz 하나). 클라이언트가 끊기면 다시 기다린다.
//
// 보내는 토픽: tf, tf_static, robot_description, map, global_costmap/costmap, local_costmap/costmap,
//              plan, local_plan, scan, imu, odom, odometry/filtered, joint_states, diagnostics, rosout
// 받는 토픽: cmd_vel
//
// 주의: Context가 Server보다 오래 살아야 한다. shutdown()은 ~Server에서도 불린다.
class Server {
 public:
  explicit Server(Context& context, std::uint16_t port = 9000);
  ~Server();

  Server(const Server&) = delete;
  Server& operator=(const Server&) = delete;

  [[nodiscard]] std::uint16_t port() const;
  [[nodiscard]] bool running() const;
  void shutdown();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 맥(PC) 쪽: Pi의 TCP(9000)에 붙어 원격 토픽을 로컬 토픽으로 재생하고, 로컬 cmd_vel을 Pi로 보낸다.
// 서버가 없으면 1초마다 다시 붙는다. 연결 전/끊긴 동안 들어온 cmd_vel은 버린다.
//
// 주의: Context가 Client보다 오래 살아야 한다. shutdown()은 ~Client에서도 불린다.
class Client {
 public:
  Client(Context& context, std::string host, std::uint16_t port = 9000);
  ~Client();

  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;

  [[nodiscard]] bool connected() const;
  [[nodiscard]] std::string endpoint() const;
  void shutdown();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace krosc::net
