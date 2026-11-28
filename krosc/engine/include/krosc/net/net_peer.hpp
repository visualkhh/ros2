#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace krosc {
class Context;
}

namespace krosc::net {

// Krosc/Context가 다른 Krosc 인스턴스와 토픽을 주고받기 위한 peer 설정.
//   - host 지정 + port=0: 임시 포트로 서버를 열고 port()로 실제 포트를 확인
//   - servers: 연결할 "host:port" 목록 (끊기면 1초마다 재연결)
struct PeerNetworkOptions {
  std::string name = "krosc";
  // 로컬 listen 주소. 비우면 이쪽으로 들어오는 연결은 받지 않는다.
  // 예: "0.0.0.0", "10.2.150.138"
  std::string host;
  std::uint16_t port = 0;  // host가 있을 때 listen 포트. 0이면 임시 포트.
  // 연결할 원격 Krosc 서버 목록. "host:port" 또는 add_server(host, port) 사용.
  std::vector<std::string> servers;

  // 원격에서 받은 토픽 이름을 로컬 이름으로 바꾼다.
  // 예: teleop Viz의 cmd_vel → 로봇의 cmd_vel_teleop
  std::map<std::string, std::string> receive_remap;

  // 원격 서비스 호출 타임아웃 (초).
  double rpc_timeout_sec = 5.0;

  void add_server(std::string server_host, std::uint16_t server_port) {
    servers.push_back(std::move(server_host) + ":" + std::to_string(server_port));
  }

  [[nodiscard]] bool listen_enabled() const { return !host.empty(); }
  [[nodiscard]] bool enabled() const { return listen_enabled() || !servers.empty(); }
};

// 여러 Krosc 프로세스 사이의 토픽 mesh.
// 로컬 TopicRegistry publish를 가로채 discover/subscribe된 peer에게 보내고,
// peer에서 온 토픽은 로컬 토픽으로 다시 발행한다 (재브로드캐스트 방지).
//
// 서비스/액션/파라미터 RPC도 같은 연결 위에 올라간다. 서비스는 type-erased payload로
// 포워딩되고, 액션은 send_goal/cancel_goal 서비스 + /_action/event 이벤트로 전달된다.
class PeerNetwork {
 public:
  struct Impl;

  PeerNetwork(Context& context, PeerNetworkOptions options);
  ~PeerNetwork();

  PeerNetwork(const PeerNetwork&) = delete;
  PeerNetwork& operator=(const PeerNetwork&) = delete;

  void start();
  void shutdown();

  [[nodiscard]] bool running() const;
  [[nodiscard]] std::uint16_t port() const;
  [[nodiscard]] std::vector<std::string> peer_endpoints() const;

  // 네트워크에서 받은 토픽을 로컬로 발행하는 중인지 (재브로드캐스트 방지).
  [[nodiscard]] static bool receiving();

  // 원격 피어가 광고한 서비스 이름 (디버깅/진단용).
  [[nodiscard]] std::vector<std::string> remote_services() const;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace krosc::net
