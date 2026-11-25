#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace krosc {
class Context;
}

namespace krosc::net {

// Krosc/Context가 다른 Krosc 인스턴스와 토픽을 주고받기 위한 peer 설정.
//   - listen=true, port=0: 임시 포트로 서버를 열고 port()로 실제 포트를 확인
//   - peers: 연결할 "host:port" 목록 (끊기면 1초마다 재연결)
struct PeerNetworkOptions {
  std::string name = "krosc";
  // 로컬 listen 주소. 비우면 이쪽으로 들어오는 연결은 받지 않는다.
  // 예: "0.0.0.0", "10.2.150.138"
  std::string host;
  std::uint16_t port = 0;  // host가 있을 때 listen 포트. 0이면 임시 포트.
  // 연결할 원격 Krosc 서버 목록. "host:port" 또는 add_server(host, port) 사용.
  std::vector<std::string> servers;

  // 기존 옵션 호환용. 새 코드는 host/servers를 쓴다.
  std::string bind_address = "0.0.0.0";
  bool listen = false;
  std::vector<std::string> peers;

  void add_server(std::string server_host, std::uint16_t server_port) {
    servers.push_back(std::move(server_host) + ":" + std::to_string(server_port));
  }

  [[nodiscard]] bool listen_enabled() const { return !host.empty() || listen; }

  [[nodiscard]] bool enabled() const {
    return listen_enabled() || !servers.empty() || !peers.empty();
  }

  [[nodiscard]] std::vector<std::string> all_servers() const {
    std::vector<std::string> result = servers;
    result.insert(result.end(), peers.begin(), peers.end());
    return result;
  }

  [[nodiscard]] std::string effective_bind_address() const {
    return host.empty() ? bind_address : host;
  }
};

// 여러 Krosc 프로세스 사이의 토픽 mesh.
// 로컬 TopicRegistry publish를 가로채 discover/subscribe된 peer에게 보내고,
// peer에서 온 토픽은 로컬 토픽으로 다시 발행한다 (재브로드캐스트 방지).
//
// 서비스/액션은 다음 단계에서 같은 peer 연결 위에 올린다. C++ 액션은 서비스+토픽으로
// 구현되어 있어 서비스 포워딩이 들어가면 함께 동작할 수 있다.
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

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace krosc::net
