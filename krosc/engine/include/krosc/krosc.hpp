#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/node.hpp"

namespace krosc {

// 부트스트랩: Context를 만들고 node_create 콜백에 넘겨 노드들을 받아 보관한다.
// 파라미터 시작 값은 Context::Options::parameters로 준다 ({노드이름: {파라미터: 값}}).
class Krosc {
 public:
  using NodeCreate = std::function<std::vector<std::unique_ptr<Node>>(Context&)>;

  Krosc();
  explicit Krosc(NodeCreate node_create, Context::Options options = {});
  // 다른 Krosc와 연결할 peer 목록("host:port"). port>0이면 이쪽도 listen한다.
  Krosc(NodeCreate node_create, std::vector<std::string> peers, std::uint16_t port = 0);
  // host를 주면 그 주소로 listen하고, servers의 Krosc들과 통신한다.
  // host/servers가 모두 비면 네트워크 없이 local-only로 동작한다.
  Krosc(NodeCreate node_create, net::PeerNetworkOptions network);
  ~Krosc();

  Krosc(const Krosc&) = delete;
  Krosc& operator=(const Krosc&) = delete;

  [[nodiscard]] Context& context() { return *context_; }
  [[nodiscard]] const Context& context() const { return *context_; }
  [[nodiscard]] const std::vector<std::unique_ptr<Node>>& nodes() const { return nodes_; }
  [[nodiscard]] std::vector<std::unique_ptr<Node>>& nodes() { return nodes_; }

  void shutdown();

 private:
  std::unique_ptr<Context> context_;
  std::vector<std::unique_ptr<Node>> nodes_;
  bool shutdown_ = false;
};

}  // namespace krosc
