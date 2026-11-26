#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "krosc/viz_web/context_link.hpp"
#include "krosc/viz_web/viz_web_plugins.hpp"
#include "krosc/viz_web/web_rpc.hpp"

#ifndef KROSC_VIZ_WEB_ROOT
#define KROSC_VIZ_WEB_ROOT "viz-web/www"
#endif

namespace krosc {
class Context;
}

namespace krosc::viz_web {

class HttpServer;

struct VizWebOptions {
  std::string robot_host = "127.0.0.1";
  std::uint16_t robot_port = 9000;
  std::string bind_address = "0.0.0.0";
  std::uint16_t port = 8080;
  std::string www_root = KROSC_VIZ_WEB_ROOT;
  // 프로젝트 www. core www보다 먼저 찾는다 (프로젝트 JS/파일 우선).
  std::string overlay_root;
  // 브라우저로 내보낼 토픽. 비우면 VizWeb이 기본 목록을 채운다.
  // 프로젝트/플러그인은 여기에 자기 토픽을 추가하면 된다.
  std::vector<ContextLink::TopicSpec> forward_topics;
};

// 웹 Viz 게이트웨이의 조립/수명주기. main.cpp는 인자만 파싱해서 이 클래스를 실행한다.
// Context/PeerNetwork는 app main의 Krosc가 소유하고, VizWeb은 ContextLink로 로컬 토픽을
// 브라우저 프레임으로 중계한다.
class VizWeb {
 public:
  VizWeb(VizWebOptions options, VizWebPluginRegistry registry, Context& context);
  ~VizWeb();

  VizWeb(const VizWeb&) = delete;
  VizWeb& operator=(const VizWeb&) = delete;

  [[nodiscard]] bool initialize(std::string& error);
  int run();
  void stop();

  [[nodiscard]] std::uint16_t port() const;
  [[nodiscard]] bool connected() const;
  [[nodiscard]] const VizWebOptions& options() const { return options_; }
  [[nodiscard]] const VizWebPluginRegistry& plugins() const { return registry_; }
  // 프로젝트 main이 MoveIt/RL 같은 추가 RPC를 등록할 수 있다.
  [[nodiscard]] WebRpc& rpc() { return *rpc_; }

 private:
  struct TopicEntry {
    ContextLink::TopicSpec spec;
    bool subscribed = false;
    std::string source = "core";
  };

  void shutdown();
  void register_topic_rpc();
  bool register_topic(const ContextLink::TopicSpec& spec, const std::string& source,
                      bool subscribe, std::string& error);
  bool set_topic_subscription(const std::string& name, bool subscribe, std::string& error);
  [[nodiscard]] Json topic_list_json() const;

  VizWebOptions options_;
  VizWebPluginRegistry registry_;
  Context* context_ = nullptr;
  std::unique_ptr<ContextLink> link_;
  std::unique_ptr<HttpServer> server_;
  std::unique_ptr<WebRpc> rpc_;
  mutable std::mutex topics_mutex_;
  std::map<std::string, TopicEntry> topics_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> robot_connected_{false};
  bool initialized_ = false;
};

}  // namespace krosc::viz_web
