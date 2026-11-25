#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "krosc/viz_web/viz_web_plugins.hpp"

#ifndef KROSC_VIZ_WEB_ROOT
#define KROSC_VIZ_WEB_ROOT "viz-web/www"
#endif

namespace krosc::net {
class FrameClient;
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
};

// 웹 Viz 게이트웨이의 조립/수명주기. main.cpp는 인자만 파싱해서 이 클래스를 실행한다.
// net::FrameClient(로봇 TCP) + HttpServer(정적 파일/WebSocket)를 소유하고 프레임을 중계한다.
class VizWeb {
 public:
  explicit VizWeb(VizWebOptions options, VizWebPluginRegistry registry = {});
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

 private:
  void shutdown();

  VizWebOptions options_;
  VizWebPluginRegistry registry_;
  std::unique_ptr<net::FrameClient> link_;
  std::unique_ptr<HttpServer> server_;
  std::atomic<bool> stop_{false};
  bool initialized_ = false;
};

}  // namespace krosc::viz_web
