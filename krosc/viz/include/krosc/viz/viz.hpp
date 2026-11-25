#pragma once

#include <memory>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/viz/viz_plugin.hpp"

class QTimer;

namespace krosc::net {
class Client;
}

namespace krosc::viz {

class VizWindow;

struct VizOptions {
  std::string server;              // HOST[:PORT], 비어 있으면 로컬
  std::string fixed_frame = "odom";
  std::string tab = "3d";
  std::string screenshot;          // 비어 있으면 스크린샷 없음
  std::vector<std::string> plugins;
  double duration = 2.0;           // screenshot 대기 시간
  double distance = 0.0;           // 카메라 거리 (0 = 기본)
};

// Viz 애플리케이션의 조립/수명주기. main.cpp는 인자만 파싱해서 이 클래스를 실행한다.
// Context/Net Client/VizWindow/플러그인 로드/상태 타이머/스크린샷을 모두 소유한다.
class Viz {
 public:
  Viz(VizOptions options, VizPluginRegistry registry);
  ~Viz();

  Viz(const Viz&) = delete;
  Viz& operator=(const Viz&) = delete;

  [[nodiscard]] bool initialize(std::string& error);
  int run();
  [[nodiscard]] VizWindow& window() { return *window_; }

 private:
  VizOptions options_;
  VizPluginRegistry registry_;
  Context context_;
  std::unique_ptr<net::Client> client_;
  std::unique_ptr<VizWindow> window_;
  QTimer* status_timer_ = nullptr;
  bool initialized_ = false;
};

}  // namespace krosc::viz
