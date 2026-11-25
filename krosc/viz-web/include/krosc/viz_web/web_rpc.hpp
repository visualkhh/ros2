#pragma once

// 브라우저 <-> 게이트웨이 JSON RPC. 저빈도 명령(지도/파라미터/액션)만 담당한다.
// 실제 처리는 로컬/원격 krosc 서비스 호출로 이어진다 (PeerNetwork RPC).

#include <functional>
#include <memory>
#include <string>

#include "krosc/viz_web/json.hpp"

namespace krosc {
class Context;
}

namespace krosc::viz_web {

class WebRpc {
 public:
  // 같은 브라우저 클라이언트로 보내는 응답/이벤트.
  using Reply = std::function<void(std::string)>;
  using Notify = std::function<void(const std::string& event, const Json& params)>;
  using Handler = std::function<Json(const Json& params, const Notify& notify)>;

  explicit WebRpc(Context& context);
  ~WebRpc();

  WebRpc(const WebRpc&) = delete;
  WebRpc& operator=(const WebRpc&) = delete;

  void add(std::string method, Handler handler);
  void start();
  void shutdown();

  // HttpServer 텍스트 프레임에서 호출한다. 파싱/실행은 워커 스레드에서 한다.
  void handle(const std::string& text, Reply reply);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 코어 기본 RPC: map_catalog, 파라미터 서비스, controller switch, RL reload.
void register_default_web_rpc(WebRpc& rpc, Context& context);

}  // namespace krosc::viz_web
