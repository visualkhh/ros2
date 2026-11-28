#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/net/topic_codec.hpp"
#include "krosc/qos.hpp"

namespace krosc::viz_web {

// 브라우저 프레임 프로토콜과 로컬 Context 토픽을 잇는 웹 Viz 전용 브리지.
// 어떤 토픽을 브라우저로 보낼지는 subscribe()로 VizWeb/플러그인이 정한다.
// 코어(engine)는 이 클래스를 모른다.
class ContextLink {
 public:
  struct TopicSpec {
    std::string name;
    net::Kind kind = net::Kind::String;
    QoS qos = QOS_DEFAULT;
  };

  using FrameHandler = std::function<void(const net::Frame&)>;
  using StatusHandler = std::function<void(bool)>;

  explicit ContextLink(Context& context);
  ~ContextLink();

  ContextLink(const ContextLink&) = delete;
  ContextLink& operator=(const ContextLink&) = delete;

  void set_frame_handler(FrameHandler handler);
  void set_status_handler(StatusHandler handler);
  void start();
  // 같은 이름이 이미 구독 중이면 false. 성공하면 true.
  bool subscribe(const TopicSpec& topic);
  bool unsubscribe(const std::string& name);
  [[nodiscard]] bool subscribed(const std::string& name) const;
  void shutdown();

  [[nodiscard]] bool connected() const;
  [[nodiscard]] std::string endpoint() const;
  [[nodiscard]] bool send(const net::Frame& frame);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace krosc::viz_web
