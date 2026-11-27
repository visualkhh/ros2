#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/topic.hpp"
#include "krosc/viz/viz_plugin.hpp"

class QDockWidget;
class QLabel;
class QTextEdit;
class QTimer;

namespace krosc_ws::viz {

// viz-web AI 패널과 같은 command/plan/action + server/routing 정보를 보여주는 Qt dock.
// llm/command 서비스로 명령을 보내고, llm/status / pick/status / ai_info를 구독한다.
class AiPlugin final : public krosc::viz::VizPlugin {
 public:
  AiPlugin() : krosc::viz::VizPlugin({"ai", "AI", "1.0", {}}) {}
  void initialize(krosc::viz::VizHost& host) override;
  void shutdown() override;

 private:
  struct State {
    std::mutex mutex;
    std::string info;
    std::string llm_status;
    std::string pick_status;
    std::string vlm_status;
    std::string context;
  };

  void send_text(const QString& text);
  void update_ui();

  krosc::viz::VizHost* host_ = nullptr;
  std::shared_ptr<State> state_;
  QDockWidget* dock_ = nullptr;
  QLabel* command_ = nullptr;
  QLabel* plan_ = nullptr;
  QLabel* action_ = nullptr;
  QLabel* vlm_ = nullptr;
  QLabel* source_ = nullptr;
  QLabel* route_ = nullptr;
  QLabel* via_ = nullptr;
  QLabel* llm_server_ = nullptr;
  QLabel* vlm_server_ = nullptr;
  QLabel* vla_server_ = nullptr;
  QLabel* llm_link_ = nullptr;
  QLabel* vlm_link_ = nullptr;
  QLabel* vla_link_ = nullptr;
  QLabel* pose_ = nullptr;
  QLabel* size_ = nullptr;
  QLabel* lidar_ = nullptr;
  QLabel* arm_ = nullptr;
  QLabel* motor_ = nullptr;
  QLabel* wheels_ = nullptr;
  QLabel* camera_ = nullptr;
  QTextEdit* input_ = nullptr;
  QTimer* timer_ = nullptr;
  krosc::Subscription<krosc::msgs::String> info_subscription_;
  krosc::Subscription<krosc::msgs::String> status_subscription_;
  krosc::Subscription<krosc::msgs::String> pick_subscription_;
  krosc::Subscription<krosc::msgs::String> vlm_subscription_;
  krosc::Subscription<krosc::msgs::String> context_subscription_;
};

}  // namespace krosc_ws::viz
