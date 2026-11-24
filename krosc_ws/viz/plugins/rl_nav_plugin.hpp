#pragma once

#include <mutex>
#include <string>

#include "krosc/msgs.hpp"
#include "krosc/topic.hpp"
#include "krosc/viz/viz_plugin.hpp"

class QComboBox;
class QDockWidget;
class QLabel;
class QPushButton;
class QTimer;

namespace krosc_ws::viz {

// RL navigation(ONNX) ON/OFF, fallback 선택, 상태 표시.
// 실제 추론은 Pi의 rl_nav_controller 노드가 하고, 이 플러그인은 토픽만 다룬다.
class RlNavPlugin final : public krosc::viz::VizPlugin {
 public:
  RlNavPlugin() : krosc::viz::VizPlugin({"rl_nav", "RL Nav", "1.0", {}}) {}
  void initialize(krosc::viz::VizHost& host) override;
  void shutdown() override;

 private:
  void refresh();
  void set_enabled(bool enabled);
  void set_fallback(const QString& fallback);
  void set_model(const QString& model);

  krosc::viz::VizHost* host_ = nullptr;
  QDockWidget* dock_ = nullptr;
  QPushButton* toggle_ = nullptr;
  QComboBox* fallback_ = nullptr;
  QComboBox* model_ = nullptr;
  QPushButton* reload_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* detail_ = nullptr;
  QTimer* timer_ = nullptr;
  krosc::Subscription<krosc::msgs::String> model_list_subscription_;
  mutable std::mutex model_mutex_;
  std::string model_list_;
  bool model_list_dirty_ = false;
  bool updating_ = false;
  bool enabled_ = false;
};

}  // namespace krosc_ws::viz
