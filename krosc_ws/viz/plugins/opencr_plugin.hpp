#pragma once

#include <string>

#include "krosc/viz/viz_plugin.hpp"

class QDockWidget;
class QLabel;
class QPushButton;
class QSpinBox;
class QTimer;

namespace krosc_ws::viz {

// OpenCR 보드 상태(배터리/전류/온도/링크) + 부저.
// 데이터는 /diagnostics 의 OpenCR, LDS-03 항목에서 읽는다.
class OpencrPlugin final : public krosc::viz::VizPlugin {
 public:
  OpencrPlugin() : krosc::viz::VizPlugin({"opencr", "OpenCR", "1.0", {"diagnostics"}}) {}
  void initialize(krosc::viz::VizHost& host) override;
  void shutdown() override;

 private:
  void refresh();
  void beep();

  krosc::viz::VizHost* host_ = nullptr;
  QDockWidget* dock_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* battery_ = nullptr;
  QLabel* current_ = nullptr;
  QLabel* temperature_ = nullptr;
  QLabel* uptime_ = nullptr;
  QLabel* link_ = nullptr;
  QLabel* lidar_ = nullptr;
  QSpinBox* frequency_ = nullptr;
  QSpinBox* duration_ = nullptr;
  QPushButton* beep_ = nullptr;
  QTimer* timer_ = nullptr;
};

}  // namespace krosc_ws::viz
