#pragma once

#include <QObject>

#include <string>

#include "krosc/viz/viz_plugin.hpp"

class QComboBox;
class QDockWidget;
class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace krosc::viz {

// SLAM / 지도 플러그인: start/stop mapping, map 목록 로드/저장/삭제, collision 토글.
// 코어 VizWindow는 이 패널을 모른다.
class SlamPlugin final : public QObject, public VizPlugin {
  Q_OBJECT

 public:
  SlamPlugin();
  ~SlamPlugin() override;

  void initialize(VizHost& host) override;
  void shutdown() override;

 private:
  void refresh();
  void update_collision_button();
  [[nodiscard]] std::string slam_status_text() const;
  [[nodiscard]] std::string map_status_text() const;

  VizHost* host_ = nullptr;
  QDockWidget* dock_ = nullptr;
  QLabel* mode_label_ = nullptr;
  QPushButton* slam_button_ = nullptr;
  QComboBox* map_combo_ = nullptr;
  QLineEdit* map_name_ = nullptr;
  QLabel* status_label_ = nullptr;
  QPushButton* collision_button_ = nullptr;
  QTimer* timer_ = nullptr;
  std::string last_map_status_;
  bool collision_enabled_ = true;
  bool active_ = false;
};

}  // namespace krosc::viz
