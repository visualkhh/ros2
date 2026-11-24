#pragma once

#include <QHash>
#include <QObject>

#include <string>

#include "krosc/viz/viz_plugin.hpp"

class QDockWidget;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;

namespace krosc::viz {

// 주행 텔레옵 플러그인: 방향 패드 + 속도 + W/A/S/D 키 전달.
// 코어 VizWindow는 이 도크를 모른다. 플러그인을 넣어야만 텔레옵이 켜진다.
class TeleopPlugin final : public QObject, public VizPlugin {
  Q_OBJECT

 public:
  TeleopPlugin();
  ~TeleopPlugin() override;

  void initialize(VizHost& host) override;
  void shutdown() override;

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  void publish_pad();

  VizHost* host_ = nullptr;
  QDockWidget* dock_ = nullptr;
  QSlider* speed_ = nullptr;
  QLabel* speed_label_ = nullptr;
  QTimer* pad_timer_ = nullptr;
  QHash<int, int> key_codes_;
  int direction_ = 0;
  double last_linear_ = 0.0;
  double last_angular_ = 0.0;
  bool active_ = false;
};

}  // namespace krosc::viz
