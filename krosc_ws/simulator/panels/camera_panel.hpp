#pragma once

#include <QWidget>

class QLabel;
class QTimer;

namespace krosc_ws::sim {

class SimulatorRealSense;

// 시뮬레이터 Qt 창 오른쪽에 붙는 D435 미리보기.
// 시뮬레이션 스레드가 만든 최신 color/depth를 복사해서 그린다.
class CameraPanel final : public QWidget {
 public:
  explicit CameraPanel(SimulatorRealSense& camera, QWidget* parent = nullptr);

 private:
  void refresh();

  SimulatorRealSense& camera_;
  QLabel* color_ = nullptr;
  QLabel* depth_ = nullptr;
  QLabel* stats_ = nullptr;
  QTimer* timer_ = nullptr;
};

}  // namespace krosc_ws::sim
