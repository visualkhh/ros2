#pragma once

#include <QWidget>

#include <array>

class QCheckBox;
class QLabel;
class QSlider;
class QTimer;

namespace srf_default_ws::sim {

class WafflePiArm;

// 시뮬레이터 진실 뷰 옆에 붙는 OpenMANIPULATOR 수동 조작 패널.
class ArmPanel final : public QWidget {
 public:
  explicit ArmPanel(WafflePiArm& robot, QWidget* parent = nullptr);

 private:
  void apply_sliders();
  void refresh();
  void set_target(const std::array<double, 5>& target);

  WafflePiArm& robot_;
  std::array<QSlider*, 5> sliders_{};
  std::array<QLabel*, 5> values_{};
  QCheckBox* manual_ = nullptr;
  QTimer* timer_ = nullptr;
  bool updating_ = false;
};

}  // namespace srf_default_ws::sim
