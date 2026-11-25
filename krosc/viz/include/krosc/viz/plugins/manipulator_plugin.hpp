#pragma once

#include <QObject>
#include <QString>

#include <array>
#include <string>

#include "krosc/viz/viz_plugin.hpp"

class QComboBox;
class QDockWidget;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;

namespace krosc::viz {

// 매니퓰레이터 수동 제어 플러그인: 관절 슬라이더, 그리퍼, Torque/STOW/HOME/E-STOP/Reset.
// arm_joint_command는 ros2_control forward_command_controller가 활성일 때 하드웨어로 간다.
class ManipulatorPlugin final : public QObject, public VizPlugin {
  Q_OBJECT

 public:
  ManipulatorPlugin();
  ~ManipulatorPlugin() override;

  void initialize(VizHost& host) override;
  void shutdown() override;

 private:
  void send_target();
  void refresh();
  void set_status(const QString& text);
  void set_slider_values(const std::array<double, 5>& positions);
  void cancel_moveit_if_executing();
  void reset_arm();
  [[nodiscard]] QString error_summary() const;

  VizHost* host_ = nullptr;
  QDockWidget* dock_ = nullptr;
  std::array<QSlider*, 4> sliders_{};
  std::array<QLabel*, 4> value_labels_{};
  QSlider* gripper_ = nullptr;
  QLabel* gripper_label_ = nullptr;
  QLabel* status_ = nullptr;
  QPushButton* torque_ = nullptr;
  QComboBox* reset_target_ = nullptr;
  QPushButton* reset_button_ = nullptr;
  QSlider* speed_ = nullptr;
  QLabel* speed_label_ = nullptr;
  QTimer* send_timer_ = nullptr;
  QTimer* refresh_timer_ = nullptr;
  std::array<double, 5> target_{};
  std::array<double, 5> output_{};
  bool moving_ = false;
  bool initialized_ = false;
  bool updating_ = false;
  bool torque_on_ = true;
  double speed_value_ = 0.35;
  bool active_ = false;
};

}  // namespace krosc::viz
