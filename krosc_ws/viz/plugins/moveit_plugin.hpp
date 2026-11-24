#pragma once

#include "krosc/viz/viz_plugin.hpp"

class QCheckBox;
class QDockWidget;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;

namespace krosc::viz {
class SceneView;
}

namespace krosc_ws::viz {

// MoveIt 패널 + MoveIt 탭. Pi의 move_group 노드와 토픽으로만 통신한다.
class MoveItPlugin final : public krosc::viz::VizPlugin {
 public:
  MoveItPlugin() : krosc::viz::VizPlugin({"moveit", "MoveIt", "1.0", {}}) {}
  void initialize(krosc::viz::VizHost& host) override;
  void shutdown() override;

 private:
  void build_dock();
  void refresh();
  void publish_target(bool preview = false);
  void update_preview();
  void set_target_from_current();
  void plan();
  void execute();
  void stop();
  void on_target_dragged(double x, double y, double z);
  void on_pitch_dragged(double degrees);
  void on_drag_finished();

  krosc::viz::VizHost* host_ = nullptr;
  krosc::viz::SceneView* view_ = nullptr;
  QDockWidget* dock_ = nullptr;
  QDoubleSpinBox* x_ = nullptr;
  QDoubleSpinBox* y_ = nullptr;
  QDoubleSpinBox* z_ = nullptr;
  QDoubleSpinBox* pitch_ = nullptr;
  QDoubleSpinBox* gripper_ = nullptr;
  QCheckBox* cartesian_ = nullptr;
  QPushButton* current_button_ = nullptr;
  QPushButton* plan_button_ = nullptr;
  QPushButton* execute_button_ = nullptr;
  QPushButton* cancel_button_ = nullptr;
  QLabel* current_label_ = nullptr;
  QLabel* status_label_ = nullptr;
  QSlider* scrub_ = nullptr;
  QTimer* refresh_timer_ = nullptr;
  bool dragging_ = false;
  double last_drag_publish_ = 0.0;
  bool executing_ = false;
  bool updating_ = false;
  bool target_initialized_ = false;
  bool has_plan_ = false;
  int points_ = 0;
};

}  // namespace krosc_ws::viz
