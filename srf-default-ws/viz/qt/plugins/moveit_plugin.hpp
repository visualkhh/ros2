#pragma once

#include <memory>

#include "srf/action.hpp"
#include "srf/msgs/moveit_msgs.hpp"
#include "srf/viz/viz_plugin.hpp"

class QCheckBox;
class QDockWidget;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;

namespace srf::viz {
class SceneView;
}

namespace srf_default_ws::viz {

// MoveIt 패널 + MoveIt 탭. 명령은 move_action 액션/scrub 서비스로,
// 상태는 moveit_* 토픽으로 받는다.
class MoveItPlugin final : public srf::viz::VizPlugin {
 public:
  MoveItPlugin() : srf::viz::VizPlugin({"moveit", "MoveIt", "1.0", {}}) {}
  void initialize(srf::viz::VizHost& host) override;
  void shutdown() override;

 private:
  void build_dock();
  void refresh();
  void publish_target(bool preview = false);
  void send_goal(bool plan_only, const std::string& preset = {});
  void send_scrub(int index);
  void update_preview();
  void set_target_from_current();
  void plan();
  void execute();
  void stop();
  void on_target_dragged(double x, double y, double z);
  void on_pitch_dragged(double degrees);
  void on_drag_finished();

  srf::viz::VizHost* host_ = nullptr;
  srf::viz::SceneView* view_ = nullptr;
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
  QTimer* preview_timer_ = nullptr;
  QTimer* scrub_timer_ = nullptr;
  using MoveClient = srf::ActionClient<srf::msgs::moveit::MoveGroupGoal,
                                         srf::msgs::moveit::MoveGroupFeedback,
                                         srf::msgs::moveit::MoveGroupResult>;
  std::unique_ptr<MoveClient> client_;
  std::shared_ptr<srf::ClientGoalHandle<srf::msgs::moveit::MoveGroupGoal,
                                          srf::msgs::moveit::MoveGroupFeedback,
                                          srf::msgs::moveit::MoveGroupResult>> handle_;
  int scrub_index_ = 0;
  bool dragging_ = false;
  double last_drag_publish_ = 0.0;
  bool executing_ = false;
  bool updating_ = false;
  bool target_initialized_ = false;
  bool has_plan_ = false;
  int points_ = 0;
};

}  // namespace srf_default_ws::viz
