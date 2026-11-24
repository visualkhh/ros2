#pragma once

#include <QHash>
#include <QMainWindow>

#include <array>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/viz/scene_view.hpp"
#include "krosc/viz/viz_model.hpp"

class QCheckBox;
class QComboBox;
class QDockWidget;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTabWidget;
class QTimer;
class QVBoxLayout;

namespace krosc::viz {

class MapView2D;

class VizWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit VizWindow(Context& context, QWidget* parent = nullptr);
  ~VizWindow() override;

  [[nodiscard]] VizModel& model() { return model_; }
  [[nodiscard]] SceneView& scene() { return *scene_; }

  void set_fixed_frame(const std::string& frame);
  void set_connection_status(bool connected, const std::string& endpoint);
  void set_tab(int index);
  void set_camera_distance(double distance);
  void set_manual_panels(bool teleop, bool manipulator, bool moveit);
  [[nodiscard]] bool save_screenshot(const QString& path);

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  void build_panel(QWidget* panel);
  void build_slam_panel(QWidget* panel, QVBoxLayout* layout);
  void refresh_topics();
  void refresh_slam();
  void update_tool_buttons();
  void update_collision_button();
  void build_manual_docks();
  void build_teleop_dock();
  void build_manipulator_dock();
  void build_moveit_dock();
  void publish_teleop_pad();
  void send_arm_target();
  void refresh_manual();
  void refresh_moveit();
  void publish_moveit_target();
  void update_moveit_preview();
  void moveit_set_target_from_current();
  void moveit_plan();
  void moveit_execute();
  void moveit_stop();
  void reset_arm();
  [[nodiscard]] QString arm_error_summary() const;
  void set_arm_slider_values(const std::array<double, 5>& positions);
  void set_tool(const std::string& tool);
  void sync_waypoints();
  void update_multi_label();
  void remove_last_multi_goal();
  void handle_tool_pose(const std::string& tool, double x, double y, double yaw, bool from_map);
  void publish_pose(const std::string& topic, double x, double y, double yaw, bool from_map);
  void publish_multi(bool preview);
  void send_pending();
  [[nodiscard]] std::string map_status_text() const;
  [[nodiscard]] std::string slam_status_text() const;

  VizModel model_;
  SceneView* scene_ = nullptr;
  QTabWidget* tabs_ = nullptr;
  MapView2D* map_view_ = nullptr;
  QComboBox* fixed_frame_ = nullptr;
  QTableWidget* table_ = nullptr;
  QLabel* stats_label_ = nullptr;
  QLabel* connection_label_ = nullptr;
  std::map<std::string, QCheckBox*> toggles_;
  QComboBox* map_combo_ = nullptr;
  QLineEdit* map_name_ = nullptr;
  QLabel* slam_mode_label_ = nullptr;
  QLabel* slam_status_label_ = nullptr;
  QLabel* multi_label_ = nullptr;
  QLabel* tf_label_ = nullptr;
  QLabel* teleop_label_ = nullptr;
  QPushButton* slam_button_ = nullptr;
  QPushButton* pose_button_ = nullptr;
  QPushButton* goal_button_ = nullptr;
  QPushButton* multi_button_ = nullptr;
  QPushButton* collision_button_ = nullptr;
  bool collision_enabled_ = true;
  QPushButton* preview_button_ = nullptr;
  QSlider* teleop_slider_ = nullptr;
  std::vector<std::tuple<double, double, double>> multi_goals_;
  std::optional<std::tuple<double, double, double>> last_single_goal_;
  bool preview_mode_ = false;
  std::string last_map_status_;
  std::string active_tool_;
  QTimer* paint_timer_ = nullptr;
  QTimer* refresh_timer_ = nullptr;
  QTimer* slam_timer_ = nullptr;

  QDockWidget* teleop_dock_ = nullptr;
  QDockWidget* manipulator_dock_ = nullptr;
  QDockWidget* moveit_dock_ = nullptr;
  QDoubleSpinBox* moveit_x_ = nullptr;
  QDoubleSpinBox* moveit_y_ = nullptr;
  QDoubleSpinBox* moveit_z_ = nullptr;
  QDoubleSpinBox* moveit_pitch_ = nullptr;
  QDoubleSpinBox* moveit_gripper_ = nullptr;
  QCheckBox* moveit_cartesian_ = nullptr;
  QPushButton* moveit_plan_button_ = nullptr;
  QPushButton* moveit_execute_button_ = nullptr;
  QPushButton* moveit_cancel_button_ = nullptr;
  QPushButton* moveit_current_button_ = nullptr;
  QLabel* moveit_current_label_ = nullptr;
  QLabel* moveit_status_label_ = nullptr;
  QSlider* moveit_scrub_ = nullptr;
  bool moveit_executing_ = false;
  bool moveit_updating_ = false;
  bool moveit_target_initialized_ = false;
  bool moveit_has_plan_ = false;
  int moveit_points_ = 0;
  std::array<QSlider*, 4> arm_sliders_{};
  std::array<QLabel*, 4> arm_value_labels_{};
  QSlider* gripper_slider_ = nullptr;
  QLabel* gripper_value_label_ = nullptr;
  QLabel* arm_status_label_ = nullptr;
  QPushButton* arm_torque_button_ = nullptr;
  QComboBox* arm_reset_target_ = nullptr;
  QPushButton* arm_reset_button_ = nullptr;
  QTimer* teleop_pad_timer_ = nullptr;
  QTimer* arm_send_timer_ = nullptr;
  QTimer* manual_refresh_timer_ = nullptr;
  int teleop_direction_ = 0;
  QHash<int, int> teleop_key_codes_;
  double teleop_last_linear_ = 0.0;
  double teleop_last_angular_ = 0.0;
  std::array<double, 5> arm_target_{};
  std::array<double, 5> arm_output_{};
  bool arm_moving_ = false;
  bool arm_initialized_ = false;
  bool arm_updating_ = false;
  bool arm_torque_on_ = true;
  double arm_speed_ = 0.35;
  QSlider* arm_speed_slider_ = nullptr;
  QLabel* arm_speed_label_ = nullptr;
};

}  // namespace krosc::viz
