#pragma once

#include <QObject>

#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/topic.hpp"
#include "krosc/viz/viz_plugin.hpp"

class QDockWidget;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTimer;

namespace krosc::viz {

// 주행 목표 플러그인: 2D Pose Estimate, 2D Goal Pose, Multi Goal, Preview/Cancel.
// SceneView/MapView2D의 tool 신호를 받아 goal 토픽으로 내보낸다.
class NavigationPlugin final : public QObject, public VizPlugin {
  Q_OBJECT

 public:
  NavigationPlugin();
  ~NavigationPlugin() override;

  void initialize(VizHost& host) override;
  void shutdown() override;

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  void set_tool(const std::string& tool);
  void sync_waypoints();
  void update_buttons();
  void update_multi_label();
  void remove_last_multi_goal();
  void handle_tool_pose(double x, double y, double yaw, bool from_map);
  void publish_pose(const std::string& topic, double x, double y, double yaw, bool from_map);
  void publish_multi(bool preview);
  void apply_costmap_params();
  void refresh_costmap_state();
  [[nodiscard]] std::string slam_status_text() const;

  VizHost* host_ = nullptr;
  QDockWidget* dock_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* multi_label_ = nullptr;
  QLabel* costmap_label_ = nullptr;
  QPushButton* pose_button_ = nullptr;
  QPushButton* preview_button_ = nullptr;
  QPushButton* move_button_ = nullptr;
  QPushButton* multi_button_ = nullptr;
  QPushButton* multi_preview_button_ = nullptr;
  QPushButton* multi_go_button_ = nullptr;
  QPushButton* clear_button_ = nullptr;
  QPushButton* costmap_apply_ = nullptr;
  QDoubleSpinBox* global_inflation_ = nullptr;
  QDoubleSpinBox* global_scaling_ = nullptr;
  QDoubleSpinBox* local_inflation_ = nullptr;
  QDoubleSpinBox* local_scaling_ = nullptr;
  QDoubleSpinBox* local_width_ = nullptr;
  QDoubleSpinBox* local_height_ = nullptr;
  QTimer* costmap_timer_ = nullptr;
  std::vector<std::tuple<double, double, double>> multi_goals_;
  std::string active_tool_;
  bool single_preview_ = false;
  bool active_ = false;
  mutable std::mutex costmap_mutex_;
  std::string costmap_state_;
  bool costmap_dirty_ = false;
  krosc::Subscription<krosc::msgs::String> costmap_subscription_;
};

}  // namespace krosc::viz
