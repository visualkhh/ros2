#pragma once

#include <QObject>

#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "krosc/viz/viz_plugin.hpp"

class QDockWidget;
class QLabel;
class QPushButton;

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
  void send_pending();
  [[nodiscard]] std::string slam_status_text() const;

  VizHost* host_ = nullptr;
  QDockWidget* dock_ = nullptr;
  QLabel* status_ = nullptr;
  QLabel* multi_label_ = nullptr;
  QPushButton* pose_button_ = nullptr;
  QPushButton* goal_button_ = nullptr;
  QPushButton* multi_button_ = nullptr;
  QPushButton* preview_button_ = nullptr;
  std::vector<std::tuple<double, double, double>> multi_goals_;
  std::optional<std::tuple<double, double, double>> last_single_goal_;
  std::string active_tool_;
  bool preview_mode_ = false;
  bool active_ = false;
};

}  // namespace krosc::viz
