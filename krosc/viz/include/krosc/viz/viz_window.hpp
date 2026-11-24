#pragma once

#include <QMainWindow>

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
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QTableWidget;
class QTimer;
class QVBoxLayout;

namespace krosc::viz {

class VizWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit VizWindow(Context& context, QWidget* parent = nullptr);
  ~VizWindow() override;

  [[nodiscard]] VizModel& model() { return model_; }
  [[nodiscard]] SceneView& scene() { return *scene_; }

  void set_fixed_frame(const std::string& frame);
  void set_connection_status(bool connected, const std::string& endpoint);
  [[nodiscard]] bool save_screenshot(const QString& path);

 private:
  void build_panel(QWidget* panel);
  void build_slam_panel(QWidget* panel, QVBoxLayout* layout);
  void refresh_topics();
  void refresh_slam();
  void update_tool_buttons();
  void publish_pose(const std::string& topic, double x, double y, double yaw);
  void publish_multi(bool preview);
  void send_pending();
  [[nodiscard]] std::string map_status_text() const;
  [[nodiscard]] std::string slam_status_text() const;

  VizModel model_;
  SceneView* scene_ = nullptr;
  QComboBox* fixed_frame_ = nullptr;
  QTableWidget* table_ = nullptr;
  QLabel* stats_label_ = nullptr;
  QLabel* connection_label_ = nullptr;
  std::map<std::string, QCheckBox*> toggles_;
  QComboBox* map_combo_ = nullptr;
  QLineEdit* map_name_ = nullptr;
  QLabel* slam_mode_label_ = nullptr;
  QLabel* slam_status_label_ = nullptr;
  QLabel* teleop_label_ = nullptr;
  QPushButton* slam_button_ = nullptr;
  QPushButton* pose_button_ = nullptr;
  QPushButton* goal_button_ = nullptr;
  QPushButton* multi_button_ = nullptr;
  QPushButton* preview_button_ = nullptr;
  QSlider* teleop_slider_ = nullptr;
  std::vector<std::tuple<double, double, double>> multi_goals_;
  std::optional<std::tuple<double, double, double>> last_single_goal_;
  bool preview_mode_ = false;
  QTimer* paint_timer_ = nullptr;
  QTimer* refresh_timer_ = nullptr;
  QTimer* slam_timer_ = nullptr;
};

}  // namespace krosc::viz
