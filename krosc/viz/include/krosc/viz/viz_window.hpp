#pragma once

#include <QMainWindow>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/viz/scene_view.hpp"
#include "krosc/viz/viz_model.hpp"
#include "krosc/viz/viz_plugin.hpp"

class QCheckBox;
class QComboBox;
class QDockWidget;
class QLabel;
class QTableWidget;
class QTabWidget;
class QTimer;

namespace krosc::viz {

class MapView2D;

// 코어 Viz 창. 화면/토픽/TF/디스플레이 토글만 담당한다.
// 텔레옵, SLAM/지도, 주행 목표, 매니퓰레이터는 플러그인으로 붙인다.
class VizWindow : public QMainWindow, public VizHost {
  Q_OBJECT

 public:
  explicit VizWindow(Context& context, QWidget* parent = nullptr);
  ~VizWindow() override;

  [[nodiscard]] VizModel& model() override { return model_; }
  [[nodiscard]] SceneView& scene() override { return *scene_; }
  [[nodiscard]] MapView2D& map_view() override { return *map_view_; }
  [[nodiscard]] QMainWindow& window() override { return *this; }
  void add_dock(QDockWidget* dock, Qt::DockWidgetArea area) override;
  void add_tab(QWidget* tab, const QString& title, bool make_current = false) override;
  void show_tab(QWidget* tab) override;
  void load_plugins(VizPluginRegistry& registry, const std::vector<std::string>& ids);

  void set_fixed_frame(const std::string& frame);
  void set_connection_status(bool connected, const std::string& endpoint);
  void set_tab(int index);
  void set_tab(const std::string& name);
  void set_camera_distance(double distance);
  // 호환용: 실제 텔레옵/매니퓰레이터는 플러그인이 만든다.
  void set_manual_panels(bool teleop, bool manipulator);
  [[nodiscard]] bool save_screenshot(const QString& path);

 private:
  void build_panel(QWidget* panel);
  void refresh_topics();

  VizModel model_;
  std::vector<std::unique_ptr<IVizPlugin>> plugins_;
  SceneView* scene_ = nullptr;
  QTabWidget* tabs_ = nullptr;
  QTabWidget* control_tabs_ = nullptr;
  MapView2D* map_view_ = nullptr;
  QComboBox* fixed_frame_ = nullptr;
  QTableWidget* table_ = nullptr;
  QLabel* stats_label_ = nullptr;
  QLabel* connection_label_ = nullptr;
  QLabel* tf_label_ = nullptr;
  std::map<std::string, QCheckBox*> toggles_;
  QTimer* paint_timer_ = nullptr;
  QTimer* refresh_timer_ = nullptr;
};

}  // namespace krosc::viz
