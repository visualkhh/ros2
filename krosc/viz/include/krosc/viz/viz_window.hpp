#pragma once

#include <QMainWindow>

#include <map>
#include <string>

#include "krosc/context.hpp"
#include "krosc/viz/scene_view.hpp"
#include "krosc/viz/viz_model.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QTableWidget;
class QTimer;

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
  void refresh_topics();

  VizModel model_;
  SceneView* scene_ = nullptr;
  QComboBox* fixed_frame_ = nullptr;
  QTableWidget* table_ = nullptr;
  QLabel* stats_label_ = nullptr;
  QLabel* connection_label_ = nullptr;
  std::map<std::string, QCheckBox*> toggles_;
  QTimer* paint_timer_ = nullptr;
  QTimer* refresh_timer_ = nullptr;
};

}  // namespace krosc::viz
