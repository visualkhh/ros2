#pragma once

#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QWidget>

#include <memory>

#include "krosc/viz/viz_model.hpp"

class QPushButton;

namespace krosc::viz {

// RViz의 Map display처럼 OccupancyGrid를 위에서 본 2D 이미지로 그린다.
// 흰색=빈 곳, 검정=벽, 회색=모름. 로봇/스캔/경로를 겹쳐 그린다.
class MapView2D final : public QWidget {
  Q_OBJECT

 public:
  explicit MapView2D(VizModel& model, QWidget* parent = nullptr);

  void fit_map();
  void set_tool(const std::string& tool);
  [[nodiscard]] std::string tool() const { return tool_; }

 signals:
  void tool_changed(QString tool);
  void tool_pose(double x, double y, double yaw);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  void rebuild_image();
  void place_fit_button();
  [[nodiscard]] QPointF to_screen(double x, double y) const;
  [[nodiscard]] QPointF to_map(const QPointF& screen) const;

  VizModel& model_;
  std::shared_ptr<const msgs::OccupancyGrid> map_cache_;
  QImage image_;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double resolution_ = 0.05;
  double zoom_ = 50.0;
  double offset_x_ = 0.0;  // map origin.x가 놓이는 화면 x
  double offset_y_ = 0.0;  // map origin.y가 놓이는 화면 y (아래쪽)
  bool dragging_ = false;
  bool fitted_once_ = false;
  QPoint last_mouse_;
  std::string tool_;
  bool tool_dragging_ = false;
  QPointF tool_start_map_;
  QPointF tool_current_map_;
  QPushButton* fit_button_ = nullptr;
};

}  // namespace krosc::viz
