#pragma once

#include <QElapsedTimer>
#include <QImage>
#include <QMatrix4x4>
#include <QPointF>
#include <QSet>
#include <QTimer>
#include <QVector3D>
#include <QWidget>

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/urdf.hpp"
#include "krosc/viz/viz_model.hpp"

class QFocusEvent;
class QKeyEvent;
class QMouseEvent;
class QPainter;
class QWheelEvent;

namespace krosc::viz {

// rviz 같은 3D 장면: QPainter로 투영해 그린다 (OpenGL 없이 CPU/래스터). 토픽만 읽는다.
class SceneView : public QWidget {
  Q_OBJECT

 public:
  explicit SceneView(VizModel& model, QWidget* parent = nullptr);

  void set_enabled(const std::string& key, bool value);
  [[nodiscard]] bool enabled(const std::string& key) const;
  void set_follow(bool follow);
  [[nodiscard]] bool follow() const { return follow_; }
  void set_camera_distance(double distance);
  void set_teleop_scale(double scale);
  [[nodiscard]] double teleop_scale() const { return teleop_scale_; }
  void set_tool(const std::string& tool);
  [[nodiscard]] std::string tool() const { return tool_; }
  void reset_camera();
  void fixed_frame_changed();
  [[nodiscard]] QString help_text() const;

 signals:
  void follow_changed(bool follow);
  void teleop_scale_changed(double scale);
  void tool_changed(QString tool);
  void tool_pose(double x, double y, double yaw);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void focusOutEvent(QFocusEvent* event) override;
  void leaveEvent(QEvent* event) override;

 private:
  struct Camera {
    double yaw = -1.0;
    double pitch = 1.0;
    double distance = 7.0;
    QVector3D target{0.0F, 0.0F, 0.1F};
  };
  struct Projected {
    QPointF point;
    bool visible = false;
  };

  void update_camera();
  void publish_teleop();
  void compute_matrix();
  [[nodiscard]] Projected project(const QVector3D& point) const;
  [[nodiscard]] std::optional<QVector3D> screen_to_ground(const QPoint& point) const;
  [[nodiscard]] std::optional<msgs::PoseStamped> robot_pose() const;
  [[nodiscard]] std::optional<QVector3D> robot_position() const;
  [[nodiscard]] msgs::Transform link_transform(const std::string& frame) const;

  void draw_grid(QPainter& painter) const;
  void draw_axes(QPainter& painter) const;
  void draw_map(QPainter& painter) const;
  void draw_grid_image(QPainter& painter, const msgs::OccupancyGrid& grid, const QImage& image) const;
  void draw_costmap(QPainter& painter, const std::shared_ptr<const msgs::OccupancyGrid>& grid, QImage& image,
                    std::shared_ptr<const msgs::OccupancyGrid>& cache, const char* label) const;
  void draw_robot(QPainter& painter);
  void draw_scan(QPainter& painter) const;
  void draw_imu(QPainter& painter) const;
  void draw_odometry_arrows(QPainter& painter) const;
  void draw_path(QPainter& painter, const std::shared_ptr<const msgs::Path>& path, const QColor& color) const;
  void draw_trail(QPainter& painter);
  void draw_logs(QPainter& painter) const;
  void draw_hud(QPainter& painter) const;

  void draw_line(QPainter& painter, const QVector3D& a, const QVector3D& b, const QColor& color,
                 double width = 1.0) const;
  void draw_face(QPainter& painter, const std::vector<QVector3D>& points, const QColor& fill,
                 const QColor& outline) const;
  void draw_box(QPainter& painter, const msgs::Transform& transform, const std::array<double, 3>& size,
                const QColor& fill, const QColor& outline) const;
  void draw_cylinder(QPainter& painter, const msgs::Transform& transform, double radius, double length,
                     const QColor& fill, const QColor& outline) const;
  void draw_yaw_arrow(QPainter& painter, const QVector3D& origin, const msgs::Quaternion& orientation,
                      double length, const QColor& color) const;
  void draw_label(QPainter& painter, const QVector3D& position, const QString& text, const QColor& color) const;

  VizModel& model_;
  Camera camera_;
  QMatrix4x4 matrix_;
  int view_width_ = 1;
  int view_height_ = 1;
  std::map<std::string, bool> enabled_;
  QPoint last_mouse_;
  bool orbiting_ = false;
  bool panning_ = false;
  QSet<int> keys_;
  bool follow_ = true;
  QVector3D follow_target_{0.0F, 0.0F, 0.1F};
  std::deque<QVector3D> trail_;
  std::unique_ptr<UrdfRobot> robot_;
  std::string robot_source_;
  mutable QImage map_image_;
  mutable QImage global_costmap_image_;
  mutable QImage local_costmap_image_;
  mutable std::shared_ptr<const msgs::OccupancyGrid> map_cached_;
  mutable std::shared_ptr<const msgs::OccupancyGrid> global_costmap_cached_;
  mutable std::shared_ptr<const msgs::OccupancyGrid> local_costmap_cached_;
  QElapsedTimer fps_clock_;
  int frame_count_ = 0;
  double fps_ = 0.0;
  double teleop_scale_ = 1.0;
  std::string tool_;
  bool tool_dragging_ = false;
  QPoint tool_start_;
  std::optional<QVector3D> tool_start_ground_;
  double last_linear_ = 0.0;
  double last_angular_ = 0.0;
  QTimer* teleop_timer_ = nullptr;
};

}  // namespace krosc::viz
