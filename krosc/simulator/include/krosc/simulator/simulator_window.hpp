#pragma once

#include <QColor>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QMatrix4x4>
#include <QPointF>
#include <QSet>
#include <QVector3D>
#include <QWidget>

#include <array>
#include <deque>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#include "krosc/simulator/snapshot.hpp"
#include "krosc/simulator/world.hpp"

class QKeyEvent;
class QMouseEvent;
class QPainter;
class QSplitter;
class QTimer;
class QVBoxLayout;
class QWheelEvent;

namespace krosc {
class Node;
}

namespace krosc::simulator {

// "진실" 화면: 월드/로봇의 실제 상태와 잡음 없는 라이다를 그린다.
// 키보드 텔레옵은 로봇을 직접 만지지 않고 ROS 2 표준 cmd_vel 토픽으로 낸다.
class SimulatorView final : public QWidget {
  Q_OBJECT

 public:
  SimulatorView(std::vector<Wall> walls, SnapshotStore& store, krosc::Node& node,
          std::string teleop_topic = "cmd_vel", QWidget* parent = nullptr);

  void reset_camera();
  void set_camera_distance(double distance);
  void set_follow(bool follow);
  [[nodiscard]] bool follow() const { return follow_; }
  [[nodiscard]] QString help_text() const;

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void focusOutEvent(QFocusEvent* event) override;

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
  struct Face {
    std::vector<QVector3D> points;
    QColor fill;
    QColor outline;
    float depth = 0.0F;
  };

  void update_camera();
  void compute_matrix();
  [[nodiscard]] Projected project(const QVector3D& point) const;
  void draw_line(QPainter& painter, const QVector3D& a, const QVector3D& b, const QColor& color,
                 double width = 1.0) const;
  void queue_face(const std::vector<QVector3D>& points, const QColor& fill, const QColor& outline);
  void queue_box(const msgs::Transform& transform, const std::array<double, 3>& size,
                 const QColor& fill, const QColor& outline);
  void queue_cylinder(const msgs::Transform& transform, double radius, double length, int segments,
                      const QColor& fill, const QColor& outline);
  void flush_faces(QPainter& painter);
  void draw_grid(QPainter& painter) const;
  void draw_walls();
  void draw_objects(const SimulatorSnapshot& snapshot);
  void draw_robot(const SimulatorSnapshot& snapshot);
  void draw_robot_arrow(QPainter& painter, const SimulatorSnapshot& snapshot) const;
  void draw_scan(QPainter& painter, const SimulatorSnapshot& snapshot) const;
  void draw_hud(QPainter& painter, const SimulatorSnapshot& snapshot) const;
  void publish_teleop();

  std::vector<Wall> walls_;
  SnapshotStore& store_;
  krosc::Node& node_;
  std::string teleop_topic_;
  Camera camera_;
  QMatrix4x4 matrix_;
  QVector3D eye_;
  std::vector<Face> faces_;
  int view_width_ = 1;
  int view_height_ = 1;
  QPoint last_mouse_;
  bool orbiting_ = false;
  bool panning_ = false;
  bool follow_ = true;
  bool show_scan_ = true;
  QVector3D follow_target_{0.0F, 0.0F, 0.1F};
  QSet<int> keys_;
  double teleop_scale_ = 1.0;
  double last_linear_ = 0.0;
  double last_angular_ = 0.0;
  QTimer* teleop_timer_ = nullptr;
  QElapsedTimer fps_clock_;
  int frame_count_ = 0;
  double fps_ = 0.0;
};

class SimulatorWindow final : public QMainWindow {
  Q_OBJECT

 public:
  SimulatorWindow(std::vector<Wall> walls, SnapshotStore& store, krosc::Node& node,
            std::string teleop_topic = "cmd_vel",
            std::vector<QWidget*> side_panels = {}, QWidget* parent = nullptr);
  ~SimulatorWindow() override;

  [[nodiscard]] SimulatorView& view() { return *view_; }
  void add_side_panel(QWidget* panel);
  [[nodiscard]] bool save_screenshot(const QString& path);

 private:
  SimulatorView* view_ = nullptr;
  QSplitter* splitter_ = nullptr;
  QWidget* panel_container_ = nullptr;
  QVBoxLayout* panel_layout_ = nullptr;
  QTimer* timer_ = nullptr;
};

}  // namespace krosc::simulator
