#include "krosc/sim/view_window.hpp"

#include <QFocusEvent>
#include <QFont>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QStatusBar>
#include <QStringList>
#include <QTimer>
#include <QVector4D>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "krosc/math.hpp"

namespace krosc::sim {

namespace {
constexpr double kPi = 3.14159265358979323846;

QColor with_alpha(QColor color, int alpha) {
  color.setAlpha(alpha);
  return color;
}
}  // namespace

SimView::SimView(std::vector<Wall> walls, TurtleBot3& robot, SnapshotStore& store, QWidget* parent)
    : QWidget(parent), walls_(std::move(walls)), robot_(robot), store_(store) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setMinimumSize(520, 420);
  fps_clock_.start();
  reset_camera();
  teleop_timer_ = new QTimer(this);
  connect(teleop_timer_, &QTimer::timeout, this, &SimView::publish_teleop);
  teleop_timer_->start(50);
}

void SimView::reset_camera() {
  camera_ = Camera{};
  follow_ = true;
  follow_target_ = camera_.target;
  update();
}

void SimView::set_camera_distance(double distance) {
  camera_.distance = std::clamp(distance, 0.4, 40.0);
  update();
}

void SimView::set_follow(bool follow) {
  follow_ = follow;
  if (follow_) {
    follow_target_ = camera_.target;
  }
  update();
}

QString SimView::help_text() const {
  return QStringLiteral("truth view · W/A/S/D drive · Q/Z speed · F follow · C overview · drag orbit · wheel zoom");
}

void SimView::update_camera() {
  if (!follow_) {
    return;
  }
  const auto snapshot = store_.latest();
  if (!snapshot) {
    return;
  }
  const QVector3D target(static_cast<float>(snapshot->robot_pose.x), static_cast<float>(snapshot->robot_pose.y), 0.1F);
  follow_target_ += (target - follow_target_) * 0.12F;
  camera_.target = follow_target_;
}

void SimView::compute_matrix() {
  view_width_ = std::max(1, width());
  view_height_ = std::max(1, height());
  const double cp = std::cos(camera_.pitch);
  const double sp = std::sin(camera_.pitch);
  const QVector3D eye = camera_.target +
                        QVector3D(static_cast<float>(cp * std::cos(camera_.yaw)),
                                  static_cast<float>(cp * std::sin(camera_.yaw)), static_cast<float>(sp)) *
                            static_cast<float>(camera_.distance);
  QMatrix4x4 view;
  view.lookAt(eye, camera_.target, QVector3D(0.0F, 0.0F, 1.0F));
  QMatrix4x4 projection;
  projection.perspective(58.0F, static_cast<float>(view_width_) / static_cast<float>(view_height_), 0.03F, 250.0F);
  matrix_ = projection * view;
}

SimView::Projected SimView::project(const QVector3D& point) const {
  const QVector4D clip = matrix_ * QVector4D(point, 1.0F);
  if (clip.w() <= 1e-6F) {
    return {QPointF{}, false};
  }
  const QVector3D ndc = clip.toVector3D() / clip.w();
  if (ndc.z() < -1.0F || ndc.z() > 1.0F) {
    return {QPointF{}, false};
  }
  return {QPointF((ndc.x() + 1.0F) * 0.5F * static_cast<float>(view_width_),
                  (1.0F - ndc.y()) * 0.5F * static_cast<float>(view_height_)),
          true};
}

void SimView::draw_line(QPainter& painter, const QVector3D& a, const QVector3D& b, const QColor& color,
                        double width) const {
  const Projected pa = project(a);
  const Projected pb = project(b);
  if (!pa.visible || !pb.visible) {
    return;
  }
  painter.setPen(QPen(color, width));
  painter.drawLine(pa.point, pb.point);
}

void SimView::draw_face(QPainter& painter, const std::vector<QVector3D>& points, const QColor& fill,
                        const QColor& outline) const {
  QPolygonF polygon;
  for (const auto& point : points) {
    const Projected projected = project(point);
    if (!projected.visible) {
      return;
    }
    polygon << projected.point;
  }
  painter.setBrush(fill);
  painter.setPen(QPen(outline, 1.0));
  painter.drawPolygon(polygon);
  painter.setBrush(Qt::NoBrush);
}

void SimView::draw_box(QPainter& painter, const msgs::Transform& transform, const std::array<double, 3>& size,
                       const QColor& fill, const QColor& outline) const {
  const double hx = size[0] / 2.0;
  const double hy = size[1] / 2.0;
  const double hz = size[2] / 2.0;
  std::array<QVector3D, 8> corners;
  int index = 0;
  for (int sx : {-1, 1}) {
    for (int sy : {-1, 1}) {
      for (int sz : {-1, 1}) {
        const msgs::Vector3 point = tfmath::apply(transform, msgs::Vector3{sx * hx, sy * hy, sz * hz});
        corners[static_cast<std::size_t>(index++)] =
            QVector3D(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z));
      }
    }
  }
  const auto face = [&](int a, int b, int c, int d) {
    draw_face(painter, {corners[static_cast<std::size_t>(a)], corners[static_cast<std::size_t>(b)],
                        corners[static_cast<std::size_t>(c)], corners[static_cast<std::size_t>(d)]},
              fill, outline);
  };
  face(0, 2, 3, 1);
  face(4, 5, 7, 6);
  face(0, 1, 5, 4);
  face(2, 6, 7, 3);
  face(0, 4, 6, 2);
  face(1, 3, 7, 5);
}

void SimView::draw_cylinder(QPainter& painter, const msgs::Transform& transform, double radius, double length,
                            int segments, const QColor& fill, const QColor& outline) const {
  const int count = std::max(3, segments);
  const double half = length / 2.0;
  std::vector<QVector3D> bottom(static_cast<std::size_t>(count));
  std::vector<QVector3D> top(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    const msgs::Vector3 low = tfmath::apply(transform, msgs::Vector3{x, y, -half});
    const msgs::Vector3 high = tfmath::apply(transform, msgs::Vector3{x, y, half});
    bottom[static_cast<std::size_t>(i)] =
        QVector3D(static_cast<float>(low.x), static_cast<float>(low.y), static_cast<float>(low.z));
    top[static_cast<std::size_t>(i)] =
        QVector3D(static_cast<float>(high.x), static_cast<float>(high.y), static_cast<float>(high.z));
  }
  for (int i = 0; i < count; ++i) {
    const int next = (i + 1) % count;
    draw_face(painter, {bottom[static_cast<std::size_t>(i)], bottom[static_cast<std::size_t>(next)],
                        top[static_cast<std::size_t>(next)], top[static_cast<std::size_t>(i)]},
              fill, outline);
  }
  draw_face(painter, top, with_alpha(fill, std::min(255, fill.alpha() + 20)), outline);
  draw_face(painter, bottom, fill, outline);
}

void SimView::draw_grid(QPainter& painter) const {
  constexpr double kStep = 0.5;
  constexpr int kHalf = 20;
  for (int i = -kHalf; i <= kHalf; ++i) {
    const double coordinate = i * kStep;
    const QColor color = i % 2 == 0 ? QColor(70, 78, 92, 65) : QColor(55, 62, 74, 40);
    draw_line(painter, QVector3D(static_cast<float>(coordinate), -static_cast<float>(kHalf) * static_cast<float>(kStep), 0.0F),
              QVector3D(static_cast<float>(coordinate), static_cast<float>(kHalf) * static_cast<float>(kStep), 0.0F), color);
    draw_line(painter, QVector3D(-static_cast<float>(kHalf) * static_cast<float>(kStep), static_cast<float>(coordinate), 0.0F),
              QVector3D(static_cast<float>(kHalf) * static_cast<float>(kStep), static_cast<float>(coordinate), 0.0F), color);
  }
  draw_line(painter, QVector3D(-10.0F, 0.0F, 0.0F), QVector3D(10.0F, 0.0F, 0.0F), QColor(230, 90, 90, 140), 1.5);
  draw_line(painter, QVector3D(0.0F, -10.0F, 0.0F), QVector3D(0.0F, 10.0F, 0.0F), QColor(90, 220, 120, 140), 1.5);
}

void SimView::draw_walls(QPainter& painter) const {
  for (const auto& wall : walls_) {
    const msgs::Transform transform{{wall.center_x, wall.center_y, wall.height / 2.0}, {}};
    draw_box(painter, transform, {wall.half_x * 2.0, wall.half_y * 2.0, wall.height},
             QColor(215, 220, 232, 235), QColor(250, 252, 255, 255));
  }
}

void SimView::draw_pucks(QPainter& painter, const SimSnapshot& snapshot) const {
  for (const auto& puck : snapshot.pucks) {
    const msgs::Transform transform{{puck.pose.x, puck.pose.y, puck.height / 2.0}, tfmath::from_yaw(puck.pose.theta)};
    const QColor color(static_cast<int>(puck.color[0] * 255.0), static_cast<int>(puck.color[1] * 255.0),
                       static_cast<int>(puck.color[2] * 255.0), 230);
    draw_cylinder(painter, transform, puck.radius, puck.height, puck.shape, color, color.lighter(130));
  }
}

void SimView::draw_robot(QPainter& painter, const SimSnapshot& snapshot) const {
  const msgs::Transform base{{snapshot.robot_pose.x, snapshot.robot_pose.y, 0.0},
                             tfmath::from_yaw(snapshot.robot_pose.theta)};
  const auto local = [&](double x, double y, double z, const msgs::Quaternion& rotation = {}) {
    return tfmath::compose(base, {{x, y, z}, rotation});
  };
  draw_box(painter, local(0.0, 0.0, 0.057), {0.266, 0.266, 0.094}, QColor(195, 202, 216, 235),
           QColor(245, 248, 255, 255));
  draw_box(painter, local(0.073, -0.011, 0.084), {0.04, 0.03, 0.03}, QColor(55, 58, 66, 235),
           QColor(120, 125, 138, 255));
  draw_cylinder(painter, local(-0.064, 0.0, 0.132), 0.035, 0.03, 16, QColor(65, 68, 78, 235),
                QColor(150, 155, 168, 255));
  const msgs::Quaternion wheel_rotation = tfmath::from_rpy(kPi / 2.0, 0.0, 0.0);
  draw_cylinder(painter, local(0.0, 0.144, 0.023, wheel_rotation), 0.033, 0.018, 16, QColor(45, 47, 54, 240),
                QColor(115, 120, 132, 255));
  draw_cylinder(painter, local(0.0, -0.144, 0.023, wheel_rotation), 0.033, 0.018, 16, QColor(45, 47, 54, 240),
                QColor(115, 120, 132, 255));
  for (double side : {-1.0, 1.0}) {
    draw_box(painter, local(-0.177, side * 0.064, 0.01), {0.03, 0.009, 0.02}, QColor(90, 94, 104, 235),
             QColor(150, 155, 168, 255));
  }

  const auto arrow_end = tfmath::apply(base, msgs::Vector3{0.45, 0.0, 0.03});
  const auto arrow_left = tfmath::apply(base, msgs::Vector3{0.32, 0.09, 0.03});
  const auto arrow_right = tfmath::apply(base, msgs::Vector3{0.32, -0.09, 0.03});
  draw_line(painter, QVector3D(static_cast<float>(base.translation.x), static_cast<float>(base.translation.y), 0.03F),
            QVector3D(static_cast<float>(arrow_end.x), static_cast<float>(arrow_end.y), 0.03F),
            QColor(80, 235, 255, 240), 2.5);
  draw_line(painter, QVector3D(static_cast<float>(arrow_end.x), static_cast<float>(arrow_end.y), 0.03F),
            QVector3D(static_cast<float>(arrow_left.x), static_cast<float>(arrow_left.y), 0.03F),
            QColor(80, 235, 255, 240), 2.0);
  draw_line(painter, QVector3D(static_cast<float>(arrow_end.x), static_cast<float>(arrow_end.y), 0.03F),
            QVector3D(static_cast<float>(arrow_right.x), static_cast<float>(arrow_right.y), 0.03F),
            QColor(80, 235, 255, 240), 2.0);
}

void SimView::draw_scan(QPainter& painter, const SimSnapshot& snapshot) const {
  if (snapshot.scan_ranges.empty()) {
    return;
  }
  QPolygonF points;
  const double increment = 2.0 * kPi / static_cast<double>(snapshot.scan_ranges.size());
  for (std::size_t i = 0; i < snapshot.scan_ranges.size(); ++i) {
    const float range = snapshot.scan_ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    const double angle = snapshot.scan_pose.theta + static_cast<double>(i) * increment;
    const double x = snapshot.scan_pose.x + static_cast<double>(range) * std::cos(angle);
    const double y = snapshot.scan_pose.y + static_cast<double>(range) * std::sin(angle);
    const Projected projected = project(QVector3D(static_cast<float>(x), static_cast<float>(y), 0.132F));
    if (projected.visible) {
      points << projected.point;
    }
  }
  painter.setPen(QPen(QColor(255, 80, 80, 240), 2.0));
  painter.drawPoints(points);
}

void SimView::draw_hud(QPainter& painter, const SimSnapshot& snapshot) const {
  double nearest = std::numeric_limits<double>::infinity();
  for (float range : snapshot.scan_ranges) {
    if (std::isfinite(range)) {
      nearest = std::min(nearest, static_cast<double>(range));
    }
  }
  QStringList lines;
  lines << "SIM (TRUTH)";
  lines << QString("t=%1 s   pose: x=%2 y=%3 yaw=%4°")
               .arg(snapshot.time, 0, 'f', 1)
               .arg(snapshot.robot_pose.x, 0, 'f', 2)
               .arg(snapshot.robot_pose.y, 0, 'f', 2)
               .arg(snapshot.robot_pose.theta * 180.0 / kPi, 0, 'f', 0);
  lines << QString("cmd: v=%1 w=%2   wheels: %3° / %4°")
               .arg(snapshot.command.linear.x, 0, 'f', 2)
               .arg(snapshot.command.angular.z, 0, 'f', 2)
               .arg(snapshot.left_wheel * 180.0 / kPi, 0, 'f', 0)
               .arg(snapshot.right_wheel * 180.0 / kPi, 0, 'f', 0);
  lines << QString("pucks: %1   nearest: %2 m   fps: %3")
               .arg(snapshot.pucks.size())
               .arg(std::isfinite(nearest) ? QString::number(nearest, 'f', 2) : QString("-"))
               .arg(fps_, 0, 'f', 1);
  lines << QString("follow: %1   teleop x%2").arg(follow_ ? "on" : "off").arg(teleop_scale_, 0, 'f', 2);

  QFont font("Menlo");
  font.setStyleHint(QFont::Monospace);
  font.setPixelSize(11);
  painter.setFont(font);
  const int line_height = 15;
  painter.fillRect(QRect(10, 10, 370, static_cast<int>(lines.size()) * line_height + 14), QColor(8, 10, 14, 180));
  painter.setPen(QColor(100, 235, 255));
  painter.drawText(18, 26, lines[0]);
  painter.setPen(QColor(225, 230, 240));
  for (int i = 1; i < lines.size(); ++i) {
    painter.drawText(18, 26 + i * line_height, lines[i]);
  }
}

void SimView::paintEvent(QPaintEvent* event) {
  (void)event;
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(8, 11, 16));
  update_camera();
  compute_matrix();

  draw_grid(painter);
  draw_walls(painter);
  const auto snapshot = store_.latest();
  if (snapshot) {
    draw_pucks(painter, *snapshot);
    draw_scan(painter, *snapshot);
    draw_robot(painter, *snapshot);
    draw_hud(painter, *snapshot);
  } else {
    QFont font("Menlo");
    font.setPixelSize(12);
    painter.setFont(font);
    painter.setPen(QColor(130, 140, 155));
    painter.drawText(18, 30, "waiting for simulation...");
  }

  ++frame_count_;
  const qint64 elapsed = fps_clock_.elapsed();
  if (elapsed >= 1000) {
    fps_ = frame_count_ * 1000.0 / static_cast<double>(elapsed);
    frame_count_ = 0;
    fps_clock_.restart();
  }
}

void SimView::mousePressEvent(QMouseEvent* event) {
  setFocus(Qt::MouseFocusReason);
  last_mouse_ = event->position().toPoint();
  if (event->button() == Qt::LeftButton) {
    orbiting_ = true;
  } else if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    panning_ = true;
  }
}

void SimView::mouseMoveEvent(QMouseEvent* event) {
  const QPoint current = event->position().toPoint();
  const QPoint delta = current - last_mouse_;
  last_mouse_ = current;
  if (orbiting_) {
    camera_.yaw -= delta.x() * 0.008;
    camera_.pitch = std::clamp(camera_.pitch + delta.y() * 0.008, 0.08, 1.5);
    update();
  } else if (panning_) {
    const double scale = camera_.distance * 0.0016;
    const QVector3D right(static_cast<float>(-std::sin(camera_.yaw)), static_cast<float>(std::cos(camera_.yaw)), 0.0F);
    const QVector3D forward(static_cast<float>(-std::cos(camera_.yaw)), static_cast<float>(-std::sin(camera_.yaw)), 0.0F);
    camera_.target -= right * static_cast<float>(delta.x() * scale) + forward * static_cast<float>(delta.y() * scale);
    if (follow_) {
      follow_target_ = camera_.target;
    }
    update();
  }
}

void SimView::mouseReleaseEvent(QMouseEvent* event) {
  (void)event;
  orbiting_ = false;
  panning_ = false;
}

void SimView::wheelEvent(QWheelEvent* event) {
  const double factor = std::exp(-event->angleDelta().y() / 600.0);
  camera_.distance = std::clamp(camera_.distance * factor, 0.4, 40.0);
  update();
}

void SimView::keyPressEvent(QKeyEvent* event) {
  if (event->isAutoRepeat()) {
    return;
  }
  switch (event->key()) {
    case Qt::Key_W:
    case Qt::Key_A:
    case Qt::Key_S:
    case Qt::Key_D:
    case Qt::Key_Up:
    case Qt::Key_Down:
    case Qt::Key_Left:
    case Qt::Key_Right:
      keys_.insert(event->key());
      event->accept();
      return;
    case Qt::Key_Q:
      teleop_scale_ = std::min(4.0, teleop_scale_ * 1.25);
      return;
    case Qt::Key_Z:
      teleop_scale_ = std::max(0.25, teleop_scale_ / 1.25);
      return;
    case Qt::Key_F:
      set_follow(!follow_);
      return;
    case Qt::Key_C:
    case Qt::Key_R:
      reset_camera();
      return;
    case Qt::Key_Escape:
      window()->close();
      return;
    default:
      QWidget::keyPressEvent(event);
      return;
  }
}

void SimView::keyReleaseEvent(QKeyEvent* event) {
  if (!event->isAutoRepeat()) {
    keys_.remove(event->key());
  }
}

void SimView::focusOutEvent(QFocusEvent* event) {
  (void)event;
  keys_.clear();
  orbiting_ = false;
  panning_ = false;
}

void SimView::publish_teleop() {
  const auto held = [this](std::initializer_list<int> keys) {
    return std::any_of(keys.begin(), keys.end(), [this](int key) { return keys_.contains(key); });
  };
  const double forward = (held({Qt::Key_W, Qt::Key_Up}) ? 1.0 : 0.0) - (held({Qt::Key_S, Qt::Key_Down}) ? 1.0 : 0.0);
  const double left = (held({Qt::Key_A, Qt::Key_Left}) ? 1.0 : 0.0) - (held({Qt::Key_D, Qt::Key_Right}) ? 1.0 : 0.0);
  const double linear = 0.26 * teleop_scale_ * forward;
  const double angular = 1.5 * teleop_scale_ * left;
  const bool active = linear != 0.0 || angular != 0.0;
  if (active || last_linear_ != 0.0 || last_angular_ != 0.0) {
    robot_.set_command(msgs::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}});
  }
  last_linear_ = linear;
  last_angular_ = angular;
}

SimWindow::SimWindow(std::vector<Wall> walls, TurtleBot3& robot, SnapshotStore& store, QWidget* parent)
    : QMainWindow(parent) {
  setWindowTitle("krosc sim (truth)");
  resize(760, 860);
  view_ = new SimView(std::move(walls), robot, store, this);
  setCentralWidget(view_);
  statusBar()->showMessage(view_->help_text());
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, view_, QOverload<>::of(&QWidget::update));
  timer_->start(33);
}

SimWindow::~SimWindow() {
  if (timer_ != nullptr) {
    timer_->stop();
  }
}

bool SimWindow::save_screenshot(const QString& path) {
  view_->update();
  return grab().save(path);
}

}  // namespace krosc::sim
