#include "view/recycling_view.hpp"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <thread>

#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/qos.hpp"

namespace srf_recycling_ws::sim {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

RecyclingView::RecyclingView(SharedScene& scene, srf::simulator::SnapshotStore& snapshots, srf::Node& node,
                             std::string teleop_topic, QWidget* parent)
    : QWidget(parent), scene_(scene), snapshots_(snapshots), node_(node), teleop_topic_(std::move(teleop_topic)) {
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(480, 320);
  auto* repaint = new QTimer(this);
  connect(repaint, &QTimer::timeout, this, [this] { update(); });
  repaint->start(40);  // 25 fps 목표
  teleop_timer_ = new QTimer(this);
  connect(teleop_timer_, &QTimer::timeout, this, [this] { publish_teleop(); });
  teleop_timer_->start(50);
  fps_clock_.start();
}

QString RecyclingView::help_text() {
  return QStringLiteral(
      "진실 화면 (카메라와 같은 렌더러) · W/A/S/D 주행 · Q/Z 속도 · F 따라가기 · L 라이다 · C 전경 · "
      "드래그 회전 · 오른쪽 드래그 이동 · 휠 줌");
}

void RecyclingView::reset_camera() {
  yaw_ = -2.2;
  pitch_ = 0.55;
  distance_ = 2.2;
  follow_ = true;
  update();
}

render::Camera RecyclingView::make_camera(int width, int height) const {
  render::Camera camera;
  camera.width = width;
  camera.height = height;
  camera.set_fov(60.0 * kPi / 180.0);
  camera.near = 0.02;
  const render::Vec3 eye{target_.x - distance_ * std::cos(pitch_) * std::cos(yaw_),
                         target_.y - distance_ * std::cos(pitch_) * std::sin(yaw_),
                         target_.z + distance_ * std::sin(pitch_)};
  camera.world_from_optical = render::Pose::scale_free_look(eye, target_);
  return camera;
}

void RecyclingView::paintEvent(QPaintEvent* event) {
  (void)event;
  const auto snapshot = snapshots_.latest();
  if (follow_ && snapshot) {
    target_.x = snapshot->robot_pose.x;
    target_.y = snapshot->robot_pose.y;
  }
  {
    std::lock_guard<std::mutex> lock(scene_.mutex);
    instances_ = scene_.instances;
  }
  // 창보다 작게 그려 늘린다 (CPU 렌더러: 큰 창에서도 프레임을 지킨다)
  const double scale = width() > 1000 ? 0.6 : 0.8;
  const int w = std::max(64, static_cast<int>(width() * scale));
  const int h = std::max(48, static_cast<int>(height() * scale));
  const render::Camera camera = make_camera(w, h);
  render::RenderOptions options;
  options.supersample = 1;
  options.threads = static_cast<int>(std::max(2U, std::thread::hardware_concurrency() / 2));
  options.vignette = 0.0F;
  QElapsedTimer timer;
  timer.start();
  render::render(camera, instances_, lighting_, frame_, options);
  render_ms_ = render_ms_ * 0.9 + timer.nsecsElapsed() / 1e6 * 0.1;

  QPainter painter(this);
  const QImage image(frame_.rgb.data(), frame_.width, frame_.height, frame_.width * 3, QImage::Format_RGB888);
  painter.setRenderHint(QPainter::SmoothPixmapTransform);
  painter.drawImage(rect(), image);

  // 라이다 (진실, 잡음 없음): 월드 점을 같은 카메라로 투영
  if (show_scan_ && snapshot && !snapshot->scan_ranges.empty()) {
    const render::Pose optical_from_world = camera.world_from_optical.inverse();
    const double sx = static_cast<double>(width()) / w, sy = static_cast<double>(height()) / h;
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(255, 60, 60, 200));
    const auto& pose = snapshot->scan_pose;
    const std::size_t beams = snapshot->scan_ranges.size();
    for (std::size_t i = 0; i < beams; ++i) {
      const float r = snapshot->scan_ranges[i];
      if (!std::isfinite(r) || r <= 0.0F) {
        continue;
      }
      const double a = pose.theta + 2 * kPi * static_cast<double>(i) / static_cast<double>(beams);
      const render::Vec3 p = optical_from_world.apply({pose.x + r * std::cos(a), pose.y + r * std::sin(a), 0.122});
      if (p.z < 0.05) {
        continue;
      }
      painter.drawEllipse(QPointF((camera.fx * p.x / p.z + camera.cx) * sx, (camera.fy * p.y / p.z + camera.cy) * sy),
                          2.0, 2.0);
    }
  }

  // HUD
  ++frames_;
  if (fps_clock_.elapsed() > 1000) {
    fps_ = frames_ * 1000.0 / static_cast<double>(fps_clock_.restart());
    frames_ = 0;
  }
  if (snapshot) {
    const QString hud = QStringLiteral(
                            "SIM (TRUTH)\nt=%1 s   pose: x=%2 y=%3 yaw=%4°\ncmd: v=%5 w=%6\nobjects: %7   "
                            "fps: %8 (%9 ms)\nfollow: %10   lidar: %11   teleop x%12")
                            .arg(snapshot->time, 0, 'f', 1)
                            .arg(snapshot->robot_pose.x, 0, 'f', 2)
                            .arg(snapshot->robot_pose.y, 0, 'f', 2)
                            .arg(snapshot->robot_pose.theta * 180.0 / kPi, 0, 'f', 0)
                            .arg(snapshot->command.linear.x, 0, 'f', 2)
                            .arg(snapshot->command.angular.z, 0, 'f', 2)
                            .arg(snapshot->objects.size())
                            .arg(fps_, 0, 'f', 1)
                            .arg(render_ms_, 0, 'f', 0)
                            .arg(follow_ ? "on" : "off")
                            .arg(show_scan_ ? "on" : "off")
                            .arg(teleop_scale_, 0, 'f', 2);
    QFont font("Menlo");
    font.setStyleHint(QFont::Monospace);
    font.setPointSize(11);
    painter.setFont(font);
    const QRect box = painter.boundingRect(QRect(12, 12, 600, 200), Qt::AlignLeft | Qt::AlignTop, hud);
    painter.fillRect(box.adjusted(-8, -6, 8, 6), QColor(10, 14, 20, 190));
    painter.setPen(QColor(230, 236, 245));
    painter.drawText(box, Qt::AlignLeft | Qt::AlignTop, hud);
  }
}

void RecyclingView::mousePressEvent(QMouseEvent* event) {
  last_mouse_ = event->position().toPoint();
  orbiting_ = event->button() == Qt::LeftButton;
  panning_ = event->button() == Qt::RightButton || event->button() == Qt::MiddleButton;
  setFocus();
}

void RecyclingView::mouseMoveEvent(QMouseEvent* event) {
  const QPoint now = event->position().toPoint();
  const QPoint delta = now - last_mouse_;
  last_mouse_ = now;
  if (orbiting_) {
    yaw_ -= delta.x() * 0.008;
    pitch_ = std::clamp(pitch_ + delta.y() * 0.006, 0.05, 1.5);
  } else if (panning_) {
    follow_ = false;
    const double k = distance_ * 0.0015;
    target_.x += (-std::sin(yaw_) * delta.x() - std::cos(yaw_) * delta.y()) * k;
    target_.y += (std::cos(yaw_) * delta.x() - std::sin(yaw_) * delta.y()) * k;
  }
  update();
}

void RecyclingView::mouseReleaseEvent(QMouseEvent* event) {
  (void)event;
  orbiting_ = panning_ = false;
}

void RecyclingView::wheelEvent(QWheelEvent* event) {
  distance_ = std::clamp(distance_ * std::pow(0.999, event->angleDelta().y()), 0.3, 40.0);
  update();
}

void RecyclingView::keyPressEvent(QKeyEvent* event) {
  if (event->isAutoRepeat()) {
    return;
  }
  switch (event->key()) {
    case Qt::Key_F: follow_ = !follow_; break;
    case Qt::Key_L: show_scan_ = !show_scan_; break;
    case Qt::Key_C:
      follow_ = false;
      target_ = {0, 0, 0};
      if (const auto snapshot = snapshots_.latest()) {
        target_ = {snapshot->robot_pose.x, snapshot->robot_pose.y, 0.0};
      }
      distance_ = 12.0;
      pitch_ = 0.9;
      break;
    case Qt::Key_Q: teleop_scale_ = std::min(4.0, teleop_scale_ * 1.25); break;
    case Qt::Key_Z: teleop_scale_ = std::max(0.1, teleop_scale_ / 1.25); break;
    default: keys_.insert(event->key());
  }
  update();
}

void RecyclingView::keyReleaseEvent(QKeyEvent* event) {
  if (!event->isAutoRepeat()) {
    keys_.remove(event->key());
  }
}

void RecyclingView::focusOutEvent(QFocusEvent* event) {
  (void)event;
  keys_.clear();  // 창을 떠나면 주행을 멈춘다
}

void RecyclingView::publish_teleop() {
  const auto held = [this](std::initializer_list<int> keys) {
    return std::any_of(keys.begin(), keys.end(), [this](int key) { return keys_.contains(key); });
  };
  const double forward = (held({Qt::Key_W, Qt::Key_Up}) ? 1.0 : 0.0) - (held({Qt::Key_S, Qt::Key_Down}) ? 1.0 : 0.0);
  const double left = (held({Qt::Key_A, Qt::Key_Left}) ? 1.0 : 0.0) - (held({Qt::Key_D, Qt::Key_Right}) ? 1.0 : 0.0);
  const double linear = 0.26 * teleop_scale_ * forward;
  const double angular = 1.5 * teleop_scale_ * left;
  if (linear != 0.0 || angular != 0.0 || last_linear_ != 0.0 || last_angular_ != 0.0) {
    node_.publish(teleop_topic_, srf::msgs::Twist{{linear, 0.0, 0.0}, {0.0, 0.0, angular}}, srf::QOS_COMMAND);
  }
  last_linear_ = linear;
  last_angular_ = angular;
}

RecyclingWindow::RecyclingWindow(SharedScene& scene, srf::simulator::SnapshotStore& snapshots, srf::Node& node,
                                 std::string teleop_topic, std::vector<QWidget*> side_panels) {
  setWindowTitle("srf recycling sim (truth)");
  resize(1500, 900);
  auto* splitter = new QSplitter(this);
  view_ = new RecyclingView(scene, snapshots, node, std::move(teleop_topic), splitter);
  splitter->addWidget(view_);
  auto* container = new QWidget;
  auto* layout = new QVBoxLayout(container);
  for (auto* panel : side_panels) {
    layout->addWidget(panel);
  }
  layout->addStretch(1);
  auto* scroll = new QScrollArea;
  scroll->setWidget(container);
  scroll->setWidgetResizable(true);
  scroll->setMinimumWidth(380);
  splitter->addWidget(scroll);
  splitter->setStretchFactor(0, 1);
  setCentralWidget(splitter);
  statusBar()->showMessage(RecyclingView::help_text());
}

}  // namespace srf_recycling_ws::sim
