#include "krosc/viz/scene_view.hpp"

#include "krosc/viz/costmap_color.hpp"
#include "krosc/viz/key_bindings.hpp"

#include <QFocusEvent>
#include <QFont>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QStringList>
#include <QTransform>
#include <QVector4D>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <utility>

#include "krosc/math.hpp"

namespace krosc::viz {

namespace {

constexpr double kPi = 3.14159265358979323846;

QColor with_alpha(QColor color, int alpha) {
  color.setAlpha(alpha);
  return color;
}

QImage build_grid_image(const msgs::OccupancyGrid& grid, bool costmap) {
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  if (width <= 0 || height <= 0 || grid.data.size() < static_cast<std::size_t>(width * height)) {
    return {};
  }
  QImage image(width, height, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int value = grid.data[static_cast<std::size_t>(y * width + x)];
      QColor color;
      if (costmap) {
        color = costmap_color(value);
        if (color.alpha() == 0) {
          continue;
        }
      } else {
        if (value < 0) {
          color = QColor(85, 88, 105, 18);  // 모름: 아주 흐리게
        } else if (value >= 50) {
          color = QColor(245, 246, 250, 255);  // 벽: 밝게
        } else {
          color = QColor(62, 88, 125, 110 + value);  // 빈 곳: 배경과 구분되게
        }
      }
      image.setPixelColor(x, height - 1 - y, color);
    }
  }
  return image;
}

msgs::Transform origin_transform(const UrdfVisual& visual) {
  msgs::Transform transform;
  transform.translation = {visual.xyz[0], visual.xyz[1], visual.xyz[2]};
  transform.rotation = tfmath::from_rpy(visual.rpy[0], visual.rpy[1], visual.rpy[2]);
  return transform;
}

msgs::Transform transform_of(const msgs::Pose& pose) {
  return {{pose.position.x, pose.position.y, pose.position.z}, pose.orientation};
}

QColor link_color(const std::string& name) {
  if (name.find("wheel") != std::string::npos) {
    return QColor(55, 58, 66);
  }
  if (name.find("caster") != std::string::npos) {
    return QColor(85, 88, 96);
  }
  if (name.find("scan") != std::string::npos || name.find("lidar") != std::string::npos) {
    return QColor(95, 100, 112);
  }
  if (name.find("camera") != std::string::npos) {
    return QColor(70, 74, 84);
  }
  if (name == "base_footprint" || name == "base_link") {
    return QColor(180, 187, 200);
  }
  return QColor(145, 150, 165);
}

}  // namespace

SceneView::SceneView(VizModel& model, QWidget* parent) : QWidget(parent), model_(model) {
  setFocusPolicy(Qt::StrongFocus);
  // 한글 IME가 키를 조합해 버리지 않게 한다 (W/A/S/D가 ㅈ/ㅁ/ㄴ/ㅇ으로 바뀌어도 key_bindings가 복원).
  setAttribute(Qt::WA_InputMethodEnabled, false);
  setMouseTracking(true);
  setMinimumSize(640, 420);
  for (const auto& key : {"grid", "tf", "model", "scan", "imu", "odom", "filtered", "map", "global_costmap",
                          "local_costmap", "plan", "local_plan", "trail", "logs"}) {
    enabled_[key] = true;
  }
  fps_clock_.start();
  reset_camera();
  teleop_timer_ = new QTimer(this);
  connect(teleop_timer_, &QTimer::timeout, this, &SceneView::publish_teleop);
  teleop_timer_->start(50);
}

void SceneView::set_enabled(const std::string& key, bool value) {
  enabled_[key] = value;
  if (key == "trail" && !value) {
    trail_.clear();
  }
  update();
}

bool SceneView::enabled(const std::string& key) const {
  auto it = enabled_.find(key);
  return it != enabled_.end() && it->second;
}

void SceneView::set_follow(bool follow) {
  follow_ = follow;
  if (follow_) {
    follow_target_ = camera_.target;
  }
  emit follow_changed(follow_);
  update();
}

void SceneView::set_camera_distance(double distance) {
  camera_.distance = std::clamp(distance, 0.4, 40.0);
  update();
}

void SceneView::set_teleop_scale(double scale) {
  teleop_scale_ = std::clamp(scale, 0.25, 1.5);
  emit teleop_scale_changed(teleop_scale_);
  update();
}

void SceneView::set_teleop_enabled(bool enabled) {
  teleop_enabled_ = enabled;
  if (!enabled) {
    keys_.clear();
    key_codes_.clear();
    if (last_linear_ != 0.0 || last_angular_ != 0.0) {
      model_.publish_cmd_vel(0.0, 0.0);
      last_linear_ = 0.0;
      last_angular_ = 0.0;
    }
  }
  update();
}

void SceneView::set_waypoints(const std::vector<std::tuple<double, double, double>>& waypoints) {
  waypoints_ = waypoints;
  update();
}

void SceneView::set_tool(const std::string& tool) {
  tool_ = (tool == "initialpose" || tool == "goal" || tool == "multi") ? tool : std::string{};
  tool_dragging_ = false;
  tool_start_ground_.reset();
  tool_current_ground_.reset();
  emit tool_changed(QString::fromStdString(tool_));
  update();
}

void SceneView::reset_camera() {
  camera_ = Camera{};
  follow_ = true;
  follow_target_ = camera_.target;
  trail_.clear();
  emit follow_changed(true);
  update();
}

void SceneView::fixed_frame_changed() {
  trail_.clear();
  map_cached_.reset();
  global_costmap_cached_.reset();
  local_costmap_cached_.reset();
  update();
}

QString SceneView::help_text() const {
  return QStringLiteral(
      "W/A/S/D: teleop   Q/Z: speed   F: follow   C: overview   Backspace: 마지막 waypoint   "
      "drag: orbit   right drag: pan   wheel: zoom");
}

void SceneView::update_camera() {
  if (!follow_) {
    return;
  }
  if (const auto position = robot_position()) {
    follow_target_ += (*position - follow_target_) * 0.12F;
    camera_.target = follow_target_;
  }
}

void SceneView::compute_matrix() {
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

std::optional<QVector3D> SceneView::screen_to_plane(const QPoint& point, float plane_z) const {
  if (view_width_ <= 0 || view_height_ <= 0) {
    return std::nullopt;
  }
  const QMatrix4x4 inverse = matrix_.inverted();
  const auto unproject = [&](float depth) -> std::optional<QVector3D> {
    const QVector4D clip(2.0F * static_cast<float>(point.x()) / static_cast<float>(view_width_) - 1.0F,
                         1.0F - 2.0F * static_cast<float>(point.y()) / static_cast<float>(view_height_), depth, 1.0F);
    const QVector4D world = inverse * clip;
    if (std::abs(world.w()) < 1e-9F) {
      return std::nullopt;
    }
    return QVector3D(world.x() / world.w(), world.y() / world.w(), world.z() / world.w());
  };
  const auto near = unproject(-1.0F);
  const auto far = unproject(1.0F);
  if (!near || !far) {
    return std::nullopt;
  }
  const QVector3D direction = *far - *near;
  if (std::abs(direction.z()) < 1e-6F) {
    return std::nullopt;
  }
  const float t = (plane_z - near->z()) / direction.z();
  if (t < 0.0F) {
    return std::nullopt;
  }
  return *near + direction * t;
}

std::optional<QVector3D> SceneView::screen_to_ground(const QPoint& point) const {
  return screen_to_plane(point, 0.0F);
}

SceneView::Projected SceneView::project(const QVector3D& point) const {
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

std::optional<msgs::PoseStamped> SceneView::robot_pose() const {
  for (const auto& frame : {"base_footprint", "base_link"}) {
    if (auto pose = model_.pose_in_fixed(frame)) {
      return pose;
    }
  }
  const auto filtered = model_.filtered();
  if (filtered) {
    const auto frame_transform = model_.transform_in_fixed(filtered->header.frame_id);
    if (frame_transform) {
      const msgs::Transform pose_transform = transform_of(filtered->pose.pose);
      const msgs::Transform composed = tfmath::compose(frame_transform->transform, pose_transform);
      msgs::PoseStamped pose;
      pose.header.frame_id = model_.fixed_frame();
      pose.pose.position = {composed.translation.x, composed.translation.y, composed.translation.z};
      pose.pose.orientation = composed.rotation;
      return pose;
    }
  }
  return std::nullopt;
}

std::optional<QVector3D> SceneView::robot_position() const {
  if (const auto pose = robot_pose()) {
    return QVector3D(static_cast<float>(pose->pose.position.x), static_cast<float>(pose->pose.position.y),
                     static_cast<float>(pose->pose.position.z));
  }
  return std::nullopt;
}

msgs::Transform SceneView::link_transform(const std::string& frame) const {
  if (const auto transform = model_.transform_in_fixed(frame)) {
    return transform->transform;
  }
  return tfmath::kIdentity;
}

void SceneView::draw_line(QPainter& painter, const QVector3D& a, const QVector3D& b, const QColor& color,
                          double width) const {
  const Projected pa = project(a);
  const Projected pb = project(b);
  if (!pa.visible || !pb.visible) {
    return;
  }
  painter.setPen(QPen(color, width));
  painter.drawLine(pa.point, pb.point);
}

void SceneView::draw_face(QPainter& painter, const std::vector<QVector3D>& points, const QColor& fill,
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

void SceneView::draw_box(QPainter& painter, const msgs::Transform& transform, const std::array<double, 3>& size,
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
  // corner index: (sx, sy, sz) = (0,0,0), (0,0,1), (0,1,0), (0,1,1), (1,0,0), (1,0,1), (1,1,0), (1,1,1)
  face(0, 2, 3, 1);  // x-
  face(4, 5, 7, 6);  // x+
  face(0, 1, 5, 4);  // y-
  face(2, 6, 7, 3);  // y+
  face(0, 4, 6, 2);  // z-
  face(1, 3, 7, 5);  // z+
}

void SceneView::draw_cylinder(QPainter& painter, const msgs::Transform& transform, double radius, double length,
                              const QColor& fill, const QColor& outline) const {
  constexpr int kSegments = 16;
  const double half = length / 2.0;
  std::array<QVector3D, kSegments> bottom;
  std::array<QVector3D, kSegments> top;
  for (int i = 0; i < kSegments; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) / kSegments;
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    const msgs::Vector3 low = tfmath::apply(transform, msgs::Vector3{x, y, -half});
    const msgs::Vector3 high = tfmath::apply(transform, msgs::Vector3{x, y, half});
    bottom[static_cast<std::size_t>(i)] = QVector3D(static_cast<float>(low.x), static_cast<float>(low.y), static_cast<float>(low.z));
    top[static_cast<std::size_t>(i)] = QVector3D(static_cast<float>(high.x), static_cast<float>(high.y), static_cast<float>(high.z));
  }
  for (int i = 0; i < kSegments; ++i) {
    const int next = (i + 1) % kSegments;
    draw_face(painter, {bottom[static_cast<std::size_t>(i)], bottom[static_cast<std::size_t>(next)],
                        top[static_cast<std::size_t>(next)], top[static_cast<std::size_t>(i)]},
              fill, outline);
  }
  draw_face(painter, std::vector<QVector3D>(top.begin(), top.end()), fill, outline);
  draw_face(painter, std::vector<QVector3D>(bottom.begin(), bottom.end()), fill, outline);
}

void SceneView::draw_yaw_arrow(QPainter& painter, const QVector3D& origin, const msgs::Quaternion& orientation,
                               double length, const QColor& color) const {
  const auto yaw = tfmath::yaw_of(orientation);
  const QVector3D tip = origin + QVector3D(static_cast<float>(std::cos(yaw) * length),
                                           static_cast<float>(std::sin(yaw) * length), 0.0F);
  draw_line(painter, origin, tip, color, 2.0);
  const double head = 0.25 * length;
  for (double offset : {2.6, -2.6}) {
    const double angle = yaw + offset;
    draw_line(painter, tip,
              tip + QVector3D(static_cast<float>(std::cos(angle) * head), static_cast<float>(std::sin(angle) * head),
                              0.0F),
              color, 2.0);
  }
}

void SceneView::draw_label(QPainter& painter, const QVector3D& position, const QString& text,
                           const QColor& color) const {
  const Projected projected = project(position);
  if (!projected.visible) {
    return;
  }
  QFont font("Helvetica");
  font.setPixelSize(10);
  painter.setFont(font);
  painter.setPen(color);
  painter.drawText(projected.point + QPointF(3.0, -3.0), text);
}

void SceneView::draw_grid(QPainter& painter) const {
  constexpr double kStep = 0.5;
  constexpr int kHalf = 20;
  for (int i = -kHalf; i <= kHalf; ++i) {
    const double coordinate = i * kStep;
    const QColor color = i % 2 == 0 ? QColor(70, 78, 92, 70) : QColor(55, 62, 74, 45);
    draw_line(painter, QVector3D(static_cast<float>(coordinate), -static_cast<float>(kHalf) * static_cast<float>(kStep), 0.0F),
              QVector3D(static_cast<float>(coordinate), static_cast<float>(kHalf) * static_cast<float>(kStep), 0.0F), color);
    draw_line(painter, QVector3D(-static_cast<float>(kHalf) * static_cast<float>(kStep), static_cast<float>(coordinate), 0.0F),
              QVector3D(static_cast<float>(kHalf) * static_cast<float>(kStep), static_cast<float>(coordinate), 0.0F), color);
  }
  draw_line(painter, QVector3D(-10.0F, 0.0F, 0.0F), QVector3D(10.0F, 0.0F, 0.0F), QColor(230, 90, 90, 150), 1.5);
  draw_line(painter, QVector3D(0.0F, -10.0F, 0.0F), QVector3D(0.0F, 10.0F, 0.0F), QColor(90, 220, 120, 150), 1.5);
}

void SceneView::draw_axes(QPainter& painter) const {
  // 화면 기준 프레임의 원점 축
  draw_line(painter, QVector3D(0, 0, 0), QVector3D(0.6F, 0, 0), QColor(255, 80, 80), 2.0);
  draw_line(painter, QVector3D(0, 0, 0), QVector3D(0, 0.6F, 0), QColor(80, 255, 120), 2.0);
  draw_line(painter, QVector3D(0, 0, 0), QVector3D(0, 0, 0.6F), QColor(90, 140, 255), 2.0);
  if (!enabled("tf")) {
    return;
  }
  for (const auto& frame : model_.tf_buffer().all_frames()) {
    const auto transform = model_.transform_in_fixed(frame);
    if (!transform) {
      continue;
    }
    const msgs::Transform& t = transform->transform;
    const QVector3D origin(static_cast<float>(t.translation.x), static_cast<float>(t.translation.y),
                           static_cast<float>(t.translation.z));
    const msgs::Vector3 x = tfmath::rotate(t.rotation, {0.12, 0.0, 0.0});
    const msgs::Vector3 y = tfmath::rotate(t.rotation, {0.0, 0.12, 0.0});
    const msgs::Vector3 z = tfmath::rotate(t.rotation, {0.0, 0.0, 0.12});
    draw_line(painter, origin, origin + QVector3D(static_cast<float>(x.x), static_cast<float>(x.y), static_cast<float>(x.z)),
              QColor(255, 90, 90, 200), 1.2);
    draw_line(painter, origin, origin + QVector3D(static_cast<float>(y.x), static_cast<float>(y.y), static_cast<float>(y.z)),
              QColor(90, 235, 120, 200), 1.2);
    draw_line(painter, origin, origin + QVector3D(static_cast<float>(z.x), static_cast<float>(z.y), static_cast<float>(z.z)),
              QColor(110, 150, 255, 200), 1.2);
    const bool important = frame == "map" || frame == "odom";
    if (important || camera_.distance <= 5.0) {
      draw_label(painter, origin, QString::fromStdString(frame), QColor(190, 200, 215, 190));
    }
  }
}

void SceneView::draw_grid_image(QPainter& painter, const msgs::OccupancyGrid& grid, const QImage& image) const {
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  if (image.isNull() || width <= 0 || height <= 0) {
    return;
  }
  msgs::Transform frame_transform = tfmath::kIdentity;
  if (grid.header.frame_id != model_.fixed_frame()) {
    const auto transform = model_.transform_in_fixed(grid.header.frame_id);
    if (!transform) {
      return;
    }
    frame_transform = transform->transform;
  }
  const double resolution = grid.info.resolution;
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  const double width_m = width * resolution;
  const double height_m = height * resolution;
  const msgs::Quaternion origin_rotation = grid.info.origin.orientation;
  const auto to_fixed = [&](double x, double y) {
    const msgs::Vector3 in_map = tfmath::apply(msgs::Transform{{ox, oy, 0.0}, origin_rotation}, msgs::Vector3{x, y, 0.0});
    const msgs::Vector3 in_fixed = tfmath::apply(frame_transform, in_map);
    return QVector3D(static_cast<float>(in_fixed.x), static_cast<float>(in_fixed.y), 0.002F);
  };
  // 이미지 row 0이 y=height_m이 되도록 뒤집어 그린다.
  const QPointF tl = project(to_fixed(0.0, height_m)).point;
  const QPointF tr = project(to_fixed(width_m, height_m)).point;
  const QPointF br = project(to_fixed(width_m, 0.0)).point;
  const QPointF bl = project(to_fixed(0.0, 0.0)).point;
  const bool visible = project(to_fixed(0.0, height_m)).visible && project(to_fixed(width_m, height_m)).visible &&
                       project(to_fixed(width_m, 0.0)).visible && project(to_fixed(0.0, 0.0)).visible;
  if (!visible) {
    return;
  }
  QPolygonF source;
  source << QPointF(0.0, 0.0) << QPointF(width, 0.0) << QPointF(width, height) << QPointF(0.0, height);
  QTransform transform;
  const bool ok = QTransform::quadToQuad(source, QPolygonF() << tl << tr << br << bl, transform);
  if (!ok) {
    return;
  }
  painter.save();
  painter.setTransform(transform);
  painter.drawImage(QPointF(0.0, 0.0), image);
  painter.restore();
}

void SceneView::draw_map(QPainter& painter) const {
  const auto grid = model_.map();
  if (!grid || !enabled("map") || grid->info.width == 0 || grid->info.height == 0) {
    return;
  }
  msgs::Transform frame_transform = tfmath::kIdentity;
  if (grid->header.frame_id != model_.fixed_frame()) {
    const auto transform = model_.transform_in_fixed(grid->header.frame_id);
    if (!transform) {
      return;
    }
    frame_transform = transform->transform;
  }
  const double resolution = grid->info.resolution;
  const double ox = grid->info.origin.position.x;
  const double oy = grid->info.origin.position.y;
  const msgs::Quaternion origin_rotation = grid->info.origin.orientation;
  const auto to_fixed = [&](double x, double y) {
    const msgs::Vector3 in_map = tfmath::apply(msgs::Transform{{ox, oy, 0.0}, origin_rotation}, msgs::Vector3{x, y, 0.0});
    const msgs::Vector3 in_fixed = tfmath::apply(frame_transform, in_map);
    return project(QVector3D(static_cast<float>(in_fixed.x), static_cast<float>(in_fixed.y), 0.01F));
  };
  const double width_m = grid->info.width * resolution;
  const double height_m = grid->info.height * resolution;
  const auto corner = [&](double x, double y) { return to_fixed(x, y); };
  const auto tl = corner(0.0, height_m);
  const auto tr = corner(width_m, height_m);
  const auto br = corner(width_m, 0.0);
  const auto bl = corner(0.0, 0.0);
  if (tl.visible && tr.visible && br.visible && bl.visible) {
    painter.setPen(QPen(QColor(0, 150, 230, 150), 1.0));
    painter.setBrush(Qt::NoBrush);
    painter.drawPolygon(QPolygonF() << tl.point << tr.point << br.point << bl.point);
  }

  const std::uint32_t width = grid->info.width;
  const std::uint32_t height = grid->info.height;
  if (grid->data.size() != static_cast<std::size_t>(width) * height) {
    return;
  }
  // RViz map 스타일: 모르는 곳=어두운 회색, 빈 곳=밝은 회색, 벽=흰색.
  // 한 프레임에 8만 칸을 project하므로 색깔별로 모아 drawPoints로 한 번에 그린다.
  std::vector<QPointF> unknown;
  std::vector<QPointF> free_space;
  std::vector<QPointF> occupied;
  const std::size_t cells = static_cast<std::size_t>(width) * height;
  unknown.reserve(cells * 3 / 4);
  free_space.reserve(cells / 4);
  occupied.reserve(cells / 20);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const std::int8_t value = grid->data[static_cast<std::size_t>(y) * width + x];
      const auto projected = to_fixed((static_cast<double>(x) + 0.5) * resolution,
                                     (static_cast<double>(y) + 0.5) * resolution);
      if (!projected.visible) {
        continue;
      }
      if (value < 0) {
        unknown.push_back(projected.point);
      } else if (value >= 50) {
        occupied.push_back(projected.point);
      } else {
        free_space.push_back(projected.point);
      }
    }
  }
  if (!unknown.empty()) {
    painter.setPen(QPen(QColor(75, 78, 88, 170), 1.0));
    painter.drawPoints(unknown.data(), static_cast<int>(unknown.size()));
  }
  if (!free_space.empty()) {
    painter.setPen(QPen(QColor(185, 190, 200, 190), 1.0));
    painter.drawPoints(free_space.data(), static_cast<int>(free_space.size()));
  }
  if (!occupied.empty()) {
    painter.setPen(QPen(QColor(248, 249, 252, 245), 2.0));
    painter.drawPoints(occupied.data(), static_cast<int>(occupied.size()));
  }
}

void SceneView::draw_costmap(QPainter& painter, const std::shared_ptr<const msgs::OccupancyGrid>& grid, QImage& image,
                             std::shared_ptr<const msgs::OccupancyGrid>& cache, const char* label) const {
  (void)label;
  if (!grid) {
    return;
  }
  if (grid != cache) {
    image = build_grid_image(*grid, true);
    cache = grid;
  }
  draw_grid_image(painter, *grid, image);
}

std::optional<std::string> SceneView::resolve_mesh_path(const std::string& filename) const {
  constexpr const char* kPrefix = "package://open_manipulator_description/";
  if (filename.rfind(kPrefix, 0) != 0) {
    return std::nullopt;  // turtlebot3 base/wheel 메시는 기존 collision 도형으로 그린다
  }
  const std::string relative = filename.substr(std::char_traits<char>::length(kPrefix));
  for (const char* root : {"krosc_ws/robot/description/", "robot/description/",
                           "../krosc_ws/robot/description/", "../../krosc_ws/robot/description/"}) {
    const std::string candidate = std::string(root) + relative;
    if (std::filesystem::is_regular_file(candidate)) {
      return candidate;
    }
  }
  return std::nullopt;
}

const std::vector<std::array<QVector3D, 3>>* SceneView::load_mesh(const UrdfGeometry& geometry) {
  auto cached = mesh_cache_.find(geometry.filename);
  if (cached != mesh_cache_.end()) {
    return &cached->second;
  }
  auto& triangles = mesh_cache_[geometry.filename];
  const auto path = resolve_mesh_path(geometry.filename);
  if (!path) {
    return &triangles;
  }
  std::ifstream file(*path, std::ios::binary);
  if (!file) {
    return &triangles;
  }
  char header[80] = {};
  std::uint32_t count = 0;
  file.read(header, sizeof(header));
  file.read(reinterpret_cast<char*>(&count), sizeof(count));
  if (!file || count == 0 || count > 500000) {
    return &triangles;
  }
  constexpr float kGrid = 0.012F;  // 12mm 격자로 간소화 (QPainter 실시간 렌더)
  std::map<std::array<int, 3>, QVector3D> points;
  std::set<std::array<int, 9>> seen;
  for (std::uint32_t i = 0; i < count; ++i) {
    float record[12] = {};  // normal[3] + vertex[9]
    std::uint16_t attribute = 0;
    file.read(reinterpret_cast<char*>(record), sizeof(record));
    file.read(reinterpret_cast<char*>(&attribute), sizeof(attribute));
    if (!file) {
      break;
    }
    std::array<std::array<int, 3>, 3> keys{};
    for (int vertex = 0; vertex < 3; ++vertex) {
      const float x = record[3 + vertex * 3] * static_cast<float>(geometry.scale[0]);
      const float y = record[4 + vertex * 3] * static_cast<float>(geometry.scale[1]);
      const float z = record[5 + vertex * 3] * static_cast<float>(geometry.scale[2]);
      keys[static_cast<std::size_t>(vertex)] = {static_cast<int>(std::lround(x / kGrid)),
                                                static_cast<int>(std::lround(y / kGrid)),
                                                static_cast<int>(std::lround(z / kGrid))};
      if (points.find(keys[static_cast<std::size_t>(vertex)]) == points.end()) {
        points.emplace(keys[static_cast<std::size_t>(vertex)],
                       QVector3D(static_cast<float>(keys[static_cast<std::size_t>(vertex)][0]) * kGrid,
                                 static_cast<float>(keys[static_cast<std::size_t>(vertex)][1]) * kGrid,
                                 static_cast<float>(keys[static_cast<std::size_t>(vertex)][2]) * kGrid));
      }
    }
    if (keys[0] == keys[1] || keys[1] == keys[2] || keys[0] == keys[2]) {
      continue;
    }
    auto sorted = keys;
    std::sort(sorted.begin(), sorted.end());
    const std::array<int, 9> triangle_key{sorted[0][0], sorted[0][1], sorted[0][2],
                                          sorted[1][0], sorted[1][1], sorted[1][2],
                                          sorted[2][0], sorted[2][1], sorted[2][2]};
    if (!seen.insert(triangle_key).second) {
      continue;
    }
    triangles.push_back({points[keys[0]], points[keys[1]], points[keys[2]]});
  }
  return &triangles;
}

void SceneView::draw_robot(QPainter& painter) {
  const auto description = model_.robot_description();
  if (description) {
    if (!robot_ || robot_source_ != description->data) {
      robot_source_ = description->data;
      try {
        robot_ = std::make_unique<UrdfRobot>(parse_urdf(robot_source_));
        mesh_cache_.clear();
      } catch (const UrdfError&) {
        robot_.reset();
      }
    }
  }

  struct MeshFace {
    float depth = 0.0F;
    QPolygonF polygon;
    QColor color;
  };
  std::vector<MeshFace> mesh_faces;
  const QVector3D light = QVector3D(0.35F, -0.45F, 0.82F).normalized();

  bool drew = false;
  if (robot_) {
    for (const auto& link : robot_->links) {
      const auto link_tf = model_.transform_in_fixed(link.name);
      if (!link_tf) {
        continue;
      }
      bool drew_link = false;
      for (const auto& geometry : link.visuals) {
        if (geometry.geometry.type == UrdfGeometryType::None) {
          continue;
        }
        if (geometry.geometry.type == UrdfGeometryType::Mesh) {
          const auto* triangles = load_mesh(geometry.geometry);
          if (triangles == nullptr || triangles->empty()) {
            continue;
          }
          const msgs::Transform transform = tfmath::compose(link_tf->transform, origin_transform(geometry));
          QColor base = link_color(link.name);
          if (!geometry.material.empty()) {
            const auto material = robot_->materials.find(geometry.material);
            if (material != robot_->materials.end()) {
              base = QColor::fromRgbF(static_cast<float>(material->second[0]),
                                      static_cast<float>(material->second[1]),
                                      static_cast<float>(material->second[2]),
                                      static_cast<float>(material->second[3]));
            }
          }
          base = base.lighter(165);
          for (const auto& triangle : *triangles) {
            std::array<QVector3D, 3> world{};
            QPolygonF polygon;
            bool visible = true;
            for (int i = 0; i < 3; ++i) {
              const msgs::Vector3 point =
                  tfmath::apply(transform, msgs::Vector3{triangle[static_cast<std::size_t>(i)].x(),
                                                         triangle[static_cast<std::size_t>(i)].y(),
                                                         triangle[static_cast<std::size_t>(i)].z()});
              world[static_cast<std::size_t>(i)] =
                  QVector3D(static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z));
              const Projected projected = project(world[static_cast<std::size_t>(i)]);
              if (!projected.visible) {
                visible = false;
                break;
              }
              polygon << projected.point;
            }
            if (!visible) {
              continue;
            }
            const QVector3D normal = QVector3D::crossProduct(world[1] - world[0], world[2] - world[0]).normalized();
            const float shade = 0.42F + 0.58F * std::abs(QVector3D::dotProduct(normal, light));
            const QVector4D clip = matrix_ * QVector4D((world[0] + world[1] + world[2]) / 3.0F, 1.0F);
            const float depth = clip.z() / std::max(clip.w(), 1e-6F);
            QColor color = base;
            color.setRedF(std::clamp(color.redF() * shade, 0.0F, 1.0F));
            color.setGreenF(std::clamp(color.greenF() * shade, 0.0F, 1.0F));
            color.setBlueF(std::clamp(color.blueF() * shade, 0.0F, 1.0F));
            color.setAlpha(235);
            mesh_faces.push_back(MeshFace{depth, polygon, color});
          }
          drew_link = true;
          continue;
        }
        const msgs::Transform transform = tfmath::compose(link_tf->transform, origin_transform(geometry));
        const QColor base = link_color(link.name);
        const QColor fill = with_alpha(base, 150);
        const QColor outline = with_alpha(base.lighter(140), 230);
        if (geometry.geometry.type == UrdfGeometryType::Box) {
          draw_box(painter, transform, geometry.geometry.size, fill, outline);
        } else if (geometry.geometry.type == UrdfGeometryType::Cylinder) {
          draw_cylinder(painter, transform, geometry.geometry.radius, geometry.geometry.length, fill, outline);
        } else if (geometry.geometry.type == UrdfGeometryType::Sphere) {
          draw_box(painter, transform,
                   {geometry.geometry.radius * 2.0, geometry.geometry.radius * 2.0, geometry.geometry.radius * 2.0},
                   fill, outline);
        }
        drew_link = true;
      }
      if (!drew_link) {
        const auto& geometries = !link.collisions.empty() ? link.collisions : link.visuals;
        for (const auto& geometry : geometries) {
          if (geometry.geometry.type == UrdfGeometryType::None || geometry.geometry.type == UrdfGeometryType::Mesh) {
            continue;
          }
          const msgs::Transform transform = tfmath::compose(link_tf->transform, origin_transform(geometry));
          const QColor base = link_color(link.name);
          const QColor fill = with_alpha(base, 150);
          const QColor outline = with_alpha(base.lighter(140), 230);
          if (geometry.geometry.type == UrdfGeometryType::Box) {
            draw_box(painter, transform, geometry.geometry.size, fill, outline);
          } else if (geometry.geometry.type == UrdfGeometryType::Cylinder) {
            draw_cylinder(painter, transform, geometry.geometry.radius, geometry.geometry.length, fill, outline);
          } else if (geometry.geometry.type == UrdfGeometryType::Sphere) {
            draw_box(painter, transform,
                     {geometry.geometry.radius * 2.0, geometry.geometry.radius * 2.0, geometry.geometry.radius * 2.0},
                     fill, outline);
          }
          drew = true;
        }
      }
      drew = drew || drew_link;
    }
  }
  std::sort(mesh_faces.begin(), mesh_faces.end(),
            [](const MeshFace& a, const MeshFace& b) { return a.depth > b.depth; });
  painter.setPen(Qt::NoPen);
  for (const auto& face : mesh_faces) {
    painter.setBrush(face.color);
    painter.drawPolygon(face.polygon);
  }
  painter.setBrush(Qt::NoBrush);
  if (!drew) {
    const auto pose = robot_pose();
    if (pose) {
      const msgs::Transform t = transform_of(pose->pose);
      draw_box(painter, t, {0.3, 0.3, 0.12}, QColor(170, 178, 195, 150), QColor(220, 225, 235, 220));
    }
  }
  const auto base = model_.transform_in_fixed("base_footprint");
  if (base) {
    draw_yaw_arrow(painter, QVector3D(static_cast<float>(base->transform.translation.x),
                                      static_cast<float>(base->transform.translation.y), 0.02F),
                   base->transform.rotation, 0.42, QColor(80, 230, 255, 230));
  } else if (const auto pose = robot_pose()) {
    draw_yaw_arrow(painter, QVector3D(static_cast<float>(pose->pose.position.x),
                                      static_cast<float>(pose->pose.position.y), 0.02F),
                   pose->pose.orientation, 0.42, QColor(80, 230, 255, 230));
  }
}

void SceneView::draw_scan(QPainter& painter) const {
  const auto scan = model_.scan();
  if (!scan) {
    return;
  }
  const auto transform = model_.transform_in_fixed(scan->header.frame_id);
  if (!transform) {
    return;
  }
  QPolygonF points;
  const double increment = scan->angle_increment;
  for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const float range = scan->ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    const double angle = scan->angle_min + static_cast<double>(i) * increment;
    const msgs::Vector3 local{static_cast<double>(range) * std::cos(angle),
                              static_cast<double>(range) * std::sin(angle), 0.0};
    const msgs::Vector3 world = tfmath::apply(transform->transform, local);
    const Projected projected = project(QVector3D(static_cast<float>(world.x), static_cast<float>(world.y), 0.04F));
    if (projected.visible) {
      points << projected.point;
    }
  }
  painter.setPen(QPen(QColor(255, 85, 85, 230), 2.0));
  painter.drawPoints(points);
}

void SceneView::draw_imu(QPainter& painter) const {
  const auto imu = model_.imu();
  if (!imu) {
    return;
  }
  const auto transform = model_.transform_in_fixed(imu->header.frame_id);
  if (!transform) {
    return;
  }
  const QVector3D origin(static_cast<float>(transform->transform.translation.x),
                         static_cast<float>(transform->transform.translation.y),
                         static_cast<float>(transform->transform.translation.z));
  draw_yaw_arrow(painter, origin, imu->orientation, 0.3, QColor(230, 90, 230, 230));
}

void SceneView::draw_odometry_arrows(QPainter& painter) const {
  struct Source {
    std::shared_ptr<const msgs::Odometry> message;
    QColor color;
    const char* name;
    const char* key;
  };
  const Source sources[] = {{model_.odom(), QColor(255, 95, 210, 230), "odom", "odom"},
                            {model_.filtered(), QColor(80, 220, 255, 230), "filtered", "filtered"}};
  for (const auto& source : sources) {
    if (!source.message || !enabled(source.key)) {
      continue;
    }
    const auto frame_transform = model_.transform_in_fixed(source.message->header.frame_id);
    if (!frame_transform) {
      continue;
    }
    const msgs::Transform pose_transform = transform_of(source.message->pose.pose);
    const msgs::Transform composed = tfmath::compose(frame_transform->transform, pose_transform);
    const QVector3D origin(static_cast<float>(composed.translation.x), static_cast<float>(composed.translation.y), 0.03F);
    draw_yaw_arrow(painter, origin, composed.rotation, 0.35, source.color);
    draw_label(painter, origin + QVector3D(0.0F, 0.0F, 0.06F), source.name, source.color);
  }
}

void SceneView::draw_path(QPainter& painter, const std::shared_ptr<const msgs::Path>& path,
                          const QColor& color) const {
  if (!path || path->poses.empty()) {
    return;
  }
  const auto frame_transform = model_.transform_in_fixed(path->header.frame_id);
  if (!frame_transform) {
    return;
  }
  QPolygonF polyline;
  std::vector<QVector3D> points;
  std::vector<double> headings;
  points.reserve(path->poses.size());
  headings.reserve(path->poses.size());
  for (const auto& pose : path->poses) {
    const msgs::Transform local = transform_of(pose.pose);
    const msgs::Transform composed = tfmath::compose(frame_transform->transform, local);
    const QVector3D point(static_cast<float>(composed.translation.x), static_cast<float>(composed.translation.y), 0.03F);
    const Projected projected = project(point);
    if (projected.visible) {
      polyline << projected.point;
    }
    points.push_back(point);
    headings.push_back(tfmath::yaw_of(composed.rotation));
  }
  painter.setPen(QPen(color, 1.5));
  painter.setBrush(Qt::NoBrush);
  painter.drawPolyline(polyline);
  double accumulated = 1e9;
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (index > 0) {
      accumulated += std::hypot(static_cast<double>(points[index].x() - points[index - 1].x()),
                                static_cast<double>(points[index].y() - points[index - 1].y()));
    }
    if (accumulated >= 0.5 || index + 1 == points.size()) {
      accumulated = 0.0;
      const double length = (index + 1 == points.size()) ? 0.22 : 0.16;
      const double yaw = headings[index];
      const Projected origin = project(points[index]);
      if (origin.visible) {
        painter.setBrush(color);
        painter.drawEllipse(origin.point, 2.0, 2.0);
        painter.setBrush(Qt::NoBrush);
      }
      const QVector3D end(points[index].x() + static_cast<float>(std::cos(yaw) * length),
                          points[index].y() + static_cast<float>(std::sin(yaw) * length), 0.035F);
      const QVector3D left(points[index].x() + static_cast<float>(std::cos(yaw + 2.56) * 0.03),
                           points[index].y() + static_cast<float>(std::sin(yaw + 2.56) * 0.03), 0.035F);
      const QVector3D right(points[index].x() + static_cast<float>(std::cos(yaw - 2.56) * 0.03),
                            points[index].y() + static_cast<float>(std::sin(yaw - 2.56) * 0.03), 0.035F);
      draw_line(painter, points[index], end, color, 1.4);
      draw_line(painter, end, left, color, 1.4);
      draw_line(painter, end, right, color, 1.4);
    }
  }
}

void SceneView::draw_trail(QPainter& painter) {
  if (!enabled("trail")) {
    return;
  }
  const auto filtered = model_.filtered();
  if (!filtered) {
    return;
  }
  const auto frame_transform = model_.transform_in_fixed(filtered->header.frame_id);
  if (!frame_transform) {
    return;
  }
  const msgs::Transform local = transform_of(filtered->pose.pose);
  const msgs::Transform composed = tfmath::compose(frame_transform->transform, local);
  const QVector3D point(static_cast<float>(composed.translation.x), static_cast<float>(composed.translation.y), 0.01F);
  if (trail_.empty() || (point - trail_.back()).length() > 0.02F) {
    trail_.push_back(point);
    while (trail_.size() > 5000) {
      trail_.pop_front();
    }
  }
  if (trail_.size() < 2) {
    return;
  }
  QPolygonF polyline;
  for (const auto& item : trail_) {
    const Projected projected = project(item);
    if (projected.visible) {
      polyline << projected.point;
    }
  }
  painter.setPen(QPen(QColor(80, 220, 255, 130), 1.5));
  painter.drawPolyline(polyline);
  double accumulated = 0.0;
  for (std::size_t index = 1; index < trail_.size(); ++index) {
    accumulated += static_cast<double>((trail_[index] - trail_[index - 1]).length());
    if (accumulated >= 1.0 && index + 1 < trail_.size()) {
      accumulated = 0.0;
      const Projected projected = project(trail_[index]);
      if (!projected.visible) continue;
      const double yaw = std::atan2(static_cast<double>(trail_[index].y() - trail_[index - 1].y()),
                                    static_cast<double>(trail_[index].x() - trail_[index - 1].x()));
      painter.setBrush(QColor(80, 220, 255, 170));
      painter.drawEllipse(projected.point, 2.0, 2.0);
      painter.setBrush(Qt::NoBrush);
      const QVector3D end(trail_[index].x() + static_cast<float>(std::cos(yaw) * 0.12),
                          trail_[index].y() + static_cast<float>(std::sin(yaw) * 0.12), 0.01F);
      const QVector3D left(trail_[index].x() + static_cast<float>(std::cos(yaw + 2.56) * 0.03),
                           trail_[index].y() + static_cast<float>(std::sin(yaw + 2.56) * 0.03), 0.01F);
      const QVector3D right(trail_[index].x() + static_cast<float>(std::cos(yaw - 2.56) * 0.03),
                            trail_[index].y() + static_cast<float>(std::sin(yaw - 2.56) * 0.03), 0.01F);
      draw_line(painter, trail_[index], end, QColor(80, 220, 255, 170), 1.4);
      draw_line(painter, end, left, QColor(80, 220, 255, 170), 1.4);
      draw_line(painter, end, right, QColor(80, 220, 255, 170), 1.4);
    }
  }
}

void SceneView::draw_logs(QPainter& painter) const {
  if (!enabled("logs")) {
    return;
  }
  const auto logs = model_.logs();
  if (logs.empty()) {
    return;
  }
  QFont font("Menlo");
  font.setStyleHint(QFont::Monospace);
  font.setPixelSize(10);
  painter.setFont(font);
  const int count = std::min<int>(6, static_cast<int>(logs.size()));
  const int line_height = 13;
  const int top = height() - 22 - count * line_height;
  painter.fillRect(QRect(10, top - 8, width() - 20, count * line_height + 12), QColor(8, 10, 14, 170));
  for (int i = 0; i < count; ++i) {
    const auto& log = logs[logs.size() - static_cast<std::size_t>(count - i)];
    QColor color(200, 205, 215);
    if (log.level >= 50) {
      color = QColor(255, 120, 255);
    } else if (log.level >= 40) {
      color = QColor(255, 110, 110);
    } else if (log.level >= 30) {
      color = QColor(255, 210, 110);
    } else if (log.level < 20) {
      color = QColor(140, 145, 155);
    }
    painter.setPen(color);
    const QString text = QString("[%1] [%2] %3")
                             .arg(log.level)
                             .arg(QString::fromStdString(log.name))
                             .arg(QString::fromStdString(log.msg).left(140));
    painter.drawText(16, top + i * line_height + 8, text);
  }
}

void SceneView::draw_hud(QPainter& painter) const {
  const auto pose = robot_pose();
  const auto scan = model_.scan();
  double nearest = std::numeric_limits<double>::infinity();
  if (scan) {
    for (float range : scan->ranges) {
      if (std::isfinite(range)) {
        nearest = std::min(nearest, static_cast<double>(range));
      }
    }
  }
  QString pose_text = "robot: TF 없음";
  if (pose) {
    const auto yaw = tfmath::to_rpy(pose->pose.orientation).yaw;
    pose_text = QString("pose: x=%1 y=%2 yaw=%3°")
                    .arg(pose->pose.position.x, 0, 'f', 2)
                    .arg(pose->pose.position.y, 0, 'f', 2)
                    .arg(yaw * 180.0 / kPi, 0, 'f', 0);
  }
  QStringList lines;
  lines << "krosc viz";
  lines << QString("fixed: %1   follow: %2   fps: %3")
               .arg(QString::fromStdString(model_.fixed_frame()))
               .arg(follow_ ? "on" : "off")
               .arg(fps_, 0, 'f', 1);
  lines << pose_text;
  lines << QString("nearest scan: %1 m   teleop x%2")
               .arg(std::isfinite(nearest) ? QString::number(nearest, 'f', 2) : QString("-"))
               .arg(teleop_scale_, 0, 'f', 2);
  if (const auto map = model_.map()) {
    lines << QString("map: %1 x %2").arg(map->info.width).arg(map->info.height);
  } else {
    lines << "map: -";
  }
  QFont font("Menlo");
  font.setStyleHint(QFont::Monospace);
  font.setPixelSize(11);
  painter.setFont(font);
  const int line_height = 15;
  painter.fillRect(QRect(10, 10, 330, static_cast<int>(lines.size()) * line_height + 14), QColor(8, 10, 14, 175));
  painter.setPen(QColor(225, 230, 240));
  for (int i = 0; i < lines.size(); ++i) {
    painter.drawText(18, 26 + i * line_height, lines[i]);
  }
}

void SceneView::paintEvent(QPaintEvent* event) {
  (void)event;
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(10, 12, 17));
  update_camera();
  compute_matrix();

  if (enabled("grid")) {
    draw_grid(painter);
  }
  draw_map(painter);
  if (enabled("global_costmap")) {
    draw_costmap(painter, model_.global_costmap(), global_costmap_image_, global_costmap_cached_, "global costmap");
  }
  if (enabled("local_costmap")) {
    draw_costmap(painter, model_.local_costmap(), local_costmap_image_, local_costmap_cached_, "local costmap");
  }
  if (enabled("plan")) {
    draw_path(painter, model_.plan(), QColor(80, 235, 130, 230));
  }
  if (enabled("local_plan")) {
    draw_path(painter, model_.local_plan(), QColor(255, 220, 90, 220));
  }
  draw_trail(painter);
  draw_axes(painter);
  if (enabled("model")) {
    draw_robot(painter);
  }
  draw_arm_preview(painter);
  if (enabled("scan")) {
    draw_scan(painter);
  }
  if (enabled("imu")) {
    draw_imu(painter);
  }
  draw_odometry_arrows(painter);
  draw_waypoints(painter);
  draw_tool_preview(painter);
  draw_logs(painter);

  ++frame_count_;
  const qint64 elapsed = fps_clock_.elapsed();
  if (elapsed >= 1000) {
    fps_ = frame_count_ * 1000.0 / static_cast<double>(elapsed);
    frame_count_ = 0;
    fps_clock_.restart();
  }
  draw_hud(painter);
}

void SceneView::draw_waypoints(QPainter& painter) const {
  if (waypoints_.empty()) {
    return;
  }
  const bool fixed_is_map = model_.fixed_frame() == "map";
  std::optional<msgs::TransformStamped> transform;
  if (!fixed_is_map) {
    transform = model_.transform_in_fixed("map");
    if (!transform) {
      return;
    }
  }
  const double offset = transform ? tfmath::yaw_of(transform->transform.rotation) : 0.0;
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const auto& [wx, wy, wyaw] = waypoints_[index];
    msgs::Vector3 point{wx, wy, 0.04};
    if (transform) {
      point = tfmath::apply(transform->transform, point);
    }
    const double yaw = wyaw + offset;
    const QVector3D origin(static_cast<float>(point.x), static_cast<float>(point.y), 0.04F);
    const QVector3D end(static_cast<float>(point.x + std::cos(yaw) * 0.20),
                        static_cast<float>(point.y + std::sin(yaw) * 0.20), 0.04F);
    const QVector3D left(static_cast<float>(point.x + std::cos(yaw + 2.55) * 0.03),
                         static_cast<float>(point.y + std::sin(yaw + 2.55) * 0.03), 0.04F);
    const QVector3D right(static_cast<float>(point.x + std::cos(yaw - 2.55) * 0.03),
                          static_cast<float>(point.y + std::sin(yaw - 2.55) * 0.03), 0.04F);
    const Projected marker = project(origin);
    if (marker.visible) {
      painter.setBrush(QColor(255, 205, 70, 230));
      painter.drawEllipse(marker.point, 3.0, 3.0);
      painter.setBrush(Qt::NoBrush);
    }
    draw_line(painter, origin, end, QColor(255, 205, 70, 230), 1.6);
    draw_line(painter, end, left, QColor(255, 205, 70, 230), 1.6);
    draw_line(painter, end, right, QColor(255, 205, 70, 230), 1.6);
    draw_label(painter, origin, QString("#%1").arg(index + 1), QColor(255, 235, 150));
  }
}

void SceneView::set_arm_preview(ArmPreview preview) {
  arm_preview_ = std::move(preview);
  update();
}

void SceneView::clear_arm_preview() {
  arm_preview_ = {};
  update();
}

void SceneView::draw_arm_preview(QPainter& painter) const {
  if (!arm_preview_.active) {
    return;
  }
  const auto base = model_.transform_in_fixed("base_link");
  if (!base) {
    return;
  }
  const auto to_fixed = [&](const msgs::Vector3& point) {
    const msgs::Vector3 moved = tfmath::apply(base->transform, point);
    return QVector3D(static_cast<float>(moved.x), static_cast<float>(moved.y), static_cast<float>(moved.z));
  };

  const QColor arm_color(90, 205, 255, 220);
  const QColor joint_color(190, 235, 255, 245);
  std::optional<QVector3D> previous;
  for (const auto& link : arm_preview_.links) {
    const QVector3D point = to_fixed(link.translation);
    if (previous) {
      draw_line(painter, *previous, point, arm_color, 4.0);
    }
    draw_line(painter, point - QVector3D(0.012F, 0.0F, 0.0F), point + QVector3D(0.012F, 0.0F, 0.0F),
              joint_color, 2.0);
    draw_line(painter, point - QVector3D(0.0F, 0.012F, 0.0F), point + QVector3D(0.0F, 0.012F, 0.0F),
              joint_color, 2.0);
    previous = point;
  }
  const QVector3D tcp = to_fixed(arm_preview_.tcp);
  if (previous) {
    draw_line(painter, *previous, tcp, arm_color, 4.0);
  }
  // TCP 프레임(빨강=x, 초록=y, 파랑=z)
  if (!arm_preview_.links.empty()) {
    const msgs::Transform& tip = arm_preview_.links.back();
    const auto axis = [&](const msgs::Vector3& local) {
      return to_fixed(tfmath::apply(tip, local));
    };
    draw_line(painter, tcp, axis({0.05, 0.0, 0.0}), QColor(255, 95, 95, 240), 2.2);
    draw_line(painter, tcp, axis({0.0, 0.05, 0.0}), QColor(95, 235, 130, 240), 2.2);
    draw_line(painter, tcp, axis({0.0, 0.0, 0.05}), QColor(110, 165, 255, 240), 2.2);
  }

  if (arm_preview_.tcp_path.size() >= 2) {
    const std::size_t stride =
        std::max<std::size_t>(1, arm_preview_.tcp_path.size() / 400);
    QVector3D last = to_fixed(arm_preview_.tcp_path.front());
    for (std::size_t i = stride; i < arm_preview_.tcp_path.size(); i += stride) {
      const QVector3D point = to_fixed(arm_preview_.tcp_path[i]);
      draw_line(painter, last, point, QColor(80, 235, 150, 215), 2.0);
      last = point;
    }
    const QVector3D end = to_fixed(arm_preview_.tcp_path.back());
    draw_line(painter, last, end, QColor(80, 235, 150, 215), 2.0);
    draw_label(painter, end, QString("TCP path (%1)").arg(arm_preview_.tcp_path.size()),
               QColor(135, 245, 175));
  }

  if (arm_preview_.target) {
    const msgs::Transform& target = *arm_preview_.target;
    const QVector3D origin = to_fixed(target.translation);
    const auto axis = [&](const msgs::Vector3& local) {
      return to_fixed(tfmath::apply(target, local));
    };
    draw_line(painter, origin, axis({0.055, 0.0, 0.0}), QColor(255, 100, 100, 250), 2.6);
    draw_line(painter, origin, axis({0.0, 0.055, 0.0}), QColor(100, 245, 140, 250), 2.6);
    draw_line(painter, origin, axis({0.0, 0.0, 0.055}), QColor(120, 175, 255, 250), 2.6);
    draw_line(painter, origin - QVector3D(0.012F, 0.0F, 0.0F), origin + QVector3D(0.012F, 0.0F, 0.0F),
              QColor(255, 220, 90, 250), 2.0);
    draw_line(painter, origin - QVector3D(0.0F, 0.012F, 0.0F), origin + QVector3D(0.0F, 0.012F, 0.0F),
              QColor(255, 220, 90, 250), 2.0);
    draw_line(painter, origin - QVector3D(0.0F, 0.0F, 0.012F), origin + QVector3D(0.0F, 0.0F, 0.012F),
              QColor(255, 220, 90, 250), 2.0);
    // 인터랙티브 마커처럼 잡기 쉬운 링
    const QColor ring_color(255, 220, 90, 180);
    std::optional<QVector3D> ring_previous;
    for (int segment = 0; segment <= 28; ++segment) {
      const double angle = 2.0 * kPi * static_cast<double>(segment) / 28.0;
      const msgs::Vector3 local{0.042 * std::cos(angle), 0.042 * std::sin(angle), 0.0};
      const QVector3D point = to_fixed(tfmath::apply(target, local));
      if (ring_previous) {
        draw_line(painter, *ring_previous, point, ring_color, 1.6);
      }
      ring_previous = point;
    }
    draw_label(painter, origin + QVector3D(0.0F, 0.0F, 0.035F), "TCP target", QColor(255, 225, 120));
  }
}

void SceneView::draw_tool_preview(QPainter& painter) const {
  if (!tool_dragging_ || !tool_start_ground_ || !tool_current_ground_ || tool_.empty()) {
    return;
  }
  QColor color(80, 235, 145, 240);
  if (tool_ == "goal") {
    color = QColor(255, 95, 205, 240);
  } else if (tool_ == "multi") {
    color = QColor(255, 210, 70, 240);
  }
  const QVector3D start = *tool_start_ground_;
  const double dx = static_cast<double>(tool_current_ground_->x() - start.x());
  const double dy = static_cast<double>(tool_current_ground_->y() - start.y());
  const double distance = std::hypot(dx, dy);
  const double yaw = distance > 0.03 ? std::atan2(dy, dx) : 0.0;
  const double length = std::max(0.6, distance);
  const QVector3D end(start.x() + static_cast<float>(std::cos(yaw) * length),
                      start.y() + static_cast<float>(std::sin(yaw) * length), 0.03F);
  draw_line(painter, start, end, color, 4.0);
  const double kHead = std::min(0.04, 0.08 * length);
  const QVector3D head_left(start.x() + static_cast<float>(std::cos(yaw + 2.55) * kHead),
                            start.y() + static_cast<float>(std::sin(yaw + 2.55) * kHead), 0.03F);
  const QVector3D head_right(start.x() + static_cast<float>(std::cos(yaw - 2.55) * kHead),
                             start.y() + static_cast<float>(std::sin(yaw - 2.55) * kHead), 0.03F);
  draw_line(painter, end, head_left, color, 4.0);
  draw_line(painter, end, head_right, color, 4.0);
  const Projected center = project(QVector3D(start.x(), start.y(), 0.03F));
  if (center.visible) {
    painter.setPen(QPen(color, 2.0));
    painter.setBrush(QColor(color.red(), color.green(), color.blue(), 55));
    painter.drawEllipse(center.point, 7.0, 7.0);
    painter.setBrush(Qt::NoBrush);
  }
}

void SceneView::mousePressEvent(QMouseEvent* event) {
  setFocus(Qt::MouseFocusReason);
  last_mouse_ = event->position().toPoint();
  if (event->button() == Qt::LeftButton && moveit_interactive_ && arm_preview_.target) {
    const auto base = model_.transform_in_fixed("base_link");
    if (base) {
      const msgs::Vector3 fixed = tfmath::apply(base->transform, arm_preview_.target->translation);
      const Projected projected = project(QVector3D(static_cast<float>(fixed.x), static_cast<float>(fixed.y),
                                                    static_cast<float>(fixed.z)));
      if (projected.visible) {
        const double distance = std::hypot(projected.point.x() - event->position().x(),
                                           projected.point.y() - event->position().y());
        if (distance <= 30.0) {
          moveit_dragging_ = true;
          return;
        }
      }
    }
  }
  if (event->button() == Qt::LeftButton && !tool_.empty()) {
    tool_dragging_ = true;
    tool_start_ = last_mouse_;
    tool_start_ground_ = screen_to_ground(tool_start_);
    tool_current_ground_ = tool_start_ground_;
    return;
  }
  if (event->button() == Qt::LeftButton) {
    orbiting_ = true;
  } else if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    panning_ = true;
  }
}

void SceneView::mouseMoveEvent(QMouseEvent* event) {
  const QPoint current = event->position().toPoint();
  const QPoint delta = current - last_mouse_;
  last_mouse_ = current;
  if (moveit_dragging_ && arm_preview_.target) {
    const auto base = model_.transform_in_fixed("base_link");
    if (!base) {
      return;
    }
    msgs::Vector3 fixed = tfmath::apply(base->transform, arm_preview_.target->translation);
    if ((event->modifiers() & Qt::ControlModifier) != 0) {
      // Ctrl+상하 드래그: 그리퍼 접근 pitch
      const msgs::Vector3 tool = tfmath::rotate(arm_preview_.target->rotation, {1.0, 0.0, 0.0});
      const double current_pitch = -std::asin(std::clamp(tool.z, -1.0, 1.0));
      const double pitch = std::clamp(current_pitch + static_cast<double>(delta.y()) * 0.004, -1.55, 1.55);
      const double yaw = tfmath::yaw_of(arm_preview_.target->rotation);
      arm_preview_.target->rotation = tfmath::from_rpy(0.0, pitch, yaw);
      update();
      emit moveit_pitch_dragged(pitch * 180.0 / kPi);
      return;
    }
    if ((event->modifiers() & Qt::ShiftModifier) != 0) {
      fixed.z += -static_cast<double>(delta.y()) * camera_.distance * 0.0015;
    } else {
      const auto point = screen_to_plane(current, static_cast<float>(fixed.z));
      if (!point) {
        return;
      }
      fixed.x = point->x();
      fixed.y = point->y();
    }
    msgs::Vector3 target = tfmath::apply(tfmath::inverse(base->transform), fixed);
    target.z = std::clamp(target.z, -0.15, 0.85);
    arm_preview_.target->translation = target;
    update();
    emit moveit_target_dragged(target.x, target.y, target.z);
    return;
  }
  if (tool_dragging_) {
    tool_current_ground_ = screen_to_ground(current);
    update();
    return;
  }
  if (orbiting_) {
    camera_.yaw -= delta.x() * 0.008;
    camera_.pitch = std::clamp(camera_.pitch + delta.y() * 0.008, 0.08, 1.5);
    update();
  } else if (panning_) {
    const double scale = camera_.distance * 0.0016;
    const QVector3D right(static_cast<float>(-std::sin(camera_.yaw)), static_cast<float>(std::cos(camera_.yaw)), 0.0F);
    const QVector3D forward(static_cast<float>(-std::cos(camera_.yaw)), static_cast<float>(-std::sin(camera_.yaw)), 0.0F);
    // 좌우는 그대로, 위아래는 반대로: 마우스를 아래로 끌면 화면도 아래로 (RViz pan과 같은 방향)
    camera_.target -= right * static_cast<float>(delta.x() * scale);
    camera_.target += forward * static_cast<float>(delta.y() * scale);
    if (follow_) {
      follow_target_ = camera_.target;
    }
    update();
  }
}

void SceneView::mouseReleaseEvent(QMouseEvent* event) {
  if (moveit_dragging_ && event->button() == Qt::LeftButton) {
    moveit_dragging_ = false;
    emit moveit_drag_finished();
    return;
  }
  if (tool_dragging_) {
    tool_dragging_ = false;
    const auto end_ground = screen_to_ground(event->position().toPoint());
    if (tool_start_ground_ && end_ground) {
      const double dx = end_ground->x() - tool_start_ground_->x();
      const double dy = end_ground->y() - tool_start_ground_->y();
      const double yaw = (dx * dx + dy * dy > 1e-4) ? std::atan2(dy, dx) : 0.0;
      emit tool_pose(static_cast<double>(tool_start_ground_->x()), static_cast<double>(tool_start_ground_->y()), yaw);
    }
    tool_current_ground_.reset();
    if (tool_ != "multi") {
      set_tool("");
    } else {
      update();
    }
    return;
  }
  orbiting_ = false;
  panning_ = false;
}

void SceneView::wheelEvent(QWheelEvent* event) {
  const double factor = std::exp(-event->angleDelta().y() / 600.0);
  camera_.distance = std::clamp(camera_.distance * factor, 0.4, 40.0);
  update();
}

void SceneView::keyPressEvent(QKeyEvent* event) {
  const int code = qwerty_key(event);
  if (!teleop_enabled_ && is_move_key(code)) {
    event->accept();
    return;
  }
  if (event->isAutoRepeat()) {
    if (is_move_key(code)) {
      keys_.insert(code);
    }
    return;
  }
  if (is_move_key(code)) {
    key_codes_[physical_key(event)] = code;
    keys_.insert(code);
    event->accept();
    return;
  }
  switch (code) {
    case Qt::Key_Q:
      set_teleop_scale(teleop_scale_ * 1.25);
      return;
    case Qt::Key_Z:
      set_teleop_scale(teleop_scale_ / 1.25);
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
    case Qt::Key_Delete:
      trail_.clear();
      update();
      return;
    default:
      QWidget::keyPressEvent(event);
      return;
  }
}

void SceneView::keyReleaseEvent(QKeyEvent* event) {
  if (event->isAutoRepeat()) {
    return;
  }
  const auto found = key_codes_.find(physical_key(event));
  int code = event->key();
  if (found != key_codes_.end()) {
    code = found.value();
    key_codes_.erase(found);
  } else {
    code = qwerty_key(event);
  }
  keys_.remove(code);
}

void SceneView::focusOutEvent(QFocusEvent* event) {
  (void)event;
  keys_.clear();
  key_codes_.clear();
  if (moveit_dragging_) {
    moveit_dragging_ = false;
    emit moveit_drag_finished();
  }
  orbiting_ = false;
  panning_ = false;
}

void SceneView::leaveEvent(QEvent* event) {
  (void)event;
}

void SceneView::set_teleop_key(int key, bool pressed) {
  if (pressed) {
    keys_.insert(key);
  } else {
    keys_.remove(key);
  }
}

void SceneView::clear_teleop_keys() { keys_.clear(); }

void SceneView::publish_teleop() {
  if (!teleop_enabled_) {
    return;
  }
  const auto held = [this](std::initializer_list<int> keys) {
    return std::any_of(keys.begin(), keys.end(), [this](int key) { return keys_.contains(key); });
  };
  const double forward = (held({Qt::Key_W, Qt::Key_Up}) ? 1.0 : 0.0) - (held({Qt::Key_S, Qt::Key_Down}) ? 1.0 : 0.0);
  const double left = (held({Qt::Key_A, Qt::Key_Left}) ? 1.0 : 0.0) - (held({Qt::Key_D, Qt::Key_Right}) ? 1.0 : 0.0);
  const double linear = 0.26 * teleop_scale_ * forward;
  const double angular = 1.5 * teleop_scale_ * left;
  // 키를 누르고 있는 동안에는 계속 보낸다. 한 번만 보내면 twist_mux/OpenCR의
  // 0.5초 명령 타임아웃에 걸려 로봇이 가다가 멈춘다.
  if (linear != 0.0 || angular != 0.0) {
    model_.publish_cmd_vel(linear, angular);
    last_linear_ = linear;
    last_angular_ = angular;
    return;
  }
  // 키를 모두 뗀 순간에는 0을 한 번 보내 즉시 정지시킨다.
  if (last_linear_ != 0.0 || last_angular_ != 0.0) {
    model_.publish_cmd_vel(0.0, 0.0);
    last_linear_ = 0.0;
    last_angular_ = 0.0;
  }
}

}  // namespace krosc::viz
