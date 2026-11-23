#include "krosc/viz/scene_view.hpp"

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
#include <cstddef>
#include <limits>
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
        if (value < 0) {
          continue;
        }
        if (value >= 100) {
          color = QColor(235, 55, 55, 230);
        } else if (value > 0) {
          const int t = std::min(value, 99);
          color = QColor(235, 90 + t, 45, 35 + t * 2);
        } else {
          continue;
        }
      } else {
        if (value < 0) {
          color = QColor(105, 105, 120, 28);
        } else if (value >= 50) {
          color = QColor(230, 232, 240, 210);
        } else {
          color = QColor(120, 150, 190, 26 + value * 2);
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
      "W/A/S/D: teleop   Q/Z: speed   F: follow   C: overview   drag: orbit   right drag: pan   wheel: zoom");
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
  const auto frame_transform = model_.transform_in_fixed(grid.header.frame_id);
  if (!frame_transform) {
    return;
  }
  const double resolution = grid.info.resolution;
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;
  const double width_m = width * resolution;
  const double height_m = height * resolution;
  const msgs::Quaternion origin_rotation = grid.info.origin.orientation;
  const auto to_fixed = [&](double x, double y) {
    const msgs::Vector3 in_map = tfmath::apply(msgs::Transform{{ox, oy, 0.0}, origin_rotation}, msgs::Vector3{x, y, 0.0});
    const msgs::Vector3 in_fixed = tfmath::apply(frame_transform->transform, in_map);
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
  QTransform transform;
  if (!QTransform::quadToQuad(QRectF(0, 0, width, height), QPolygonF() << tl << tr << br << bl, transform)) {
    return;
  }
  painter.save();
  painter.setTransform(transform);
  painter.drawImage(0, 0, image);
  painter.restore();
}

void SceneView::draw_map(QPainter& painter) const {
  const auto grid = model_.map();
  if (!grid || !enabled("map")) {
    return;
  }
  if (grid != map_cached_) {
    map_image_ = build_grid_image(*grid, false);
    map_cached_ = grid;
  }
  draw_grid_image(painter, *grid, map_image_);
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

void SceneView::draw_robot(QPainter& painter) {
  const auto description = model_.robot_description();
  if (description) {
    if (!robot_ || robot_source_ != description->data) {
      robot_source_ = description->data;
      try {
        robot_ = std::make_unique<UrdfRobot>(parse_urdf(robot_source_));
      } catch (const UrdfError&) {
        robot_.reset();
      }
    }
  }
  bool drew = false;
  if (robot_) {
    for (const auto& link : robot_->links) {
      const auto link_tf = model_.transform_in_fixed(link.name);
      if (!link_tf) {
        continue;
      }
      const auto& geometries = !link.collisions.empty() ? link.collisions : link.visuals;
      for (const auto& geometry : geometries) {
        if (geometry.geometry.type == UrdfGeometryType::None || geometry.geometry.type == UrdfGeometryType::Mesh) {
          continue;
        }
        const msgs::Transform t = tfmath::compose(link_tf->transform, origin_transform(geometry));
        const QColor base = link_color(link.name);
        const QColor fill = with_alpha(base, 150);
        const QColor outline = with_alpha(base.lighter(140), 230);
        if (geometry.geometry.type == UrdfGeometryType::Box) {
          draw_box(painter, t, geometry.geometry.size, fill, outline);
        } else if (geometry.geometry.type == UrdfGeometryType::Cylinder) {
          draw_cylinder(painter, t, geometry.geometry.radius, geometry.geometry.length, fill, outline);
        } else if (geometry.geometry.type == UrdfGeometryType::Sphere) {
          draw_box(painter, t, {geometry.geometry.radius * 2.0, geometry.geometry.radius * 2.0,
                                geometry.geometry.radius * 2.0},
                   fill, outline);
        }
        drew = true;
      }
    }
  }
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
  for (const auto& pose : path->poses) {
    const msgs::Transform local = transform_of(pose.pose);
    const msgs::Transform composed = tfmath::compose(frame_transform->transform, local);
    const Projected projected = project(
        QVector3D(static_cast<float>(composed.translation.x), static_cast<float>(composed.translation.y), 0.03F));
    if (projected.visible) {
      polyline << projected.point;
    }
  }
  painter.setPen(QPen(color, 2.2));
  painter.setBrush(Qt::NoBrush);
  painter.drawPolyline(polyline);
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
  painter.setPen(QPen(QColor(80, 220, 255, 130), 1.8));
  painter.drawPolyline(polyline);
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
  if (enabled("scan")) {
    draw_scan(painter);
  }
  if (enabled("imu")) {
    draw_imu(painter);
  }
  draw_odometry_arrows(painter);
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

void SceneView::mousePressEvent(QMouseEvent* event) {
  setFocus(Qt::MouseFocusReason);
  last_mouse_ = event->position().toPoint();
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

void SceneView::mouseReleaseEvent(QMouseEvent* event) {
  (void)event;
  orbiting_ = false;
  panning_ = false;
}

void SceneView::wheelEvent(QWheelEvent* event) {
  const double factor = std::exp(-event->angleDelta().y() / 600.0);
  camera_.distance = std::clamp(camera_.distance * factor, 0.4, 40.0);
  update();
}

void SceneView::keyPressEvent(QKeyEvent* event) {
  if (event->isAutoRepeat()) {
    if (event->key() == Qt::Key_W || event->key() == Qt::Key_A || event->key() == Qt::Key_S ||
        event->key() == Qt::Key_D || event->key() == Qt::Key_Up || event->key() == Qt::Key_Down ||
        event->key() == Qt::Key_Left || event->key() == Qt::Key_Right) {
      keys_.insert(event->key());
    }
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
      update();
      return;
    case Qt::Key_Z:
      teleop_scale_ = std::max(0.25, teleop_scale_ / 1.25);
      update();
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
  if (!event->isAutoRepeat()) {
    keys_.remove(event->key());
  }
}

void SceneView::focusOutEvent(QFocusEvent* event) {
  (void)event;
  keys_.clear();
  orbiting_ = false;
  panning_ = false;
}

void SceneView::leaveEvent(QEvent* event) {
  (void)event;
}

void SceneView::publish_teleop() {
  const auto held = [this](std::initializer_list<int> keys) {
    return std::any_of(keys.begin(), keys.end(), [this](int key) { return keys_.contains(key); });
  };
  const double forward = (held({Qt::Key_W, Qt::Key_Up}) ? 1.0 : 0.0) - (held({Qt::Key_S, Qt::Key_Down}) ? 1.0 : 0.0);
  const double left = (held({Qt::Key_A, Qt::Key_Left}) ? 1.0 : 0.0) - (held({Qt::Key_D, Qt::Key_Right}) ? 1.0 : 0.0);
  const double linear = 0.26 * teleop_scale_ * forward;
  const double angular = 1.5 * teleop_scale_ * left;
  if (linear != last_linear_ || angular != last_angular_) {
    model_.publish_cmd_vel(linear, angular);
    last_linear_ = linear;
    last_angular_ = angular;
  }
}

}  // namespace krosc::viz
