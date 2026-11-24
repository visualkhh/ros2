#include "krosc/viz/map_view.hpp"

#include <QFocusEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPolygonF>
#include <QPainter>
#include <QPushButton>
#include <QResizeEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "krosc/math.hpp"
#include "krosc/tf.hpp"

namespace krosc::viz {

MapView2D::MapView2D(VizModel& model, QWidget* parent) : QWidget(parent), model_(model) {
  setMinimumSize(240, 200);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  fit_button_ = new QPushButton("Fit map", this);
  fit_button_->setStyleSheet(
      "QPushButton { background:#171c26; border:1px solid #2a3242; border-radius:4px; padding:4px 10px; color:#d7dde8; }"
      "QPushButton:hover { background:#202838; }");
  fit_button_->adjustSize();
  connect(fit_button_, &QPushButton::clicked, this, &MapView2D::fit_map);
  place_fit_button();
}

void MapView2D::place_fit_button() {
  if (fit_button_ != nullptr) {
    fit_button_->move(width() - fit_button_->width() - 12, 10);
    fit_button_->raise();
  }
}

void MapView2D::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  place_fit_button();
  if (!fitted_once_) {
    fit_map();
  }
}

void MapView2D::set_tool(const std::string& tool) {
  tool_ = (tool == "initialpose" || tool == "goal" || tool == "multi") ? tool : std::string{};
  tool_dragging_ = false;
  emit tool_changed(QString::fromStdString(tool_));
  update();
}

void MapView2D::set_waypoints(const std::vector<std::tuple<double, double, double>>& waypoints) {
  waypoints_ = waypoints;
  update();
}

void MapView2D::set_show_tf(bool show) {
  show_tf_ = show;
  update();
}

void MapView2D::set_show_odom(bool show) {
  show_odom_ = show;
  update();
}

void MapView2D::set_show_filtered(bool show) {
  show_filtered_ = show;
  update();
}

void MapView2D::draw_arrow(QPainter& painter, const QPointF& start, double angle, double length, const QColor& color) const {
  const QPointF end(start.x() + std::cos(angle) * length, start.y() - std::sin(angle) * length);
  painter.setPen(QPen(color, 2.0));
  painter.drawLine(start, end);
  const double head = std::min(0.04 * zoom_, 0.18 * length);
  const double a1 = angle + 2.6;
  const double a2 = angle - 2.6;
  painter.drawLine(end, QPointF(end.x() + std::cos(a1) * head, end.y() - std::sin(a1) * head));
  painter.drawLine(end, QPointF(end.x() + std::cos(a2) * head, end.y() - std::sin(a2) * head));
}

void MapView2D::fit_map() {
  const auto map = model_.map();
  if (!map || map->info.width == 0 || map->info.height == 0 || map->info.resolution <= 0.0) {
    return;
  }
  const double width_m = map->info.width * map->info.resolution;
  const double height_m = map->info.height * map->info.resolution;
  if (width_m <= 0.0 || height_m <= 0.0) {
    return;
  }
  const double margin = 32.0;
  zoom_ = std::min((static_cast<double>(width()) - margin) / width_m,
                   (static_cast<double>(height()) - margin) / height_m);
  if (!(zoom_ > 0.0)) {
    zoom_ = 20.0;
  }
  offset_x_ = (static_cast<double>(width()) - width_m * zoom_) / 2.0;
  offset_y_ = (static_cast<double>(height()) + height_m * zoom_) / 2.0;
  fitted_once_ = true;
  update();
}

void MapView2D::rebuild_image() {
  const auto map = model_.map();
  if (map == map_cache_) {
    return;
  }
  map_cache_ = map;
  image_ = QImage();
  if (!map || map->info.width == 0 || map->info.height == 0 ||
      map->data.size() != static_cast<std::size_t>(map->info.width) * map->info.height) {
    return;
  }
  const std::uint32_t width = map->info.width;
  const std::uint32_t height = map->info.height;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;
  resolution_ = map->info.resolution;
  image_ = QImage(static_cast<int>(width), static_cast<int>(height), QImage::Format_ARGB32);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const std::int8_t value = map->data[static_cast<std::size_t>(y) * width + x];
      QColor color;
      if (value < 0) {
        color = QColor(205, 205, 205);  // 모름 (PGM의 205)
      } else if (value >= 50) {
        color = QColor(18, 18, 22);  // 벽 (PGM의 검정)
      } else {
        color = QColor(252, 252, 252);  // 빈 곳 (PGM의 흰색)
      }
      image_.setPixelColor(static_cast<int>(x), static_cast<int>(height - 1 - y), color);
    }
  }
  if (!fitted_once_) {
    fit_map();
  }
}

QPointF MapView2D::to_screen(double x, double y) const {
  return {offset_x_ + (x - origin_x_) * zoom_, offset_y_ - (y - origin_y_) * zoom_};
}

QPointF MapView2D::to_map(const QPointF& screen) const {
  return {origin_x_ + (screen.x() - offset_x_) / zoom_, origin_y_ + (offset_y_ - screen.y()) / zoom_};
}

void MapView2D::paintEvent(QPaintEvent* event) {
  (void)event;
  QPainter painter(this);
  painter.fillRect(rect(), QColor(10, 12, 17));
  rebuild_image();
  if (image_.isNull()) {
    painter.setPen(QColor(150, 160, 175));
    painter.drawText(rect(), Qt::AlignCenter, "no map");
    return;
  }
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  const double width_m = image_.width() * resolution_ * zoom_;
  const double height_m = image_.height() * resolution_ * zoom_;
  const QRectF target(offset_x_, offset_y_ - height_m, width_m, height_m);
  painter.drawImage(target, image_);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, false);

  // 1m 격자 (지도 위에 흐리게)
  painter.setPen(QPen(QColor(70, 80, 95, 55), 1.0));
  const double x0 = origin_x_;
  const double y0 = origin_y_;
  const double x1 = origin_x_ + image_.width() * resolution_;
  const double y1 = origin_y_ + image_.height() * resolution_;
  for (double x = std::ceil(x0); x <= x1; x += 1.0) {
    painter.drawLine(to_screen(x, y0), to_screen(x, y1));
  }
  for (double y = std::ceil(y0); y <= y1; y += 1.0) {
    painter.drawLine(to_screen(x0, y), to_screen(x1, y));
  }
  painter.setPen(QPen(QColor(0, 150, 230, 150), 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(target);

  const auto tf = [&](const std::string& target_frame, const std::string& source_frame) -> std::optional<msgs::TransformStamped> {
    try {
      return model_.tf_buffer().lookup_transform(target_frame, source_frame);
    } catch (const TransformException&) {
      return std::nullopt;
    }
  };

  // TF 프레임 축 (map 기준, RViz TF display처럼)
  if (show_tf_) {
    painter.save();
    QFont tf_font("Menlo");
    tf_font.setPixelSize(10);
    painter.setFont(tf_font);
    for (const auto& frame : model_.tf_buffer().all_frames()) {
      const auto transform = tf("map", frame);
      if (!transform) {
        continue;
      }
      const double x = transform->transform.translation.x;
      const double y = transform->transform.translation.y;
      const double yaw = tfmath::yaw_of(transform->transform.rotation);
      const QPointF origin = to_screen(x, y);
      if (origin.x() < -20.0 || origin.y() < -20.0 || origin.x() > width() + 20.0 || origin.y() > height() + 20.0) {
        continue;
      }
      const double length = 0.16;
      painter.setPen(QPen(QColor(255, 90, 90, 220), 1.6));
      painter.drawLine(origin, to_screen(x + std::cos(yaw) * length, y + std::sin(yaw) * length));
      painter.setPen(QPen(QColor(90, 235, 120, 220), 1.6));
      painter.drawLine(origin, to_screen(x + std::cos(yaw + 1.5707963) * length,
                                        y + std::sin(yaw + 1.5707963) * length));
      if (frame == "map" || frame == "odom" || frame == "base_footprint" || frame == "base_link") {
        painter.setPen(QColor(190, 200, 215, 200));
        painter.drawText(origin + QPointF(7.0, -7.0), QString::fromStdString(frame));
      }
    }
    painter.restore();
  }

  // 계획 경로들 (map 좌표)
  const auto draw_path = [&](const std::shared_ptr<const msgs::Path>& path, const QColor& color) {
    if (!path || path->poses.empty()) {
      return;
    }
    std::vector<QPointF> points;
    points.reserve(path->poses.size());
    std::optional<msgs::TransformStamped> transform;
    if (!path->header.frame_id.empty() && path->header.frame_id != "map") {
      transform = tf("map", path->header.frame_id);
      if (!transform) {
        return;
      }
    }
    std::vector<std::tuple<double, double, double>> world;
    world.reserve(path->poses.size());
    const double yaw_offset = transform ? tfmath::yaw_of(transform->transform.rotation) : 0.0;
    for (const auto& pose : path->poses) {
      msgs::Vector3 p{pose.pose.position.x, pose.pose.position.y, 0.0};
      if (transform) {
        p = tfmath::apply(transform->transform, p);
      }
      points.push_back(to_screen(p.x, p.y));
      world.emplace_back(p.x, p.y, tfmath::yaw_of(pose.pose.orientation) + yaw_offset);
    }
    painter.setPen(QPen(color, 1.5));
    painter.setBrush(Qt::NoBrush);
    painter.drawPolyline(points.data(), static_cast<int>(points.size()));
    double accumulated = 1e9;  // 첫 웨이포인트도 표시
    const QColor arrow_color(color.red(), color.green(), color.blue(), 210);
    for (std::size_t index = 0; index < world.size(); ++index) {
      if (index > 0) {
        accumulated += std::hypot(std::get<0>(world[index]) - std::get<0>(world[index - 1]),
                                  std::get<1>(world[index]) - std::get<1>(world[index - 1]));
      }
      if (accumulated >= 0.5 || index + 1 == world.size()) {
        accumulated = 0.0;
        const QPointF point = to_screen(std::get<0>(world[index]), std::get<1>(world[index]));
        painter.setPen(QPen(color, 1.5));
        painter.setBrush(arrow_color);
        painter.drawEllipse(point, 2.4, 2.4);  // 웨이포인트 점
        painter.setBrush(Qt::NoBrush);
        const double length = (index + 1 == world.size() ? 0.22 : 0.16) * zoom_;
        draw_arrow(painter, point, std::get<2>(world[index]), length, arrow_color);
      }
    }
  };
  draw_path(model_.plan(), QColor(80, 235, 130, 230));
  draw_path(model_.local_plan(), QColor(255, 220, 90, 200));

  // Multi Goal로 찍은 웨이포인트 (방향 포함)
  QFont waypoint_font("Menlo");
  waypoint_font.setPixelSize(11);
  painter.setFont(waypoint_font);
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const auto& [x, y, yaw] = waypoints_[index];
    const QPointF point = to_screen(x, y);
    painter.setPen(QPen(QColor(255, 205, 60, 240), 2.0));
    painter.setBrush(QColor(255, 205, 60, 220));
    painter.drawEllipse(point, 4.0, 4.0);
    painter.setBrush(Qt::NoBrush);
    draw_arrow(painter, point, yaw, 0.20 * zoom_, QColor(255, 205, 70, 230));
    painter.setPen(QColor(255, 235, 150));
    painter.drawText(point + QPointF(8.0, -8.0), QString("#%1").arg(index + 1));
  }

  // 스캔
  if (const auto scan = model_.scan()) {
    if (const auto sensor = tf("map", scan->header.frame_id)) {
      const double yaw = tfmath::yaw_of(sensor->transform.rotation);
      const double sx = sensor->transform.translation.x;
      const double sy = sensor->transform.translation.y;
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      std::vector<QPointF> points;
      points.reserve(scan->ranges.size());
      for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
        const double range = static_cast<double>(scan->ranges[index]);
        if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
          continue;
        }
        const double angle = scan->angle_min + static_cast<double>(index) * scan->angle_increment;
        const double px = range * std::cos(angle);
        const double py = range * std::sin(angle);
        points.push_back(to_screen(sx + c * px - s * py, sy + s * px + c * py));
      }
      painter.setPen(QPen(QColor(255, 80, 80, 210), 2.0));
      painter.drawPoints(points.data(), static_cast<int>(points.size()));
    }
  }

  // 실제 주행 궤적 (진짜 경로)
  if (const auto robot = tf("map", "base_footprint")) {
    const double rx = robot->transform.translation.x;
    const double ry = robot->transform.translation.y;
    if (trail_.empty() || std::hypot(rx - trail_.back().first, ry - trail_.back().second) > 0.05) {
      trail_.emplace_back(rx, ry);
      while (trail_.size() > 4000) {
        trail_.pop_front();
      }
    }
    if (trail_.size() > 1) {
      std::vector<QPointF> points;
      points.reserve(trail_.size());
      for (const auto& [x, y] : trail_) {
        points.push_back(to_screen(x, y));
      }
      painter.setPen(QPen(QColor(140, 185, 255, 190), 1.6));
      painter.drawPolyline(points.data(), static_cast<int>(points.size()));
      double accumulated = 0.0;
      for (std::size_t index = 1; index < trail_.size(); ++index) {
        accumulated += std::hypot(trail_[index].first - trail_[index - 1].first,
                                  trail_[index].second - trail_[index - 1].second);
        if (accumulated >= 1.0 && index + 1 < trail_.size()) {
          accumulated = 0.0;
          const double angle = std::atan2(trail_[index].second - trail_[index - 1].second,
                                          trail_[index].first - trail_[index - 1].first);
          const QPointF point = to_screen(trail_[index].first, trail_[index].second);
          painter.setBrush(QColor(140, 185, 255, 210));
          painter.drawEllipse(point, 2.0, 2.0);
          painter.setBrush(Qt::NoBrush);
          draw_arrow(painter, point, angle, 0.12 * zoom_, QColor(140, 185, 255, 210));
        }
      }
    }
  }

  // /odom, EKF filtered 궤적 화살표 (map 기준)
  const auto draw_odom_arrow = [&](const std::shared_ptr<const msgs::Odometry>& message, const QColor& color,
                                   const char* name) {
    if (!message) {
      return;
    }
    const auto frame_transform = tf("map", message->header.frame_id);
    if (!frame_transform) {
      return;
    }
    const msgs::Transform local{{message->pose.pose.position.x, message->pose.pose.position.y,
                                 message->pose.pose.position.z},
                                message->pose.pose.orientation};
    const msgs::Transform composed = tfmath::compose(frame_transform->transform, local);
    const double x = composed.translation.x;
    const double y = composed.translation.y;
    const double yaw = tfmath::yaw_of(composed.rotation);
    const QPointF origin = to_screen(x, y);
    const QPointF end = to_screen(x + std::cos(yaw) * 0.36, y + std::sin(yaw) * 0.36);
    painter.setPen(QPen(color, 2.0));
    painter.drawLine(origin, end);
    const double head = 0.05 * zoom_;
    painter.drawLine(end, QPointF(end.x() - std::cos(yaw - 0.5) * head, end.y() + std::sin(yaw - 0.5) * head));
    painter.drawLine(end, QPointF(end.x() - std::cos(yaw + 0.5) * head, end.y() + std::sin(yaw + 0.5) * head));
    painter.setPen(QColor(color.red(), color.green(), color.blue(), 210));
    painter.drawText(origin + QPointF(7.0, 10.0), QString::fromUtf8(name));
  };
  if (show_odom_) {
    draw_odom_arrow(model_.odom(), QColor(255, 95, 210, 230), "wheel odom");
  }
  if (show_filtered_) {
    draw_odom_arrow(model_.filtered(), QColor(80, 220, 255, 230), "EKF filtered");
  }

  // 로봇 (base_footprint)
  if (const auto robot = tf("map", "base_footprint")) {
    const double yaw = tfmath::yaw_of(robot->transform.rotation);
    const double x = robot->transform.translation.x;
    const double y = robot->transform.translation.y;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const auto point = [&](double forward, double left) { return to_screen(x + c * forward - s * left, y + s * forward + c * left); };
    QPolygonF triangle;
    triangle << point(0.27, 0.0) << point(-0.14, 0.12) << point(-0.14, -0.12);
    painter.setBrush(QColor(70, 140, 255, 200));
    painter.setPen(QPen(QColor(210, 225, 255, 240), 1.5));
    painter.drawPolygon(triangle);
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(point(0.0, 0.0), 3.0, 3.0);
  }

  // 도구 드래그 미리보기 (2D Pose Estimate / Goal / Multi)
  if (tool_dragging_) {
    QColor color(80, 235, 145, 240);
    if (tool_ == "goal") {
      color = QColor(255, 95, 205, 240);
    } else if (tool_ == "multi") {
      color = QColor(255, 210, 70, 240);
    }
    const double dx = tool_current_map_.x() - tool_start_map_.x();
    const double dy = tool_current_map_.y() - tool_start_map_.y();
    const double distance = std::hypot(dx, dy);
    const double yaw = distance > 0.03 ? std::atan2(dy, dx) : 0.0;
    const double length = std::max(0.6, distance);
    const QPointF start = to_screen(tool_start_map_.x(), tool_start_map_.y());
    const QPointF end = to_screen(tool_start_map_.x() + std::cos(yaw) * length,
                                  tool_start_map_.y() + std::sin(yaw) * length);
    painter.setPen(QPen(color, 3.0));
    painter.drawLine(start, end);
    const double head = std::min(0.04 * zoom_, 0.15 * length * zoom_);
    painter.drawLine(end, QPointF(end.x() - std::cos(yaw - 0.5) * head, end.y() + std::sin(yaw - 0.5) * head));
    painter.drawLine(end, QPointF(end.x() - std::cos(yaw + 0.5) * head, end.y() + std::sin(yaw + 0.5) * head));
    painter.setBrush(QColor(color.red(), color.green(), color.blue(), 60));
    painter.drawEllipse(start, 6.0, 6.0);
    painter.setBrush(Qt::NoBrush);
  }

  // HUD
  painter.setPen(QColor(220, 228, 240));
  QFont font("Menlo");
  font.setStyleHint(QFont::Monospace);
  font.setPixelSize(11);
  painter.setFont(font);
  const QString info = QString("map %1x%2  res %3 m  zoom %4")
                           .arg(image_.width())
                           .arg(image_.height())
                           .arg(resolution_, 0, 'f', 2)
                           .arg(zoom_, 0, 'f', 1);
  painter.fillRect(QRect(8, 8, painter.fontMetrics().horizontalAdvance(info) + 16, 24), QColor(8, 10, 14, 185));
  painter.drawText(16, 25, info);
}

void MapView2D::mousePressEvent(QMouseEvent* event) {
  setFocus(Qt::MouseFocusReason);
  if (event->button() == Qt::LeftButton && !tool_.empty()) {
    tool_dragging_ = true;
    tool_start_map_ = to_map(event->position());
    tool_current_map_ = tool_start_map_;
    update();
    return;
  }
  if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton) {
    dragging_ = true;
    last_mouse_ = event->position().toPoint();
    setCursor(Qt::ClosedHandCursor);
  }
}

void MapView2D::mouseMoveEvent(QMouseEvent* event) {
  if (tool_dragging_) {
    tool_current_map_ = to_map(event->position());
    update();
    return;
  }
  if (!dragging_) {
    return;
  }
  const QPoint current = event->position().toPoint();
  offset_x_ += current.x() - last_mouse_.x();
  offset_y_ += current.y() - last_mouse_.y();
  last_mouse_ = current;
  update();
}

void MapView2D::mouseReleaseEvent(QMouseEvent* event) {
  if (tool_dragging_) {
    tool_dragging_ = false;
    tool_current_map_ = to_map(event->position());
    const double dx = tool_current_map_.x() - tool_start_map_.x();
    const double dy = tool_current_map_.y() - tool_start_map_.y();
    const double yaw = (dx * dx + dy * dy > 1e-4) ? std::atan2(dy, dx) : 0.0;
    emit tool_pose(tool_start_map_.x(), tool_start_map_.y(), yaw);
    if (tool_ != "multi") {
      set_tool("");
    }
    update();
    return;
  }
  dragging_ = false;
  unsetCursor();
}

void MapView2D::keyPressEvent(QKeyEvent* event) {
  if (event->isAutoRepeat()) {
    event->accept();
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
      emit teleop_key(event->key(), true);
      event->accept();
      return;
    default:
      QWidget::keyPressEvent(event);
      return;
  }
}

void MapView2D::keyReleaseEvent(QKeyEvent* event) {
  if (event->isAutoRepeat()) {
    event->accept();
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
      emit teleop_key(event->key(), false);
      event->accept();
      return;
    default:
      QWidget::keyReleaseEvent(event);
      return;
  }
}

void MapView2D::focusOutEvent(QFocusEvent* event) {
  (void)event;
  emit teleop_focus_lost();
}

void MapView2D::wheelEvent(QWheelEvent* event) {
  if (image_.isNull()) {
    return;
  }
  const QPointF cursor = event->position();
  const double before_x = origin_x_ + (cursor.x() - offset_x_) / zoom_;
  const double before_y = origin_y_ + (offset_y_ - cursor.y()) / zoom_;
  const double factor = std::exp(event->angleDelta().y() / 600.0);
  zoom_ = std::clamp(zoom_ * factor, 1.0, 800.0);
  offset_x_ = cursor.x() - (before_x - origin_x_) * zoom_;
  offset_y_ = cursor.y() + (before_y - origin_y_) * zoom_;
  update();
}

}  // namespace krosc::viz
