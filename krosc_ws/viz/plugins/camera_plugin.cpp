#include "camera_plugin.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "krosc/viz/viz_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace krosc_ws::viz {

namespace {

QString size_text(const krosc::msgs::Image& image) {
  return QString("%1x%2").arg(image.width).arg(image.height);
}

}  // namespace

void CameraPlugin::initialize(krosc::viz::VizHost& host) {
  images_ = std::make_shared<Images>();
  auto* panel = new QWidget();
  auto* layout = new QVBoxLayout(panel);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* grid = new QGridLayout();
  grid->setHorizontalSpacing(8);
  grid->setVerticalSpacing(6);

  const auto add_pane = [&](const QString& title, QLabel*& image_label,
                            QLabel*& info_label, int column) {
    auto* box = new QVBoxLayout();
    auto* header = new QLabel(title, panel);
    header->setStyleSheet("color:#e8eef8;font-weight:600;");
    info_label = new QLabel("대기", panel);
    info_label->setStyleSheet("color:#8e9bb0;font-variant-numeric:tabular-nums;");
    auto* row = new QHBoxLayout();
    row->addWidget(header);
    row->addStretch(1);
    row->addWidget(info_label);
    box->addLayout(row);
    image_label = new QLabel(panel);
    image_label->setMinimumSize(320, 240);
    image_label->setAlignment(Qt::AlignCenter);
    image_label->setStyleSheet("background:#000;border:1px solid #263041;color:#667286;");
    image_label->setText("영상 없음");
    box->addWidget(image_label, 1);
    grid->addLayout(box, 0, column);
  };
  add_pane("Color", color_label_, color_info_, 0);
  add_pane("Depth", depth_label_, depth_info_, 1);
  layout->addLayout(grid, 1);

  host.add_tab(panel, "Camera", false);

  color_subscription_ = host.model().node().subscribe<krosc::msgs::Image>(
      "camera/color/image_raw",
      [images = images_](const krosc::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(images->mutex);
        images->color = message;
        images->color_seen = true;
      },
      krosc::QOS_SENSOR_DATA);
  depth_subscription_ = host.model().node().subscribe<krosc::msgs::Image>(
      "camera/depth/image_rect_raw",
      [images = images_](const krosc::msgs::Image& message) {
        std::lock_guard<std::mutex> lock(images->mutex);
        images->depth = message;
        images->depth_seen = true;
      },
      krosc::QOS_SENSOR_DATA);

  timer_ = new QTimer(panel);
  QObject::connect(timer_, &QTimer::timeout, panel, [this] { render(); });
  timer_->start(100);
}

void CameraPlugin::shutdown() {
  if (timer_ != nullptr) {
    timer_->stop();
  }
  color_subscription_.cancel();
  depth_subscription_.cancel();
}

QImage CameraPlugin::to_image(const krosc::msgs::Image& image) {
  if (image.data.empty() || image.width == 0 || image.height == 0) {
    return {};
  }
  if (image.encoding == "jpeg" || image.encoding == "jpg" || image.encoding == "png") {
    QImage decoded;
    if (decoded.loadFromData(image.data.data(), static_cast<int>(image.data.size()))) {
      return decoded;
    }
    return {};
  }
  const int width = static_cast<int>(image.width);
  const int height = static_cast<int>(image.height);
  const int step = image.step > 0 ? static_cast<int>(image.step) : width * 3;
  if (image.encoding == "rgb8" || image.encoding == "bgr8") {
    const QImage::Format format =
        image.encoding == "rgb8" ? QImage::Format_RGB888 : QImage::Format_BGR888;
    return QImage(image.data.data(), width, height, step, format).copy();
  }
  return {};
}

QImage CameraPlugin::depth_image(const krosc::msgs::Image& image) {
  if (image.data.empty() || image.width == 0 || image.height == 0 ||
      image.encoding != "16UC1") {
    return {};
  }
  const int width = static_cast<int>(image.width);
  const int height = static_cast<int>(image.height);
  const int step = image.step > 0 ? static_cast<int>(image.step) : width * 2;
  QImage result(width, height, QImage::Format_RGB32);
  for (int y = 0; y < height; ++y) {
    const auto* row = reinterpret_cast<const std::uint8_t*>(image.data.data()) +
                      static_cast<std::size_t>(y) * step;
    for (int x = 0; x < width; ++x) {
      const std::uint16_t mm = static_cast<std::uint16_t>(row[x * 2]) |
                               (static_cast<std::uint16_t>(row[x * 2 + 1]) << 8);
      // 웹과 같은 LUT: 0.2m(파랑) ~ 3.0m(빨강). 0은 invalid.
      int red = 0;
      int green = 0;
      int blue = 0;
      if (mm > 0) {
        const double meters = static_cast<double>(mm) / 1000.0;
        const double normalized =
            std::clamp((meters - 0.2) / 2.8, 0.0, 1.0);
        const auto channel = [normalized](double offset) {
          return static_cast<int>(std::clamp(1.5 - std::abs(4.0 * normalized - offset),
                                             0.0, 1.0) * 255.0);
        };
        red = channel(3.0);
        green = channel(2.0);
        blue = channel(1.0);
      }
      result.setPixel(x, y, qRgb(red, green, blue));
    }
  }
  return result;
}

void CameraPlugin::render() {
  if (!images_) {
    return;
  }
  krosc::msgs::Image color;
  krosc::msgs::Image depth;
  bool color_seen = false;
  bool depth_seen = false;
  {
    std::lock_guard<std::mutex> lock(images_->mutex);
    color = images_->color;
    depth = images_->depth;
    color_seen = images_->color_seen;
    depth_seen = images_->depth_seen;
  }

  if (color_seen && color_label_ != nullptr) {
    const QImage image = to_image(color);
    if (!image.isNull()) {
      const QSize target = color_label_->size();
      color_label_->setPixmap(QPixmap::fromImage(image).scaled(
          target, Qt::KeepAspectRatio, Qt::SmoothTransformation));
      color_info_->setText(QString("%1  %2  %3 KB")
                               .arg(size_text(color))
                               .arg(QString::fromStdString(color.encoding))
                               .arg(color.data.size() / 1024));
    }
  }
  if (depth_seen && depth_label_ != nullptr) {
    const QImage image = depth_image(depth);
    if (!image.isNull()) {
      const QSize target = depth_label_->size();
      depth_label_->setPixmap(QPixmap::fromImage(image).scaled(
          target, Qt::KeepAspectRatio, Qt::SmoothTransformation));
      depth_info_->setText(QString("%1  %2  %3 KB")
                               .arg(size_text(depth))
                               .arg(QString::fromStdString(depth.encoding))
                               .arg(depth.data.size() / 1024));
    }
  }
}

}  // namespace krosc_ws::viz
