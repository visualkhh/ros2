#include "panels/camera_panel.hpp"

#include <QFont>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "hardwares/realsense/simulator_realsense.hpp"

namespace krosc_ws::sim {

namespace {

constexpr int kImageWidth = 256;
constexpr int kImageHeight = 192;

QRgb depth_color(std::uint16_t millimeters) {
  if (millimeters == 0) {
    return qRgb(0, 0, 0);
  }
  const double t = std::clamp((static_cast<double>(millimeters) / 1000.0 - 0.2) / 2.8, 0.0, 1.0);
  const double clamp = std::clamp(t, 0.0, 1.0);
  const auto red = static_cast<int>(std::clamp(1.5 - std::abs(4.0 * clamp - 3.0), 0.0, 1.0) * 255.0);
  const auto green = static_cast<int>(std::clamp(1.5 - std::abs(4.0 * clamp - 2.0), 0.0, 1.0) * 255.0);
  const auto blue = static_cast<int>(std::clamp(1.5 - std::abs(4.0 * clamp - 1.0), 0.0, 1.0) * 255.0);
  return qRgb(red, green, blue);
}

}  // namespace

CameraPanel::CameraPanel(SimulatorRealSense& camera, QWidget* parent)
    : QWidget(parent), camera_(camera) {
  setObjectName("camera-panel");
  setMinimumWidth(280);
  setMaximumWidth(360);
  setStyleSheet(
      "#camera-panel { background: #11161f; color: #dbe4f0; }"
      "QLabel { color: #b8c4d6; }"
      "QLabel#camera-image { background: #000; border: 1px solid #2a3648; }");

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(6);

  auto* title = new QLabel("REALSENSE D435 (SIM)", this);
  QFont title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);
  layout->addWidget(title);

  stats_ = new QLabel("waiting", this);
  stats_->setStyleSheet("color: #8e9bb0; font-size: 11px;");
  layout->addWidget(stats_);

  auto* color_title = new QLabel("color", this);
  color_title->setStyleSheet("color: #8e9bb0; font-size: 11px;");
  layout->addWidget(color_title);
  color_ = new QLabel("no frames", this);
  color_->setObjectName("camera-image");
  color_->setFixedSize(kImageWidth, kImageHeight);
  color_->setAlignment(Qt::AlignCenter);
  color_->setScaledContents(true);
  layout->addWidget(color_);

  auto* depth_title = new QLabel("depth  0.2m(파랑) → 3.0m(빨강)", this);
  depth_title->setStyleSheet("color: #8e9bb0; font-size: 11px;");
  layout->addWidget(depth_title);
  depth_ = new QLabel("no frames", this);
  depth_->setObjectName("camera-image");
  depth_->setFixedSize(kImageWidth, kImageHeight);
  depth_->setAlignment(Qt::AlignCenter);
  depth_->setScaledContents(true);
  layout->addWidget(depth_);
  layout->addStretch(1);

  timer_ = new QTimer(this);
  timer_->setInterval(100);
  connect(timer_, &QTimer::timeout, this, [this] { refresh(); });
  timer_->start();
  refresh();
}

void CameraPanel::refresh() {
  if (const auto frame = camera_.snapshot_color()) {
    if (frame->encoding == "rgb8" && frame->width > 0 && frame->height > 0) {
      const QImage image(reinterpret_cast<const uchar*>(frame->data.data()),
                         static_cast<int>(frame->width), static_cast<int>(frame->height),
                         static_cast<int>(frame->step), QImage::Format_RGB888);
      color_->setPixmap(QPixmap::fromImage(image.copy()));
    }
  }

  if (const auto frame = camera_.snapshot_depth()) {
    if (frame->encoding == "16UC1" && frame->width > 0 && frame->height > 0) {
      QImage image(static_cast<int>(frame->width), static_cast<int>(frame->height),
                   QImage::Format_RGB32);
      image.fill(Qt::black);
      for (std::uint32_t y = 0; y < frame->height; ++y) {
        auto* line = reinterpret_cast<QRgb*>(image.scanLine(static_cast<int>(y)));
        for (std::uint32_t x = 0; x < frame->width; ++x) {
          if ((x & 1u) != 0 || (y & 1u) != 0) {
            continue;  // 2x2 샘플 = particle
          }
          const std::size_t offset =
              static_cast<std::size_t>(y) * frame->step + static_cast<std::size_t>(x) * 2;
          const std::uint16_t millimeters =
              static_cast<std::uint16_t>(frame->data[offset]) |
              static_cast<std::uint16_t>(frame->data[offset + 1] << 8);
          const QRgb color = depth_color(millimeters);
          line[x] = color;
        }
      }
      depth_->setPixmap(QPixmap::fromImage(image));
    }
  }

  const RealSenseStats stats = camera_.stats();
  char text[160];
  std::snprintf(text, sizeof(text), "color %llu · depth %llu · age %.2fs / %.2fs",
                static_cast<unsigned long long>(stats.color_frames),
                static_cast<unsigned long long>(stats.depth_frames),
                stats.color_age, stats.depth_age);
  stats_->setText(QString::fromUtf8(text));
}

}  // namespace krosc_ws::sim
