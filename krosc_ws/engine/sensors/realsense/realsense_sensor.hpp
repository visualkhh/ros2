#pragma once

#include <cstdint>
#include <string>

#include "krosc/context.hpp"
#include "krosc/sensor/sensor.hpp"

namespace krosc_ws {

class RealSenseCamera;

// RealSense의 Sensor 논리 레이어.
// 물리 장치에서 최신 color/depth 프레임을 꺼내 Image 토픽으로 발행하고,
// 1초마다 diagnostics에 카메라 상태를 남긴다.
class RealSenseSensor final : public krosc::sensor::Sensor {
 public:
  RealSenseSensor(krosc::Context& context, RealSenseCamera& device,
                  std::string color_topic = "camera/color/image_raw",
                  std::string depth_topic = "camera/depth/image_rect_raw",
                  std::string color_frame = "camera_color_optical_frame",
                  std::string depth_frame = "camera_depth_optical_frame",
                  std::string diagnostics_topic = "diagnostics",
                  double depth_period = 0.2);

  [[nodiscard]] bool connected() const override;
  bool read(double time, double period) override;

  [[nodiscard]] std::uint64_t color_published() const { return color_published_; }
  [[nodiscard]] std::uint64_t depth_published() const { return depth_published_; }

 private:
  void publish_diagnostics(double time);
  void update_rates(double time);

  RealSenseCamera& device_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string color_frame_;
  std::string depth_frame_;
  std::string diagnostics_topic_;
  double depth_period_;
  double last_depth_ = -1.0;
  double last_diagnostics_ = -1.0;
  double last_rate_time_ = -1.0;
  std::uint64_t color_published_ = 0;
  std::uint64_t depth_published_ = 0;
  std::uint64_t rate_color_ = 0;
  std::uint64_t rate_depth_ = 0;
  double color_rate_ = 0.0;
  double depth_rate_ = 0.0;
};

}  // namespace krosc_ws
