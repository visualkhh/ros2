#pragma once

#include <memory>
#include <string>

#include "realsense_camera.hpp"

namespace krosc_ws {

// Intel RealSense D435. UVC 비디오 스트림을 librealsense2로 읽어
// color는 JPEG, depth는 16UC1 raw로 최신 프레임만 보관한다.
// 토픽 발행/프레임/진단은 Sensor 레이어(RealSenseSensor)가 담당한다.
class RealSenseDevice final : public RealSenseCamera {
 public:
  struct Options {
    // D435가 안정적으로 지원하는 스트림은 640x480이다.
    // 발행 전에 color/depth를 decimation해서 Pi 대역폭과 JPEG 비용을 줄인다.
    int width = 640;
    int height = 480;
    int fps = 15;
    int jpeg_quality = 70;
    int color_decimation = 2;  // 640x480 -> 320x240 JPEG
    int depth_decimation = 4;  // 640x480 -> 160x120 16UC1
    bool enable_color = true;
    bool enable_depth = true;
    // 비우면 첫 장치를 쓴다. 여러 대일 때만 지정한다.
    std::string serial;
  };

  RealSenseDevice();
  explicit RealSenseDevice(Options options);
  ~RealSenseDevice() override;

  RealSenseDevice(const RealSenseDevice&) = delete;
  RealSenseDevice& operator=(const RealSenseDevice&) = delete;

  void start() override;
  void stop() override;
  void connect() override;
  void disconnect() override;
  [[nodiscard]] bool is_connected() const override;

  [[nodiscard]] std::optional<RealSenseImage> take_color() override;
  [[nodiscard]] std::optional<RealSenseImage> take_depth() override;
  [[nodiscard]] RealSenseStats stats() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace krosc_ws
