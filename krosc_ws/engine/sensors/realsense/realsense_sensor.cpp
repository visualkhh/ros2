#include "realsense_sensor.hpp"

#include <cstdio>
#include <utility>
#include <vector>

#include "hardwares/realsense/realsense_camera.hpp"
#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"

namespace krosc_ws {

namespace {

std::string fixed(double value, int digits) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.*f", digits, value);
  return text;
}

}  // namespace

RealSenseSensor::RealSenseSensor(krosc::Context& context, RealSenseCamera& device,
                                 std::string color_topic, std::string depth_topic,
                                 std::string color_frame, std::string depth_frame,
                                 std::string diagnostics_topic, double depth_period)
    : krosc::sensor::Sensor(context), device_(device), color_topic_(std::move(color_topic)),
      depth_topic_(std::move(depth_topic)), color_frame_(std::move(color_frame)),
      depth_frame_(std::move(depth_frame)), diagnostics_topic_(std::move(diagnostics_topic)),
      depth_period_(depth_period > 0.0 ? depth_period : 0.2) {}

bool RealSenseSensor::is_connected() const { return device_.is_connected(); }

bool RealSenseSensor::read(double time, double period) {
  (void)period;
  const double stamp = now();

  if (auto frame = device_.take_color()) {
    krosc::msgs::Image message;
    message.header = krosc::msgs::Header{stamp, color_frame_};
    message.width = frame->width;
    message.height = frame->height;
    message.encoding = std::move(frame->encoding);
    message.step = frame->step;
    message.data = std::move(frame->data);
    if (!color_topic_.empty()) {
      context().topics().publish<krosc::msgs::Image>(color_topic_, std::move(message),
                                                     krosc::QOS_SENSOR_DATA);
    }
    ++color_published_;
  }

  if (time - last_depth_ >= depth_period_ - 1e-9) {
    if (auto frame = device_.take_depth()) {
      krosc::msgs::Image message;
      message.header = krosc::msgs::Header{stamp, depth_frame_};
      message.width = frame->width;
      message.height = frame->height;
      message.encoding = std::move(frame->encoding);
      message.step = frame->step;
      message.data = std::move(frame->data);
      if (!depth_topic_.empty()) {
        context().topics().publish<krosc::msgs::Image>(depth_topic_, std::move(message),
                                                       krosc::QOS_SENSOR_DATA);
      }
      ++depth_published_;
    }
    last_depth_ = time;
  }

  update_rates(time);
  if (time - last_diagnostics_ >= 1.0) {
    publish_diagnostics(time);
    last_diagnostics_ = time;
  }
  return device_.is_connected();
}

void RealSenseSensor::update_rates(double time) {
  if (last_rate_time_ < 0.0) {
    last_rate_time_ = time;
    rate_color_ = color_published_;
    rate_depth_ = depth_published_;
    return;
  }
  const double elapsed = time - last_rate_time_;
  if (elapsed < 1.0) {
    return;
  }
  color_rate_ = static_cast<double>(color_published_ - rate_color_) / elapsed;
  depth_rate_ = static_cast<double>(depth_published_ - rate_depth_) / elapsed;
  rate_color_ = color_published_;
  rate_depth_ = depth_published_;
  last_rate_time_ = time;
}

void RealSenseSensor::publish_diagnostics(double time) {
  const RealSenseStats stats = device_.stats();
  if (stats.serial.empty() && !stats.running) {
    return;  // 카메라가 아예 없는 구성이면 diagnostics를 만들지 않는다.
  }
  krosc::msgs::DiagnosticStatus status;
  status.name = "RealSense D435";
  status.hardware_id = stats.serial;
  const bool fresh = stats.color_age <= 2.0 || stats.depth_age <= 2.0;
  if (!stats.running) {
    status.level = 2;
    status.message = "disconnected";
  } else if (!fresh) {
    status.level = 1;
    status.message = "no frames";
  } else {
    status.level = 0;
    status.message = "streaming";
  }
  status.values = {
      {"serial", stats.serial.empty() ? "-" : stats.serial},
      {"firmware", stats.firmware.empty() ? "-" : stats.firmware},
      {"frames", std::to_string(stats.frames)},
      {"color_frames", std::to_string(stats.color_frames)},
      {"depth_frames", std::to_string(stats.depth_frames)},
      {"color_fps", fixed(color_rate_, 1)},
      {"depth_fps", fixed(depth_rate_, 1)},
      {"color_age", fixed(stats.color_age, 2)},
      {"depth_age", fixed(stats.depth_age, 2)},
      {"color_topic", color_topic_},
      {"depth_topic", depth_topic_},
  };
  krosc::msgs::DiagnosticArray array;
  array.header = krosc::msgs::Header{now(), color_frame_};
  array.status.push_back(std::move(status));
  if (!diagnostics_topic_.empty()) {
    context().topics().publish<krosc::msgs::DiagnosticArray>(
        diagnostics_topic_, std::move(array), krosc::QOS_DEFAULT);
  }
  (void)time;
}

}  // namespace krosc_ws
