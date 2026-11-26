#include "hardwares/realsense/simulator_realsense.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

namespace krosc_ws::sim {

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr double kCameraMountX = 0.085;
constexpr double kCameraMountZ = 0.108;
constexpr double kHorizontalFov = 69.0 * kPi / 180.0;

double seconds_since(Clock::time_point time) {
  return std::chrono::duration<double>(Clock::now() - time).count();
}

std::uint8_t clamp_byte(double value) {
  return static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
}

}  // namespace

SimulatorRealSense::SimulatorRealSense(WafflePiArm& robot, bool noisy, std::uint32_t seed,
                                       std::uint32_t width, std::uint32_t height)
    : robot_(robot), noisy_(noisy), width_(width < 32 ? 32 : width),
      height_(height < 24 ? 24 : height), rng_(seed) {
  robot_.set_camera_callback([this](double time) { render(time); });
}

SimulatorRealSense::~SimulatorRealSense() { robot_.set_camera_callback({}); }

void SimulatorRealSense::start() { running_.store(true); }

void SimulatorRealSense::stop() { running_.store(false); }

void SimulatorRealSense::render(double time) {
  (void)time;
  if (!running_.load()) {
    return;
  }
  const auto* world = robot_.world;
  if (world == nullptr) {
    return;
  }
  const auto& pose = robot_.pose;
  const double camera_x = pose.x + kCameraMountX * std::cos(pose.theta);
  const double camera_y = pose.y + kCameraMountX * std::sin(pose.theta);
  const double camera_z = kCameraMountZ;

  const int width = static_cast<int>(width_);
  const int height = static_cast<int>(height_);
  const double tan_half_h = std::tan(kHorizontalFov * 0.5);
  const double tan_half_v = tan_half_h * static_cast<double>(height) / static_cast<double>(width);

  RealSenseImage color;
  color.width = width_;
  color.height = height_;
  color.encoding = "rgb8";
  color.step = width_ * 3;
  color.data.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);

  RealSenseImage depth;
  depth.width = width_;
  depth.height = height_;
  depth.encoding = "16UC1";
  depth.step = width_ * 2;
  depth.data.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 2);

  std::normal_distribution<double> normal(0.0, 1.0);
  for (int y = 0; y < height; ++y) {
    const double ndc_y = 1.0 - (static_cast<double>(y) + 0.5) * 2.0 / static_cast<double>(height);
    const double vertical = std::atan(ndc_y * tan_half_v);
    const double tan_v = std::tan(vertical);
    for (int x = 0; x < width; ++x) {
      const double ndc_x = (static_cast<double>(x) + 0.5) * 2.0 / static_cast<double>(width) - 1.0;
      const double horizontal = std::atan(ndc_x * tan_half_h);
      // optical frame: +x 오른쪽, +y 아래, +z 전방. 월드 yaw는 왼쪽이 +다.
      const double angle = pose.theta - horizontal;
      const double dx = std::cos(angle);
      const double dy = std::sin(angle);

      double best = std::numeric_limits<double>::infinity();
      const krosc::simulator::WorldObject* hit_object = nullptr;
      bool hit_wall = false;
      bool hit_floor = false;
      double hit_height = 0.0;

      for (const auto& wall : world->walls()) {
        const double t = krosc::simulator::ray_hit_box(wall, camera_x, camera_y, dx, dy);
        if (!std::isfinite(t) || t >= best) {
          continue;
        }
        const double z = camera_z + t * tan_v;
        if (z < 0.0 || z > wall.height) {
          continue;
        }
        best = t;
        hit_wall = true;
        hit_object = nullptr;
        hit_height = wall.height;
      }
      for (const auto& object : world->objects()) {
        if (object->radius <= 0.0 || object->height <= 0.0) {
          continue;
        }
        const double t = krosc::simulator::ray_hit_circle(
            object->pose.x, object->pose.y, object->radius, camera_x, camera_y, dx, dy);
        if (!std::isfinite(t) || t >= best) {
          continue;
        }
        const double z = camera_z + t * tan_v;
        if (z < 0.0 || z > object->height) {
          continue;
        }
        best = t;
        hit_wall = false;
        hit_object = object.get();
        hit_floor = false;
        hit_height = object->height;
      }
      // 아래를 보는 ray는 바닥(z=0)과 만난다. 바닥이 없으면 color가 검게 보여
      // 시뮬레이터 카메라가 고장난 것처럼 보인다.
      if (tan_v < -1e-6) {
        const double floor_distance = camera_z / (-tan_v);
        if (std::isfinite(floor_distance) && floor_distance > 0.0 && floor_distance < best) {
          best = floor_distance;
          hit_wall = false;
          hit_object = nullptr;
          hit_floor = true;
          hit_height = 0.0;
        }
      }

      double red = 0.0;
      double green = 0.0;
      double blue = 0.0;
      if (!std::isfinite(best)) {
        // 위쪽 하늘. 아래에서 위로 갈수록 밝아진다.
        const double up = (ndc_y + 1.0) * 0.5;
        red = 34.0 + 30.0 * up;
        green = 40.0 + 34.0 * up;
        blue = 54.0 + 42.0 * up;
      } else if (hit_floor) {
        const double floor_x = camera_x + best * dx;
        const double floor_y = camera_y + best * dy;
        const bool checker =
            ((static_cast<int>(std::floor(floor_x / 0.25)) +
              static_cast<int>(std::floor(floor_y / 0.25))) & 1) != 0;
        const double base = checker ? 86.0 : 64.0;
        const double shade = 1.0 / (1.0 + 0.10 * best * best);
        red = base * shade;
        green = base * shade;
        blue = (base + 8.0) * shade;
      } else {
        const double shade = 1.0 / (1.0 + 0.18 * best * best);
        const double vertical_shade =
            hit_height > 0.05
                ? 0.70 + 0.30 * std::clamp((camera_z + best * tan_v) / hit_height, 0.0, 1.0)
                : 1.0;
        const double light = shade * vertical_shade;
        if (hit_wall) {
          red = 178.0 * light;
          green = 186.0 * light;
          blue = 198.0 * light;
        } else if (hit_object != nullptr) {
          red = hit_object->color[0] * 255.0 * light;
          green = hit_object->color[1] * 255.0 * light;
          blue = hit_object->color[2] * 255.0 * light;
        }
      }

      const std::size_t pixel = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                                static_cast<std::size_t>(x);
      color.data[pixel * 3 + 0] = clamp_byte(red);
      color.data[pixel * 3 + 1] = clamp_byte(green);
      color.data[pixel * 3 + 2] = clamp_byte(blue);

      double millimeters = std::isfinite(best) ? best * 1000.0 : 0.0;
      if (millimeters > 0.0 && noisy_) {
        millimeters += normal(rng_) * depth_sigma_ * 1000.0;
      }
      if (!std::isfinite(millimeters) || millimeters < 1.0) {
        millimeters = 0.0;
      }
      millimeters = std::min(millimeters, 65535.0);
      const auto mm = static_cast<std::uint16_t>(millimeters);
      depth.data[pixel * 2 + 0] = static_cast<std::uint8_t>(mm & 0xFFu);
      depth.data[pixel * 2 + 1] = static_cast<std::uint8_t>((mm >> 8) & 0xFFu);
    }
  }

  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  color_ = std::move(color);
  depth_ = std::move(depth);
  ++color_seq_;
  ++depth_seq_;
  ++frames_;
  ++color_frames_;
  ++depth_frames_;
  color_time_ = now;
  depth_time_ = now;
}

std::optional<RealSenseImage> SimulatorRealSense::take_color() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (color_seq_ == color_taken_ || color_.data.empty()) {
    return std::nullopt;
  }
  color_taken_ = color_seq_;
  return color_;
}

std::optional<RealSenseImage> SimulatorRealSense::take_depth() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (depth_seq_ == depth_taken_ || depth_.data.empty()) {
    return std::nullopt;
  }
  depth_taken_ = depth_seq_;
  return depth_;
}

std::optional<RealSenseImage> SimulatorRealSense::snapshot_color() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (color_.data.empty()) {
    return std::nullopt;
  }
  return color_;
}

std::optional<RealSenseImage> SimulatorRealSense::snapshot_depth() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (depth_.data.empty()) {
    return std::nullopt;
  }
  return depth_;
}

RealSenseStats SimulatorRealSense::stats() const {
  RealSenseStats result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.running = running_.load();
  result.frames = frames_;
  result.color_frames = color_frames_;
  result.depth_frames = depth_frames_;
  result.color_age = color_frames_ > 0 ? seconds_since(color_time_) : 1e9;
  result.depth_age = depth_frames_ > 0 ? seconds_since(depth_time_) : 1e9;
  result.serial = "sim";
  result.firmware = "sim";
  result.product = "Simulated D435";
  return result;
}

std::string SimulatorRealSense::noise_summary() const {
  if (!noisy_) {
    return "off";
  }
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), "depth sigma=%.1fmm %ux%u@%.0fHz",
                depth_sigma_ * 1000.0, width_, height_, 1.0 / WafflePiArm::kCameraPeriod);
  return buffer;
}

}  // namespace krosc_ws::sim
