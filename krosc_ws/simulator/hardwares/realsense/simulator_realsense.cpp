#include "hardwares/realsense/simulator_realsense.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

#include "krosc/tf/math.hpp"

namespace krosc_ws::sim {

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr double kCameraMountX = 0.1167547;  // camera_color_optical_frame
constexpr double kCameraMountY = 0.0325;
constexpr double kCameraMountZ = 0.0775;
constexpr double kHorizontalFov = 69.0 * kPi / 180.0;

double seconds_since(Clock::time_point time) {
  return std::chrono::duration<double>(Clock::now() - time).count();
}

std::uint8_t clamp_byte(double value) {
  return static_cast<std::uint8_t>(std::clamp(value, 0.0, 255.0));
}

// 로봇 visual 상자(로봇 yaw로 회전)와 2D ray의 교차. t는 카메라 평면 거리.
bool ray_box_2d(double cx, double cy, double yaw, double size_x, double size_y,
                double x0, double y0, double dx, double dy, double& t) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double origin_x = c * (x0 - cx) + s * (y0 - cy);
  const double origin_y = -s * (x0 - cx) + c * (y0 - cy);
  const double ray_x = c * dx + s * dy;
  const double ray_y = -s * dx + c * dy;
  const double half_x = size_x * 0.5;
  const double half_y = size_y * 0.5;
  double t0 = 0.0;
  double t1 = std::numeric_limits<double>::infinity();
  const double origins[2] = {origin_x, origin_y};
  const double directions[2] = {ray_x, ray_y};
  const double lows[2] = {-half_x, -half_y};
  const double highs[2] = {half_x, half_y};
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(directions[axis]) < 1e-12) {
      if (origins[axis] < lows[axis] || origins[axis] > highs[axis]) {
        return false;
      }
      continue;
    }
    double near = (lows[axis] - origins[axis]) / directions[axis];
    double far = (highs[axis] - origins[axis]) / directions[axis];
    if (near > far) {
      std::swap(near, far);
    }
    t0 = std::max(t0, near);
    t1 = std::min(t1, far);
    if (t0 > t1) {
      return false;
    }
  }
  if (t1 < 0.0) {
    return false;
  }
  t = t0 > 1e-9 ? t0 : t1;
  return t > 0.0;
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
  const double camera_x = pose.x + kCameraMountX * std::cos(pose.theta) -
                          kCameraMountY * std::sin(pose.theta);
  const double camera_y = pose.y + kCameraMountX * std::sin(pose.theta) +
                          kCameraMountY * std::cos(pose.theta);
  const double camera_z = kCameraMountZ;

  // 로봇 자신의 팔/그리퍼도 카메라에 보여야 한다. World의 퍽/벽만 그리면
  // D435에는 팔이 사라진다.
  const krosc::simulator::RobotState robot_state = robot_.state();

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

  // 카메라가 자기 몸체(베이스/상판/마운트) 안에 있으면 그 면이 화면을 가득
  // 채운다. 카메라 원점을 품는 visual은 렌더 목록에서 뺀다. 팔/그리퍼는
  // 카메라 밖에 있으므로 그대로 남는다.
  std::vector<const krosc::simulator::RobotVisual*> camera_visuals;
  camera_visuals.reserve(robot_state.visuals.size());
  for (const auto& visual : robot_state.visuals) {
    const double vx = visual.transform.translation.x;
    const double vy = visual.transform.translation.y;
    bool inside = false;
    if (visual.type == krosc::simulator::RobotVisualType::Cylinder) {
      inside = visual.radius > 0.0 &&
               std::hypot(camera_x - vx, camera_y - vy) < visual.radius;
    } else {
      const double yaw = krosc::tfmath::yaw_of(visual.transform.rotation);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      const double local_x = c * (camera_x - vx) + s * (camera_y - vy);
      const double local_y = -s * (camera_x - vx) + c * (camera_y - vy);
      inside = std::abs(local_x) < visual.size[0] * 0.5 &&
               std::abs(local_y) < visual.size[1] * 0.5;
    }
    if (!inside) {
      camera_visuals.push_back(&visual);
    }
  }

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
      const krosc::simulator::RobotVisual* hit_visual = nullptr;
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
        if (z < object->z || z > object->z + object->height) {
          continue;
        }
        best = t;
        hit_wall = false;
        hit_object = object.get();
        hit_visual = nullptr;
        hit_floor = false;
        hit_height = object->z + object->height;
      }
      // 로봇 링크(팔/그리퍼/바퀴)도 같은 2.5D 레이로 그린다.
      for (const auto* visual_ptr : camera_visuals) {
        const auto& visual = *visual_ptr;
        const double vx = visual.transform.translation.x;
        const double vy = visual.transform.translation.y;
        const double vz = visual.transform.translation.z;
        double t = 0.0;
        double half_z = 0.0;
        bool hit = false;
        if (visual.type == krosc::simulator::RobotVisualType::Cylinder) {
          if (visual.radius <= 0.0 || visual.length <= 0.0) {
            continue;
          }
          t = krosc::simulator::ray_hit_circle(vx, vy, visual.radius, camera_x,
                                               camera_y, dx, dy);
          hit = std::isfinite(t) && t > 0.0;
          half_z = visual.length * 0.5;
        } else {
          const double yaw = krosc::tfmath::yaw_of(visual.transform.rotation);
          hit = ray_box_2d(vx, vy, yaw, visual.size[0], visual.size[1], camera_x,
                           camera_y, dx, dy, t);
          half_z = visual.size[2] * 0.5;
        }
        if (!hit || t >= best) {
          continue;
        }
        const double z = camera_z + t * tan_v;
        if (z < vz - half_z || z > vz + half_z) {
          continue;
        }
        best = t;
        hit_wall = false;
        hit_object = nullptr;
        hit_visual = &visual;
        hit_floor = false;
        hit_height = vz + half_z;
      }
      // 아래를 보는 ray는 바닥(z=0)과 만난다. 바닥이 없으면 color가 검게 보여
      // 시뮬레이터 카메라가 고장난 것처럼 보인다.
      if (tan_v < -1e-6) {
        const double floor_distance = camera_z / (-tan_v);
        if (std::isfinite(floor_distance) && floor_distance > 0.0 && floor_distance < best) {
          best = floor_distance;
          hit_wall = false;
          hit_object = nullptr;
          hit_visual = nullptr;
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
        } else if (hit_visual != nullptr) {
          red = hit_visual->color[0] * 255.0 * light;
          green = hit_visual->color[1] * 255.0 * light;
          blue = hit_visual->color[2] * 255.0 * light;
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
