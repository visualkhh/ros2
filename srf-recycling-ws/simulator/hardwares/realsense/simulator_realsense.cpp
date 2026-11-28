#include "hardwares/realsense/simulator_realsense.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

#include "world/recycling_world.hpp"

namespace srf_recycling_ws::sim {

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr double kHorizontalFov = 69.0 * kPi / 180.0;  // D435 color

double seconds_since(Clock::time_point time) {
  return std::chrono::duration<double>(Clock::now() - time).count();
}

}  // namespace

SimulatorRealSense::SimulatorRealSense(WafflePiArm& robot, const std::string& urdf_path, bool noisy,
                                       std::uint32_t seed, std::uint32_t width, std::uint32_t height)
    : robot_(robot), noisy_(noisy), width_(width < 32 ? 32 : width),
      height_(height < 24 ? 24 : height), rng_(seed),
      robot_mesh_(std::make_unique<render::RobotMesh>(urdf_path)) {
  robot_mesh_->hide_link("end_effector_link");  // URDF 표시용 빨간 상자 (실물 카메라에는 없다)
  robot_.set_camera_callback([this](double time) { render(time); });
}

SimulatorRealSense::~SimulatorRealSense() { robot_.set_camera_callback({}); }

void SimulatorRealSense::start() { running_.store(true); }

void SimulatorRealSense::stop() { running_.store(false); }

// 시뮬레이션 스레드에서 불린다 (월드를 읽어도 안전하다).
void SimulatorRealSense::render(double time) {
  (void)time;
  if (!running_.load()) {
    return;
  }
  const auto* world = robot_.world;
  if (world == nullptr) {
    return;
  }
  // 장면: 재활용 월드면 방·매트·수거함·물품을 그대로, 아니면 벽은 상자·물체는 원기둥으로.
  instances_.clear();
  if (const auto* recycling = dynamic_cast<const RecyclingWorld*>(world)) {
    recycling->add_render_instances(instances_);
  } else {
    for (const auto& wall : world->walls()) {
      instances_.push_back({render::make_box(wall.half_x * 2, wall.half_y * 2, wall.height),
                            render::Pose::translation(wall.center_x, wall.center_y, wall.height / 2),
                            {0.85F, 0.84F, 0.80F}});
    }
    instances_.push_back({render::make_box(20.0, 20.0, 0.01), render::Pose::translation(0, 0, -0.005),
                          {0.66F, 0.66F, 0.68F}});
    for (const auto& object : world->objects()) {
      if (object.get() == &robot_ || object->radius <= 0.0 || object->height <= 0.0) {
        continue;
      }
      instances_.push_back({render::make_cylinder(object->radius, object->height),
                            render::Pose::translation(object->pose.x, object->pose.y, object->z + object->height / 2),
                            {static_cast<float>(object->color[0]), static_cast<float>(object->color[1]),
                             static_cast<float>(object->color[2])}});
    }
  }
  // 로봇 자신(URDF 메시): 팔·그리퍼가 카메라에 보인다.
  const auto joints = robot_.joint_positions();
  robot_mesh_->add_instances(robot_.pose, joints, instances_);

  render::Camera camera;
  camera.width = static_cast<int>(width_);
  camera.height = static_cast<int>(height_);
  camera.set_fov(kHorizontalFov);
  camera.world_from_optical = robot_mesh_->link_pose("camera_color_optical_frame", robot_.pose, joints);
  render::render(camera, instances_, lighting_, frame_);

  RealSenseImage color;
  color.width = width_;
  color.height = height_;
  color.encoding = "rgb8";
  color.step = width_ * 3;
  color.data = frame_.rgb;
  RealSenseImage depth;
  depth.width = width_;
  depth.height = height_;
  depth.encoding = "16UC1";
  depth.step = width_ * 2;
  depth.data.resize(frame_.depth_mm.size() * 2);
  std::normal_distribution<double> normal(0.0, 1.0);
  for (std::size_t i = 0; i < frame_.depth_mm.size(); ++i) {
    double millimeters = frame_.depth_mm[i];
    if (millimeters > 0.0 && noisy_) {
      millimeters = std::clamp(millimeters + normal(rng_) * depth_sigma_ * 1000.0, 1.0, 65535.0);
    }
    const auto mm = static_cast<std::uint16_t>(millimeters);
    depth.data[i * 2 + 0] = static_cast<std::uint8_t>(mm & 0xFFu);  // 16UC1 little-endian
    depth.data[i * 2 + 1] = static_cast<std::uint8_t>((mm >> 8) & 0xFFu);
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

}  // namespace srf_recycling_ws::sim
