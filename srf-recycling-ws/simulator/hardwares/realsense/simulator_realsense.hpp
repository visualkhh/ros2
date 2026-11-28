#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "hardwares/realsense/realsense_camera.hpp"
#include "render/scene_renderer.hpp"
#include "robot/waffle_pi_arm.hpp"

namespace srf_recycling_ws::sim {

// 시뮬레이터의 D435 스텁. URDF 로봇 메시(관절 FK)와 월드(재활용 월드면 방·수거함·물품)를
// 3D 소프트웨어 렌더러로 camera_color_optical_frame에서 그려 color(RGB8)/depth(16UC1, mm)를 만든다.
// 실기와 같은 RealSenseCamera 인터페이스를 쓴다.
// 렌더는 WafflePiArm::tick의 camera callback(시뮬레이션 스레드)에서 한다.
class SimulatorRealSense final : public RealSenseCamera {
 public:
  SimulatorRealSense(WafflePiArm& robot, const std::string& urdf_path, bool noisy = true,
                     std::uint32_t seed = 42, std::uint32_t width = 424, std::uint32_t height = 240);
  ~SimulatorRealSense() override;

  SimulatorRealSense(const SimulatorRealSense&) = delete;
  SimulatorRealSense& operator=(const SimulatorRealSense&) = delete;

  void start() override;
  void stop() override;
  void connect() override { start(); }
  void disconnect() override { stop(); }
  [[nodiscard]] bool is_connected() const override { return running_.load(); }

  [[nodiscard]] std::optional<RealSenseImage> take_color() override;
  [[nodiscard]] std::optional<RealSenseImage> take_depth() override;
  [[nodiscard]] RealSenseStats stats() const override;

  // Qt 패널용. Sensor가 가져간 프레임과 무관하게 최신 프레임을 복사해서 준다.
  [[nodiscard]] std::optional<RealSenseImage> snapshot_color() const;
  [[nodiscard]] std::optional<RealSenseImage> snapshot_depth() const;
  [[nodiscard]] std::string noise_summary() const;
  // 진실 화면이 같은 로봇 메시를 쓴다 (한 번만 읽는다).
  [[nodiscard]] const render::RobotMesh& robot_mesh() const { return *robot_mesh_; }

 private:
  void render(double time);

  WafflePiArm& robot_;
  const bool noisy_;
  const std::uint32_t width_;
  const std::uint32_t height_;
  std::mt19937 rng_;
  std::atomic<bool> running_{false};

  mutable std::mutex mutex_;
  RealSenseImage color_;
  RealSenseImage depth_;
  std::uint64_t color_seq_ = 0;
  std::uint64_t depth_seq_ = 0;
  std::uint64_t color_taken_ = 0;
  std::uint64_t depth_taken_ = 0;
  std::uint64_t frames_ = 0;
  std::uint64_t color_frames_ = 0;
  std::uint64_t depth_frames_ = 0;
  std::chrono::steady_clock::time_point color_time_{};
  std::chrono::steady_clock::time_point depth_time_{};

  double depth_sigma_ = 0.005;

  std::unique_ptr<render::RobotMesh> robot_mesh_;
  std::vector<render::Instance> instances_;
  render::Lighting lighting_;
  render::Frame frame_;
};

}  // namespace srf_recycling_ws::sim
