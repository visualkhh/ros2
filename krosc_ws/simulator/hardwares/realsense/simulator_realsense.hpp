#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <random>
#include <string>

#include "hardwares/realsense/realsense_camera.hpp"
#include "robot/waffle_pi_arm.hpp"

namespace krosc_ws::sim {

// 시뮬레이터의 D435 스텁. WafflePiArm의 pose와 World의 벽/퍽을 2.5D 레이캐스트해서
// color(RGB8)/depth(16UC1)를 만든다. 실기와 같은 RealSenseCamera 인터페이스를 쓴다.
// 렌더는 WafflePiArm::tick의 camera callback(시뮬레이션 스레드)에서 한다.
class SimulatorRealSense final : public RealSenseCamera {
 public:
  explicit SimulatorRealSense(WafflePiArm& robot, bool noisy = true,
                              std::uint32_t seed = 42, std::uint32_t width = 160,
                              std::uint32_t height = 120);
  ~SimulatorRealSense() override;

  SimulatorRealSense(const SimulatorRealSense&) = delete;
  SimulatorRealSense& operator=(const SimulatorRealSense&) = delete;

  void start() override;
  void stop() override;
  [[nodiscard]] bool connected() const override { return running_.load(); }

  [[nodiscard]] std::optional<RealSenseImage> take_color() override;
  [[nodiscard]] std::optional<RealSenseImage> take_depth() override;
  [[nodiscard]] RealSenseStats stats() const override;

  // Qt 패널용. Sensor가 가져간 프레임과 무관하게 최신 프레임을 복사해서 준다.
  [[nodiscard]] std::optional<RealSenseImage> snapshot_color() const;
  [[nodiscard]] std::optional<RealSenseImage> snapshot_depth() const;
  [[nodiscard]] std::string noise_summary() const;

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
  double dropout_ = 0.0;
};

}  // namespace krosc_ws::sim
