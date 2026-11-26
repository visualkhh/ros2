#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "hardwares/opencr/opencr_device.hpp"
#include "krosc/msgs.hpp"
#include "krosc/simulator/robot.hpp"
#include "krosc/simulator/world.hpp"

namespace krosc_ws::sim {

class UrdfRobotVisualizer;

// krosc_ws 실기 모델: TurtleBot3 Waffle Pi 차동 구동 + OpenMANIPULATOR-X 4DOF/gripper.
// generic 시뮬레이터 로봇(TurtleBot)을 상속하지 않고 SimulatorRobot을 직접 구현한다.
class WafflePiArm final : public krosc::simulator::SimulatorRobot {
 public:
  explicit WafflePiArm(bool noisy = false, std::optional<std::uint32_t> seed = std::nullopt,
                       std::string urdf_path = {});
  ~WafflePiArm() override;

  [[nodiscard]] std::string kind() const override { return "waffle_pi_arm"; }

  void set_command(const krosc::msgs::Twist& command);
  [[nodiscard]] krosc::msgs::Twist command() const;
  void tick(double time, double dt) override;
  [[nodiscard]] krosc::simulator::RobotState state() const override;

  [[nodiscard]] std::map<std::string, double> joint_positions() const;
  [[nodiscard]] double left_position() const;
  [[nodiscard]] double right_position() const;
  [[nodiscard]] double left_velocity() const;
  [[nodiscard]] double right_velocity() const;
  [[nodiscard]] krosc::msgs::Pose2D scan_pose() const;

  void set_scan_callback(std::function<void(const krosc::msgs::LaserScan&)> callback);
  void set_imu_callback(std::function<void(const krosc::msgs::Imu&)> callback);
  // 시뮬레이터 카메라가 월드 레이캐스트를 같은 시뮬레이션 스레드에서 하도록
  // pose가 갱신된 뒤 호출한다.
  void set_camera_callback(std::function<void(double)> callback);

  // 수동 조작(패널)과 MoveIt/컨트롤러 명령의 우선순위를 전환한다.
  void set_manual_arm(bool manual);
  [[nodiscard]] bool manual_arm() const;
  void set_arm_target(const std::array<double, 5>& target);            // 수동
  void set_controller_arm_target(const std::array<double, 5>& target); // SimulatorOpenCr write_arm
  [[nodiscard]] ArmStatus arm_status() const;
  [[nodiscard]] std::string noise_summary() const;

  static constexpr std::array<double, 5> kHome{0.5, -1.8, 1.4, 0.8, 0.0};
  static constexpr std::array<double, 5> kStow{0.0, -1.9024, 1.3440, 0.9023, 0.0};

  static constexpr double kWheelRadius = 0.033;
  static constexpr double kWheelSeparation = 0.288;
  static constexpr double kBodyRadius = 0.22;
  static constexpr double kBodyHeight = 0.141;
  static constexpr double kLidarMountX = -0.064;
  static constexpr double kLidarMountHeight = 0.132;
  static constexpr int kScanBeams = 360;
  static constexpr double kScanPeriod = 0.1;
  static constexpr double kScanRangeMin = 0.12;
  static constexpr double kScanRangeMax = 3.5;
  static constexpr double kImuPeriod = 0.01;
  static constexpr double kCameraPeriod = 0.1;  // 10Hz (웹 표시용)

 private:
  struct Noise {
    double sigma = 0.0;
    double bias = 0.0;
    double scale = 0.0;

    [[nodiscard]] double apply(double value, std::mt19937& rng) const {
      if (sigma == 0.0 && bias == 0.0 && scale == 0.0) {
        return value;
      }
      std::normal_distribution<double> normal(0.0, 1.0);
      return value * (1.0 + scale) + bias + normal(rng) * sigma;
    }

    [[nodiscard]] double variance() const { return sigma * sigma + bias * bias; }
  };

  void apply_motion(double dt, double linear, double angular);
  void sample_scan(double time);
  void sample_imu(double time, double imu_dt);
  static void step_towards(std::array<double, 5>& position,
                           const std::array<double, 5>& target, double dt);

  mutable std::mutex command_mutex_;
  mutable std::mutex state_mutex_;
  krosc::msgs::Twist command_;
  bool noisy_ = false;
  std::mt19937 rng_;
  Noise left_slip_{0.012, 0.0, -0.015};
  Noise right_slip_{0.012, 0.0, 0.018};

  double left_angle_ = 0.0;
  double right_angle_ = 0.0;
  double left_position_ = 0.0;
  double right_position_ = 0.0;
  double left_velocity_ = 0.0;
  double right_velocity_ = 0.0;
  double imu_accum_ = 0.0;
  double scan_accum_ = 0.0;
  double camera_accum_ = 0.0;
  double last_linear_velocity_ = 0.0;
  double last_angular_velocity_ = 0.0;
  double last_imu_linear_velocity_ = 0.0;

  std::function<void(const krosc::msgs::LaserScan&)> scan_callback_;
  std::function<void(const krosc::msgs::Imu&)> imu_callback_;
  std::function<void(double)> camera_callback_;
  std::vector<float> truth_scan_ranges_;
  krosc::msgs::Pose2D truth_scan_pose_;

  std::unique_ptr<UrdfRobotVisualizer> visualizer_;
  mutable std::mutex arm_mutex_;
  bool manual_ = true;
  std::array<double, 5> manual_position_{};
  std::array<double, 5> manual_target_{};
  std::array<double, 5> controller_position_{};
  std::array<double, 5> controller_target_{};
};

}  // namespace krosc_ws::sim
