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
#include "srf/msgs.hpp"
#include "srf/simulator/robot.hpp"
#include "srf/simulator/world.hpp"

namespace srf_recycling_ws::sim {

class UrdfRobotVisualizer;
class ArmCollision;

// srf_recycling_ws 실기 모델: TurtleBot3 Waffle Pi 차동 구동 + OpenMANIPULATOR-X 4DOF/gripper.
// generic 시뮬레이터 로봇(TurtleBot)을 상속하지 않고 SimulatorRobot을 직접 구현한다.
class WafflePiArm final : public srf::simulator::SimulatorRobot {
 public:
  // srdf_path까지 주면 팔 충돌(자기 몸·카메라 마운트·바닥)을 검사해 닿는 관절은 멈춘다 (실기 서보처럼).
  explicit WafflePiArm(bool noisy = false, std::optional<std::uint32_t> seed = std::nullopt,
                       std::string urdf_path = {}, std::string srdf_path = {});
  ~WafflePiArm() override;

  [[nodiscard]] std::string kind() const override { return "waffle_pi_arm"; }

  void set_command(const srf::msgs::Twist& command);
  [[nodiscard]] srf::msgs::Twist command() const;
  void tick(double time, double dt) override;
  [[nodiscard]] srf::simulator::RobotState state() const override;

  [[nodiscard]] std::map<std::string, double> joint_positions() const;
  [[nodiscard]] double left_position() const;
  [[nodiscard]] double right_position() const;
  [[nodiscard]] double left_velocity() const;
  [[nodiscard]] double right_velocity() const;
  [[nodiscard]] srf::msgs::Pose2D scan_pose() const;

  void set_scan_callback(std::function<void(const srf::msgs::LaserScan&)> callback);
  void set_imu_callback(std::function<void(const srf::msgs::Imu&)> callback);
  // 시뮬레이터 카메라가 월드 레이캐스트를 같은 시뮬레이션 스레드에서 하도록
  // pose가 갱신된 뒤 호출한다.
  void set_camera_callback(std::function<void(double)> callback);

  // 수동 조작(패널)과 MoveIt/컨트롤러 명령의 우선순위를 전환한다.
  void set_manual_arm(bool manual);
  [[nodiscard]] bool manual_arm() const;
  void set_arm_target(const std::array<double, 5>& target);            // 수동
  void set_controller_arm_target(const std::array<double, 5>& target); // SimulatorOpenCr write_arm
  [[nodiscard]] ArmStatus arm_status() const;
  // 그리퍼가 물체를 쥐고 있나 (진실값: 시범 조교·채점용).
  [[nodiscard]] bool holding() const { return held_ != nullptr; }
  // 쥔 물체·상판 물체를 놓는다 (월드 물체를 지우기 전에 — randomize).
  void release_all();
  // 마지막 tick에서 팔을 막은 충돌 ("link5↔camera_link, ..."), 안 막혔으면 빈 문자열.
  [[nodiscard]] std::string arm_blocked_by() const;
  [[nodiscard]] std::string noise_summary() const;

  static constexpr std::array<double, 5> kHome{0.5, -1.75, 1.35, 0.8, 0.0};
  static constexpr std::array<double, 5> kStow{0.0, -1.75, 1.3440, 0.9023, 0.0};

  static constexpr double kWheelRadius = 0.033;
  static constexpr double kWheelSeparation = 0.288;
  // 바닥 퍽과의 물리 충돌은 실제 차체 상자(URDF base 0.266 x 0.266)의 내접원만 쓴다.
  // 0.22는 팔을 편 Nav2 footprint라 시뮬 물리에서 쓰면 퍽이 허공에서 밀린다.
  static constexpr double kBodyRadius = 0.133;
  static constexpr double kBodyHeight = 0.141;
  static constexpr double kLidarMountX = -0.024;
  static constexpr double kLidarMountHeight = 0.122;
  static constexpr int kScanBeams = 360;
  static constexpr double kScanPeriod = 0.1;
  static constexpr double kScanRangeMin = 0.12;
  static constexpr double kScanRangeMax = 3.5;
  static constexpr double kImuPeriod = 0.01;
  static constexpr double kCameraPeriod = 0.2;  // 5Hz (웹/Qt 표시용, 저해상도)

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

  struct Tcp {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
  };

  void apply_motion(double dt, double linear, double angular);
  void sample_scan(double time);
  void sample_imu(double time, double imu_dt);
  [[nodiscard]] Tcp tcp_pose(const std::array<double, 5>& joints) const;
  void update_grasp();
  void update_payload();
  static void step_towards(std::array<double, 5>& position,
                           const std::array<double, 5>& target, double dt);
  // position → next 로 옮기되 새로 닿는 자세는 건너뛴다 (arm_mutex_ 잡은 채로).
  void move_arm(std::array<double, 5>& position, const std::array<double, 5>& next);

  mutable std::mutex command_mutex_;
  mutable std::mutex state_mutex_;
  srf::msgs::Twist command_;
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

  std::function<void(const srf::msgs::LaserScan&)> scan_callback_;
  std::function<void(const srf::msgs::Imu&)> imu_callback_;
  std::function<void(double)> camera_callback_;
  std::vector<float> truth_scan_ranges_;
  srf::msgs::Pose2D truth_scan_pose_;

  std::unique_ptr<UrdfRobotVisualizer> visualizer_;
  std::unique_ptr<ArmCollision> collision_;
  std::string blocked_by_;
  std::array<double, 5> arm_velocity_{};        // 마지막 tick 관절 속도 (rad/s, 그리퍼 m/s)
  std::array<bool, 5> arm_stalled_{};           // 목표가 있는데 충돌로 못 움직인 관절
  mutable std::mutex arm_mutex_;
  bool manual_ = true;
  std::array<double, 5> manual_position_{};
  std::array<double, 5> manual_target_{};
  std::array<double, 5> controller_position_{};
  std::array<double, 5> controller_target_{};

  // grasp attach: 그리퍼가 닫히면 가까운 graspable 물체를 TCP에 붙인다.
  srf::simulator::WorldObject* held_ = nullptr;
  double held_dx_ = 0.0;
  double held_dy_ = 0.0;
  double held_dz_ = 0.0;
  double held_dyaw_ = 0.0;

  // 로봇 상판에 내려놓은 물체들. 각자 로봇 pose 기준 상대 위치를 들고 따라간다.
  struct Payload {
    srf::simulator::WorldObject* object = nullptr;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    double dyaw = 0.0;
  };
  std::vector<Payload> payloads_;
};

}  // namespace srf_recycling_ws::sim
