#include "real_turtlebot3.hpp"

#include <stdexcept>
#include <utility>

#include "hardwares/sensors/opencr_imu/opencr_imu.hpp"

namespace krosc_ws {

RealTurtleBot3::RealTurtleBot3(krosc::Context& context, std::string opencr_port,
                               std::string lidar_port, bool enable_lidar, SensorOptions options)
    : context_(context),
      opencr_(kWheelRadius, kWheelSeparation, std::move(opencr_port), 115200, 0.02, 0.5, -1.0, -1.0, kMaxWheel) {
  // 이 로봇은 두 모터 모두 +속도 = 뒤로 (ROS robot.yaml left_dir/right_dir = -1).
  krosc::sensor::SensorInfo imu_info;
  imu_info.name = "imu";
  imu_info.type = "opencr_imu";
  imu_info.topic = options.imu_topic;
  imu_info.frame_id = options.imu_frame;
  imu_info.period = options.imu_period;
  auto imu = std::make_unique<OpenCrImu>(context_, opencr_);
  if (!sensors_.add(imu_info, std::move(imu))) {
    throw std::runtime_error("RealTurtleBot3: IMU 센서를 등록하지 못했다 (imu 주기 확인)");
  }

  if (enable_lidar) {
    auto lidar = std::make_unique<Lds03>(context_, std::move(lidar_port), kLidarMountX,
                                         options.scan_frame, true, true);
    lidar_ = lidar.get();
    krosc::sensor::SensorInfo lidar_info;
    lidar_info.name = "lidar";
    lidar_info.type = "lds03";
    lidar_info.topic = options.scan_topic;
    lidar_info.frame_id = options.scan_frame;
    lidar_info.period = options.scan_period;
    if (!sensors_.add(lidar_info, std::move(lidar))) {
      throw std::runtime_error("RealTurtleBot3: 라이다 센서를 등록하지 못했다 (scan 주기 확인)");
    }
  }
  opencr_.start();
  if (!sensors_.start()) {
    stop();
    throw std::runtime_error("RealTurtleBot3: 센서 초기화 실패");
  }
}

RealTurtleBot3::~RealTurtleBot3() { stop(); }

void RealTurtleBot3::tick() { opencr_.tick(); }

void RealTurtleBot3::stop() {
  // 프로세스 종료 경로다. Context가 살아 있을 때 센서 on_finish()까지 부른다.
  sensors_.finish();
  opencr_.stop();
}

std::map<std::string, double> RealTurtleBot3::joint_positions() const {
  std::map<std::string, double> joints{{"wheel_left_joint", opencr_.left_position()},
                                       {"wheel_right_joint", opencr_.right_position()}};
  const ArmStatus arm = opencr_.arm_status();
  if (arm.present) {
    for (std::size_t i = 0; i < 4; ++i) {
      if (std::isfinite(arm.position[i])) {
        joints["joint" + std::to_string(i + 1)] = arm.position[i];
      }
    }
    if (std::isfinite(arm.position[4])) {
      joints["gripper_left_joint"] = arm.position[4];
      joints["gripper_right_joint"] = arm.position[4];
    }
  }
  return joints;
}

}  // namespace krosc_ws
