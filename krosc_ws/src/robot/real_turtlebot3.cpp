#include "real_turtlebot3.hpp"

#include <utility>

namespace krosc_ws {

RealTurtleBot3::RealTurtleBot3(std::string opencr_port, std::string lidar_port, bool enable_lidar)
    : opencr_(kWheelRadius, kWheelSeparation, std::move(opencr_port), 115200, 0.05, 0.5, -1.0, -1.0, kMaxWheel) {
  // 이 로봇은 두 모터 모두 +속도 = 뒤로 (ROS robot.yaml left_dir/right_dir = -1).
  if (enable_lidar) {
    lidar_ = std::make_unique<Lds03>(std::move(lidar_port), kLidarMountX, kLidarMountHeight, "base_scan", true, true);
  }
  opencr_.start();
  if (lidar_) {
    lidar_->start();
  }
}

RealTurtleBot3::~RealTurtleBot3() { stop(); }

void RealTurtleBot3::tick() { opencr_.tick(); }

void RealTurtleBot3::stop() {
  opencr_.stop();
  if (lidar_) {
    lidar_->stop();
  }
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

std::optional<krosc::msgs::LaserScan> RealTurtleBot3::scan(double stamp) {
  if (!lidar_) {
    return std::nullopt;
  }
  return lidar_->take_scan(stamp);
}

}  // namespace krosc_ws
