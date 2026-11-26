#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "krosc/context.hpp"

namespace krosc::control {
class ControlManager;
class ControllerManager;
}
namespace krosc::sensor {
class SensorManager;
}
namespace krosc::nodes {
class Cartographer;
class Ekf;
class MapCatalog;
class MapSaver;
class MapServer;
class RobotStatePublisher;
class TwistMux;
class VelocitySmoother;
class CollisionMonitor;
}
namespace krosc::moveit {
class MoveGroup;
}
namespace krosc::nodes::nav2 {
class BehaviorServer;
class BtNavigator;
class ControllerServer;
class Costmap2DROS;
class PlannerServer;
}

namespace krosc_ws {

class OpenCrDevice;
class LdsDevice;
class RealSenseCamera;
class OpenCrControl;
class OpenCrImu;
class Lds03Sensor;
class RealSenseSensor;
class RobotStatus;
class LlmInput;

struct EngineOptions {
  std::string urdf_path;
  std::string maps_dir;
  bool slam = false;
  bool enable_moveit = true;
  bool enable_map = true;
  bool enable_rl = true;
  bool enable_arm = true;
  bool enable_status = true;
  bool enable_robot_state_publisher = true;
  bool enable_camera = true;
  bool enable_llm = true;
  // llm_input 노드가 직접 호출하는 Ollama 서버. 원격 서버는 KROSC_OLLAMA_URL로 덮어쓴다.
  std::string llm_ollama_url = "http://127.0.0.1:11434";
  std::string llm_model = "gemma3:4b";
  double llm_timeout = 10.0;
  double wheel_radius = 0.033;
  double wheel_separation = 0.287;

  std::string scan_topic = "scan";
  std::string scan_frame = "base_scan";
  double scan_period = 0.1;
  std::string imu_topic = "imu";
  std::string imu_frame = "imu_link";
  double imu_period = 0.02;
  std::string camera_color_topic = "camera/color/image_raw";
  std::string camera_depth_topic = "camera/depth/image_rect_raw";
  std::string camera_color_frame = "camera_color_optical_frame";
  std::string camera_depth_frame = "camera_depth_optical_frame";
  double camera_period = 0.2;        // color 5Hz
  double camera_depth_period = 0.5;  // depth 2Hz (160x120)
  double control_read_period = 0.02;
  double control_write_period = 0.02;
  double control_period = 0.01;
};

// 실기/시뮬레이터 공통 엔진.
// 외부에서 Hardware(OpenCR/LDS/RealSense)를 주입하면
// Control/Sensor/nav2/MoveIt/로봇 상태를 조립한다.
class Engine {
 public:
  Engine(krosc::Context& context, OpenCrDevice& opencr, LdsDevice* lidar,
         EngineOptions options, RealSenseCamera* camera = nullptr);
  ~Engine();

  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;

  bool start();
  void stop();
  void log_status();

  // OpenCR/LDS/RealSense가 모두 연결됐는지. 선택 하드웨어(NULL/비활성)는 통과한다.
  [[nodiscard]] bool is_connected() const;
  // 링크만 연결/해제한다. 센서/컨트롤 루프는 계속 돈다 (수동 펌웨어 업로드 등).
  void connect_hardware();
  void disconnect_hardware();

  [[nodiscard]] OpenCrDevice& opencr() { return opencr_; }
  [[nodiscard]] LdsDevice* lidar() { return lidar_; }

 private:
  krosc::Context& context_;
  OpenCrDevice& opencr_;
  LdsDevice* lidar_;
  RealSenseCamera* camera_;
  EngineOptions options_;
  std::string urdf_;

  std::unique_ptr<OpenCrControl> control_;
  std::unique_ptr<OpenCrImu> imu_;
  std::unique_ptr<Lds03Sensor> lds_sensor_;
  std::unique_ptr<RealSenseSensor> camera_sensor_;
  std::unique_ptr<RobotStatus> status_;
  std::unique_ptr<LlmInput> llm_;
  std::unique_ptr<krosc::sensor::SensorManager> sensors_;
  std::unique_ptr<krosc::control::ControlManager> controls_;
  std::unique_ptr<krosc::control::ControllerManager> controller_manager_;
  std::unique_ptr<krosc::nodes::Ekf> ekf_;
  std::unique_ptr<krosc::moveit::MoveGroup> move_group_;

  std::unique_ptr<krosc::nodes::Cartographer> cartographer_;
  std::unique_ptr<krosc::nodes::MapServer> map_server_;
  std::unique_ptr<krosc::nodes::MapSaver> map_saver_;
  std::unique_ptr<krosc::nodes::MapCatalog> map_catalog_;
  std::unique_ptr<krosc::nodes::RobotStatePublisher> robot_state_;

  std::unique_ptr<krosc::nodes::nav2::Costmap2DROS> global_costmap_;
  std::unique_ptr<krosc::nodes::nav2::Costmap2DROS> local_costmap_;
  std::unique_ptr<krosc::nodes::nav2::PlannerServer> planner_;
  std::unique_ptr<krosc::nodes::nav2::ControllerServer> controller_;
  std::unique_ptr<krosc::nodes::nav2::BehaviorServer> behavior_;
  std::unique_ptr<krosc::nodes::nav2::BtNavigator> bt_navigator_;
  std::unique_ptr<krosc::nodes::TwistMux> mux_;
  std::unique_ptr<krosc::nodes::VelocitySmoother> smoother_;
  std::unique_ptr<krosc::nodes::CollisionMonitor> collision_;

  bool started_ = false;
  std::chrono::steady_clock::time_point last_status_{};
};

}  // namespace krosc_ws
