#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <typeinfo>
#include <vector>

// krosc 메시지: ROS 메시지와 같은 이름과 필드를 쓰는 값 타입이다.
// 공유는 std::shared_ptr<const T>로 해서 구독자마다 복사하지 않는다.
namespace krosc::msgs {

using Time = double;
using Duration = double;

struct Header {
  Time stamp = 0.0;
  std::string frame_id;
};

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose {
  Point position;
  Quaternion orientation;
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct Twist {
  Vector3 linear;
  Vector3 angular;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct TransformStamped {
  Header header;
  std::string child_frame_id;
  Transform transform;
};

struct PointStamped {
  Header header;
  Point point;
};

struct PoseStamped {
  Header header;
  Pose pose;
};

struct PoseWithCovariance {
  Pose pose;
  std::array<double, 36> covariance{};
};

struct TwistWithCovariance {
  Twist twist;
  std::array<double, 36> covariance{};
};

struct PoseWithCovarianceStamped {
  Header header;
  PoseWithCovariance pose;
};

struct TwistStamped {
  Header header;
  Twist twist;
};

struct Odometry {
  Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};

struct LaserScan {
  Header header;
  double angle_min = 0.0;
  double angle_max = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  // ranges[i]는 angle_min + i * angle_increment 방향의 거리(m). 범위 밖은 inf.
  std::vector<float> ranges;
  std::vector<float> intensities;
};

struct JointState {
  Header header;
  std::vector<std::string> name;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

struct Imu {
  Header header;
  Quaternion orientation;
  std::array<double, 9> orientation_covariance{};
  Vector3 angular_velocity;
  std::array<double, 9> angular_velocity_covariance{};
  Vector3 linear_acceleration;
  std::array<double, 9> linear_acceleration_covariance{};
};

struct TFMessage {
  std::vector<TransformStamped> transforms;
};

struct MapMetaData {
  Time map_load_time = 0.0;
  double resolution = 0.05;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  Pose origin;
};

struct OccupancyGrid {
  Header header;
  MapMetaData info;
  // 칸마다 점유 확률(0~100, 모르면 -1)을 행 우선으로. 인덱스 = y * width + x.
  std::vector<std::int8_t> data;
};

struct Path {
  Header header;
  std::vector<PoseStamped> poses;
};

struct String {
  std::string data;
};

struct Bool {
  bool data = false;
};

struct Empty {};

struct KeyValue {
  std::string key;
  std::string value;
};

struct DiagnosticStatus {
  // 0 OK, 1 WARN, 2 ERROR, 3 STALE
  std::int8_t level = 0;
  std::string name;
  std::string message;
  std::string hardware_id;
  std::vector<KeyValue> values;
};

struct DiagnosticArray {
  Header header;
  std::vector<DiagnosticStatus> status;
};

struct Log {
  // 파이썬 logging과 같은 숫자: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
  Time stamp = 0.0;
  std::int8_t level = 20;
  std::string name;
  std::string msg;
  std::string file;
  std::string function;
  std::uint32_t line = 0;
};

// std_srvs
struct SetBoolRequest {
  bool data = false;
};
struct SetBoolResponse {
  bool success = false;
  std::string message;
};
struct TriggerRequest {};
struct TriggerResponse {
  bool success = false;
  std::string message;
};

// krosc_ws 하드웨어 서비스 (std_srvs에 없는 명령)
struct ArmResetRequest {
  std::string target;
};
struct ArmResetResponse {
  bool success = false;
  std::string message;
};
struct BuzzerRequest {
  std::int32_t frequency = 2000;
  std::int32_t duration = 150;
};
struct BuzzerResponse {
  bool success = false;
  std::string message;
};

// tf2_msgs / nav_msgs / sensor_msgs 와 같은 이름을 패널과 로그에 보여 주기 위한 타입 이름.
template <class T>
constexpr std::string_view message_type_name() {
  return typeid(T).name();
}

#define KROSC_MSG_NAME(Type, Name)                                   \
  template <>                                                        \
  constexpr std::string_view message_type_name<Type>() {             \
    return Name;                                                     \
  }

KROSC_MSG_NAME(Header, "std_msgs/Header")
KROSC_MSG_NAME(String, "std_msgs/String")
KROSC_MSG_NAME(Bool, "std_msgs/Bool")
KROSC_MSG_NAME(Empty, "std_msgs/Empty")
KROSC_MSG_NAME(Log, "rcl_interfaces/Log")
KROSC_MSG_NAME(Vector3, "geometry_msgs/Vector3")
KROSC_MSG_NAME(Point, "geometry_msgs/Point")
KROSC_MSG_NAME(PointStamped, "geometry_msgs/PointStamped")
KROSC_MSG_NAME(Quaternion, "geometry_msgs/Quaternion")
KROSC_MSG_NAME(Pose, "geometry_msgs/Pose")
KROSC_MSG_NAME(Pose2D, "geometry_msgs/Pose2D")
KROSC_MSG_NAME(PoseStamped, "geometry_msgs/PoseStamped")
KROSC_MSG_NAME(PoseWithCovariance, "geometry_msgs/PoseWithCovariance")
KROSC_MSG_NAME(PoseWithCovarianceStamped, "geometry_msgs/PoseWithCovarianceStamped")
KROSC_MSG_NAME(Twist, "geometry_msgs/Twist")
KROSC_MSG_NAME(TwistStamped, "geometry_msgs/TwistStamped")
KROSC_MSG_NAME(TwistWithCovariance, "geometry_msgs/TwistWithCovariance")
KROSC_MSG_NAME(Transform, "geometry_msgs/Transform")
KROSC_MSG_NAME(TransformStamped, "geometry_msgs/TransformStamped")
KROSC_MSG_NAME(Odometry, "nav_msgs/Odometry")
KROSC_MSG_NAME(Path, "nav_msgs/Path")
KROSC_MSG_NAME(OccupancyGrid, "nav_msgs/OccupancyGrid")
KROSC_MSG_NAME(LaserScan, "sensor_msgs/LaserScan")
KROSC_MSG_NAME(JointState, "sensor_msgs/JointState")
KROSC_MSG_NAME(Imu, "sensor_msgs/Imu")
KROSC_MSG_NAME(TFMessage, "tf2_msgs/TFMessage")
KROSC_MSG_NAME(DiagnosticArray, "diagnostic_msgs/DiagnosticArray")
KROSC_MSG_NAME(DiagnosticStatus, "diagnostic_msgs/DiagnosticStatus")
KROSC_MSG_NAME(KeyValue, "diagnostic_msgs/KeyValue")
KROSC_MSG_NAME(SetBoolRequest, "std_srvs/SetBool_Request")
KROSC_MSG_NAME(SetBoolResponse, "std_srvs/SetBool_Response")
KROSC_MSG_NAME(TriggerRequest, "std_srvs/Trigger_Request")
KROSC_MSG_NAME(TriggerResponse, "std_srvs/Trigger_Response")
KROSC_MSG_NAME(ArmResetRequest, "krosc_ws/srv/ArmReset_Request")
KROSC_MSG_NAME(ArmResetResponse, "krosc_ws/srv/ArmReset_Response")
KROSC_MSG_NAME(BuzzerRequest, "krosc_ws/srv/Buzzer_Request")
KROSC_MSG_NAME(BuzzerResponse, "krosc_ws/srv/Buzzer_Response")

#undef KROSC_MSG_NAME

}  // namespace krosc::msgs
