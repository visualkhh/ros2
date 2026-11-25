#pragma once

// krosc msgs RPC 코덱이 공유하는 바이너리 필드 헬퍼.
// 전송 계층(net/rpc)은 이 파일을 모른다.

#include <cstdint>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/net/rpc_codec.hpp"

namespace krosc::msgs::codec {

using net::RpcReader;
using net::RpcWriter;

inline void encode_header(RpcWriter& w, const msgs::Header& header) {
  w.f64(header.stamp);
  w.str(header.frame_id);
}
inline void encode_quaternion(RpcWriter& w, const msgs::Quaternion& value) {
  w.f64(value.x);
  w.f64(value.y);
  w.f64(value.z);
  w.f64(value.w);
}
inline void encode_point(RpcWriter& w, const msgs::Point& value) {
  w.f64(value.x);
  w.f64(value.y);
  w.f64(value.z);
}
inline void encode_pose(RpcWriter& w, const msgs::Pose& value) {
  encode_point(w, value.position);
  encode_quaternion(w, value.orientation);
}
inline void encode_pose_stamped(RpcWriter& w, const msgs::PoseStamped& value) {
  encode_header(w, value.header);
  encode_pose(w, value.pose);
}
inline void encode_doubles(RpcWriter& w, const std::vector<double>& values) {
  w.u32(static_cast<std::uint32_t>(values.size()));
  for (const double value : values) {
    w.f64(value);
  }
}
inline void encode_strings(RpcWriter& w, const std::vector<std::string>& values) {
  w.u32(static_cast<std::uint32_t>(values.size()));
  for (const auto& value : values) {
    w.str(value);
  }
}

inline msgs::Header decode_header(RpcReader& r) {
  msgs::Header header;
  header.stamp = r.f64();
  header.frame_id = r.str();
  return header;
}
inline msgs::Quaternion decode_quaternion(RpcReader& r) { return {r.f64(), r.f64(), r.f64(), r.f64()}; }
inline msgs::Point decode_point(RpcReader& r) { return {r.f64(), r.f64(), r.f64()}; }
inline msgs::Pose decode_pose(RpcReader& r) {
  msgs::Pose pose;
  pose.position = decode_point(r);
  pose.orientation = decode_quaternion(r);
  return pose;
}
inline msgs::PoseStamped decode_pose_stamped(RpcReader& r) {
  msgs::PoseStamped pose;
  pose.header = decode_header(r);
  pose.pose = decode_pose(r);
  return pose;
}
inline std::vector<double> decode_doubles(RpcReader& r) {
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return {};
  }
  std::vector<double> values;
  values.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    values.push_back(r.f64());
  }
  return values;
}
inline std::vector<std::string> decode_strings(RpcReader& r) {
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return {};
  }
  std::vector<std::string> values;
  values.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    values.push_back(r.str());
  }
  return values;
}

inline void encode_joint_trajectory_point(RpcWriter& w, const msgs::control::JointTrajectoryPoint& point) {
  encode_doubles(w, point.positions);
  encode_doubles(w, point.velocities);
  encode_doubles(w, point.accelerations);
  encode_doubles(w, point.effort);
  w.f64(point.time_from_start);
}

inline msgs::control::JointTrajectoryPoint decode_joint_trajectory_point(RpcReader& r) {
  msgs::control::JointTrajectoryPoint point;
  point.positions = decode_doubles(r);
  point.velocities = decode_doubles(r);
  point.accelerations = decode_doubles(r);
  point.effort = decode_doubles(r);
  point.time_from_start = r.f64();
  return point;
}

inline void encode_joint_trajectory(RpcWriter& w, const msgs::control::JointTrajectory& trajectory) {
  encode_header(w, trajectory.header);
  encode_strings(w, trajectory.joint_names);
  w.u32(static_cast<std::uint32_t>(trajectory.points.size()));
  for (const auto& point : trajectory.points) {
    encode_joint_trajectory_point(w, point);
  }
}

inline msgs::control::JointTrajectory decode_joint_trajectory(RpcReader& r) {
  msgs::control::JointTrajectory trajectory;
  trajectory.header = decode_header(r);
  trajectory.joint_names = decode_strings(r);
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return trajectory;
  }
  trajectory.points.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    trajectory.points.push_back(decode_joint_trajectory_point(r));
  }
  return trajectory;
}

inline void encode_tolerances(RpcWriter& w, const std::vector<msgs::control::JointTolerance>& values) {
  w.u32(static_cast<std::uint32_t>(values.size()));
  for (const auto& value : values) {
    w.str(value.name);
    w.f64(value.position);
    w.f64(value.velocity);
    w.f64(value.acceleration);
  }
}

inline std::vector<msgs::control::JointTolerance> decode_tolerances(RpcReader& r) {
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return {};
  }
  std::vector<msgs::control::JointTolerance> values;
  values.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    msgs::control::JointTolerance value;
    value.name = r.str();
    value.position = r.f64();
    value.velocity = r.f64();
    value.acceleration = r.f64();
    values.push_back(std::move(value));
  }
  return values;
}

inline void encode_parameter_value(RpcWriter& w, const msgs::rcl_interfaces::ParameterValue& value) {
  w.u8(value.type);
  using Type = msgs::rcl_interfaces::ParameterType;
  switch (static_cast<Type>(value.type)) {
    case Type::Bool:
      w.u8(value.bool_value ? 1U : 0U);
      break;
    case Type::Integer:
      w.i64(value.integer_value);
      break;
    case Type::Double:
      w.f64(value.double_value);
      break;
    case Type::String:
      w.str(value.string_value);
      break;
    case Type::BoolArray:
      w.u32(static_cast<std::uint32_t>(value.bool_array_value.size()));
      for (const bool item : value.bool_array_value) {
        w.u8(item ? 1U : 0U);
      }
      break;
    case Type::IntegerArray:
      w.u32(static_cast<std::uint32_t>(value.integer_array_value.size()));
      for (const std::int64_t item : value.integer_array_value) {
        w.i64(item);
      }
      break;
    case Type::DoubleArray:
      encode_doubles(w, value.double_array_value);
      break;
    case Type::StringArray:
      encode_strings(w, value.string_array_value);
      break;
    case Type::NotSet:
    default:
      break;
  }
}

inline msgs::rcl_interfaces::ParameterValue decode_parameter_value(RpcReader& r) {
  using Type = msgs::rcl_interfaces::ParameterType;
  msgs::rcl_interfaces::ParameterValue value;
  value.type = r.u8();
  switch (static_cast<Type>(value.type)) {
    case Type::Bool:
      value.bool_value = r.u8() != 0;
      break;
    case Type::Integer:
      value.integer_value = r.i64();
      break;
    case Type::Double:
      value.double_value = r.f64();
      break;
    case Type::String:
      value.string_value = r.str();
      break;
    case Type::BoolArray: {
      const std::uint32_t count = r.u32();
      if (count > 1000000) {
        r.skip(r.remaining());
        break;
      }
      value.bool_array_value.reserve(count);
      for (std::uint32_t i = 0; i < count; ++i) {
        value.bool_array_value.push_back(r.u8() != 0);
      }
      break;
    }
    case Type::IntegerArray: {
      const std::uint32_t count = r.u32();
      if (count > 1000000) {
        r.skip(r.remaining());
        break;
      }
      value.integer_array_value.reserve(count);
      for (std::uint32_t i = 0; i < count; ++i) {
        value.integer_array_value.push_back(r.i64());
      }
      break;
    }
    case Type::DoubleArray:
      value.double_array_value = decode_doubles(r);
      break;
    case Type::StringArray:
      value.string_array_value = decode_strings(r);
      break;
    case Type::NotSet:
    default:
      break;
  }
  return value;
}


}  // namespace krosc::msgs::codec
