#pragma once

#include <cmath>

#include "krosc/msgs.hpp"

// tf2와 같은 규약: Transform은 "자식 좌표계의 점을 부모 좌표계로 옮기는" 변환이다.
// 쿼터니언은 (x, y, z, w), 오른손 법칙, w가 스칼라 부분이다.
namespace krosc::tfmath {

constexpr double kPi = 3.14159265358979323846;
inline constexpr msgs::Transform kIdentity{};

inline msgs::Quaternion normalize(const msgs::Quaternion& q) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm == 0.0) {
    return msgs::Quaternion{};
  }
  return {q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

inline msgs::Quaternion conjugate(const msgs::Quaternion& q) {
  return {-q.x, -q.y, -q.z, q.w};
}

inline msgs::Quaternion multiply(const msgs::Quaternion& a, const msgs::Quaternion& b) {
  return {
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

inline msgs::Vector3 rotate(const msgs::Quaternion& q, const msgs::Vector3& v) {
  const msgs::Quaternion r = multiply(multiply(q, msgs::Quaternion{v.x, v.y, v.z, 0.0}), conjugate(q));
  return {r.x, r.y, r.z};
}

inline msgs::Quaternion from_axis_angle(const msgs::Vector3& axis, double angle) {
  const double norm = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  if (norm == 0.0) {
    return {};
  }
  const double s = std::sin(angle / 2.0) / norm;
  return {axis.x * s, axis.y * s, axis.z * s, std::cos(angle / 2.0)};
}

inline msgs::Quaternion from_rpy(double roll, double pitch, double yaw) {
  const msgs::Quaternion qx = from_axis_angle({1.0, 0.0, 0.0}, roll);
  const msgs::Quaternion qy = from_axis_angle({0.0, 1.0, 0.0}, pitch);
  const msgs::Quaternion qz = from_axis_angle({0.0, 0.0, 1.0}, yaw);
  return multiply(qz, multiply(qy, qx));
}

inline msgs::Transform compose(const msgs::Transform& outer, const msgs::Transform& inner) {
  const msgs::Vector3 moved = rotate(outer.rotation, inner.translation);
  return {
      {outer.translation.x + moved.x, outer.translation.y + moved.y, outer.translation.z + moved.z},
      normalize(multiply(outer.rotation, inner.rotation)),
  };
}

inline msgs::Transform inverse(const msgs::Transform& t) {
  const msgs::Quaternion q = conjugate(t.rotation);
  const msgs::Vector3 v = rotate(q, t.translation);
  return {{-v.x, -v.y, -v.z}, q};
}

inline msgs::Vector3 apply(const msgs::Transform& t, const msgs::Vector3& point) {
  const msgs::Vector3 r = rotate(t.rotation, point);
  return {r.x + t.translation.x, r.y + t.translation.y, r.z + t.translation.z};
}

inline msgs::Point apply(const msgs::Transform& t, const msgs::Point& point) {
  const msgs::Vector3 r = apply(t, msgs::Vector3{point.x, point.y, point.z});
  return {r.x, r.y, r.z};
}

inline msgs::Quaternion slerp(const msgs::Quaternion& a_in, const msgs::Quaternion& b_in, double ratio) {
  msgs::Quaternion b = b_in;
  double dot = a_in.x * b.x + a_in.y * b.y + a_in.z * b.z + a_in.w * b.w;
  if (dot < 0.0) {
    b = {-b.x, -b.y, -b.z, -b.w};
    dot = -dot;
  }
  if (dot > 0.9995) {
    return normalize({a_in.x + ratio * (b.x - a_in.x), a_in.y + ratio * (b.y - a_in.y),
                      a_in.z + ratio * (b.z - a_in.z), a_in.w + ratio * (b.w - a_in.w)});
  }
  const double theta = std::acos(dot);
  const double sin_theta = std::sin(theta);
  const double wa = std::sin((1.0 - ratio) * theta) / sin_theta;
  const double wb = std::sin(ratio * theta) / sin_theta;
  return {wa * a_in.x + wb * b.x, wa * a_in.y + wb * b.y, wa * a_in.z + wb * b.z, wa * a_in.w + wb * b.w};
}

inline msgs::Transform interpolate(const msgs::Transform& a, const msgs::Transform& b, double ratio) {
  return {
      {a.translation.x + ratio * (b.translation.x - a.translation.x),
       a.translation.y + ratio * (b.translation.y - a.translation.y),
       a.translation.z + ratio * (b.translation.z - a.translation.z)},
      slerp(a.rotation, b.rotation, ratio),
  };
}

struct Rpy {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

inline Rpy to_rpy(const msgs::Quaternion& q_in) {
  const msgs::Quaternion q = normalize(q_in);
  const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
  if (std::abs(sin_pitch) >= 1.0 - 1e-12) {
    return {0.0, std::copysign(kPi / 2.0, sin_pitch),
            std::atan2(-2.0 * (q.x * q.y - q.w * q.z), 1.0 - 2.0 * (q.x * q.x + q.z * q.z))};
  }
  return {std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
          std::asin(sin_pitch),
          std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

inline msgs::Quaternion from_yaw(double yaw) {
  return {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

inline double yaw_of(const msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace krosc::tfmath
