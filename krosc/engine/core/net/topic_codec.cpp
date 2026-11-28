#include "krosc/net/topic_codec.hpp"

#include <optional>
#include <type_traits>
#include <unordered_map>


namespace krosc::net {

using detail::Reader;
using detail::Writer;

namespace {

void encode(Writer& w, const msgs::Header& header) {
  w.f64(header.stamp);
  w.str(header.frame_id);
}
void encode(Writer& w, const msgs::Vector3& v) {
  w.f64(v.x);
  w.f64(v.y);
  w.f64(v.z);
}
void encode(Writer& w, const msgs::Quaternion& q) {
  w.f64(q.x);
  w.f64(q.y);
  w.f64(q.z);
  w.f64(q.w);
}
void encode(Writer& w, const msgs::Pose& pose) {
  encode(w, msgs::Vector3{pose.position.x, pose.position.y, pose.position.z});
  encode(w, pose.orientation);
}
void encode(Writer& w, const msgs::Twist& twist) {
  encode(w, twist.linear);
  encode(w, twist.angular);
}
void encode(Writer& w, const msgs::Transform& transform) {
  encode(w, transform.translation);
  encode(w, transform.rotation);
}
void encode(Writer& w, const msgs::TransformStamped& stamped) {
  encode(w, stamped.header);
  w.str(stamped.child_frame_id);
  encode(w, stamped.transform);
}

void encode(Writer& w, const msgs::String& message) { w.str(message.data); }
void encode(Writer& w, const msgs::PoseStamped& message) {
  encode(w, message.header);
  encode(w, message.pose);
}
void encode(Writer& w, const msgs::PoseWithCovarianceStamped& message) {
  encode(w, message.header);
  encode(w, message.pose.pose);
  for (const double value : message.pose.covariance) {
    w.f64(value);
  }
}
void encode(Writer& w, const msgs::Bool& message) { w.u8(message.data ? 1U : 0U); }
void encode(Writer& w, const msgs::Empty&) { (void)w; }

void encode(Writer& w, const msgs::TFMessage& message) {
  w.u32(static_cast<std::uint32_t>(message.transforms.size()));
  for (const auto& transform : message.transforms) {
    encode(w, transform);
  }
}

void encode(Writer& w, const msgs::LaserScan& message) {
  encode(w, message.header);
  w.f64(message.angle_min);
  w.f64(message.angle_max);
  w.f64(message.angle_increment);
  w.f64(message.range_min);
  w.f64(message.range_max);
  w.u32(static_cast<std::uint32_t>(message.ranges.size()));
  for (float range : message.ranges) {
    w.f32(range);
  }
  w.u32(static_cast<std::uint32_t>(message.intensities.size()));
  for (float intensity : message.intensities) {
    w.f32(intensity);
  }
}

void encode(Writer& w, const msgs::Image& message) {
  encode(w, message.header);
  w.u32(message.width);
  w.u32(message.height);
  w.str(message.encoding);
  w.u32(message.step);
  w.u32(static_cast<std::uint32_t>(message.data.size()));
  w.bytes(message.data);
}

void encode(Writer& w, const msgs::Imu& message) {
  encode(w, message.header);
  encode(w, message.orientation);
  for (double value : message.orientation_covariance) {
    w.f64(value);
  }
  encode(w, message.angular_velocity);
  for (double value : message.angular_velocity_covariance) {
    w.f64(value);
  }
  encode(w, message.linear_acceleration);
  for (double value : message.linear_acceleration_covariance) {
    w.f64(value);
  }
}

void encode(Writer& w, const msgs::Odometry& message) {
  encode(w, message.header);
  w.str(message.child_frame_id);
  encode(w, message.pose.pose);
  for (double value : message.pose.covariance) {
    w.f64(value);
  }
  encode(w, message.twist.twist);
  for (double value : message.twist.covariance) {
    w.f64(value);
  }
}

void encode(Writer& w, const msgs::JointState& message) {
  encode(w, message.header);
  w.u32(static_cast<std::uint32_t>(message.name.size()));
  for (const auto& name : message.name) {
    w.str(name);
  }
  const auto write_doubles = [&](const std::vector<double>& values) {
    w.u32(static_cast<std::uint32_t>(values.size()));
    for (double value : values) {
      w.f64(value);
    }
  };
  write_doubles(message.position);
  write_doubles(message.velocity);
  write_doubles(message.effort);
}

void encode(Writer& w, const msgs::OccupancyGrid& message) {
  encode(w, message.header);
  w.f64(message.info.map_load_time);
  w.f64(message.info.resolution);
  w.u32(message.info.width);
  w.u32(message.info.height);
  encode(w, message.info.origin);
  w.u32(static_cast<std::uint32_t>(message.data.size()));
  w.bytes(reinterpret_cast<const std::uint8_t*>(message.data.data()), message.data.size());
}

void encode(Writer& w, const msgs::Path& message) {
  encode(w, message.header);
  w.u32(static_cast<std::uint32_t>(message.poses.size()));
  for (const auto& pose : message.poses) {
    encode(w, pose.header);
    encode(w, pose.pose);
  }
}

void encode(Writer& w, const msgs::DiagnosticArray& message) {
  encode(w, message.header);
  w.u32(static_cast<std::uint32_t>(message.status.size()));
  for (const auto& status : message.status) {
    w.u8(static_cast<std::uint8_t>(status.level));
    w.str(status.name);
    w.str(status.message);
    w.str(status.hardware_id);
    w.u32(static_cast<std::uint32_t>(status.values.size()));
    for (const auto& value : status.values) {
      w.str(value.key);
      w.str(value.value);
    }
  }
}

void encode(Writer& w, const msgs::Log& message) {
  w.f64(message.stamp);
  w.u8(static_cast<std::uint8_t>(message.level));
  w.str(message.name);
  w.str(message.msg);
  w.str(message.file);
  w.str(message.function);
  w.u32(message.line);
}

msgs::Header decode_header(Reader& r) {
  msgs::Header header;
  header.stamp = r.f64();
  header.frame_id = r.str();
  return header;
}
msgs::Vector3 decode_vector3(Reader& r) { return {r.f64(), r.f64(), r.f64()}; }
msgs::Quaternion decode_quaternion(Reader& r) { return {r.f64(), r.f64(), r.f64(), r.f64()}; }
msgs::Pose decode_pose(Reader& r) {
  msgs::Pose pose;
  pose.position = {r.f64(), r.f64(), r.f64()};
  pose.orientation = decode_quaternion(r);
  return pose;
}
msgs::Twist decode_twist(Reader& r) {
  msgs::Twist twist;
  twist.linear = decode_vector3(r);
  twist.angular = decode_vector3(r);
  return twist;
}
msgs::Transform decode_transform(Reader& r) {
  msgs::Transform transform;
  transform.translation = decode_vector3(r);
  transform.rotation = decode_quaternion(r);
  return transform;
}

std::optional<msgs::TFMessage> decode_tf(Reader& r) {
  const std::uint32_t count = r.u32();
  msgs::TFMessage message;
  if (count > 10000) {
    return std::nullopt;
  }
  message.transforms.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    msgs::TransformStamped stamped;
    stamped.header = decode_header(r);
    stamped.child_frame_id = r.str();
    stamped.transform = decode_transform(r);
    message.transforms.push_back(std::move(stamped));
  }
  return r.ok() ? std::optional<msgs::TFMessage>(std::move(message)) : std::nullopt;
}

std::optional<msgs::String> decode_string(Reader& r) {
  msgs::String message{r.str()};
  return r.ok() ? std::optional<msgs::String>(std::move(message)) : std::nullopt;
}

std::optional<msgs::PoseStamped> decode_pose_stamped(Reader& r) {
  msgs::PoseStamped message;
  message.header = decode_header(r);
  message.pose = decode_pose(r);
  return r.ok() ? std::optional<msgs::PoseStamped>(std::move(message)) : std::nullopt;
}

std::optional<msgs::PoseWithCovarianceStamped> decode_pose_covariance(Reader& r) {
  msgs::PoseWithCovarianceStamped message;
  message.header = decode_header(r);
  message.pose.pose = decode_pose(r);
  for (double& value : message.pose.covariance) {
    value = r.f64();
  }
  return r.ok() ? std::optional<msgs::PoseWithCovarianceStamped>(std::move(message)) : std::nullopt;
}

std::optional<msgs::Bool> decode_bool(Reader& r) {
  const std::uint8_t value = r.u8();
  return r.ok() ? std::optional<msgs::Bool>(msgs::Bool{value != 0}) : std::nullopt;
}

std::optional<msgs::LaserScan> decode_scan(Reader& r) {
  msgs::LaserScan message;
  message.header = decode_header(r);
  message.angle_min = r.f64();
  message.angle_max = r.f64();
  message.angle_increment = r.f64();
  message.range_min = r.f64();
  message.range_max = r.f64();
  const std::uint32_t ranges = r.u32();
  if (ranges > 1000000) {
    return std::nullopt;
  }
  message.ranges.resize(ranges);
  for (auto& range : message.ranges) {
    range = r.f32();
  }
  const std::uint32_t intensities = r.u32();
  if (intensities > 1000000) {
    return std::nullopt;
  }
  message.intensities.resize(intensities);
  for (auto& intensity : message.intensities) {
    intensity = r.f32();
  }
  return r.ok() ? std::optional<msgs::LaserScan>(std::move(message)) : std::nullopt;
}

std::optional<msgs::Image> decode_image(Reader& r) {
  msgs::Image message;
  message.header = decode_header(r);
  message.width = r.u32();
  message.height = r.u32();
  message.encoding = r.str();
  message.step = r.u32();
  const std::uint32_t count = r.u32();
  if (!r.ok() || count > static_cast<std::uint32_t>(kMaxFrame) ||
      r.remaining() < count) {
    return std::nullopt;
  }
  message.data.resize(count);
  if (count > 0) {
    std::memcpy(message.data.data(), r.rest(), count);
    r.skip(count);
  }
  return r.ok() ? std::optional<msgs::Image>(std::move(message)) : std::nullopt;
}

std::optional<msgs::Imu> decode_imu(Reader& r) {
  msgs::Imu message;
  message.header = decode_header(r);
  message.orientation = decode_quaternion(r);
  for (double& value : message.orientation_covariance) {
    value = r.f64();
  }
  message.angular_velocity = decode_vector3(r);
  for (double& value : message.angular_velocity_covariance) {
    value = r.f64();
  }
  message.linear_acceleration = decode_vector3(r);
  for (double& value : message.linear_acceleration_covariance) {
    value = r.f64();
  }
  return r.ok() ? std::optional<msgs::Imu>(std::move(message)) : std::nullopt;
}

std::optional<msgs::Odometry> decode_odometry(Reader& r) {
  msgs::Odometry message;
  message.header = decode_header(r);
  message.child_frame_id = r.str();
  message.pose.pose = decode_pose(r);
  for (double& value : message.pose.covariance) {
    value = r.f64();
  }
  message.twist.twist = decode_twist(r);
  for (double& value : message.twist.covariance) {
    value = r.f64();
  }
  return r.ok() ? std::optional<msgs::Odometry>(std::move(message)) : std::nullopt;
}

std::optional<msgs::JointState> decode_joint_state(Reader& r) {
  msgs::JointState message;
  message.header = decode_header(r);
  const std::uint32_t names = r.u32();
  if (names > 100000) {
    return std::nullopt;
  }
  message.name.reserve(names);
  for (std::uint32_t i = 0; i < names; ++i) {
    message.name.push_back(r.str());
  }
  const auto read_doubles = [&](std::vector<double>& values) {
    const std::uint32_t count = r.u32();
    if (count > 100000) {
      return false;
    }
    values.resize(count);
    for (double& value : values) {
      value = r.f64();
    }
    return true;
  };
  if (!read_doubles(message.position) || !read_doubles(message.velocity) || !read_doubles(message.effort)) {
    return std::nullopt;
  }
  return r.ok() ? std::optional<msgs::JointState>(std::move(message)) : std::nullopt;
}

std::optional<msgs::OccupancyGrid> decode_grid(Reader& r) {
  msgs::OccupancyGrid message;
  message.header = decode_header(r);
  message.info.map_load_time = r.f64();
  message.info.resolution = r.f64();
  message.info.width = r.u32();
  message.info.height = r.u32();
  message.info.origin = decode_pose(r);
  const std::uint32_t count = r.u32();
  if (count > static_cast<std::uint32_t>(kMaxFrame)) {
    return std::nullopt;
  }
  message.data.resize(count);
  const std::uint8_t* data = r.rest();
  if (r.remaining() < count) {
    return std::nullopt;
  }
  std::memcpy(message.data.data(), data, count);
  r.skip(count);
  return r.ok() ? std::optional<msgs::OccupancyGrid>(std::move(message)) : std::nullopt;
}

std::optional<msgs::Path> decode_path(Reader& r) {
  msgs::Path message;
  message.header = decode_header(r);
  const std::uint32_t count = r.u32();
  if (count > 100000) {
    return std::nullopt;
  }
  message.poses.resize(count);
  for (auto& pose : message.poses) {
    pose.header = decode_header(r);
    pose.pose = decode_pose(r);
  }
  return r.ok() ? std::optional<msgs::Path>(std::move(message)) : std::nullopt;
}

std::optional<msgs::DiagnosticArray> decode_diagnostics(Reader& r) {
  msgs::DiagnosticArray message;
  message.header = decode_header(r);
  const std::uint32_t count = r.u32();
  if (count > 10000) {
    return std::nullopt;
  }
  message.status.resize(count);
  for (auto& status : message.status) {
    status.level = static_cast<std::int8_t>(r.u8());
    status.name = r.str();
    status.message = r.str();
    status.hardware_id = r.str();
    const std::uint32_t values = r.u32();
    if (values > 10000) {
      return std::nullopt;
    }
    status.values.resize(values);
    for (auto& value : status.values) {
      value.key = r.str();
      value.value = r.str();
    }
  }
  return r.ok() ? std::optional<msgs::DiagnosticArray>(std::move(message)) : std::nullopt;
}

std::optional<msgs::Log> decode_log(Reader& r) {
  msgs::Log message;
  message.stamp = r.f64();
  message.level = static_cast<std::int8_t>(r.u8());
  message.name = r.str();
  message.msg = r.str();
  message.file = r.str();
  message.function = r.str();
  message.line = r.u32();
  return r.ok() ? std::optional<msgs::Log>(std::move(message)) : std::nullopt;
}


// decode 함수는 T 또는 optional<T>를 돌려준다. 끝까지 읽었는지(reader.ok)는 여기서 한 번만 본다.
// Decode는 캡처 없는 람다 타입 (C++20부터 기본 생성 가능).
template <class T, class Decode>
TopicType entry(Kind kind, Decode) {
  TopicType type{kind, std::type_index(typeid(T)), msgs::message_type_name<T>()};
  type.encode = [](RpcWriter& out, const void* message) { encode(out, *static_cast<const T*>(message)); };
  type.decode = [](const std::uint8_t* data, std::size_t size) -> std::shared_ptr<const void> {
    Reader reader(data, size);
    auto value = Decode{}(reader);
    if (!reader.ok()) {
      return {};
    }
    if constexpr (std::is_same_v<decltype(value), T>) {
      return std::make_shared<const T>(std::move(value));
    } else {
      return value ? std::make_shared<const T>(std::move(*value)) : nullptr;
    }
  };
  type.publish = [](TopicRegistry& topics, const std::string& topic, std::shared_ptr<const void> message,
                    QoS qos) {
    topics.publish<T>(topic, std::static_pointer_cast<const T>(std::move(message)), qos);
  };
  type.subscribe = [](TopicRegistry& topics, const std::string& topic,
                      std::function<void(const void*)> callback, QoS qos) {
    return topics
        .subscribe<T>(topic, [callback = std::move(callback)](const T& message) { callback(&message); }, qos)
        .base();
  };
  return type;
}

// 메시지 타입 하나 = 이 표의 한 줄. Kind 숫자는 브라우저(www/app.js)와 공유하므로 바꾸지 않는다.
const std::vector<TopicType>& table() {
  static const std::vector<TopicType> types{
      entry<msgs::Twist>(Kind::Twist, [](Reader& r) { return decode_twist(r); }),
      entry<msgs::String>(Kind::String, [](Reader& r) { return decode_string(r); }),
      entry<msgs::TFMessage>(Kind::TF, [](Reader& r) { return decode_tf(r); }),
      entry<msgs::LaserScan>(Kind::Scan, [](Reader& r) { return decode_scan(r); }),
      entry<msgs::Imu>(Kind::Imu, [](Reader& r) { return decode_imu(r); }),
      entry<msgs::Odometry>(Kind::Odometry, [](Reader& r) { return decode_odometry(r); }),
      entry<msgs::JointState>(Kind::JointState, [](Reader& r) { return decode_joint_state(r); }),
      entry<msgs::OccupancyGrid>(Kind::Grid, [](Reader& r) { return decode_grid(r); }),
      entry<msgs::Path>(Kind::Path, [](Reader& r) { return decode_path(r); }),
      entry<msgs::DiagnosticArray>(Kind::Diagnostics, [](Reader& r) { return decode_diagnostics(r); }),
      entry<msgs::Log>(Kind::Log, [](Reader& r) { return decode_log(r); }),
      entry<msgs::PoseStamped>(Kind::Pose, [](Reader& r) { return decode_pose_stamped(r); }),
      entry<msgs::PoseWithCovarianceStamped>(Kind::PoseCov, [](Reader& r) { return decode_pose_covariance(r); }),
      entry<msgs::Empty>(Kind::Empty, [](Reader&) { return msgs::Empty{}; }),
      entry<msgs::Bool>(Kind::Bool, [](Reader& r) { return decode_bool(r); }),
      entry<msgs::Image>(Kind::Image, [](Reader& r) { return decode_image(r); }),
  };
  return types;
}

}  // namespace

const TopicType* find_topic_type(Kind kind) {
  for (const auto& type : table()) {
    if (type.kind == kind) {
      return &type;
    }
  }
  return nullptr;
}

const TopicType* find_topic_type(std::type_index type) {
  static const auto by_type = [] {
    std::unordered_map<std::type_index, const TopicType*> result;
    for (const auto& entry : table()) {
      result.emplace(entry.type, &entry);
    }
    return result;
  }();
  const auto found = by_type.find(type);
  return found == by_type.end() ? nullptr : found->second;
}

}  // namespace krosc::net
