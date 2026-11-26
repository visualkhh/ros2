#include "krosc/net/net.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/net/rpc_codec.hpp"
#include "krosc/msgs.hpp"
#include "krosc/qos.hpp"
#include "krosc/topic.hpp"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace krosc::net {

using detail::Reader;
using detail::Writer;

namespace {

constexpr std::size_t kMaxQueue = 512;
constexpr std::size_t kMaxClients = 8;

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

std::vector<std::uint8_t> make_frame(std::uint8_t type, const std::string& topic, Kind kind,
                                     const std::vector<std::uint8_t>& payload) {
  return encode_frame(Frame{type, topic, kind, payload});
}

bool send_all(int fd, const std::vector<std::uint8_t>& data) {
  std::size_t offset = 0;
  while (offset < data.size()) {
    const ssize_t sent = ::send(fd, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(sent);
  }
  return true;
}

bool read_frame(int fd, std::uint8_t& type, std::string& topic, Kind& kind,
                std::vector<std::uint8_t>& payload) {
  Frame frame;
  if (!krosc::net::read_frame(fd, frame)) {
    return false;
  }
  type = frame.type;
  topic = std::move(frame.topic);
  kind = frame.kind;
  payload = std::move(frame.payload);
  return true;
}

void configure_socket(int fd) {
  int enabled = 1;
  (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
  timeval timeout{};
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  // 읽기는 blocking으로 둔다. SO_RCVTIMEO를 주면 cmd_vel이 없는 1초마다 끊긴 것으로 오해한다.
  timeval no_timeout{};
  (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &no_timeout, sizeof(no_timeout));
#ifdef SO_NOSIGPIPE
  (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof(enabled));
#endif
}

void close_socket(int fd) {
  if (fd >= 0) {
    (void)::shutdown(fd, SHUT_RDWR);
    (void)::close(fd);
  }
}

QoS qos_for_topic(const std::string& topic) {
  if (topic == "tf") {
    return QOS_TF;
  }
  if (topic == "tf_static") {
    return QOS_TF_STATIC;
  }
  if (topic == "robot_description" || topic == "map" || topic == "global_costmap/costmap" ||
      topic == "local_costmap/costmap" || topic == "plan" || topic == "map_list" ||
      topic == "map_status" || topic == "slam_status" || topic == "goal_status" ||
      topic == "collision_status") {
    return QOS_LATCHED;
  }
  if (topic == "scan" || topic == "imu" || topic == "odom" || topic == "odometry/filtered" ||
      topic == "local_plan") {
    return QOS_SENSOR_DATA;
  }
  return QOS_DEFAULT;
}

// 클라이언트(웹/레거시 TCP)가 보낸 publish 프레임을 Kind에 따라 로컬 토픽으로 반영한다.
// 토픽 이름 화이트리스트는 두지 않는다. 어떤 토픽을 구독/노출할지는 Viz/VizWeb/플러그인이 정한다.
bool publish_frame(Context& context, const Frame& frame, bool remap_cmd_vel) {
  if (frame.type != kFramePublish) {
    return false;
  }
  const auto message = decode_message(frame.kind, frame.payload.data(), frame.payload.size());
  if (!message) {
    return false;
  }
  std::string topic = frame.topic;
  if (remap_cmd_vel && topic == "cmd_vel") {
    topic = "cmd_vel_teleop";
  }
  const QoS qos = frame.kind == Kind::Twist ? QOS_COMMAND : QOS_DEFAULT;
  switch (frame.kind) {
    case Kind::Twist:
      context.topics().publish<msgs::Twist>(
          topic, std::static_pointer_cast<const msgs::Twist>(message), qos);
      break;
    case Kind::String:
      context.topics().publish<msgs::String>(
          topic, std::static_pointer_cast<const msgs::String>(message), qos);
      break;
    case Kind::TF:
      context.topics().publish<msgs::TFMessage>(
          topic, std::static_pointer_cast<const msgs::TFMessage>(message), qos);
      break;
    case Kind::Scan:
      context.topics().publish<msgs::LaserScan>(
          topic, std::static_pointer_cast<const msgs::LaserScan>(message), qos);
      break;
    case Kind::Image:
      context.topics().publish<msgs::Image>(
          topic, std::static_pointer_cast<const msgs::Image>(message), qos);
      break;
    case Kind::Imu:
      context.topics().publish<msgs::Imu>(
          topic, std::static_pointer_cast<const msgs::Imu>(message), qos);
      break;
    case Kind::Odometry:
      context.topics().publish<msgs::Odometry>(
          topic, std::static_pointer_cast<const msgs::Odometry>(message), qos);
      break;
    case Kind::JointState:
      context.topics().publish<msgs::JointState>(
          topic, std::static_pointer_cast<const msgs::JointState>(message), qos);
      break;
    case Kind::Grid:
      context.topics().publish<msgs::OccupancyGrid>(
          topic, std::static_pointer_cast<const msgs::OccupancyGrid>(message), qos);
      break;
    case Kind::Path:
      context.topics().publish<msgs::Path>(
          topic, std::static_pointer_cast<const msgs::Path>(message), qos);
      break;
    case Kind::Diagnostics:
      context.topics().publish<msgs::DiagnosticArray>(
          topic, std::static_pointer_cast<const msgs::DiagnosticArray>(message), qos);
      break;
    case Kind::Log:
      context.topics().publish<msgs::Log>(
          topic, std::static_pointer_cast<const msgs::Log>(message), qos);
      break;
    case Kind::Pose:
      context.topics().publish<msgs::PoseStamped>(
          topic, std::static_pointer_cast<const msgs::PoseStamped>(message), qos);
      break;
    case Kind::PoseCov:
      context.topics().publish<msgs::PoseWithCovarianceStamped>(
          topic, std::static_pointer_cast<const msgs::PoseWithCovarianceStamped>(message), qos);
      break;
    case Kind::Bool:
      context.topics().publish<msgs::Bool>(
          topic, std::static_pointer_cast<const msgs::Bool>(message), qos);
      break;
    case Kind::Empty:
      context.topics().publish<msgs::Empty>(
          topic, std::static_pointer_cast<const msgs::Empty>(message), qos);
      break;
    default:
      return false;
  }
  return true;
}

}  // namespace

// ============================================================================
// Server
// ============================================================================

struct Server::Impl {
  explicit Impl(Context& context_value, std::uint16_t port_value) : context(context_value), requested_port(port_value) {}
  ~Impl() { shutdown(); }

  // 클라이언트마다 송신 큐와 writer 스레드를 둔다. 느린 클라이언트가 다른 클라이언트의
  // 센서 스트림을 막지 않고, 새 클라이언트가 붙어도 기존 연결을 끊지 않는다.
  struct Connection {
    int fd = -1;
    std::string peer;
    std::atomic<bool> closed{false};
    std::mutex lifecycle_mutex;  // reader/writer 스레드가 모두 붙기 전에 close하지 않게 한다
    std::mutex send_mutex;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::deque<std::vector<std::uint8_t>> queue;
    std::thread reader;
    std::thread writer;
  };

  Context& context;
  std::uint16_t requested_port = 9000;
  std::uint16_t bound_port = 0;
  std::atomic<bool> stopping{false};
  std::atomic<bool> reaper_stop{false};
  std::atomic<int> listen_fd{-1};
  std::thread accept_thread;
  std::thread reaper_thread;
  std::mutex clients_mutex;
  std::vector<std::shared_ptr<Connection>> clients;
  std::mutex retired_mutex;
  std::condition_variable retired_cv;
  std::deque<std::shared_ptr<Connection>> retired;
  std::mutex latched_mutex;
  std::unordered_map<std::string, std::deque<std::vector<std::uint8_t>>> latched;
  std::vector<std::shared_ptr<krosc::detail::SubscriptionBase>> subscriptions;

  template <class T, class Encode>
  void subscribe_topic(const std::string& name, Kind kind, QoS qos, bool store_latched, Encode encode_message) {
    auto subscription = context.topics().subscribe<T>(
        name,
        [this, name, kind, store_latched, encode_message](const T& message) {
          if (stopping.load()) {
            return;
          }
          Writer writer;
          encode_message(writer, message);
          std::vector<std::uint8_t> frame = make_frame(kFrameTopic, name, kind, writer.data);
          if (store_latched) {
            std::lock_guard<std::mutex> lock(latched_mutex);
            auto& history = latched[name];
            history.push_back(frame);
            // tf_static은 RSP 고정 관절 + map->odom처럼 여러 메시지가 필요하다.
            // 나머지 래치 토픽은 최신 한 개만 있으면 되고, plan 같은 과거 기록을
            // 새 클라이언트에 100개씩 재생하지 않도록 한다.
            const std::size_t limit = name == "tf_static" ? 100 : 1;
            while (history.size() > limit) {
              history.pop_front();
            }
          }
          enqueue(frame);
        },
        qos);
    subscriptions.push_back(subscription.base());
  }

  void enqueue_to(const std::shared_ptr<Connection>& connection, const std::vector<std::uint8_t>& frame) {
    if (connection->closed.load()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(connection->queue_mutex);
      if (connection->queue.size() >= kMaxQueue) {
        connection->queue.pop_front();
      }
      connection->queue.push_back(frame);
    }
    connection->queue_cv.notify_one();
  }

  void enqueue(const std::vector<std::uint8_t>& frame) {
    if (stopping.load()) {
      return;
    }
    std::vector<std::shared_ptr<Connection>> snapshot;
    {
      std::lock_guard<std::mutex> lock(clients_mutex);
      snapshot = clients;
    }
    for (const auto& connection : snapshot) {
      enqueue_to(connection, frame);
    }
  }

  void close_client(const std::shared_ptr<Connection>& connection) {
    std::lock_guard<std::mutex> lifecycle(connection->lifecycle_mutex);
    if (connection->closed.exchange(true)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(connection->send_mutex);
      if (connection->fd >= 0) {
        close_socket(connection->fd);
        connection->fd = -1;
      }
    }
    connection->queue_cv.notify_all();
    {
      std::lock_guard<std::mutex> lock(clients_mutex);
      clients.erase(std::remove(clients.begin(), clients.end(), connection), clients.end());
    }
    std::printf("[krosc/net] client disconnected: %s\n", connection->peer.c_str());
    std::fflush(stdout);
    {
      std::lock_guard<std::mutex> lock(retired_mutex);
      retired.push_back(connection);
    }
    retired_cv.notify_one();
  }

  void reader_loop(const std::shared_ptr<Connection>& connection) {
    const int fd = connection->fd;
    while (!stopping.load() && !connection->closed.load()) {
      std::uint8_t type = 0;
      std::string topic;
      Kind kind = Kind::Twist;
      std::vector<std::uint8_t> payload;
      if (!read_frame(fd, type, topic, kind, payload)) {
        break;
      }
      if (type != kFramePublish) {
        continue;
      }
      Frame frame;
      frame.type = type;
      frame.topic = std::move(topic);
      frame.kind = kind;
      frame.payload = std::move(payload);
      (void)publish_frame(context, frame, true);
    }
    close_client(connection);
  }

  void writer_loop(const std::shared_ptr<Connection>& connection) {
    while (!stopping.load() && !connection->closed.load()) {
      std::vector<std::uint8_t> frame;
      {
        std::unique_lock<std::mutex> lock(connection->queue_mutex);
        connection->queue_cv.wait(lock, [this, &connection] {
          return stopping.load() || connection->closed.load() || !connection->queue.empty();
        });
        if (stopping.load() || connection->closed.load()) {
          return;
        }
        if (connection->queue.empty()) {
          continue;
        }
        frame = std::move(connection->queue.front());
        connection->queue.pop_front();
      }
      bool failed = false;
      {
        std::lock_guard<std::mutex> lock(connection->send_mutex);
        if (connection->closed.load()) {
          return;
        }
        failed = !send_all(connection->fd, frame);
      }
      if (failed) {
        close_client(connection);
        return;
      }
    }
  }

  void reaper_loop() {
    while (true) {
      std::shared_ptr<Connection> connection;
      {
        std::unique_lock<std::mutex> lock(retired_mutex);
        retired_cv.wait(lock, [this] { return reaper_stop.load() || !retired.empty(); });
        if (retired.empty()) {
          if (reaper_stop.load()) {
            return;
          }
          continue;
        }
        connection = retired.front();
        retired.pop_front();
      }
      if (connection->reader.joinable()) {
        connection->reader.join();
      }
      if (connection->writer.joinable()) {
        connection->writer.join();
      }
    }
  }

  void accept_loop() {
    while (!stopping.load()) {
      sockaddr_in address{};
      socklen_t length = sizeof(address);
      const int fd = ::accept(listen_fd.load(), reinterpret_cast<sockaddr*>(&address), &length);
      if (fd < 0) {
        if (stopping.load()) {
          return;
        }
        continue;
      }
      if (stopping.load()) {
        close_socket(fd);
        return;
      }
      {
        std::lock_guard<std::mutex> lock(clients_mutex);
        if (clients.size() >= kMaxClients) {
          close_socket(fd);
          std::printf("[krosc/net] client rejected (max %zu clients)\n", kMaxClients);
          std::fflush(stdout);
          continue;
        }
      }
      configure_socket(fd);
      auto connection = std::make_shared<Connection>();
      connection->fd = fd;
      connection->peer = ::inet_ntoa(address.sin_addr);
      std::lock_guard<std::mutex> lifecycle(connection->lifecycle_mutex);
      // 스레드를 먼저 띄우고 lifecycle 락을 잡은 채 clients에 등록한다. reader가
      // 즉시 끊겨도 close_client가 스레드 생성 중간에 join하지 않는다.
      connection->reader = std::thread([this, connection] { reader_loop(connection); });
      connection->writer = std::thread([this, connection] { writer_loop(connection); });
      std::size_t count = 0;
      {
        std::lock_guard<std::mutex> lock(clients_mutex);
        clients.push_back(connection);
        count = clients.size();
      }
      {
        std::lock_guard<std::mutex> lock(latched_mutex);
        for (const auto& [name, history] : latched) {
          (void)name;
          for (const auto& frame : history) {
            enqueue_to(connection, frame);
          }
        }
      }
      std::printf("[krosc/net] client connected: %s (%zu clients)\n", connection->peer.c_str(), count);
      std::fflush(stdout);
    }
  }

  void shutdown() {
    if (stopping.exchange(true)) {
      return;
    }
    const int fd = listen_fd.exchange(-1);
    if (fd >= 0) {
      close_socket(fd);
    }
    if (accept_thread.joinable()) {
      accept_thread.join();
    }
    std::vector<std::shared_ptr<Connection>> snapshot;
    {
      std::lock_guard<std::mutex> lock(clients_mutex);
      snapshot = clients;
      clients.clear();
    }
    for (const auto& connection : snapshot) {
      close_client(connection);
    }
    {
      std::lock_guard<std::mutex> lock(retired_mutex);
      reaper_stop.store(true);
    }
    retired_cv.notify_all();
    if (reaper_thread.joinable()) {
      reaper_thread.join();
    }
    for (auto& subscription : subscriptions) {
      subscription->cancel();
    }
    subscriptions.clear();
  }
};

Server::Server(Context& context, std::uint16_t port) : impl_(std::make_unique<Impl>(context, port)) {
  impl_->subscribe_topic<msgs::TFMessage>(
      "tf", Kind::TF, QOS_TF, false, [](Writer& w, const msgs::TFMessage& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::TFMessage>(
      "tf_static", Kind::TF, QOS_TF_STATIC, true, [](Writer& w, const msgs::TFMessage& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "robot_description", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::OccupancyGrid>(
      "map", Kind::Grid, QOS_LATCHED, true,
      [](Writer& w, const msgs::OccupancyGrid& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::OccupancyGrid>(
      "global_costmap/costmap", Kind::Grid, QOS_LATCHED, true,
      [](Writer& w, const msgs::OccupancyGrid& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::OccupancyGrid>(
      "local_costmap/costmap", Kind::Grid, QOS_LATCHED, true,
      [](Writer& w, const msgs::OccupancyGrid& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Path>(
      "plan", Kind::Path, QOS_LATCHED, true, [](Writer& w, const msgs::Path& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Path>(
      "local_plan", Kind::Path, QOS_SENSOR_DATA, false,
      [](Writer& w, const msgs::Path& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::LaserScan>(
      "scan", Kind::Scan, QOS_SENSOR_DATA, false,
      [](Writer& w, const msgs::LaserScan& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Imu>(
      "imu", Kind::Imu, QOS_SENSOR_DATA, false, [](Writer& w, const msgs::Imu& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Odometry>(
      "odom", Kind::Odometry, QOS_SENSOR_DATA, false,
      [](Writer& w, const msgs::Odometry& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Odometry>(
      "odometry/filtered", Kind::Odometry, QOS_SENSOR_DATA, false,
      [](Writer& w, const msgs::Odometry& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::JointState>(
      "joint_states", Kind::JointState, QOS_DEFAULT, false,
      [](Writer& w, const msgs::JointState& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::DiagnosticArray>(
      "diagnostics", Kind::Diagnostics, QOS_DEFAULT, false,
      [](Writer& w, const msgs::DiagnosticArray& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Log>(
      "rosout", Kind::Log, QOS_DEFAULT, false, [](Writer& w, const msgs::Log& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "map_list", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "map_status", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "slam_status", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "goal_status", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "collision_status", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "arm_status", Kind::String, QOS_DEFAULT, false,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "rl_nav_status", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "rl_nav_model_list", Kind::String, QOS_LATCHED, true,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "moveit_status", Kind::String, QOS_DEFAULT, false,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::String>(
      "moveit_state", Kind::String, QOS_DEFAULT, false,
      [](Writer& w, const msgs::String& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::PoseStamped>(
      "moveit_target_pose", Kind::Pose, QOS_DEFAULT, false,
      [](Writer& w, const msgs::PoseStamped& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Path>(
      "moveit_arm", Kind::Path, QOS_DEFAULT, false,
      [](Writer& w, const msgs::Path& message) { encode(w, message); });
  impl_->subscribe_topic<msgs::Path>(
      "moveit_path", Kind::Path, QOS_DEFAULT, false,
      [](Writer& w, const msgs::Path& message) { encode(w, message); });

  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    throw std::runtime_error("net::Server: socket() failed");
  }
  int enabled = 1;
  (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port);
  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    close_socket(fd);
    throw std::runtime_error("net::Server: bind failed (port " + std::to_string(port) + ")");
  }
  if (::listen(fd, 16) != 0) {
    close_socket(fd);
    throw std::runtime_error("net::Server: listen failed");
  }
  socklen_t length = sizeof(address);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) == 0) {
    impl_->bound_port = ntohs(address.sin_port);
  } else {
    impl_->bound_port = port;
  }
  impl_->listen_fd.store(fd);
  impl_->reaper_thread = std::thread([impl = impl_.get()] { impl->reaper_loop(); });
  impl_->accept_thread = std::thread([impl = impl_.get()] { impl->accept_loop(); });
}

Server::~Server() { shutdown(); }

std::uint16_t Server::port() const { return impl_ ? impl_->bound_port : 0; }

bool Server::running() const { return impl_ && impl_->listen_fd.load() >= 0 && !impl_->stopping.load(); }

void Server::shutdown() {
  if (impl_) {
    impl_->shutdown();
  }
}

// ============================================================================
// Client
// ============================================================================

struct Client::Impl {
  Impl(Context& context_value, std::string host_value, std::uint16_t port_value)
      : context(context_value), host(std::move(host_value)), port(port_value) {}

  Context& context;
  std::string host;
  std::uint16_t port = 9000;
  std::atomic<bool> stopping{false};
  std::atomic<bool> connected{false};
  std::atomic<int> fd{-1};
  std::thread reader_thread;
  std::thread writer_thread;
  std::mutex socket_mutex;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<std::vector<std::uint8_t>> queue;
  std::shared_ptr<krosc::detail::SubscriptionBase> command_subscription;
  std::vector<std::shared_ptr<krosc::detail::SubscriptionBase>> control_subscriptions;

  template <class T, class Encode>
  void forward_topic(const std::string& topic, Kind kind, Encode encode_message) {
    auto subscription = context.topics().subscribe<T>(
        topic,
        [this, topic, kind, encode_message](const T& message) {
          if (!connected.load() || stopping.load()) {
            return;
          }
          Writer writer;
          encode_message(writer, message);
          auto frame = make_frame(kFramePublish, topic, kind, writer.data);
          {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (queue.size() >= kMaxQueue) {
              queue.pop_front();
            }
            queue.push_back(std::move(frame));
          }
          queue_cv.notify_one();
        },
        QOS_DEFAULT);
    control_subscriptions.push_back(subscription.base());
  }

  void close_socket() {
    const int current = fd.exchange(-1);
    connected.store(false);
    if (current >= 0) {
      std::lock_guard<std::mutex> lock(socket_mutex);
      net::close_socket(current);
    }
    queue_cv.notify_all();
  }

  int connect_to_server() {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* result = nullptr;
    const std::string service = std::to_string(port);
    if (::getaddrinfo(host.c_str(), service.c_str(), &hints, &result) != 0 || result == nullptr) {
      return -1;
    }
    int socket_fd = -1;
    for (addrinfo* address = result; address != nullptr; address = address->ai_next) {
      socket_fd = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
      if (socket_fd < 0) {
        continue;
      }
      configure_socket(socket_fd);
      if (::connect(socket_fd, address->ai_addr, address->ai_addrlen) == 0) {
        break;
      }
      net::close_socket(socket_fd);
      socket_fd = -1;
    }
    ::freeaddrinfo(result);
    return socket_fd;
  }

  void publish_remote(const std::string& topic, Kind kind, const std::vector<std::uint8_t>& payload) {
    switch (kind) {
      case Kind::TF: {
        Reader reader(payload);
        if (auto message = decode_tf(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::String: {
        Reader reader(payload);
        if (auto message = decode_string(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Scan: {
        Reader reader(payload);
        if (auto message = decode_scan(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Image: {
        Reader reader(payload);
        if (auto message = decode_image(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Imu: {
        Reader reader(payload);
        if (auto message = decode_imu(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Odometry: {
        Reader reader(payload);
        if (auto message = decode_odometry(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::JointState: {
        Reader reader(payload);
        if (auto message = decode_joint_state(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Grid: {
        Reader reader(payload);
        if (auto message = decode_grid(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Path: {
        Reader reader(payload);
        if (auto message = decode_path(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Diagnostics: {
        Reader reader(payload);
        if (auto message = decode_diagnostics(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Log: {
        Reader reader(payload);
        if (auto message = decode_log(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Pose: {
        Reader reader(payload);
        if (auto message = decode_pose_stamped(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::PoseCov: {
        Reader reader(payload);
        if (auto message = decode_pose_covariance(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Empty:
        context.topics().publish(topic, msgs::Empty{}, qos_for_topic(topic));
        break;
      case Kind::Bool: {
        Reader reader(payload);
        if (auto message = decode_bool(reader)) {
          context.topics().publish(topic, std::move(*message), qos_for_topic(topic));
        }
        break;
      }
      case Kind::Twist:
        break;
      default:
        break;
    }
  }

  void reader_loop() {
    while (!stopping.load()) {
      const int socket_fd = connect_to_server();
      if (socket_fd < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(socket_mutex);
        if (stopping.load()) {
          net::close_socket(socket_fd);
          return;
        }
        fd.store(socket_fd);
        connected.store(true);
      }
      std::printf("[krosc/net] connected to %s:%u\n", host.c_str(), static_cast<unsigned>(port));
      std::fflush(stdout);
      queue_cv.notify_all();
      while (!stopping.load()) {
        std::uint8_t type = 0;
        std::string topic;
        Kind kind = Kind::Twist;
        std::vector<std::uint8_t> payload;
        if (!read_frame(socket_fd, type, topic, kind, payload)) {
          break;
        }
        if (type == kFrameTopic) {
          publish_remote(topic, kind, payload);
        }
      }
      connected.store(false);
      {
        std::lock_guard<std::mutex> lock(socket_mutex);
        if (fd.load() == socket_fd) {
          fd.store(-1);
        }
        net::close_socket(socket_fd);
      }
      queue_cv.notify_all();
      if (!stopping.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }

  void writer_loop() {
    while (!stopping.load()) {
      std::vector<std::uint8_t> frame;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [this] { return stopping.load() || (!queue.empty() && connected.load()); });
        if (stopping.load()) {
          return;
        }
        if (queue.empty() || !connected.load()) {
          continue;
        }
        frame = std::move(queue.front());
        queue.pop_front();
      }
      const int socket_fd = fd.load();
      if (socket_fd < 0) {
        continue;
      }
      std::lock_guard<std::mutex> lock(socket_mutex);
      if (fd.load() != socket_fd) {
        continue;
      }
      if (!send_all(socket_fd, frame)) {
        fd.store(-1);
        connected.store(false);
        net::close_socket(socket_fd);
        queue_cv.notify_all();
      }
    }
  }

  void shutdown() {
    if (stopping.exchange(true)) {
      return;
    }
    close_socket();
    if (reader_thread.joinable()) {
      reader_thread.join();
    }
    if (writer_thread.joinable()) {
      writer_thread.join();
    }
    if (command_subscription) {
      command_subscription->cancel();
      command_subscription.reset();
    }
    for (auto& subscription : control_subscriptions) {
      if (subscription) {
        subscription->cancel();
      }
    }
    control_subscriptions.clear();
  }
};

Client::Client(Context& context, std::string host, std::uint16_t port)
    : impl_(std::make_unique<Impl>(context, std::move(host), port)) {
  auto command_subscription = context.topics().subscribe<msgs::Twist>(
      "cmd_vel",
      [this](const msgs::Twist& twist) {
        if (!impl_->connected.load() || impl_->stopping.load()) {
          return;
        }
        Writer writer;
        encode(writer, twist);
        std::vector<std::uint8_t> frame = make_frame(kFramePublish, "cmd_vel", Kind::Twist, writer.data);
        {
          std::lock_guard<std::mutex> lock(impl_->queue_mutex);
          if (impl_->queue.size() >= kMaxQueue) {
            impl_->queue.pop_front();
          }
          impl_->queue.push_back(std::move(frame));
        }
        impl_->queue_cv.notify_one();
      },
      QOS_COMMAND);
  impl_->command_subscription = command_subscription.base();
  impl_->forward_topic<msgs::PoseWithCovarianceStamped>(
      "initialpose", Kind::PoseCov,
      [](Writer& writer, const msgs::PoseWithCovarianceStamped& message) { encode(writer, message); });
  impl_->forward_topic<msgs::JointState>(
      "arm_joint_command", Kind::JointState,
      [](Writer& writer, const msgs::JointState& message) { encode(writer, message); });
  impl_->reader_thread = std::thread([impl = impl_.get()] { impl->reader_loop(); });
  impl_->writer_thread = std::thread([impl = impl_.get()] { impl->writer_loop(); });
}

Client::~Client() { shutdown(); }

bool Client::connected() const { return impl_ && impl_->connected.load(); }

std::string Client::endpoint() const {
  return impl_ ? impl_->host + ":" + std::to_string(impl_->port) : std::string{};
}

void Client::shutdown() {
  if (impl_) {
    impl_->shutdown();
  }
}


// ---------------------------------------------------------------------------
// 타입 소거 메시지 코덱 (peer 네트워크용)
// ---------------------------------------------------------------------------

std::optional<Kind> message_kind(std::type_index type) {
  if (type == typeid(msgs::Twist)) return Kind::Twist;
  if (type == typeid(msgs::String)) return Kind::String;
  if (type == typeid(msgs::TFMessage)) return Kind::TF;
  if (type == typeid(msgs::LaserScan)) return Kind::Scan;
  if (type == typeid(msgs::Image)) return Kind::Image;
  if (type == typeid(msgs::Imu)) return Kind::Imu;
  if (type == typeid(msgs::Odometry)) return Kind::Odometry;
  if (type == typeid(msgs::JointState)) return Kind::JointState;
  if (type == typeid(msgs::OccupancyGrid)) return Kind::Grid;
  if (type == typeid(msgs::Path)) return Kind::Path;
  if (type == typeid(msgs::DiagnosticArray)) return Kind::Diagnostics;
  if (type == typeid(msgs::Log)) return Kind::Log;
  if (type == typeid(msgs::PoseStamped)) return Kind::Pose;
  if (type == typeid(msgs::PoseWithCovarianceStamped)) return Kind::PoseCov;
  if (type == typeid(msgs::Bool)) return Kind::Bool;
  if (type == typeid(msgs::Empty)) return Kind::Empty;
  return std::nullopt;
}

std::vector<std::uint8_t> encode_message(std::type_index type, const void* message) {
  Writer writer;
  if (type == typeid(msgs::Twist)) {
    encode(writer, *static_cast<const msgs::Twist*>(message));
  } else if (type == typeid(msgs::String)) {
    encode(writer, *static_cast<const msgs::String*>(message));
  } else if (type == typeid(msgs::TFMessage)) {
    encode(writer, *static_cast<const msgs::TFMessage*>(message));
  } else if (type == typeid(msgs::LaserScan)) {
    encode(writer, *static_cast<const msgs::LaserScan*>(message));
  } else if (type == typeid(msgs::Image)) {
    encode(writer, *static_cast<const msgs::Image*>(message));
  } else if (type == typeid(msgs::Imu)) {
    encode(writer, *static_cast<const msgs::Imu*>(message));
  } else if (type == typeid(msgs::Odometry)) {
    encode(writer, *static_cast<const msgs::Odometry*>(message));
  } else if (type == typeid(msgs::JointState)) {
    encode(writer, *static_cast<const msgs::JointState*>(message));
  } else if (type == typeid(msgs::OccupancyGrid)) {
    encode(writer, *static_cast<const msgs::OccupancyGrid*>(message));
  } else if (type == typeid(msgs::Path)) {
    encode(writer, *static_cast<const msgs::Path*>(message));
  } else if (type == typeid(msgs::DiagnosticArray)) {
    encode(writer, *static_cast<const msgs::DiagnosticArray*>(message));
  } else if (type == typeid(msgs::Log)) {
    encode(writer, *static_cast<const msgs::Log*>(message));
  } else if (type == typeid(msgs::PoseStamped)) {
    encode(writer, *static_cast<const msgs::PoseStamped*>(message));
  } else if (type == typeid(msgs::PoseWithCovarianceStamped)) {
    encode(writer, *static_cast<const msgs::PoseWithCovarianceStamped*>(message));
  } else if (type == typeid(msgs::Bool)) {
    encode(writer, *static_cast<const msgs::Bool*>(message));
  } else if (type == typeid(msgs::Empty)) {
    encode(writer, *static_cast<const msgs::Empty*>(message));
  }
  return writer.data;
}

std::shared_ptr<const void> decode_message(Kind kind, const std::uint8_t* data, std::size_t size) {
  Reader reader(data, size);
  switch (kind) {
    case Kind::Twist:
      if (const auto message = decode_twist(reader); reader.ok()) {
        return std::make_shared<const msgs::Twist>(message);
      }
      break;
    case Kind::String:
      if (auto message = decode_string(reader)) return std::make_shared<const msgs::String>(std::move(*message));
      break;
    case Kind::TF:
      if (auto message = decode_tf(reader)) return std::make_shared<const msgs::TFMessage>(std::move(*message));
      break;
    case Kind::Scan:
      if (auto message = decode_scan(reader)) return std::make_shared<const msgs::LaserScan>(std::move(*message));
      break;
    case Kind::Image:
      if (auto message = decode_image(reader)) return std::make_shared<const msgs::Image>(std::move(*message));
      break;
    case Kind::Imu:
      if (auto message = decode_imu(reader)) return std::make_shared<const msgs::Imu>(std::move(*message));
      break;
    case Kind::Odometry:
      if (auto message = decode_odometry(reader)) return std::make_shared<const msgs::Odometry>(std::move(*message));
      break;
    case Kind::JointState:
      if (auto message = decode_joint_state(reader)) return std::make_shared<const msgs::JointState>(std::move(*message));
      break;
    case Kind::Grid:
      if (auto message = decode_grid(reader)) return std::make_shared<const msgs::OccupancyGrid>(std::move(*message));
      break;
    case Kind::Path:
      if (auto message = decode_path(reader)) return std::make_shared<const msgs::Path>(std::move(*message));
      break;
    case Kind::Diagnostics:
      if (auto message = decode_diagnostics(reader)) return std::make_shared<const msgs::DiagnosticArray>(std::move(*message));
      break;
    case Kind::Log:
      if (auto message = decode_log(reader)) return std::make_shared<const msgs::Log>(std::move(*message));
      break;
    case Kind::Pose:
      if (auto message = decode_pose_stamped(reader)) return std::make_shared<const msgs::PoseStamped>(std::move(*message));
      break;
    case Kind::PoseCov:
      if (auto message = decode_pose_covariance(reader)) return std::make_shared<const msgs::PoseWithCovarianceStamped>(std::move(*message));
      break;
    case Kind::Bool:
      if (auto message = decode_bool(reader)) return std::make_shared<const msgs::Bool>(*message);
      break;
    case Kind::Empty:
      return std::make_shared<const msgs::Empty>();
    default:
      break;
  }
  return {};
}

}  // namespace krosc::net
