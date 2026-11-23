#include "krosc/nodes/ekf.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace krosc::nodes {

namespace {

constexpr int X = 0;
constexpr int Y = 1;
constexpr int YAW = 2;
constexpr int V = 3;
constexpr int W = 4;
constexpr double kFloor = 1e-9;
constexpr std::array<int, 5> kConfigIndex = {0, 1, 5, 6, 11};  // robot_localization 15개 순서에서의 자리

double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

using Matrix5 = std::array<double, 25>;

Matrix5 identity5() {
  Matrix5 result{};
  for (int i = 0; i < 5; ++i) {
    result[static_cast<std::size_t>(i * 5 + i)] = 1.0;
  }
  return result;
}

Matrix5 multiply5(const Matrix5& a, const Matrix5& b) {
  Matrix5 result{};
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      double sum = 0.0;
      for (int k = 0; k < 5; ++k) {
        sum += a[static_cast<std::size_t>(r * 5 + k)] * b[static_cast<std::size_t>(k * 5 + c)];
      }
      result[static_cast<std::size_t>(r * 5 + c)] = sum;
    }
  }
  return result;
}

Matrix5 transpose5(const Matrix5& a) {
  Matrix5 result{};
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      result[static_cast<std::size_t>(r * 5 + c)] = a[static_cast<std::size_t>(c * 5 + r)];
    }
  }
  return result;
}

Matrix5 diagonal5(const std::vector<double>& values15) {
  Matrix5 result{};
  for (int i = 0; i < 5; ++i) {
    result[static_cast<std::size_t>(i * 5 + i)] = values15[static_cast<std::size_t>(kConfigIndex[i])];
  }
  return result;
}

std::vector<double> inverse(const std::vector<double>& input, int n) {
  std::vector<double> matrix(static_cast<std::size_t>(n * n * 2));
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      matrix[static_cast<std::size_t>(r * (2 * n) + c)] = input[static_cast<std::size_t>(r * n + c)];
      matrix[static_cast<std::size_t>(r * (2 * n) + n + c)] = r == c ? 1.0 : 0.0;
    }
  }
  for (int col = 0; col < n; ++col) {
    int pivot = col;
    double best = std::abs(matrix[static_cast<std::size_t>(col * (2 * n) + col)]);
    for (int r = col + 1; r < n; ++r) {
      const double value = std::abs(matrix[static_cast<std::size_t>(r * (2 * n) + col)]);
      if (value > best) {
        best = value;
        pivot = r;
      }
    }
    if (best < 1e-15) {
      // 공분산이 특이하면 단위행렬로 대신한다 (수치적으로 망가진 측정은 버린다).
      return std::vector<double>(static_cast<std::size_t>(n * n), 0.0);
    }
    if (pivot != col) {
      for (int c = 0; c < 2 * n; ++c) {
        std::swap(matrix[static_cast<std::size_t>(col * (2 * n) + c)],
                  matrix[static_cast<std::size_t>(pivot * (2 * n) + c)]);
      }
    }
    const double scale = matrix[static_cast<std::size_t>(col * (2 * n) + col)];
    for (int c = 0; c < 2 * n; ++c) {
      matrix[static_cast<std::size_t>(col * (2 * n) + c)] /= scale;
    }
    for (int r = 0; r < n; ++r) {
      if (r == col) {
        continue;
      }
      const double factor = matrix[static_cast<std::size_t>(r * (2 * n) + col)];
      if (factor == 0.0) {
        continue;
      }
      for (int c = 0; c < 2 * n; ++c) {
        matrix[static_cast<std::size_t>(r * (2 * n) + c)] -=
            factor * matrix[static_cast<std::size_t>(col * (2 * n) + c)];
      }
    }
  }
  std::vector<double> result(static_cast<std::size_t>(n * n));
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      result[static_cast<std::size_t>(r * n + c)] = matrix[static_cast<std::size_t>(r * (2 * n) + n + c)];
    }
  }
  return result;
}

double yaw_of(const msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool valid_flags(const std::vector<bool>& value) { return value.size() == 15; }
bool valid_diagonal(const std::vector<double>& value) {
  if (value.size() != 15) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](double item) { return item >= 0.0; });
}

}  // namespace

Ekf::Ekf(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : Node(context, std::move(name), std::move(parameters)),
      frequency_(declare_parameter<double>("frequency", 30.0, [](double value) { return value > 0.0; })),
      odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
      base_link_frame_(declare_parameter<std::string>("base_link_frame", "base_footprint")),
      publish_tf_(declare_parameter<bool>("publish_tf", true)),
      odom0_(declare_parameter<std::string>("odom0", "")),
      odom0_config_(declare_parameter<std::vector<bool>>("odom0_config", std::vector<bool>(15, false), valid_flags)),
      odom0_pose_rejection_threshold_(
          declare_parameter<double>("odom0_pose_rejection_threshold", std::numeric_limits<double>::infinity(),
                                    [](double value) { return value > 0.0; })),
      odom0_twist_rejection_threshold_(
          declare_parameter<double>("odom0_twist_rejection_threshold", std::numeric_limits<double>::infinity(),
                                    [](double value) { return value > 0.0; })),
      imu0_(declare_parameter<std::string>("imu0", "")),
      imu0_config_(declare_parameter<std::vector<bool>>("imu0_config", std::vector<bool>(15, false), valid_flags)),
      imu0_relative_(declare_parameter<bool>("imu0_relative", false)),
      imu0_pose_rejection_threshold_(
          declare_parameter<double>("imu0_pose_rejection_threshold", std::numeric_limits<double>::infinity(),
                                    [](double value) { return value > 0.0; })),
      imu0_twist_rejection_threshold_(
          declare_parameter<double>("imu0_twist_rejection_threshold", std::numeric_limits<double>::infinity(),
                                    [](double value) { return value > 0.0; })),
      process_noise_covariance_(declare_parameter<std::vector<double>>(
          "process_noise_covariance",
          {0.05, 0.05, 0.06, 0.03, 0.03, 0.06, 0.025, 0.025, 0.04, 0.01, 0.01, 0.02, 0.01, 0.01, 0.015},
          valid_diagonal)),
      initial_estimate_covariance_(declare_parameter<std::vector<double>>(
          "initial_estimate_covariance", std::vector<double>(15, 1e-9), valid_diagonal)),
      broadcaster_(*this) {
  covariance_ = diagonal5(initial_estimate_covariance_.value());
  if (!odom0_.value().empty()) {
    odom_subscription_ =
        subscribe<msgs::Odometry>(odom0_.value(), [this](const msgs::Odometry& message) { on_odom(message); });
  }
  if (!imu0_.value().empty()) {
    imu_subscription_ = subscribe<msgs::Imu>(imu0_.value(), [this](const msgs::Imu& message) { on_imu(message); });
  }
  timer_ = create_timer(1.0 / frequency_.value(), [this] { on_timer(); });
}

Ekf::Measurement Ekf::pick(const std::vector<bool>& config, const Measurement& values) const {
  Measurement picked;
  for (const auto& item : values) {
    if (const auto index = static_cast<std::size_t>(kConfigIndex[static_cast<std::size_t>(item.state_index)]);
        index < config.size() && config[index]) {
      picked.push_back({item.state_index, item.value, std::max(item.variance, kFloor)});
    }
  }
  return picked;
}

void Ekf::enqueue(double stamp, Measurement pose, Measurement twist, double pose_threshold, double twist_threshold) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  queue_.push_back(Pending{stamp, std::move(pose), std::move(twist), pose_threshold, twist_threshold});
}

void Ekf::on_odom(const msgs::Odometry& message) {
  Measurement pose_values = {
      {X, message.pose.pose.position.x, message.pose.covariance[0]},
      {Y, message.pose.pose.position.y, message.pose.covariance[7]},
      {YAW, yaw_of(message.pose.pose.orientation), message.pose.covariance[35]},
  };
  Measurement twist_values = {
      {V, message.twist.twist.linear.x, message.twist.covariance[0]},
      {W, message.twist.twist.angular.z, message.twist.covariance[35]},
  };
  enqueue(message.header.stamp, pick(odom0_config_.value(), pose_values),
          pick(odom0_config_.value(), twist_values), odom0_pose_rejection_threshold_.value(),
          odom0_twist_rejection_threshold_.value());
}

void Ekf::on_imu(const msgs::Imu& message) {
  Measurement pose_values;
  if (message.orientation_covariance[0] != -1.0) {  // ROS: -1이면 방향 값이 없다
    double yaw = yaw_of(message.orientation);
    if (imu0_relative_.value()) {
      if (!imu_yaw_offset_) {
        imu_yaw_offset_ = yaw;
      }
      yaw = wrap_angle(yaw - *imu_yaw_offset_);
    }
    pose_values.push_back({YAW, yaw, message.orientation_covariance[8]});
  }
  Measurement twist_values = {{W, message.angular_velocity.z, message.angular_velocity_covariance[8]}};
  enqueue(message.header.stamp, pick(imu0_config_.value(), pose_values),
          pick(imu0_config_.value(), twist_values), imu0_pose_rejection_threshold_.value(),
          imu0_twist_rejection_threshold_.value());
}

void Ekf::on_timer() {
  std::vector<Pending> queue;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue.swap(queue_);
  }
  std::stable_sort(queue.begin(), queue.end(),
                   [](const Pending& a, const Pending& b) { return a.stamp < b.stamp; });
  for (const auto& pending : queue) {
    if (!time_) {
      time_ = pending.stamp;  // 첫 값의 시각에서 원점으로 시작한다
    }
    if (pending.stamp < *time_) {
      late_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    predict(pending.stamp - *time_);
    time_ = pending.stamp;
    correct(pending.pose, pending.pose_threshold);
    correct(pending.twist, pending.twist_threshold);
  }
  if (time_) {
    publish_filtered(*time_);
  }
}

void Ekf::predict(double dt) {
  if (dt <= 0.0) {
    return;
  }
  const double x = state_[X];
  const double y = state_[Y];
  const double yaw = state_[YAW];
  const double v = state_[V];
  const double w = state_[W];
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  state_ = {x + v * c * dt, y + v * s * dt, wrap_angle(yaw + w * dt), v, w};

  Matrix5 f = identity5();
  f[static_cast<std::size_t>(X * 5 + YAW)] = -v * s * dt;
  f[static_cast<std::size_t>(X * 5 + V)] = c * dt;
  f[static_cast<std::size_t>(Y * 5 + YAW)] = v * c * dt;
  f[static_cast<std::size_t>(Y * 5 + V)] = s * dt;
  f[static_cast<std::size_t>(YAW * 5 + W)] = dt;
  const Matrix5 q = diagonal5(process_noise_covariance_.value());
  covariance_ = multiply5(multiply5(f, covariance_), transpose5(f));
  for (int i = 0; i < 5; ++i) {
    covariance_[static_cast<std::size_t>(i * 5 + i)] += q[static_cast<std::size_t>(i * 5 + i)] * dt;
  }
}

bool Ekf::correct(const Measurement& measurement, double threshold) {
  if (measurement.empty()) {
    return false;
  }
  const int n = static_cast<int>(measurement.size());
  std::vector<int> indices(static_cast<std::size_t>(n));
  std::vector<double> innovation(static_cast<std::size_t>(n));
  std::vector<double> s(static_cast<std::size_t>(n * n));
  for (int r = 0; r < n; ++r) {
    const auto& item = measurement[static_cast<std::size_t>(r)];
    indices[static_cast<std::size_t>(r)] = item.state_index;
    const double difference = item.value - state_[static_cast<std::size_t>(item.state_index)];
    innovation[static_cast<std::size_t>(r)] =
        item.state_index == YAW ? wrap_angle(difference) : difference;
    for (int c = 0; c < n; ++c) {
      const auto& other = measurement[static_cast<std::size_t>(c)];
      double value = covariance_[static_cast<std::size_t>(item.state_index * 5 + other.state_index)];
      if (r == c) {
        value += item.variance;
      }
      s[static_cast<std::size_t>(r * n + c)] = value;
    }
  }
  const std::vector<double> s_inv = inverse(s, n);
  double mahalanobis = 0.0;
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      mahalanobis += innovation[static_cast<std::size_t>(r)] * s_inv[static_cast<std::size_t>(r * n + c)] *
                     innovation[static_cast<std::size_t>(c)];
    }
  }
  if (mahalanobis > threshold * threshold) {
    rejected_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  std::vector<double> gain(static_cast<std::size_t>(5 * n));
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < n; ++c) {
      double value = 0.0;
      for (int k = 0; k < n; ++k) {
        value += covariance_[static_cast<std::size_t>(r * 5 + indices[static_cast<std::size_t>(k)])] *
                 s_inv[static_cast<std::size_t>(k * n + c)];
      }
      gain[static_cast<std::size_t>(r * n + c)] = value;
    }
  }
  std::array<double, 5> correction{};
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < n; ++c) {
      correction[static_cast<std::size_t>(r)] +=
          gain[static_cast<std::size_t>(r * n + c)] * innovation[static_cast<std::size_t>(c)];
    }
  }
  for (int i = 0; i < 5; ++i) {
    state_[static_cast<std::size_t>(i)] += correction[static_cast<std::size_t>(i)];
  }
  state_[YAW] = wrap_angle(state_[YAW]);

  Matrix5 reduced{};
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      double value = 0.0;
      for (int k = 0; k < n; ++k) {
        value += gain[static_cast<std::size_t>(r * n + k)] *
                 covariance_[static_cast<std::size_t>(indices[static_cast<std::size_t>(k)] * 5 + c)];
      }
      reduced[static_cast<std::size_t>(r * 5 + c)] = value;
    }
  }
  Matrix5 updated{};
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      updated[static_cast<std::size_t>(r * 5 + c)] =
          covariance_[static_cast<std::size_t>(r * 5 + c)] - reduced[static_cast<std::size_t>(r * 5 + c)];
    }
  }
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      covariance_[static_cast<std::size_t>(r * 5 + c)] =
          (updated[static_cast<std::size_t>(r * 5 + c)] + updated[static_cast<std::size_t>(c * 5 + r)]) / 2.0;
    }
  }
  return true;
}

void Ekf::publish_filtered(double stamp) {
  const double x = state_[X];
  const double y = state_[Y];
  const double yaw = state_[YAW];
  const double v = state_[V];
  const double w = state_[W];

  msgs::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_.value();
  odometry.child_frame_id = base_link_frame_.value();
  odometry.pose.pose.position = {x, y, 0.0};
  odometry.pose.pose.orientation = tfmath::from_yaw(yaw);
  for (const int a : {X, Y, YAW}) {
    for (const int b : {X, Y, YAW}) {
      const int row = a == X ? 0 : (a == Y ? 1 : 5);
      const int col = b == X ? 0 : (b == Y ? 1 : 5);
      odometry.pose.covariance[static_cast<std::size_t>(row * 6 + col)] =
          covariance_[static_cast<std::size_t>(a * 5 + b)];
    }
  }
  odometry.twist.twist.linear.x = v;
  odometry.twist.twist.angular.z = w;
  odometry.twist.covariance[0] = covariance_[static_cast<std::size_t>(V * 5 + V)];
  odometry.twist.covariance[5] = covariance_[static_cast<std::size_t>(V * 5 + W)];
  odometry.twist.covariance[30] = covariance_[static_cast<std::size_t>(W * 5 + V)];
  odometry.twist.covariance[35] = covariance_[static_cast<std::size_t>(W * 5 + W)];
  publish(FILTERED_TOPIC_NAME, std::move(odometry));

  if (publish_tf_.value()) {
    msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_.value();
    transform.child_frame_id = base_link_frame_.value();
    transform.transform.translation = {x, y, 0.0};
    transform.transform.rotation = tfmath::from_yaw(yaw);
    broadcaster_.send_transform(transform);
  }
}

}  // namespace krosc::nodes
