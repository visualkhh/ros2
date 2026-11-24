#include "opencr_serial.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <thread>
#include <utility>

#include "serial.hpp"

namespace krosc_ws {

namespace {
constexpr double kRetrySeconds = 1.0;

std::pair<double, double> wheel_speeds(const krosc::msgs::Twist& command, double wheel_radius,
                                       double wheel_separation, double max_wheel) {
  const double half = wheel_separation / 2.0;
  double left = (command.linear.x - command.angular.z * half) / wheel_radius;
  double right = (command.linear.x + command.angular.z * half) / wheel_radius;
  const double peak = std::max(std::abs(left), std::abs(right));
  if (max_wheel > 0.0 && peak > max_wheel) {
    left = left * max_wheel / peak;
    right = right * max_wheel / peak;
  }
  return {left, right};
}
}  // namespace

OpenCrSerial::OpenCrSerial(double wheel_radius, double wheel_separation, std::string port, int baud,
                           double command_period, double command_timeout, double left_dir, double right_dir,
                           double max_wheel)
    : wheel_radius_(wheel_radius),
      wheel_separation_(wheel_separation),
      port_(std::move(port)),
      baud_(baud),
      command_period_(command_period),
      command_timeout_(command_timeout),
      left_dir_(left_dir),
      right_dir_(right_dir),
      max_wheel_(max_wheel),
      last_write_(std::chrono::steady_clock::now()),
      last_send_(std::chrono::steady_clock::now()) {}

OpenCrSerial::~OpenCrSerial() { stop(); }

void OpenCrSerial::start() {
  if (reader_thread_.joinable()) {
    return;
  }
  stop_.store(false);
  reader_thread_ = std::thread([this] { reader_loop(); });
}

void OpenCrSerial::stop() {
  if (stop_.exchange(true)) {
    return;
  }
  if (connected_.load()) {
    (void)write_command(format_command(0.0, 0.0));
  }
  close_fd();
  if (reader_thread_.joinable() && reader_thread_.get_id() != std::this_thread::get_id()) {
    reader_thread_.join();
  }
}

void OpenCrSerial::write_arm(const std::array<double, 5>& position) {
  {
    std::lock_guard<std::mutex> lock(arm_command_mutex_);
    bool same = has_arm_target_;
    for (std::size_t i = 0; i < position.size() && same; ++i) {
      if (!std::isfinite(position[i])) {
        continue;  // NaN = 유지, 비교하지 않는다
      }
      same = std::isfinite(last_arm_target_[i]) &&
             std::abs(position[i] - last_arm_target_[i]) <= (i == 4 ? 0.0005 : 0.001);
    }
    if (same) {
      return;  // 같은 목표는 다시 보내지 않는다
    }
    last_arm_target_ = position;
    has_arm_target_ = true;
  }
  std::string line = "A";
  char buffer[32];
  for (double value : position) {
    line += ",";
    if (std::isfinite(value)) {
      std::snprintf(buffer, sizeof(buffer), "%.5f", value);
      line += buffer;
    }
  }
  (void)write_command(line);
}

void OpenCrSerial::set_arm_torque(bool enabled) {
  (void)write_command(enabled ? "AT,1" : "AT,0");
}

ArmStatus OpenCrSerial::arm_status() const {
  std::lock_guard<std::mutex> lock(arm_mutex_);
  return arm_status_;
}

void OpenCrSerial::write(const krosc::msgs::Twist& command) {
  const auto [left, right] = wheel_speeds(command, wheel_radius_, wheel_separation_, max_wheel_);
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    target_left_ = left * left_dir_;
    target_right_ = right * right_dir_;
    last_write_ = std::chrono::steady_clock::now();
  }
}

void OpenCrSerial::tick() {
  const auto now = std::chrono::steady_clock::now();
  double left = 0.0;
  double right = 0.0;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (now - last_write_ > std::chrono::duration<double>(command_timeout_)) {
      target_left_ = 0.0;
      target_right_ = 0.0;
      last_write_ = now;
    }
    if (now - last_send_ < std::chrono::duration<double>(command_period_)) {
      return;
    }
    last_send_ = now;
    left = target_left_;
    right = target_right_;
  }
  const bool sent = write_command(format_command(left, right));
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!sent && !send_failed_) {
      send_failed_ = true;
    } else if (sent) {
      send_failed_ = false;
    }
  }
}

bool OpenCrSerial::write_command(const std::string& line) {
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ < 0) {
    return false;
  }
  if (!write_line(fd_, line)) {
    std::lock_guard<std::mutex> error_lock(error_mutex_);
    last_error_ = "OpenCR 쓰기 실패";
    error_count_.fetch_add(1);
    return false;
  }
  return true;
}

void OpenCrSerial::close_fd() {
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ >= 0) {
    close_serial(fd_);
    fd_ = -1;
  }
  connected_.store(false);
}

void OpenCrSerial::reader_loop() {
  std::string buffer;
  while (!stop_.load()) {
    int fd = -1;
    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      fd = fd_;
    }
    if (fd < 0) {
      const int opened = open_serial(port_, baud_);
      if (opened < 0) {
        for (int i = 0; i < 10 && !stop_.load(); ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(fd_mutex_);
        if (stop_.load()) {
          close_serial(opened);
          return;
        }
        fd_ = opened;
      }
      connected_.store(true);
      buffer.clear();
      std::fprintf(stderr, "[krosc_ws] OpenCR connected: %s (%d baud)\n", port_.c_str(), baud_);
      std::fflush(stderr);
    }
    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      fd = fd_;
    }
    if (fd < 0) {
      continue;
    }
    pollfd descriptor{fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 100);
    if (ready <= 0) {
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      close_fd();
      continue;
    }
    char data[4096];
    const ssize_t count = ::read(fd, data, sizeof(data));
    if (count <= 0) {
      if (count < 0 && (errno == EINTR || errno == EAGAIN)) {
        continue;
      }
      close_fd();
      continue;
    }
    buffer.append(data, static_cast<std::size_t>(count));
    std::size_t newline = 0;
    while ((newline = buffer.find('\n')) != std::string::npos) {
      std::string line = buffer.substr(0, newline);
      buffer.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (!line.empty()) {
        on_line(line);
      }
    }
    if (buffer.size() > 4096) {
      buffer.clear();
    }
  }
  close_fd();
}

void OpenCrSerial::on_line(const std::string& line) {
  const auto parsed = split_line(line);
  if (!parsed) {
    bad_lines_.fetch_add(1);
    return;
  }
  const auto& [kind, values] = *parsed;
  if (kind == "S") {
    const auto status_value = parse_status(values);
    if (!status_value) {
      bad_lines_.fetch_add(1);
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    status_ = *status_value;
    status_time_ = std::chrono::steady_clock::now();
    left_position_ = status_value->left_position * left_dir_;
    right_position_ = status_value->right_position * right_dir_;
    left_velocity_ = status_value->left_velocity * left_dir_;
    right_velocity_ = status_value->right_velocity * right_dir_;
  } else if (kind == "I") {
    const auto imu = parse_imu(values);
    if (!imu) {
      bad_lines_.fetch_add(1);
      return;
    }
    std::lock_guard<std::mutex> lock(imu_mutex_);
    pending_imu_ = *imu;
  } else if (kind == "M") {
    if (values.size() < 15) {
      bad_lines_.fetch_add(1);
      return;
    }
    std::lock_guard<std::mutex> lock(arm_mutex_);
    for (std::size_t i = 0; i < 5; ++i) {
      arm_status_.position[i] = values[i * 3];
      arm_status_.velocity[i] = values[i * 3 + 1];
      arm_status_.current[i] = values[i * 3 + 2];
      if (15 + i < values.size()) {
        arm_status_.temperature[i] = values[15 + i];
      }
    }
    arm_status_.present = false;
    for (std::size_t i = 0; i < 4; ++i) {
      arm_status_.present = arm_status_.present || std::isfinite(arm_status_.position[i]);
    }
  } else if (kind == "F") {
    if (values.size() < 5) {
      bad_lines_.fetch_add(1);
      return;
    }
    std::lock_guard<std::mutex> lock(arm_mutex_);
    for (std::size_t i = 0; i < 5; ++i) {
      arm_status_.error[i] = std::isfinite(values[i]) ? static_cast<int>(values[i]) : -1;
    }
  } else if (kind == "E") {
    const auto errors = parse_errors(values);
    if (!errors) {
      bad_lines_.fetch_add(1);
      return;
    }
    if (*errors != previous_errors_) {
      previous_errors_ = *errors;
      for (int code : {errors->first, errors->second}) {
        if (code != 0) {
          error_count_.fetch_add(1);
          std::lock_guard<std::mutex> lock(error_mutex_);
          last_error_ = "바퀴 에러: " + error_text(code);
        }
      }
    }
  }
  // M(arm)은 이 워크스페이스에서 쓰지 않는다. 모르는 줄은 무시한다 (펌웨어가 줄을 추가해도 안 깨진다).
}

std::optional<krosc::msgs::Imu> OpenCrSerial::consume_imu() {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (!pending_imu_) {
    return std::nullopt;
  }
  auto value = pending_imu_;
  pending_imu_.reset();
  return value;
}

double OpenCrSerial::left_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return left_position_;
}
double OpenCrSerial::right_position() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return right_position_;
}
double OpenCrSerial::left_velocity() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return left_velocity_;
}
double OpenCrSerial::right_velocity() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return right_velocity_;
}
std::optional<BoardStatus> OpenCrSerial::status() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return status_;
}
double OpenCrSerial::status_age() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!status_) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - status_time_).count();
}
std::string OpenCrSerial::last_error() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

}  // namespace krosc_ws
