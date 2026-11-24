#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "krosc/msgs.hpp"

#include "opencr_protocol.hpp"

namespace krosc_ws {

// OpenCR M/F 줄의 팔 상태. position[0..3] = joint1..4, position[4] = gripper [m].
struct ArmStatus {
  std::array<double, 5> position{std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 5> velocity{};
  std::array<double, 5> current{};
  std::array<double, 5> temperature{};
  std::array<int, 5> error{-1, -1, -1, -1, -1};
  bool present = false;
};

// ROBOTIS OpenCR 1.0 + tb3_wheels 펌웨어를 USB 시리얼로 쓰는 드라이버.
// - write(Twist): 역기구학으로 바퀴 목표 속도를 계산해 두고, tick()이 20Hz로 C 줄을 보낸다.
// - 읽기 스레드: S(위치/속도/배터리), I(IMU), E(바퀴 에러)를 파싱한다.
// - 0.5초 동안 write가 없으면 마지막 명령을 버리고 0을 보낸다 (펌웨어 watchdog와 별개로 앱에서도).
class OpenCrSerial {
 public:
  OpenCrSerial(double wheel_radius, double wheel_separation, std::string port, int baud = 115200,
               double command_period = 0.05, double command_timeout = 0.5, double left_dir = 1.0,
               double right_dir = 1.0, double max_wheel = 8.0);
  ~OpenCrSerial();

  OpenCrSerial(const OpenCrSerial&) = delete;
  OpenCrSerial& operator=(const OpenCrSerial&) = delete;

  void start();
  void stop();

  void write(const krosc::msgs::Twist& command);
  void write_arm(const std::array<double, 5>& position);  // NaN = 그 관절 유지
  void set_arm_torque(bool enabled);
  void tick();

  [[nodiscard]] bool connected() const { return connected_.load(); }
  [[nodiscard]] std::optional<krosc::msgs::Imu> consume_imu();
  [[nodiscard]] ArmStatus arm_status() const;

  [[nodiscard]] double left_position() const;
  [[nodiscard]] double right_position() const;
  [[nodiscard]] double left_velocity() const;
  [[nodiscard]] double right_velocity() const;
  [[nodiscard]] std::optional<BoardStatus> status() const;
  [[nodiscard]] double status_age() const;
  [[nodiscard]] std::uint64_t error_count() const { return error_count_.load(); }
  [[nodiscard]] std::uint64_t bad_lines() const { return bad_lines_.load(); }
  [[nodiscard]] std::string last_error() const;
  [[nodiscard]] const std::string& port() const { return port_; }

 private:
  void reader_loop();
  void on_line(const std::string& line);
  void close_fd();
  bool write_command(const std::string& line);

  const double wheel_radius_;
  const double wheel_separation_;
  const std::string port_;
  const int baud_;
  const double command_period_;
  const double command_timeout_;
  const double left_dir_;
  const double right_dir_;
  const double max_wheel_;

  std::atomic<bool> stop_{false};
  std::atomic<bool> connected_{false};
  std::atomic<std::uint64_t> error_count_{0};
  std::atomic<std::uint64_t> bad_lines_{0};
  std::thread reader_thread_;

  mutable std::mutex fd_mutex_;
  int fd_ = -1;

  mutable std::mutex state_mutex_;
  double left_position_ = 0.0;
  double right_position_ = 0.0;
  double left_velocity_ = 0.0;
  double right_velocity_ = 0.0;
  std::optional<BoardStatus> status_;
  std::chrono::steady_clock::time_point status_time_{};

  std::mutex imu_mutex_;
  std::optional<krosc::msgs::Imu> pending_imu_;

  mutable std::mutex arm_mutex_;
  ArmStatus arm_status_;

  std::mutex command_mutex_;
  double target_left_ = 0.0;
  double target_right_ = 0.0;
  std::chrono::steady_clock::time_point last_write_{};
  std::chrono::steady_clock::time_point last_send_{};
  bool send_failed_ = false;

  mutable std::mutex error_mutex_;
  std::string last_error_;
  std::pair<int, int> previous_errors_{0, 0};
};

}  // namespace krosc_ws
