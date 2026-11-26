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
#include <vector>

#include "krosc/msgs.hpp"

#include "lds_device.hpp"

namespace krosc_ws {

// LDS-03 (COIN-D4) 패킷 파서. 시리얼 없이 시험할 수 있는 순수 파서다.
class LdsParser {
 public:
  static constexpr int kBeams = 360;
  static constexpr double kRangeMin = 0.05;
  static constexpr double kRangeMax = 12.0;
  static constexpr int kMaxSamples = 128;

  explicit LdsParser(int beams = kBeams, bool reverse = true);

  void feed(const std::uint8_t* data, std::size_t size);
  [[nodiscard]] std::optional<std::vector<float>> take_scan();
  [[nodiscard]] std::uint64_t bad_packets() const { return bad_packets_.load(); }
  [[nodiscard]] int beams() const { return beams_; }

 private:
  struct Packet {
    std::vector<std::pair<double, double>> points;
    bool ring_start = false;
  };
  [[nodiscard]] std::optional<Packet> parse(const std::uint8_t* header, const std::uint8_t* payload, int count);
  void complete();

  const int beams_;
  const bool reverse_;
  std::vector<std::uint8_t> buffer_;
  std::vector<std::pair<double, double>> pending_;
  double last_interval_ = 0.0;
  std::optional<std::vector<float>> scan_;
  mutable std::mutex scan_mutex_;
  std::atomic<std::uint64_t> bad_packets_{0};
};

// 실제 LDS-03 시리얼 장치: 230400, 10Hz. USB2LDS는 DTR을 내려야 데이터를 보낸다.
// 로봇 몸통이 찍힌 점은 base_link 기준 상자 안이면 지운다.
// 토픽/프레임은 sensor 논리 레이어(Lds03Sensor)가 정한다.
class Lds03Device final : public LdsDevice {
 public:
  explicit Lds03Device(std::string port, double mount_x = 0.060, bool reverse = true,
                       bool body_filter = true, int baud = 230400);
  ~Lds03Device() override;

  Lds03Device(const Lds03Device&) = delete;
  Lds03Device& operator=(const Lds03Device&) = delete;

  void start() override;
  void stop() override;
  [[nodiscard]] bool connected() const override { return connected_.load(); }

  // header.stamp만 채운다. frame_id/topic은 Sensor 레이어가 정한다.
  [[nodiscard]] std::optional<krosc::msgs::LaserScan> take_scan(double stamp) override;
  [[nodiscard]] std::uint64_t scans() const override { return scans_.load(); }
  [[nodiscard]] std::uint64_t bad_packets() const override { return parser_.bad_packets(); }
  [[nodiscard]] double scan_age() const override;
  [[nodiscard]] const std::string& port() const override { return port_; }

 private:
  void reader_loop();
  void close_fd();
  bool send_line(const std::uint8_t* data, std::size_t size);

  static constexpr std::array<double, 4> kBodyBox{-0.21, 0.075, -0.17, 0.17};

  std::string port_;
  const double mount_x_;
  const bool body_filter_;
  const int baud_;
  LdsParser parser_;

  std::atomic<bool> stop_{false};
  std::atomic<bool> connected_{false};
  std::atomic<std::uint64_t> scans_{0};
  std::thread reader_thread_;
  mutable std::mutex fd_mutex_;
  int fd_ = -1;
  std::chrono::steady_clock::time_point last_data_{};
  mutable std::mutex scan_time_mutex_;
  std::chrono::steady_clock::time_point last_scan_time_{};
  bool has_scan_time_ = false;
};

}  // namespace krosc_ws
