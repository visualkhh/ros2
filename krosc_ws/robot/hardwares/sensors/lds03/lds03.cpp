#include "lds03.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <thread>
#include <utility>

#include "hardwares/serial.hpp"
#include "krosc/qos.hpp"

namespace krosc_ws {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::uint8_t kHeader0 = 0xAA;
constexpr std::uint8_t kHeader1 = 0x55;
constexpr std::array<std::uint8_t, 4> kStart{0xAA, 0x55, 0xF0, 0x0F};
constexpr std::array<std::uint8_t, 4> kStop{0xAA, 0x55, 0xF5, 0x0A};
}  // namespace

LdsParser::LdsParser(int beams, bool reverse) : beams_(beams), reverse_(reverse) {
  pending_.reserve(static_cast<std::size_t>(beams_));
}

void LdsParser::feed(const std::uint8_t* data, std::size_t size) {
  buffer_.insert(buffer_.end(), data, data + size);
  if (buffer_.size() > 65536) {
    buffer_.erase(buffer_.begin(), buffer_.end() - 4096);
  }
  for (;;) {
    std::size_t header = std::string::npos;
    for (std::size_t i = 0; i + 1 < buffer_.size(); ++i) {
      if (buffer_[i] == kHeader0 && buffer_[i + 1] == kHeader1) {
        header = i;
        break;
      }
    }
    if (header == std::string::npos) {
      if (buffer_.size() > 1) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      }
      return;
    }
    if (header > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(header));
    }
    if (buffer_.size() < 10) {
      return;
    }
    const std::uint8_t* raw = buffer_.data();
    const int count = raw[3];
    if (count == 0 || count > kMaxSamples) {
      buffer_.erase(buffer_.begin());
      bad_packets_.fetch_add(1);
      continue;
    }
    const std::size_t packet_size = 10 + static_cast<std::size_t>(count) * 3;
    if (buffer_.size() < packet_size) {
      return;
    }
    std::array<std::uint8_t, 10> header_bytes{};
    std::memcpy(header_bytes.data(), raw, 10);
    std::vector<std::uint8_t> payload(static_cast<std::size_t>(count) * 3);
    std::memcpy(payload.data(), raw + 10, payload.size());
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(packet_size));
    auto packet = parse(header_bytes.data(), payload.data(), count);
    if (!packet) {
      bad_packets_.fetch_add(1);
      continue;
    }
    if (packet->ring_start && !pending_.empty()) {
      complete();
    }
    pending_.insert(pending_.end(), packet->points.begin(), packet->points.end());
  }
}

std::optional<LdsParser::Packet> LdsParser::parse(const std::uint8_t* header, const std::uint8_t* payload,
                                                  int count) {
  const std::uint8_t ct = header[2];
  const std::uint16_t first_raw = static_cast<std::uint16_t>(header[4]) |
                                  static_cast<std::uint16_t>(header[5] << 8);
  const std::uint16_t last_raw = static_cast<std::uint16_t>(header[6]) |
                                 static_cast<std::uint16_t>(header[7] << 8);
  const std::uint16_t checksum = static_cast<std::uint16_t>(header[8]) |
                                 static_cast<std::uint16_t>(header[9] << 8);
  std::uint16_t calculated = static_cast<std::uint16_t>(0x55AA ^ first_raw ^ (ct | (count << 8)) ^ last_raw);
  for (int i = 0; i < count; ++i) {
    const std::uint8_t b0 = payload[i * 3];
    const std::uint8_t b1 = payload[i * 3 + 1];
    const std::uint8_t b2 = payload[i * 3 + 2];
    calculated ^= b0;
    calculated ^= static_cast<std::uint16_t>(b1 | (b2 << 8));
  }
  if (calculated != checksum) {
    return std::nullopt;
  }
  if ((first_raw & 0x01) == 0 || (last_raw & 0x01) == 0) {
    return std::nullopt;
  }
  const double first = static_cast<double>(first_raw >> 1);
  const double last = static_cast<double>(last_raw >> 1);
  double interval = 0.0;
  if (count == 1) {
    interval = 0.0;
  } else if (last < first) {
    if (first > 270.0 * 64.0 && last < 90.0 * 64.0) {
      interval = (360.0 * 64.0 + last - first) / static_cast<double>(count - 1);
    } else {
      interval = last_interval_;
    }
  } else {
    interval = (last - first) / static_cast<double>(count - 1);
    last_interval_ = interval;
  }
  Packet packet;
  packet.ring_start = (ct & 0x01) != 0;
  packet.points.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const std::uint8_t b1 = payload[i * 3 + 1];
    const std::uint8_t b2 = payload[i * 3 + 2];
    const double distance = static_cast<double>((b2 << 6) | (b1 >> 2)) / 1000.0;
    double angle = (first + static_cast<double>(i) * interval) / 64.0;
    if (reverse_) {
      angle = 360.0 - angle;
    }
    angle = std::fmod(angle, 360.0);
    if (angle < 0.0) {
      angle += 360.0;
    }
    if (distance >= kRangeMin && distance <= kRangeMax) {
      packet.points.emplace_back(angle, distance);
    }
  }
  return packet;
}

void LdsParser::complete() {
  std::vector<float> bins(static_cast<std::size_t>(beams_), std::numeric_limits<float>::infinity());
  for (const auto& [angle, distance] : pending_) {
    if (!std::isfinite(distance)) {
      continue;
    }
    const int index = static_cast<int>(angle / 360.0 * static_cast<double>(beams_)) % beams_;
    if (distance < bins[static_cast<std::size_t>(index)]) {
      bins[static_cast<std::size_t>(index)] = static_cast<float>(distance);
    }
  }
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_ = std::move(bins);
  }
  pending_.clear();
}

std::optional<std::vector<float>> LdsParser::take_scan() {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!scan_) {
    return std::nullopt;
  }
  auto scan = std::move(scan_);
  scan_.reset();
  return scan;
}

Lds03::Lds03(krosc::Context& context, std::string port, double mount_x, std::string frame_id,
             bool reverse, bool body_filter, int baud)
    : krosc::sensor::Sensor(context), port_(std::move(port)), mount_x_(mount_x),
      frame_id_(std::move(frame_id)),
      body_filter_(body_filter),
      baud_(baud),
      parser_(LdsParser::kBeams, reverse) {
  if (port_.empty()) {
    port_ = port_exists("/dev/tb3_lidar") ? "/dev/tb3_lidar" : "/dev/ttyUSB0";
  }
}

Lds03::~Lds03() { stop(); }

bool Lds03::on_init(const krosc::sensor::SensorInfo& info) {
  if (!krosc::sensor::Sensor::on_init(info)) {
    return false;
  }
  if (!info.topic.empty()) {
    topic_ = info.topic;
  }
  if (!info.frame_id.empty()) {
    frame_id_ = info.frame_id;
  }
  return true;
}

bool Lds03::on_activate() {
  start();
  return true;
}

bool Lds03::on_deactivate() {
  stop();
  return true;
}

bool Lds03::on_update(double time, double period) {
  (void)time;
  (void)period;
  if (auto scan = take_scan(now())) {
    if (!topic_.empty()) {
      context().topics().publish<krosc::msgs::LaserScan>(topic_, std::move(*scan),
                                                         krosc::QOS_SENSOR_DATA);
    }
  }
  return connected_.load() && scan_age() <= 1.0;
}

void Lds03::start() {
  if (reader_thread_.joinable()) {
    return;
  }
  stop_.store(false);
  reader_thread_ = std::thread([this] { reader_loop(); });
}

void Lds03::stop() {
  if (stop_.exchange(true)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) {
      (void)write_bytes(fd_, kStop.data(), kStop.size());
    }
  }
  close_fd();
  if (reader_thread_.joinable() && reader_thread_.get_id() != std::this_thread::get_id()) {
    reader_thread_.join();
  }
}

void Lds03::close_fd() {
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ >= 0) {
    close_serial(fd_);
    fd_ = -1;
  }
  connected_.store(false);
}

bool Lds03::send_line(const std::uint8_t* data, std::size_t size) {
  std::lock_guard<std::mutex> lock(fd_mutex_);
  if (fd_ < 0) {
    return false;
  }
  return write_bytes(fd_, data, size);
}

void Lds03::reader_loop() {
  std::uint8_t buffer[4096];
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
      set_dtr_low(opened);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      (void)send_line(kStart.data(), kStart.size());
      last_data_ = std::chrono::steady_clock::now();
      std::fprintf(stderr, "[krosc_ws] LDS-03 connected: %s (%d baud)\n", port_.c_str(), baud_);
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
    if (ready < 0) {
      if (errno != EINTR) {
        close_fd();
      }
      continue;
    }
    if (ready == 0) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_data_).count() > 3.0) {
        (void)send_line(kStart.data(), kStart.size());
        last_data_ = std::chrono::steady_clock::now();
      }
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      close_fd();
      continue;
    }
    const ssize_t count = ::read(fd, buffer, sizeof(buffer));
    if (count <= 0) {
      if (count < 0 && (errno == EINTR || errno == EAGAIN)) {
        continue;
      }
      close_fd();
      continue;
    }
    last_data_ = std::chrono::steady_clock::now();
    parser_.feed(buffer, static_cast<std::size_t>(count));
  }
  close_fd();
}

std::optional<krosc::msgs::LaserScan> Lds03::take_scan(double stamp) {
  auto bins = parser_.take_scan();
  if (!bins) {
    return std::nullopt;
  }
  const double increment = 2.0 * kPi / static_cast<double>(parser_.beams());
  if (body_filter_) {
    for (std::size_t i = 0; i < bins->size(); ++i) {
      const float distance = (*bins)[i];
      if (!std::isfinite(distance)) {
        continue;
      }
      const double angle = static_cast<double>(i) * increment;
      const double x = mount_x_ + static_cast<double>(distance) * std::cos(angle);
      const double y = static_cast<double>(distance) * std::sin(angle);
      if (x >= kBodyBox[0] && x <= kBodyBox[1] && y >= kBodyBox[2] && y <= kBodyBox[3]) {
        (*bins)[i] = std::numeric_limits<float>::infinity();
      }
    }
  }
  krosc::msgs::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = frame_id_;
  scan.angle_min = 0.0;
  scan.angle_max = 2.0 * kPi - increment;
  scan.angle_increment = increment;
  scan.range_min = LdsParser::kRangeMin;
  scan.range_max = LdsParser::kRangeMax;
  scan.ranges = std::move(*bins);
  scans_.fetch_add(1);
  {
    std::lock_guard<std::mutex> lock(scan_time_mutex_);
    last_scan_time_ = std::chrono::steady_clock::now();
    has_scan_time_ = true;
  }
  return scan;
}

double Lds03::scan_age() const {
  std::lock_guard<std::mutex> lock(scan_time_mutex_);
  if (!has_scan_time_) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_scan_time_).count();
}

}  // namespace krosc_ws
