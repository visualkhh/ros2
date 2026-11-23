#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "hardwares/lds03.hpp"
#include "hardwares/opencr_protocol.hpp"

#include "test_framework.hpp"

namespace {

std::vector<std::uint8_t> make_packet(bool ring_start, double start_deg, double end_deg,
                                      const std::vector<double>& distances_m) {
  const int count = static_cast<int>(distances_m.size());
  const auto first_raw = static_cast<std::uint16_t>((static_cast<int>(start_deg * 64.0) << 1) | 1);
  const auto last_raw = static_cast<std::uint16_t>((static_cast<int>(end_deg * 64.0) << 1) | 1);
  const std::uint8_t ct = ring_start ? 1 : 0;
  std::vector<std::uint8_t> samples;
  std::uint16_t checksum = static_cast<std::uint16_t>(0x55AA ^ first_raw ^ (ct | (count << 8)) ^ last_raw);
  for (double distance : distances_m) {
    const auto mm = static_cast<int>(std::llround(distance * 1000.0));
    const auto b0 = static_cast<std::uint8_t>(0);
    const auto b1 = static_cast<std::uint8_t>((mm & 0x3F) << 2);
    const auto b2 = static_cast<std::uint8_t>(mm >> 6);
    samples.push_back(b0);
    samples.push_back(b1);
    samples.push_back(b2);
    checksum ^= b0;
    checksum ^= static_cast<std::uint16_t>(b1 | (b2 << 8));
  }
  std::vector<std::uint8_t> packet;
  packet.push_back(0xAA);
  packet.push_back(0x55);
  packet.push_back(ct);
  packet.push_back(static_cast<std::uint8_t>(count));
  packet.push_back(static_cast<std::uint8_t>(first_raw & 0xFF));
  packet.push_back(static_cast<std::uint8_t>(first_raw >> 8));
  packet.push_back(static_cast<std::uint8_t>(last_raw & 0xFF));
  packet.push_back(static_cast<std::uint8_t>(last_raw >> 8));
  packet.push_back(static_cast<std::uint8_t>(checksum & 0xFF));
  packet.push_back(static_cast<std::uint8_t>(checksum >> 8));
  packet.insert(packet.end(), samples.begin(), samples.end());
  return packet;
}

}  // namespace

KROSC_TEST(opencr_protocol_parse_and_format) {
  const auto parsed = krosc_ws::split_line("S,1.5,-2.0,3.0,4.0,100,0.1,0.2,30,31,12.3");
  CHECK(parsed.has_value());
  if (parsed) {
    CHECK_EQ(std::string("S"), parsed->first);
    CHECK_EQ(10u, parsed->second.size());
  }
  CHECK(!krosc_ws::split_line("S,1.0,oops").has_value());

  const auto status = krosc_ws::parse_status({1.0, 2.0, 3.0, 4.0, 5.0});
  CHECK(status.has_value());
  if (status) {
    CHECK_NEAR(1.0, status->left_position, 1e-12);
    CHECK_NEAR(4.0, status->right_velocity, 1e-12);
    CHECK(std::isnan(status->battery_voltage));
  }
  const auto imu = krosc_ws::parse_imu({0.1, 0.2, 0.3, 0.0, 0.0, 9.81, 1.0, 0.0, 0.0, 0.0});
  CHECK(imu.has_value());
  if (imu) {
    CHECK_NEAR(0.1, imu->angular_velocity.x, 1e-12);
    CHECK_NEAR(9.81, imu->linear_acceleration.z, 1e-12);
    CHECK_NEAR(1.0, imu->orientation.w, 1e-12);
  }
  const auto errors = krosc_ws::parse_errors({0.0, 4.0});
  CHECK(errors.has_value());
  if (errors) {
    CHECK_EQ(4, errors->second);
    CHECK_EQ(std::string("과열"), krosc_ws::error_text(errors->second));
  }
  CHECK_EQ(std::string("C,1.5000,-2.0000"), krosc_ws::format_command(1.5, -2.0));
}

KROSC_TEST(lds03_parser_completes_scan_on_ring_start) {
  krosc_ws::LdsParser parser(360, false);
  const auto first = make_packet(true, 0.0, 90.0, {1.0, 2.0});
  const auto second = make_packet(true, 180.0, 180.0, {3.0});
  const auto third = make_packet(true, 0.0, 0.0, {4.0});
  parser.feed(first.data(), first.size());
  CHECK(!parser.take_scan().has_value());
  parser.feed(second.data(), second.size());
  const auto scan = parser.take_scan();
  CHECK(scan.has_value());
  if (scan) {
    CHECK_EQ(360u, scan->size());
    CHECK_NEAR(1.0, (*scan)[0], 1e-3);
    CHECK_NEAR(2.0, (*scan)[90], 1e-3);
    CHECK(!std::isfinite((*scan)[180]));
  }
  parser.feed(third.data(), third.size());
  const auto next = parser.take_scan();
  CHECK(next.has_value());
  if (next) {
    CHECK_NEAR(3.0, (*next)[180], 1e-3);
  }
  CHECK_EQ(0u, parser.bad_packets());
}

KROSC_TEST(lds03_reverse_maps_angles) {
  krosc_ws::LdsParser parser(360, true);
  const auto first = make_packet(true, 0.0, 90.0, {1.0, 2.0});
  const auto second = make_packet(true, 0.0, 0.0, {3.0});
  parser.feed(first.data(), first.size());
  parser.feed(second.data(), second.size());
  const auto scan = parser.take_scan();
  CHECK(scan.has_value());
  if (scan) {
    CHECK_NEAR(1.0, (*scan)[0], 1e-3);     // 0 -> 360 -> 0
    CHECK_NEAR(2.0, (*scan)[270], 1e-3);   // 90 -> 270 (시계방향 장치 보정)
  }
}
