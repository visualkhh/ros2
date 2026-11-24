#include <cmath>
#include <numbers>
#include <utility>
#include <vector>

#include "krosc/cartographer/probability_grid.hpp"
#include "krosc/cartographer/scan_matcher.hpp"

#include "test_framework.hpp"

namespace {

using krosc::cartographer::Pose2;
using krosc::cartographer::ProbabilityGrid;
using krosc::cartographer::ScanPoints;

std::vector<std::pair<double, double>> walls() {
  std::vector<std::pair<double, double>> points;
  for (int index = 0; index < 40; ++index) {
    const double t = 0.025 + 0.05 * static_cast<double>(index);  // 칸 중심 (0.025 .. 1.975)
    points.emplace_back(0.025, t);
    points.emplace_back(1.975, t);
    points.emplace_back(t, 0.025);
    points.emplace_back(t, 1.975);
  }
  return points;
}

ProbabilityGrid room_grid() {
  ProbabilityGrid grid(0.05);
  grid.insert({1.0, 1.0}, walls());
  return grid;
}

ScanPoints room_scan(const Pose2& sensor) {
  ScanPoints points;
  for (const auto& wall : walls()) {
    // map 점을 센서 좌표로: 센서 자세의 역변환
    const double dx = wall.first - sensor[0];
    const double dy = wall.second - sensor[1];
    const double c = std::cos(sensor[2]);
    const double s = std::sin(sensor[2]);
    points.emplace_back(c * dx + s * dy, -s * dx + c * dy);
  }
  return points;
}

}  // namespace

KROSC_TEST(probability_grid_marks_walls_and_clears_free_space) {
  auto grid = room_grid();
  CHECK(!grid.empty());
  CHECK(grid.probability(0.025, 1.025) > 0.65);   // 벽 (칸 중심)
  CHECK(grid.probability(0.5, 1.0) < 0.45);   // 빈 곳
}

KROSC_TEST(probability_grid_message_roundtrip) {
  auto grid = room_grid();
  const auto message = grid.to_message(krosc::msgs::Header{1.0, "map"});
  CHECK(message.info.width > 0);
  CHECK(message.info.height > 0);
  CHECK_EQ(message.data.size(), static_cast<std::size_t>(message.info.width) * message.info.height);
  CHECK_EQ(message.header.frame_id, std::string("map"));
  auto back = ProbabilityGrid::from_message(message);
  CHECK(back.probability(0.025, 1.025) > 0.6);
  CHECK(back.probability(0.5, 1.0) < 0.5);
}

KROSC_TEST(scan_matcher_finds_the_true_pose_from_a_wrong_guess) {
  const auto grid = room_grid();
  const Pose2 truth{1.0, 1.0, 0.0};
  const auto scan = room_scan(truth);
  const Pose2 guess{1.06, 0.97, 0.05};  // 6cm, 3도쯤 틀린 예상
  const auto [found, value] = krosc::cartographer::match(grid, scan, guess);
  CHECK_NEAR(truth[0], found[0], 0.015);
  CHECK_NEAR(truth[1], found[1], 0.015);
  CHECK_NEAR(truth[2], found[2], std::numbers::pi / 180.0);
  CHECK(value > 0.65);
}
