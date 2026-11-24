#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "krosc/cartographer/probability_grid.hpp"

namespace krosc::cartographer {

using Pose2 = std::array<double, 3>;  // (x, y, yaw)
using ScanPoints = std::vector<std::pair<double, double>>;

inline double wrap_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// a 좌표계 안의 b를 a의 부모 좌표계로.
inline Pose2 compose(const Pose2& a, const Pose2& b) {
  const double c = std::cos(a[2]);
  const double s = std::sin(a[2]);
  return {a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2])};
}

inline Pose2 inverse(const Pose2& a) {
  const double c = std::cos(a[2]);
  const double s = std::sin(a[2]);
  return {-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]};
}

// 센서가 pose에 있을 때 스캔 점(센서 좌표)들이 놓이는 칸의 평균 벽 확률.
inline double score(const ProbabilityGrid& grid, const ScanPoints& points, const Pose2& pose) {
  if (points.empty()) {
    return 0.0;
  }
  const double c = std::cos(pose[2]);
  const double s = std::sin(pose[2]);
  double sum = 0.0;
  for (const auto& [px, py] : points) {
    sum += grid.probability(pose[0] + c * px - s * py, pose[1] + s * px + c * py);
  }
  return sum / static_cast<double>(points.size());
}

// cartographer의 실시간 상관 스캔 매칭 흉내: 예상 위치 둘레를 격자로 뒤져 스캔 점들이 벽에 가장 잘
// 얹히는 위치를 찾는다. 거친 간격으로 본 뒤 앞 단계 격자의 절반만 점점 촘촘히 본다.
inline std::pair<Pose2, double> match(const ProbabilityGrid& grid, const ScanPoints& points, const Pose2& guess,
                                      double linear_window = 0.1, double angular_window = 0.1) {
  Pose2 best = guess;
  double best_score = score(grid, points, guess);
  if (points.empty()) {
    return {best, best_score};
  }
  double window_linear = linear_window;
  double window_angular = angular_window;
  static constexpr double kStages[][2] = {{0.05, 0.025}, {0.0125, 0.00625}, {0.003, 0.0015}};
  std::vector<std::pair<double, double>> rotated(points.size());
  for (const auto& stage : kStages) {
    const double linear_step = stage[0];
    const double angular_step = stage[1];
    const Pose2 center = best;
    const int n = static_cast<int>(std::llround(window_linear / linear_step));
    const int m = static_cast<int>(std::llround(window_angular / angular_step));
    for (int k = -m; k <= m; ++k) {
      const double yaw = center[2] + static_cast<double>(k) * angular_step;
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      for (std::size_t index = 0; index < points.size(); ++index) {
        const double px = points[index].first;
        const double py = points[index].second;
        rotated[index] = {c * px - s * py, s * px + c * py};
      }
      for (int i = -n; i <= n; ++i) {
        const double x = center[0] + static_cast<double>(i) * linear_step;
        for (int j = -n; j <= n; ++j) {
          const double y = center[1] + static_cast<double>(j) * linear_step;
          double sum = 0.0;
          for (const auto& [rx, ry] : rotated) {
            sum += grid.probability(x + rx, y + ry);
          }
          const double value = sum / static_cast<double>(rotated.size());
          if (value > best_score) {
            best = {x, y, yaw};
            best_score = value;
          }
        }
      }
    }
    window_linear = linear_step / 2.0;
    window_angular = angular_step / 2.0;
  }
  return {best, best_score};
}

}  // namespace krosc::cartographer
