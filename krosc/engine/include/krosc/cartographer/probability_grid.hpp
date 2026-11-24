#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"

namespace krosc::cartographer {

// cartographer의 ProbabilityGrid 흉내: 칸마다 "벽일 확률"을 log-odds로 쌓는 2D 격자.
// 칸은 필요할 때만 생긴다 (unordered_map). 스캔 매칭이 자주 읽는 확률은 같이 캐시한다.
class ProbabilityGrid {
 public:
  static constexpr double kHit = 0.85;
  static constexpr double kMiss = -0.4;
  static constexpr double kLimitLow = -2.0;
  static constexpr double kLimitHigh = 3.5;

  explicit ProbabilityGrid(double resolution) : resolution_(resolution) {}

  [[nodiscard]] double resolution() const { return resolution_; }
  [[nodiscard]] bool empty() const { return cells_.empty(); }
  [[nodiscard]] std::size_t size() const { return cells_.size(); }

  [[nodiscard]] std::pair<int, int> cell(double x, double y) const {
    return {static_cast<int>(std::floor(x / resolution_)), static_cast<int>(std::floor(y / resolution_))};
  }

  // (x, y)에서 벽일 확률을 둘레 네 칸 중심에서 이중선형 보간한다 (스캔 매칭 점수가 칸 경계에서 계단처럼
  // 튀지 않게). 모르는 칸은 0.5.
  [[nodiscard]] double probability(double x, double y) const {
    const double u = x / resolution_ - 0.5;
    const double v = y / resolution_ - 0.5;
    const int i = static_cast<int>(std::floor(u));
    const int j = static_cast<int>(std::floor(v));
    const double fu = u - i;
    const double fv = v - j;
    return probability_cell(i, j) * (1.0 - fu) * (1.0 - fv) +
           probability_cell(i + 1, j) * fu * (1.0 - fv) +
           probability_cell(i, j + 1) * (1.0 - fu) * fv +
           probability_cell(i + 1, j + 1) * fu * fv;
  }

  // 센서 위치에서 각 끝점까지 레이를 그어, 지나간 칸은 빈 곳 쪽으로, 끝점 칸은 벽 쪽으로.
  // sensor/hits는 같은 좌표계(보통 map)의 미터 단위다.
  void insert(std::pair<double, double> sensor, const std::vector<std::pair<double, double>>& hits) {
    if (hits.empty()) {
      return;
    }
    const auto start = cell(sensor.first, sensor.second);
    std::unordered_set<std::uint64_t> ends;
    std::unordered_set<std::uint64_t> misses;
    ends.reserve(hits.size());
    misses.reserve(hits.size() * 32);
    for (const auto& hit : hits) {
      const auto end = cell(hit.first, hit.second);
      const std::uint64_t end_key = key(end.first, end.second);
      ends.insert(end_key);
      add_line(start.first, start.second, end.first, end.second, misses);
    }
    for (const std::uint64_t key_value : misses) {
      if (ends.find(key_value) == ends.end()) {
        add_key(key_value, kMiss);
      }
    }
    for (const std::uint64_t key_value : ends) {
      add_key(key_value, kHit);
    }
  }

  // 닿은 칸을 모두 담는 가장 작은 사각형으로 OccupancyGrid를 만든다 (확률 0~100, 닿지 않은 칸 -1).
  [[nodiscard]] msgs::OccupancyGrid to_message(const msgs::Header& header) const {
    msgs::OccupancyGrid message;
    message.header = header;
    message.info.map_load_time = header.stamp;
    message.info.resolution = resolution_;
    if (cells_.empty()) {
      message.info.width = 0;
      message.info.height = 0;
      message.info.origin = msgs::Pose{};
      message.info.origin.orientation.w = 1.0;
      return message;
    }
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
    bool first = true;
    for (const auto& [key_value, entry] : cells_) {
      (void)entry;
      const auto [x, y] = decode(key_value);
      if (first) {
        x0 = x1 = x;
        y0 = y1 = y;
        first = false;
      } else {
        x0 = std::min(x0, x);
        y0 = std::min(y0, y);
        x1 = std::max(x1, x);
        y1 = std::max(y1, y);
      }
    }
    const int width = x1 - x0 + 1;
    const int height = y1 - y0 + 1;
    message.info.width = static_cast<std::uint32_t>(width);
    message.info.height = static_cast<std::uint32_t>(height);
    message.info.origin.position.x = x0 * resolution_;
    message.info.origin.position.y = y0 * resolution_;
    message.info.origin.orientation.w = 1.0;
    message.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                        static_cast<std::int8_t>(-1));
    for (const auto& [key_value, entry] : cells_) {
      const auto [x, y] = decode(key_value);
      const int i = x - x0;
      const int j = y - y0;
      message.data[static_cast<std::size_t>(j) * static_cast<std::size_t>(width) + static_cast<std::size_t>(i)] =
          static_cast<std::int8_t>(std::lround(entry.probability * 100.0));
    }
    return message;
  }

  // 저장한 지도(map_server)로 만든다: 칸 중심을 이 격자의 칸에 놓는다. 지도 origin의 회전은 무시한다.
  static ProbabilityGrid from_message(const msgs::OccupancyGrid& grid) {
    ProbabilityGrid made(grid.info.resolution);
    const auto& info = grid.info;
    if (info.width == 0 || info.height == 0) {
      return made;
    }
    for (std::uint32_t j = 0; j < info.height; ++j) {
      for (std::uint32_t i = 0; i < info.width; ++i) {
        const std::int8_t value = grid.data[static_cast<std::size_t>(j) * info.width + i];
        if (value < 0) {
          continue;
        }
        const double p = std::min(0.99, std::max(0.01, static_cast<double>(value) / 100.0));
        const double log_odds = std::log(p / (1.0 - p));
        const double x = info.origin.position.x + (static_cast<double>(i) + 0.5) * info.resolution;
        const double y = info.origin.position.y + (static_cast<double>(j) + 0.5) * info.resolution;
        const auto cell_index = made.cell(x, y);
        made.add(cell_index.first, cell_index.second, log_odds);
      }
    }
    return made;
  }

 private:
  struct Entry {
    double log_odds = 0.0;
    double probability = 0.5;
  };

  static std::uint64_t key(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
           static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
  }

  static std::pair<int, int> decode(std::uint64_t value) {
    return {static_cast<std::int32_t>(value >> 32), static_cast<std::int32_t>(value & 0xffffffffu)};
  }

  [[nodiscard]] double probability_cell(int x, int y) const {
    const auto found = cells_.find(key(x, y));
    return found == cells_.end() ? 0.5 : found->second.probability;
  }

  static double probability_of_log_odds(double log_odds) { return 1.0 - 1.0 / (1.0 + std::exp(log_odds)); }

  void add(int x, int y, double amount) {
    auto [found, inserted] = cells_.try_emplace(key(x, y));
    (void)inserted;
    found->second.log_odds = std::min(kLimitHigh, std::max(kLimitLow, found->second.log_odds + amount));
    found->second.probability = probability_of_log_odds(found->second.log_odds);
  }

  void add_key(std::uint64_t key_value, double amount) {
    const auto [x, y] = decode(key_value);
    add(x, y, amount);
  }

  // 브레젠험 직선: start부터 end 앞까지의 칸 (레이가 지나간 칸).
  static void add_line(int x, int y, int x1, int y1, std::unordered_set<std::uint64_t>& out) {
    const int dx = std::abs(x1 - x);
    const int dy = -std::abs(y1 - y);
    const int sx = x < x1 ? 1 : -1;
    const int sy = y < y1 ? 1 : -1;
    int error = dx + dy;
    while (x != x1 || y != y1) {
      out.insert(key(x, y));
      const int e2 = 2 * error;
      if (e2 >= dy) {
        error += dy;
        x += sx;
      }
      if (e2 <= dx) {
        error += dx;
        y += sy;
      }
    }
  }

  double resolution_;
  std::unordered_map<std::uint64_t, Entry> cells_;
};

}  // namespace krosc::cartographer
