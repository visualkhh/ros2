#pragma once

// 움직이는 사람·장비: 걷는 작업자(셀 담당 작업자 포함), 지게차.
//   월드 물체(원 반지름·높이)라 라이다·카메라에 보인다. 앞에 로봇·사람이 있으면 멈춘다.
#include <string>
#include <vector>

#include "world/recycling_world.hpp"

namespace srf_recycling_ws::sim {

class Worker final : public VisualObject {
 public:
  std::vector<Waypoint> route;
  std::size_t next = 0;
  bool loop = true;        // false면 마지막 지점에서 멈추고 route를 비운다 (셀 담당 작업)
  double speed = 1.1;
  double pause_left = 0.0;
  bool station = false;    // 컨베이어 작업자: 제자리에서 물품을 집는다
  double phase = 0.0;      // 걸음·팔 동작
  double stuck_time = 0.0; // 지점에 가까워지지 못한 시간
  double last_dist = 1e9;
  Carry carrying = Carry::None;
  render::Color carry_color{0.2F, 0.45F, 0.85F};
  render::Material shirt, pants, skin, helmet, vest, shoes;

  Worker();
  [[nodiscard]] std::string kind() const override { return "worker"; }
  [[nodiscard]] bool idle() const { return route.empty() && pause_left <= 0.0; }
  void tick(double time, double dt) override;
  void add_render(std::vector<render::Instance>& out, double time) const override;
};

class Forklift final : public VisualObject {
 public:
  double lane_y = 9.25;
  double x_min = 3.0;
  double x_max = 21.0;
  int direction = 1;  // +1 동쪽(짐 싣고), -1 서쪽
  double pause_left = 2.0;
  double fork_height = 0.1;
  bool loaded = true;
  double speed = 1.5;

  Forklift();
  [[nodiscard]] std::string kind() const override { return "forklift"; }
  void tick(double time, double dt) override;
  void add_render(std::vector<render::Instance>& out, double time) const override;
};

// 작업자에게 옷을 입힌다 (시드로 정해진 피부색·셔츠·안전모).
void dress_worker(Worker& worker, std::mt19937& rng);

}  // namespace srf_recycling_ws::sim
