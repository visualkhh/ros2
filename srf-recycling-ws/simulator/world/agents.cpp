// 움직이는 사람·장비: 걷는 작업자, 컨베이어 작업자, 셀 담당 작업자(공급·비우기), 지게차.
#include "world/agents.hpp"

#include <cmath>
#include <random>

#include "render/textures.hpp"

namespace srf_recycling_ws::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;
using render::Color;
using render::Material;
using render::Pose;

Material tex(const std::string& name, Color tint = {1, 1, 1}, float specular = 0.05F) {
  Material m = render::solid(tint, specular);
  m.texture = textures::get(name);
  return m;
}

double wrap_angle(double a) {
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}

// 앞쪽(heading 기준 ±45°) distance 안에 로봇·다른 사람·장비가 있나
bool blocked(const srf::simulator::World* world, const srf::simulator::WorldObject* self, double heading,
             double distance) {
  if (world == nullptr) {
    return false;
  }
  for (const auto& object : world->objects()) {
    if (object.get() == self || object->radius <= 0.0) {
      continue;
    }
    const std::string kind = object->kind();
    if (kind != "waffle_pi_arm" && kind != "worker" && kind != "forklift") {
      continue;
    }
    const double dx = object->pose.x - self->pose.x, dy = object->pose.y - self->pose.y;
    const double d = std::hypot(dx, dy) - object->radius - self->radius;
    if (d < distance && std::cos(std::atan2(dy, dx) - heading) > 0.7) {
      return true;
    }
  }
  return false;
}

}  // namespace

// ---- 작업자 ----

Worker::Worker() {
  radius = 0.25;
  height = 1.75;
  is_static = false;
  movable = false;
  sides = 0;  // 진실 화면은 add_render로 그린다
}

void Worker::tick(double time, double dt) {
  (void)time;
  if (station) {
    phase += dt * 1.6;
    return;
  }
  if (pause_left > 0) {
    pause_left -= dt;
    return;
  }
  if (route.empty()) {
    return;
  }
  const Waypoint& target = route[next];
  const double dx = target.x - pose.x, dy = target.y - pose.y;
  const double dist = std::hypot(dx, dy);
  // 안전장치: 막힌 것도 아닌데 3초 동안 가까워지지 못하면 (벽에 밀려 못 닿는 지점) 도착으로 친다
  if (dist < 0.08 || stuck_time > 3.0) {
    stuck_time = 0.0;
    last_dist = 1e9;
    pause_left = target.pause;
    if (target.action != CellAction::None) {
      // 지점에서 할 일: 들고 있는 것은 바로 바꾸고, 물체 추가·삭제는 월드가 스텝 뒤에 한다
      switch (target.action) {
        case CellAction::PickSupply: carrying = Carry::ToteFull; break;
        case CellAction::DumpSupply: carrying = Carry::ToteEmpty; break;
        case CellAction::CollectBin: carrying = Carry::Crate; break;
        case CellAction::DumpWheelie: carrying = Carry::None; break;
        default: break;
      }
      if (auto* recycling = dynamic_cast<RecyclingWorld*>(world)) {
        recycling->queue_action(target.action, target.bin);
      }
    }
    if (next + 1 >= route.size() && !loop) {
      route.clear();
      next = 0;
      return;
    }
    next = (next + 1) % route.size();
    return;
  }
  const double heading = std::atan2(dy, dx);
  pose.theta += std::clamp(wrap_angle(heading - pose.theta), -3.0 * dt, 3.0 * dt);
  if (blocked(world, this, pose.theta, 0.7)) {
    return;  // 로봇·사람이 앞에 있으면 기다린다
  }
  const double step = std::min(dist, speed * dt * std::max(0.0, std::cos(wrap_angle(heading - pose.theta))));
  pose.x += std::cos(pose.theta) * step;
  pose.y += std::sin(pose.theta) * step;
  phase += step / 0.75 * kPi;  // 보폭 0.75 m
  stuck_time = dist < last_dist - 1e-3 ? 0.0 : stuck_time + dt;
  last_dist = std::min(last_dist, dist);
}

void Worker::add_render(std::vector<render::Instance>& out, double time) const {
  (void)time;
  static const auto leg = render::make_box(0.13, 0.15, 0.84);
  static const auto shoe = render::make_box(0.26, 0.12, 0.08);
  static const auto torso = render::make_box(0.24, 0.38, 0.58);
  static const auto vest_mesh = render::make_box(0.26, 0.40, 0.42);
  static const auto arm = render::make_box(0.09, 0.09, 0.6);
  static const auto hand = render::make_box(0.08, 0.06, 0.1);
  static const auto head = render::make_lumpy_sphere(0.105, 0.0, 0, 12, 16);
  static const auto hat = render::make_frustum(0.13, 0.10, 0.08, 20);
  static const auto brim = render::make_cylinder(0.15, 0.012, 20);
  static const auto tote = render::make_box(0.42, 0.32, 0.22);
  static const auto tote_fill = render::make_lumpy_sphere(0.16, 0.35, 11, 8, 12);
  const Pose base = Pose::translation(pose.x, pose.y, 0) * Pose::yaw(pose.theta);
  const bool carry = carrying != Carry::None;
  const double swing = station ? 0.0 : 0.45 * std::sin(phase);
  for (const int side : {-1, 1}) {
    const double y = side * 0.1;
    const Pose hip = base * Pose::translation(0, y, 0.88) * Pose::axis_angle({0, 1, 0}, side * swing);
    out.push_back({leg, hip * Pose::translation(0, 0, -0.42), pants});
    out.push_back({shoe, hip * Pose::translation(0.05, 0, -0.84), shoes});
    // 팔: 걸을 때는 다리와 반대로, 컨베이어 작업자는 앞으로 뻗어 집고, 짐을 들면 앞으로 받친다
    const double reach = carry ? 1.0 : station ? (side > 0 ? 0.9 + 0.5 * std::sin(phase) : 0.4) : -side * swing;
    const Pose shoulder = base * Pose::translation(0, side * 0.24, 1.42) * Pose::axis_angle({0, 1, 0}, -reach);
    out.push_back({arm, shoulder * Pose::translation(0, 0, -0.3), shirt});
    out.push_back({hand, shoulder * Pose::translation(0, 0, -0.64), skin});
  }
  out.push_back({torso, base * Pose::translation(0, 0, 1.16), shirt});
  out.push_back({vest_mesh, base * Pose::translation(0, 0, 1.22), vest});
  out.push_back({head, base * Pose::translation(0.01, 0, 1.6), skin});
  out.push_back({hat, base * Pose::translation(0.01, 0, 1.69), helmet});
  out.push_back({brim, base * Pose::translation(0.04, 0, 1.655), helmet});
  if (carry) {  // 토트(공급)·바구니(비우기)를 앞에 들고 간다
    const Pose held = base * Pose::translation(0.5, 0, 1.0);
    out.push_back({tote, held, render::solid(carry_color, 0.2F, 30)});
    if (carrying != Carry::ToteEmpty) {
      out.push_back({tote_fill, held * Pose::translation(0, 0, 0.1), tex("bale_pet", {1, 1, 1}, 0.2F)});
    }
  }
}

void dress_worker(Worker& w, std::mt19937& rng) {
  std::uniform_real_distribution<double> u(0, 1);
  static const std::array<Color, 6> skins{{{0.96F, 0.80F, 0.66F}, {0.88F, 0.70F, 0.55F}, {0.72F, 0.53F, 0.40F},
                                           {0.55F, 0.38F, 0.27F}, {0.93F, 0.76F, 0.62F}, {0.80F, 0.62F, 0.48F}}};
  w.skin = render::solid(skins[static_cast<std::size_t>(rng() % skins.size())], 0.05F);
  w.shirt = tex(u(rng) < 0.5 ? "shirt_blue" : "shirt_gray");
  w.pants = tex("denim");
  w.vest = tex("hi_vis_vest", {1, 1, 1}, 0.1F);
  w.helmet = render::solid(u(rng) < 0.6 ? Color{0.95F, 0.82F, 0.1F} : Color{0.95F, 0.95F, 0.93F}, 0.5F, 60);
  w.shoes = render::solid({0.1F, 0.08F, 0.07F}, 0.2F);
  w.speed = 0.95 + u(rng) * 0.35;
}

// ---- 지게차 ----

Forklift::Forklift() {
  radius = 0.7;
  height = 2.1;
  is_static = false;
  movable = false;
  sides = 0;
}

void Forklift::tick(double time, double dt) {
  (void)time;
  const double target_heading = direction > 0 ? 0.0 : kPi;
  if (pause_left > 0) {  // 끝에서 멈춰 포크를 올렸다 내리고 돈다
    pause_left -= dt;
    fork_height = 0.1 + 0.6 * std::max(0.0, std::sin(std::min(1.0, pause_left / 4.0) * kPi));
    pose.theta += std::clamp(wrap_angle(target_heading - pose.theta), -0.8 * dt, 0.8 * dt);
    return;
  }
  fork_height = 0.12;
  pose.theta += std::clamp(wrap_angle(target_heading - pose.theta), -0.8 * dt, 0.8 * dt);
  if (std::abs(wrap_angle(target_heading - pose.theta)) > 0.05) {
    return;  // 다 돌 때까지
  }
  if (blocked(world, this, pose.theta, 1.6)) {
    return;  // 앞에 사람·로봇
  }
  pose.x += direction * speed * dt;
  pose.y = lane_y;
  if ((direction > 0 && pose.x >= x_max) || (direction < 0 && pose.x <= x_min)) {
    direction = -direction;
    loaded = direction > 0;  // 하역장에서 싣고 적재장에 내린다
    pause_left = 4.0;
  }
}

void Forklift::add_render(std::vector<render::Instance>& out, double time) const {
  (void)time;
  static const auto chassis = render::make_box(1.5, 1.1, 0.6);
  static const auto counterweight = render::make_box(0.45, 1.1, 0.75);
  static const auto post = render::make_box(0.06, 0.06, 1.3);
  static const auto roof = render::make_box(1.1, 1.0, 0.05);
  static const auto rail = render::make_box(0.08, 0.08, 2.2);
  static const auto carriage = render::make_box(0.05, 0.9, 0.45);
  static const auto fork = render::make_box(1.0, 0.1, 0.045);
  static const auto wheel_front = render::make_cylinder(0.28, 0.22, 24);
  static const auto wheel_rear = render::make_cylinder(0.22, 0.2, 24);
  static const auto seat = render::make_box(0.4, 0.5, 0.15);
  static const auto beacon = render::make_cylinder(0.06, 0.08, 16);
  static const auto pallet = render::make_box(1.2, 1.0, 0.14);
  static const auto bag = render::make_box(0.95, 0.95, 0.85);
  const Material body = tex("forklift_body", {1, 1, 1}, 0.35F);
  const Material dark = render::solid({0.12F, 0.12F, 0.13F}, 0.3F, 40);
  const Material metal = render::solid({0.55F, 0.56F, 0.58F}, 0.6F, 60);
  const Material tire = render::solid({0.06F, 0.06F, 0.06F}, 0.1F);
  Material light = render::solid({1.0F, 0.55F, 0.1F});
  light.emissive = 0.9F;
  const Pose base = Pose::translation(pose.x, pose.y, 0) * Pose::yaw(pose.theta);
  out.push_back({chassis, base * Pose::translation(-0.05, 0, 0.5), body});
  out.push_back({counterweight, base * Pose::translation(-0.8, 0, 0.55), dark});
  out.push_back({seat, base * Pose::translation(-0.25, 0, 0.88), dark});
  for (const double px : {-0.55, 0.45}) {
    for (const double py : {-0.48, 0.48}) {
      out.push_back({post, base * Pose::translation(px, py, 1.45), dark});
    }
  }
  out.push_back({roof, base * Pose::translation(-0.05, 0, 2.12), dark});
  out.push_back({beacon, base * Pose::translation(-0.3, 0, 2.19), light});
  for (const double py : {-0.35, 0.35}) {
    out.push_back({rail, base * Pose::translation(0.78, py, 1.15), metal});
  }
  out.push_back({carriage, base * Pose::translation(0.84, 0, fork_height + 0.25), metal});
  for (const double py : {-0.28, 0.28}) {
    out.push_back({fork, base * Pose::translation(1.35, py, fork_height), metal});
  }
  for (const double py : {-0.5, 0.5}) {
    out.push_back({wheel_front, base * Pose::translation(0.45, py, 0.28) * Pose::axis_angle({1, 0, 0}, kPi / 2), tire});
    out.push_back({wheel_rear, base * Pose::translation(-0.65, py, 0.22) * Pose::axis_angle({1, 0, 0}, kPi / 2), tire});
  }
  if (loaded) {
    out.push_back({pallet, base * Pose::translation(1.4, 0, fork_height + 0.09), tex("pallet_wood")});
    out.push_back({bag, base * Pose::translation(1.4, 0, fork_height + 0.6), tex("jumbo_bag")});
  }
}

// ---- 배치 ----

void RecyclingWorld::spawn_agents(const SceneConfig& config) {
  std::mt19937 rng(config.seed * 31U + 7U);
  std::uniform_real_distribution<double> u(0, 1);
  // 걷는 작업자 경로 (통로·하역장·셀 점검·수거함 구역)
  const std::array<std::vector<Waypoint>, 4> routes{{
      {{0.8, 7.7, 1.0}, {23.0, 7.7, 2.0}},                                     // 보행자 통로 왕복
      {{2.5, 10.3, 3.0}, {7.2, 10.35, 1.0}, {7.2, 7.9, 0.0}, {2.5, 7.9, 1.0}},  // 하역장 ↔ 통로
      {{9.6, 7.6, 2.0}, {6.2, 7.3, 0.0}, {6.0, 2.0, 4.0}, {6.2, 7.3, 0.0}},      // 셀 점검: 매트와 바구니 사이를 지난다
      {{11.0, 2.4, 2.0}, {17.2, 2.4, 1.0}, {17.2, 5.6, 2.0}, {11.0, 5.6, 1.0}},  // 수거함 구역
  }};
  for (int i = 0; i < std::clamp(config.people, 0, 4); ++i) {
    auto worker = std::make_unique<Worker>();
    dress_worker(*worker, rng);
    worker->route = routes[static_cast<std::size_t>(i)];
    worker->next = 1;
    const auto& start = worker->route[0];
    worker->pause_left = u(rng) * 3.0;  // 출발 시각을 흩는다
    add(std::move(worker), srf::msgs::Pose2D{start.x, start.y, 0.0});
  }
  for (const double x : {10.0, 12.5, 15.0}) {  // 컨베이어 작업자 (항상)
    auto worker = std::make_unique<Worker>();
    dress_worker(*worker, rng);
    worker->station = true;
    worker->phase = u(rng) * 6.0;
    add(std::move(worker), srf::msgs::Pose2D{x, 11.05, kPi / 2});
  }
  // 셀 담당 작업자 (항상): 셀 북서쪽 구석에서 대기하다가 공급·비우기를 한다
  auto cell_worker = std::make_unique<Worker>();
  dress_worker(*cell_worker, rng);
  cell_worker->loop = false;
  cell_worker->helmet = render::solid({0.2F, 0.45F, 0.9F}, 0.5F, 60);  // 파란 안전모: 셀 담당
  const auto& idle = layout_.operator_idle;
  operator_ = static_cast<Worker*>(add(std::move(cell_worker), srf::msgs::Pose2D{idle.x, idle.y, -kPi / 2}));
  if (config.forklifts > 0) {
    add(std::make_unique<Forklift>(), srf::msgs::Pose2D{3.0, 9.25, 0.0});
  }
}

}  // namespace srf_recycling_ws::sim
