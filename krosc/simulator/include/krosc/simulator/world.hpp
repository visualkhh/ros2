#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "krosc/msgs.hpp"

namespace krosc::simulator {

// 월드 축에 평행한 벽 상자 (반폭/반높이).
struct Wall {
  double center_x = 0.0;
  double center_y = 0.0;
  double half_x = 0.0;
  double half_y = 0.0;
  double height = 0.25;  // 바닥에서 벽 윗면까지 (시각/충돌 높이)
};

class World;

class WorldObject {
 public:
  virtual ~WorldObject() = default;

  msgs::Pose2D pose;
  double radius = 0.0;    // 충돌 판정용 원 반지름 (0이면 충돌하지 않는다)
  double height = 0.0;    // 바닥에서 윗면까지 (센서 높이보다 낮으면 센서가 못 본다)
  double z = 0.0;         // 바닥 기준 아래면 높이 (들었을 때만 변한다)
  double mass = 1.0;
  double friction = 0.3;
  bool is_static = false;  // 월드 지형: 움직이지 않는다
  bool movable = false;    // 월드 물리가 움직인다 (로봇은 false: 스스로 움직인다)
  bool graspable = false;  // 매니퓰레이터가 집을 수 있는 물체
  bool held = false;       // 로봇 그리퍼가 들고 있다 (충돌/벽 밀림 제외)
  bool on_robot = false;   // 로봇 상판에 실려 로봇과 함께 움직인다
  std::pair<double, double> velocity{0.0, 0.0};
  // 화면 표시 힌트. sides가 0이면 스냅샷에 넣지 않는다.
  int sides = 32;
  std::array<double, 3> color{0.75, 0.78, 0.84};
  World* world = nullptr;

  virtual void tick(double time, double dt) {
    (void)time;
    (void)dt;
  }

  [[nodiscard]] virtual std::string kind() const { return "object"; }
};

// 월드: 벽과 객체를 들고, 레이/원 충돌을 푼다. krosc 토픽과 무관하다.
class World {
 public:
  explicit World(double gravity = 9.81, double ground_friction = 0.3);
  virtual ~World() = default;

  World(const World&) = delete;
  World& operator=(const World&) = delete;

  void add_wall(const Wall& wall);
  WorldObject* add(std::unique_ptr<WorldObject> object, std::optional<msgs::Pose2D> pose = std::nullopt);

  void step(double dt);

  [[nodiscard]] double time() const { return time_; }
  [[nodiscard]] double gravity() const { return gravity_; }
  [[nodiscard]] double ground_friction() const { return ground_friction_; }
  [[nodiscard]] const std::vector<Wall>& walls() const { return walls_; }
  [[nodiscard]] const std::vector<std::unique_ptr<WorldObject>>& objects() const { return objects_; }
  [[nodiscard]] std::vector<std::unique_ptr<WorldObject>>& objects() { return objects_; }

  [[nodiscard]] double ray_distance(double x, double y, double angle, double height = 0.0) const;
  void ray_distances(double x, double y, const std::vector<double>& angles, double height,
                     std::vector<float>& out) const;
  [[nodiscard]] std::pair<double, double> push_out(double x, double y, double radius) const;

 private:
  void resolve_objects(double dt);
  void resolve_walls();
  double friction_with_ground(const WorldObject& object) const;

  double gravity_;
  double ground_friction_;
  double time_ = 0.0;
  std::vector<Wall> walls_;
  std::vector<std::unique_ptr<WorldObject>> objects_;
};

// 원과 상자/원의 충돌. (도형에서 원 쪽 단위 법선 nx, ny, 깊이)
struct Push {
  double nx = 0.0;
  double ny = 0.0;
  double depth = 0.0;
};

[[nodiscard]] std::optional<Push> circle_vs_box(const Wall& box, double x, double y, double radius);
[[nodiscard]] std::optional<Push> circle_vs_circle(double cx, double cy, double cr, double x, double y, double radius);
[[nodiscard]] double ray_hit_box(const Wall& box, double x, double y, double dx, double dy);
[[nodiscard]] double ray_hit_circle(double cx, double cy, double radius, double x, double y, double dx, double dy);

}  // namespace krosc::simulator
