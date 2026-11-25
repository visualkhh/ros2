#include "krosc/simulator/world.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace krosc::simulator {

namespace {
constexpr double kEpsilon = 1e-12;

double clamp(double value, double lo, double hi) { return std::min(hi, std::max(lo, value)); }
}  // namespace

std::optional<Push> circle_vs_box(const Wall& box, double x, double y, double radius) {
  const double min_x = box.center_x - box.half_x;
  const double max_x = box.center_x + box.half_x;
  const double min_y = box.center_y - box.half_y;
  const double max_y = box.center_y + box.half_y;
  const double px = clamp(x, min_x, max_x);
  const double py = clamp(y, min_y, max_y);
  const double dx = x - px;
  const double dy = y - py;
  const double distance = std::hypot(dx, dy);
  if (distance > 1e-9) {
    if (distance >= radius) {
      return std::nullopt;
    }
    return Push{dx / distance, dy / distance, radius - distance};
  }
  const std::pair<double, Push> sides[] = {
      {x - min_x, {-1.0, 0.0, 0.0}},
      {max_x - x, {1.0, 0.0, 0.0}},
      {y - min_y, {0.0, -1.0, 0.0}},
      {max_y - y, {0.0, 1.0, 0.0}},
  };
  const auto best = std::min_element(std::begin(sides), std::end(sides),
                                     [](const auto& a, const auto& b) { return a.first < b.first; });
  return Push{best->second.nx, best->second.ny, best->first + radius};
}

std::optional<Push> circle_vs_circle(double cx, double cy, double cr, double x, double y, double radius) {
  const double dx = x - cx;
  const double dy = y - cy;
  const double distance = std::hypot(dx, dy);
  const double depth = radius + cr - distance;
  if (depth <= 0.0) {
    return std::nullopt;
  }
  if (distance > 1e-9) {
    return Push{dx / distance, dy / distance, depth};
  }
  return Push{1.0, 0.0, depth};
}

double ray_hit_box(const Wall& box, double x, double y, double dx, double dy) {
  const double min_x = box.center_x - box.half_x;
  const double max_x = box.center_x + box.half_x;
  const double min_y = box.center_y - box.half_y;
  const double max_y = box.center_y + box.half_y;
  if (x > min_x && x < max_x && y > min_y && y < max_y) {
    return std::numeric_limits<double>::infinity();
  }
  double t0 = 0.0;
  double t1 = std::numeric_limits<double>::infinity();
  const double origins[2] = {x, y};
  const double directions[2] = {dx, dy};
  const double lows[2] = {min_x, min_y};
  const double highs[2] = {max_x, max_y};
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(directions[axis]) < kEpsilon) {
      if (origins[axis] < lows[axis] || origins[axis] > highs[axis]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double ta = (lows[axis] - origins[axis]) / directions[axis];
    double tb = (highs[axis] - origins[axis]) / directions[axis];
    if (ta > tb) {
      std::swap(ta, tb);
    }
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
    if (t0 > t1) {
      return std::numeric_limits<double>::infinity();
    }
  }
  return t0 > 0.0 ? t0 : std::numeric_limits<double>::infinity();
}

double ray_hit_circle(double cx, double cy, double radius, double x, double y, double dx, double dy) {
  const double fx = cx - x;
  const double fy = cy - y;
  const double along = fx * dx + fy * dy;
  const double off2 = fx * fx + fy * fy - along * along;
  if (fx * fx + fy * fy <= radius * radius || off2 > radius * radius || along < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return along - std::sqrt(std::max(0.0, radius * radius - off2));
}

World::World(double gravity, double ground_friction) : gravity_(gravity), ground_friction_(ground_friction) {}

void World::add_wall(const Wall& wall) { walls_.push_back(wall); }

WorldObject* World::add(std::unique_ptr<WorldObject> object, std::optional<msgs::Pose2D> pose) {
  if (pose) {
    object->pose = *pose;
  }
  object->world = this;
  WorldObject* raw = object.get();
  objects_.push_back(std::move(object));
  return raw;
}

void World::step(double dt) {
  if (dt <= 0.0) {
    return;
  }
  time_ += dt;
  for (auto& object : objects_) {
    object->tick(time_, dt);
  }
  resolve_objects(dt);
  resolve_walls();
}

double World::ray_distance(double x, double y, double angle, double height) const {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double best = std::numeric_limits<double>::infinity();
  for (const auto& wall : walls_) {
    best = std::min(best, ray_hit_box(wall, x, y, dx, dy));
  }
  for (const auto& object : objects_) {
    if (object->radius > 0.0 && object->height > height) {
      best = std::min(best, ray_hit_circle(object->pose.x, object->pose.y, object->radius, x, y, dx, dy));
    }
  }
  return best;
}

void World::ray_distances(double x, double y, const std::vector<double>& angles, double height,
                          std::vector<float>& out) const {
  out.resize(angles.size());
  for (std::size_t i = 0; i < angles.size(); ++i) {
    out[i] = static_cast<float>(ray_distance(x, y, angles[i], height));
  }
}

std::pair<double, double> World::push_out(double x, double y, double radius) const {
  for (int iteration = 0; iteration < 4; ++iteration) {
    for (const auto& wall : walls_) {
      if (const auto hit = circle_vs_box(wall, x, y, radius); hit) {
        x += hit->nx * hit->depth;
        y += hit->ny * hit->depth;
      }
    }
  }
  return {x, y};
}

double World::friction_with_ground(const WorldObject& object) const {
  return std::sqrt(object.friction * ground_friction_);
}

void World::resolve_objects(double dt) {
  // movable 객체(퍽)가 미끄러진다: 벽/서로/로봇에 막힌 만큼만 실제 속도가 된다.
  for (auto& object : objects_) {
    if (!object->movable || object->is_static) {
      continue;
    }
    const double vx = object->velocity.first;
    const double vy = object->velocity.second;
    if (vx == 0.0 && vy == 0.0) {
      continue;
    }
    const double nx = object->pose.x + vx * dt;
    const double ny = object->pose.y + vy * dt;
    const auto [px, py] = push_out(nx, ny, object->radius);
    const double moved_x = px - object->pose.x;
    const double moved_y = py - object->pose.y;
    object->pose.x = px;
    object->pose.y = py;
    const double speed = std::hypot(moved_x, moved_y) / dt;
    if (speed <= 1e-9) {
      object->velocity = {0.0, 0.0};
      continue;
    }
    const double slowed = std::max(0.0, speed - friction_with_ground(*object) * gravity_ * dt) / speed;
    object->velocity = {moved_x / dt * slowed, moved_y / dt * slowed};
  }

  // 원 충돌: 로봇(비 movable)은 밀리지 않고, movable 쪽이 밀린다.
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    for (std::size_t j = i + 1; j < objects_.size(); ++j) {
      WorldObject& a = *objects_[i];
      WorldObject& b = *objects_[j];
      if (a.radius <= 0.0 || b.radius <= 0.0 || a.is_static || b.is_static || (!a.movable && !b.movable)) {
        continue;
      }
      const double dx = b.pose.x - a.pose.x;
      const double dy = b.pose.y - a.pose.y;
      const double distance = std::hypot(dx, dy);
      const double gap = a.radius + b.radius;
      if (distance >= gap) {
        continue;
      }
      const double nx = distance > 1e-9 ? dx / distance : 1.0;
      const double ny = distance > 1e-9 ? dy / distance : 0.0;
      const double overlap = gap - distance;
      const double share_a = !a.movable ? 0.0 : (b.movable ? b.mass / (a.mass + b.mass) : 1.0);
      const double share_b = 1.0 - share_a;
      const auto [ax, ay] = a.movable ? push_out(a.pose.x - nx * overlap * share_a, a.pose.y - ny * overlap * share_a, a.radius)
                                      : std::pair<double, double>{a.pose.x, a.pose.y};
      const auto [bx, by] = b.movable ? push_out(b.pose.x + nx * overlap * share_b, b.pose.y + ny * overlap * share_b, b.radius)
                                      : std::pair<double, double>{b.pose.x, b.pose.y};
      if (a.movable) {
        a.pose.x = ax;
        a.pose.y = ay;
      }
      if (b.movable) {
        b.pose.x = bx;
        b.pose.y = by;
      }
    }
  }
}

void World::resolve_walls() {
  for (auto& object : objects_) {
    if (!object->is_static && object->radius > 0.0) {
      const auto [x, y] = push_out(object->pose.x, object->pose.y, object->radius);
      object->pose.x = x;
      object->pose.y = y;
    }
  }
}

}  // namespace krosc::simulator
