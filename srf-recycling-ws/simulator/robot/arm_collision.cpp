#include "robot/arm_collision.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace srf_recycling_ws::sim {

using render::Pose;
using render::Vec3;

namespace {

Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 scale(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double axis_of(const Vec3& v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

std::string read_file(const std::string& path, const char* what) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error(std::string(what) + "를 열 수 없다: " + path);
  }
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

using Tri = std::array<Vec3, 3>;

// 분리축 정리 (볼록 다각형 둘): 축 위 투영 구간이 떨어지면 안 닿는다.
bool separated_on(const Vec3& axis, const Vec3* a, int na, const Vec3* b, int nb) {
  if (dot(axis, axis) < 1e-18) {
    return false;
  }
  double amin = std::numeric_limits<double>::infinity(), amax = -amin, bmin = amin, bmax = -amin;
  for (int i = 0; i < na; ++i) {
    const double p = dot(a[i], axis);
    amin = std::min(amin, p);
    amax = std::max(amax, p);
  }
  for (int i = 0; i < nb; ++i) {
    const double p = dot(b[i], axis);
    bmin = std::min(bmin, p);
    bmax = std::max(bmax, p);
  }
  return amax < bmin || bmax < amin;
}

bool triangles_touch(const Tri& a, const Tri& b) {
  const Vec3 na = cross(sub(a[1], a[0]), sub(a[2], a[0]));
  const Vec3 nb = cross(sub(b[1], b[0]), sub(b[2], b[0]));
  if (separated_on(na, a.data(), 3, b.data(), 3) || separated_on(nb, a.data(), 3, b.data(), 3)) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    const Vec3 ea = sub(a[static_cast<std::size_t>((i + 1) % 3)], a[static_cast<std::size_t>(i)]);
    for (int j = 0; j < 3; ++j) {
      const Vec3 eb = sub(b[static_cast<std::size_t>((j + 1) % 3)], b[static_cast<std::size_t>(j)]);
      if (separated_on(cross(ea, eb), a.data(), 3, b.data(), 3)) {
        return false;
      }
    }
    // 같은 평면일 때: 평면 안 모서리 법선
    if (separated_on(cross(na, ea), a.data(), 3, b.data(), 3)) {
      return false;
    }
    const Vec3 eb = sub(b[static_cast<std::size_t>((i + 1) % 3)], b[static_cast<std::size_t>(i)]);
    if (separated_on(cross(nb, eb), a.data(), 3, b.data(), 3)) {
      return false;
    }
  }
  return true;
}

// 축 정렬 상자 A(A 좌표계) vs 상자 B(중심·축이 A 좌표계로 주어짐).
bool boxes_touch(const Vec3& ca, const Vec3& ha, const Vec3& cb, const std::array<Vec3, 3>& ub, const Vec3& hb) {
  const std::array<Vec3, 3> ua{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  const std::array<double, 3> ha_{ha.x, ha.y, ha.z};
  const std::array<double, 3> hb_{hb.x, hb.y, hb.z};
  const Vec3 d = sub(cb, ca);
  auto separated = [&](const Vec3& axis) {
    if (dot(axis, axis) < 1e-18) {
      return false;
    }
    double ra = 0.0, rb = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      ra += ha_[i] * std::abs(dot(ua[i], axis));
      rb += hb_[i] * std::abs(dot(ub[i], axis));
    }
    return std::abs(dot(d, axis)) > ra + rb;
  };
  for (std::size_t i = 0; i < 3; ++i) {
    if (separated(ua[i]) || separated(ub[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      if (separated(cross(ua[i], ub[j]))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

// 메시 좌표계의 축 정렬 경계 상자 트리. 잎은 삼각형 몇 개.
struct ArmCollision::Bvh {
  struct Node {
    Vec3 center;
    Vec3 half;
    int left = -1;  // -1이면 잎
    int right = -1;
    int first = 0;
    int count = 0;
  };
  std::vector<Tri> tris;
  std::vector<Node> nodes;

  explicit Bvh(std::vector<Tri> triangles) : tris(std::move(triangles)) {
    nodes.reserve(tris.size() * 2 / 3 + 1);
    build(0, static_cast<int>(tris.size()));
  }

  int build(int first, int count) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    Vec3 lo{kInf, kInf, kInf}, hi{-kInf, -kInf, -kInf};
    for (int i = first; i < first + count; ++i) {
      for (const auto& p : tris[static_cast<std::size_t>(i)]) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
      }
    }
    const int index = static_cast<int>(nodes.size());
    nodes.push_back({scale(add(lo, hi), 0.5), scale(sub(hi, lo), 0.5), -1, -1, first, count});
    if (count <= 4) {
      return index;
    }
    const Vec3 extent = sub(hi, lo);
    const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : extent.y >= extent.z ? 1 : 2;
    const auto begin = tris.begin() + first;
    const auto middle = begin + count / 2;
    std::nth_element(begin, middle, begin + count, [axis](const Tri& a, const Tri& b) {
      return axis_of(a[0], axis) + axis_of(a[1], axis) + axis_of(a[2], axis) <
             axis_of(b[0], axis) + axis_of(b[1], axis) + axis_of(b[2], axis);
    });
    const int left = build(first, count / 2);
    const int right = build(first + count / 2, count - count / 2);
    nodes[static_cast<std::size_t>(index)].left = left;
    nodes[static_cast<std::size_t>(index)].right = right;
    return index;
  }

  // a_from_b: B 메시 좌표 → A 메시 좌표
  static bool touch(const Bvh& a, const Bvh& b, const Pose& a_from_b) {
    const std::array<Vec3, 3> ub{{{a_from_b.r[0][0], a_from_b.r[1][0], a_from_b.r[2][0]},
                                  {a_from_b.r[0][1], a_from_b.r[1][1], a_from_b.r[2][1]},
                                  {a_from_b.r[0][2], a_from_b.r[1][2], a_from_b.r[2][2]}}};
    std::vector<std::pair<int, int>> stack{{0, 0}};
    while (!stack.empty()) {
      const auto [ia, ib] = stack.back();
      stack.pop_back();
      const Node& na = a.nodes[static_cast<std::size_t>(ia)];
      const Node& nb = b.nodes[static_cast<std::size_t>(ib)];
      if (!boxes_touch(na.center, na.half, a_from_b.apply(nb.center), ub, nb.half)) {
        continue;
      }
      const bool leaf_a = na.left < 0;
      const bool leaf_b = nb.left < 0;
      if (leaf_a && leaf_b) {
        for (int j = nb.first; j < nb.first + nb.count; ++j) {
          const Tri& tb = b.tris[static_cast<std::size_t>(j)];
          const Tri moved{a_from_b.apply(tb[0]), a_from_b.apply(tb[1]), a_from_b.apply(tb[2])};
          for (int i = na.first; i < na.first + na.count; ++i) {
            if (triangles_touch(a.tris[static_cast<std::size_t>(i)], moved)) {
              return true;
            }
          }
        }
        continue;
      }
      // 큰 쪽을 내려간다
      const double size_a = dot(na.half, na.half);
      const double size_b = dot(nb.half, nb.half);
      if (leaf_b || (!leaf_a && size_a >= size_b)) {
        stack.emplace_back(na.left, ib);
        stack.emplace_back(na.right, ib);
      } else {
        stack.emplace_back(ia, nb.left);
        stack.emplace_back(ia, nb.right);
      }
    }
    return false;
  }

  // world_from_mesh 로 놓았을 때 바닥(z<0) 아래로 내려간 꼭짓점이 있나.
  bool below_floor(const Pose& world_from_mesh) const {
    std::vector<int> stack{0};
    while (!stack.empty()) {
      const Node& node = nodes[static_cast<std::size_t>(stack.back())];
      stack.pop_back();
      const Vec3 c = world_from_mesh.apply(node.center);
      const double reach = node.half.x * std::abs(world_from_mesh.r[2][0]) +
                           node.half.y * std::abs(world_from_mesh.r[2][1]) +
                           node.half.z * std::abs(world_from_mesh.r[2][2]);
      if (c.z - reach >= 0.0) {
        continue;
      }
      if (node.left >= 0) {
        stack.push_back(node.left);
        stack.push_back(node.right);
        continue;
      }
      for (int i = node.first; i < node.first + node.count; ++i) {
        for (const auto& p : tris[static_cast<std::size_t>(i)]) {
          if (world_from_mesh.apply(p).z < 0.0) {
            return true;
          }
        }
      }
    }
    return false;
  }
};

std::set<std::pair<std::string, std::string>> ArmCollision::parse_disabled_pairs(const std::string& srdf_text) {
  std::set<std::pair<std::string, std::string>> pairs;
  const std::regex pattern(R"re(<disable_collisions\s+link1="([^"]+)"\s+link2="([^"]+)")re");
  for (auto it = std::sregex_iterator(srdf_text.begin(), srdf_text.end(), pattern); it != std::sregex_iterator(); ++it) {
    const std::string a = (*it)[1];
    const std::string b = (*it)[2];
    pairs.insert(std::minmax(a, b));
  }
  return pairs;
}

ArmCollision::ArmCollision(const std::string& urdf_path, const std::string& srdf_path) {
  robot_ = srf::parse_urdf(read_file(urdf_path, "URDF"));
  const auto disabled = parse_disabled_pairs(read_file(srdf_path, "SRDF"));
  const std::string urdf_dir = std::filesystem::path(urdf_path).parent_path().string();

  std::map<std::string, std::shared_ptr<const Bvh>> cache;
  for (const auto& link : robot_.links) {
    for (const auto& collision : link.collisions) {
      const auto& g = collision.geometry;
      std::shared_ptr<const render::Mesh> mesh;
      switch (g.type) {
        case srf::UrdfGeometryType::Box: mesh = render::make_box(g.size[0], g.size[1], g.size[2]); break;
        case srf::UrdfGeometryType::Cylinder: mesh = render::make_cylinder(g.radius, g.length, 24); break;
        case srf::UrdfGeometryType::Sphere: mesh = render::make_lumpy_sphere(g.radius, 0.0, 0, 8, 12); break;
        case srf::UrdfGeometryType::Mesh: {
          const std::string key = g.filename + "@" + std::to_string(g.scale[0]);
          if (auto found = cache.find(key); found != cache.end()) {
            bodies_.push_back({link.name, Pose::rpy(collision.xyz, collision.rpy), found->second});
            continue;
          }
          mesh = render::load_stl(render::resolve_mesh_path(urdf_dir, g.filename), g.scale);
          break;
        }
        case srf::UrdfGeometryType::None: continue;
      }
      if (!mesh || mesh->positions.empty()) {
        continue;
      }
      std::vector<Tri> tris;
      tris.reserve(mesh->positions.size());
      for (const auto& t : mesh->positions) {
        tris.push_back({Vec3{t[0].x, t[0].y, t[0].z}, Vec3{t[1].x, t[1].y, t[1].z}, Vec3{t[2].x, t[2].y, t[2].z}});
      }
      auto bvh = std::make_shared<const Bvh>(std::move(tris));
      if (g.type == srf::UrdfGeometryType::Mesh) {
        cache[g.filename + "@" + std::to_string(g.scale[0])] = bvh;
      }
      bodies_.push_back({link.name, Pose::rpy(collision.xyz, collision.rpy), bvh});
    }
  }

  // 팔 링크: 회전·직선 관절(바퀴 continuous 제외) 아래 전부
  bool grew = true;
  while (grew) {
    grew = false;
    for (const auto& joint : robot_.joints) {
      const bool moving = joint.type == "revolute" || joint.type == "prismatic";
      if ((moving || arm_links_.count(joint.parent) != 0) && arm_links_.insert(joint.child).second) {
        grew = true;
      }
    }
  }

  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    for (std::size_t j = i + 1; j < bodies_.size(); ++j) {
      const auto& a = bodies_[i].link;
      const auto& b = bodies_[j].link;
      if (a == b || (arm_links_.count(a) == 0 && arm_links_.count(b) == 0) || disabled.count(std::minmax(a, b)) != 0) {
        continue;
      }
      pairs_.emplace_back(i, j);
    }
  }
}

ArmCollision::~ArmCollision() = default;

std::vector<ArmCollision::Contact> ArmCollision::contacts(const std::map<std::string, double>& joints) const {
  const auto poses = render::forward_kinematics(robot_, srf::msgs::Pose2D{}, joints);
  std::vector<Pose> world(bodies_.size());
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    world[i] = poses.at(bodies_[i].link) * bodies_[i].offset;
  }
  std::vector<Contact> result;
  for (const auto& [i, j] : pairs_) {
    if (Bvh::touch(*bodies_[i].bvh, *bodies_[j].bvh, world[i].inverse() * world[j])) {
      result.push_back({bodies_[i].link, bodies_[j].link});
    }
  }
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    if (arm_links_.count(bodies_[i].link) != 0 && bodies_[i].bvh->below_floor(world[i])) {
      result.push_back({bodies_[i].link, "floor"});
    }
  }
  return result;
}

std::string to_string(const std::vector<ArmCollision::Contact>& contacts) {
  std::string text;
  for (const auto& contact : contacts) {
    text += (text.empty() ? "" : ", ") + contact.a + "↔" + contact.b;
  }
  return text;
}

}  // namespace srf_recycling_ws::sim
