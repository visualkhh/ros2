#include "world/recycling_world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <sstream>

#include "render/textures.hpp"

namespace srf_recycling_ws::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;
constexpr double kDropHeight = 0.03;  // 수거함 밖에서 이보다 높이서 떨어지면 낙하

using render::Pose;

Pose up(double z) { return Pose::translation(0, 0, z); }

std::string fixed(double value, int digits) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.*f", digits, value);
  return text;
}

render::Material shadow_material(float strength) {
  render::Material m;
  m.texture = textures::get("shadow_blob");
  m.blend = render::Blend::Multiply;
  m.opacity = strength;
  return m;
}

}  // namespace

const char* category_name(Category category) {
  switch (category) {
    case Category::Can: return "can";
    case Category::Pet: return "pet";
    case Category::Paper: return "paper";
    case Category::Unknown: return "unknown";
    case Category::Other: return "other";
  }
  return "unknown";
}

std::optional<Category> parse_category(const std::string& text) {
  for (const auto c : {Category::Can, Category::Pet, Category::Paper, Category::Unknown, Category::Other}) {
    if (text == category_name(c)) {
      return c;
    }
  }
  return std::nullopt;
}

const char* scene_name(Scene scene) {
  switch (scene) {
    case Scene::Normal: return "normal";
    case Scene::Multi: return "multi";
    case Scene::Ambiguous: return "ambiguous";
    case Scene::Absent: return "absent";
  }
  return "normal";
}

std::optional<Scene> parse_scene(const std::string& text) {
  for (const auto s : {Scene::Normal, Scene::Multi, Scene::Ambiguous, Scene::Absent}) {
    if (text == scene_name(s)) {
      return s;
    }
  }
  return std::nullopt;
}

// ---- 물품 ----

render::Pose RecyclableItem::world_pose() const {
  const Pose base = Pose::translation(pose.x, pose.y, z) * Pose::yaw(pose.theta);
  if (!lying) {
    return base;
  }
  // 옆으로 눕힘: 물품 +z 축이 월드 x로, 바닥에 닿도록 반지름만큼 올리고 길이 중심을 원점에.
  return base * up(diameter / 2) * Pose::axis_angle({0, 1, 0}, kPi / 2) * up(-length / 2);
}

void RecyclableItem::add_render(std::vector<render::Instance>& out, double time) const {
  (void)time;
  const Pose pose_world = world_pose();
  for (const auto& part : parts) {
    out.push_back({part.mesh, pose_world * part.offset, part.material});
  }
}

void RecyclableItem::tick(double time, double dt) {
  (void)time;
  if (held || on_robot) {
    was_held = was_held || held;
    velocity = {0.0, 0.0};
    return;
  }
  if (was_held) {  // 방금 그리퍼에서 놓였다
    was_held = false;
    ++releases;
    last_release_height = z;
  }
  if (z <= 0.0) {
    return;
  }
  fall_speed += kGravity * dt;  // 자유낙하. 바닥(바구니 안도 바닥 높이)에 닿으면 낙하 여부를 판정한다.
  z = std::max(0.0, z - fall_speed * dt);
  if (z == 0.0) {
    fall_speed = 0.0;
    const auto* recycling = dynamic_cast<const RecyclingWorld*>(world);
    const bool in_bin = recycling != nullptr && recycling->bin_containing(*this) != nullptr;
    if (!in_bin && last_release_height > kDropHeight) {
      dropped = true;
    }
  }
}

// ---- 월드 ----

RecyclingWorld::RecyclingWorld(RecyclingLayout layout) : layout_(std::move(layout)) {
  build_facility();
  // 미션 수거 바구니: 벽 4개(충돌·라이다) + 라벨 텍스처 + 바닥
  for (const auto& bin : layout_.bins) {
    const double hx = bin.inner_x / 2, hy = bin.inner_y / 2, wt = bin.wall;
    render::Material outside = render::solid({1, 1, 1}, 0.15F, 30);
    outside.texture = textures::get(bin.texture);
    const std::array<srf::simulator::Wall, 4> walls{{
        {bin.x, bin.y - hy - wt / 2, hx + wt, wt / 2, bin.height},
        {bin.x, bin.y + hy + wt / 2, hx + wt, wt / 2, bin.height},
        {bin.x - hx - wt / 2, bin.y, wt / 2, hy, bin.height},
        {bin.x + hx + wt / 2, bin.y, wt / 2, hy, bin.height},
    }};
    for (const auto& wall : walls) {
      add_wall(wall);
      add_box(wall.center_x, wall.center_y, wall.height / 2, wall.half_x * 2, wall.half_y * 2, wall.height, outside);
    }
    const render::Color dark{bin.color[0] * 0.55F, bin.color[1] * 0.55F, bin.color[2] * 0.55F};
    add_box(bin.x, bin.y, 0.003, bin.inner_x, bin.inner_y, 0.006, render::solid(dark));
    add_static({render::make_quad(bin.inner_x * 1.9, bin.inner_y * 1.9), Pose::translation(bin.x, bin.y, 0.0035),
                shadow_material(0.35F)});
  }
}

void RecyclingWorld::add_box(double cx, double cy, double cz, double sx, double sy, double sz,
                             const render::Material& material, bool collide) {
  add_static({render::make_box(sx, sy, sz), Pose::translation(cx, cy, cz), material});
  if (collide) {
    add_wall({cx, cy, sx / 2, sy / 2, cz + sz / 2});
  }
}

void RecyclingWorld::randomize(const SceneConfig& config) {
  scene_ = config;
  auto& all = objects();
  all.erase(std::remove_if(all.begin(), all.end(),
                           [](const auto& object) {
                             return object && (object->kind() == "recyclable" || object->kind() == "worker" ||
                                               object->kind() == "forklift");
                           }),
            all.end());
  std::mt19937 rng(config.seed);

  // 1) 종류 목록
  const std::array<Category, 3> kinds{Category::Can, Category::Pet, Category::Paper};
  const auto random_kind = [&](std::optional<Category> exclude) {
    for (;;) {
      const Category c = kinds[std::uniform_int_distribution<std::size_t>(0, 2)(rng)];
      if (!exclude || c != *exclude) {
        return c;
      }
    }
  };
  std::vector<Category> list;
  const int count = std::max(config.items, 0);
  switch (config.scene) {
    case Scene::Normal:
      if (config.target) {  // 지시 대상이 정확히 하나
        list.push_back(*config.target);
        while (static_cast<int>(list.size()) < std::max(count, 1)) list.push_back(random_kind(config.target));
      } else {              // 세 종류가 다 나오게
        for (int i = 0; i < count; ++i) list.push_back(i < 3 ? kinds[static_cast<std::size_t>(i)] : random_kind({}));
      }
      break;
    case Scene::Multi: {  // 같은 종류 후보가 둘 이상
      const Category c = config.target ? *config.target : random_kind({});
      list = {c, c};
      while (static_cast<int>(list.size()) < std::max(count, 2)) list.push_back(random_kind({}));
      break;
    }
    case Scene::Ambiguous:  // 분류가 불확실한 물품 하나 + 나머지
      list.push_back(Category::Unknown);
      while (static_cast<int>(list.size()) < std::max(count, 1)) list.push_back(random_kind(config.target));
      break;
    case Scene::Absent:  // 대상이 없음 (대상 지정 없으면 작업 영역이 빈다)
      if (config.target) {
        for (int i = 0; i < count; ++i) list.push_back(random_kind(config.target));
      }
      break;
  }
  for (int i = 0; i < config.distractors && !(config.scene == Scene::Absent && !config.target); ++i) {
    list.push_back(Category::Other);
  }
  std::shuffle(list.begin(), list.end(), rng);

  // 2) 작업 매트 안 임의 위치·방향 (겹치지 않게)
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const auto& area = layout_.work_area;
  std::vector<std::pair<double, double>> placed;
  std::vector<double> radii;
  int next_id = 1;
  for (const Category category : list) {
    auto item = std::make_unique<RecyclableItem>();
    item->id = next_id++;
    item->category = category;
    build_item(*item, rng);
    item->lying = item->can_lie && unit(rng) < config.lying_ratio;
    item->radius = item->lying ? item->length / 2 : item->diameter / 2 * std::sqrt(2.0) * 0.85;
    item->height = item->lying ? item->diameter : item->length;
    item->mass = 0.05;
    item->friction = 0.5;
    item->movable = true;
    item->graspable = true;
    item->sides = 24;
    const auto& main_color = item->parts[item->parts.size() > 1 ? 1 : 0].material.color;
    item->color = {main_color[0], main_color[1], main_color[2]};
    srf::msgs::Pose2D pose;
    bool ok = false;
    for (int attempt = 0; attempt < 400 && !ok; ++attempt) {
      pose.x = area.min_x + item->radius + 0.02 + unit(rng) * (area.max_x - area.min_x - 2 * (item->radius + 0.02));
      pose.y = area.min_y + item->radius + 0.02 + unit(rng) * (area.max_y - area.min_y - 2 * (item->radius + 0.02));
      pose.theta = (unit(rng) * 2 - 1) * kPi;
      ok = true;
      for (std::size_t i = 0; i < placed.size(); ++i) {
        if (std::hypot(pose.x - placed[i].first, pose.y - placed[i].second) < radii[i] + item->radius + 0.03) {
          ok = false;
          break;
        }
      }
    }
    if (!ok) {
      std::fprintf(stderr, "[recycling] 물품 %d를 놓을 자리가 없다 (작업 매트가 좁다)\n", item->id);
      continue;
    }
    placed.emplace_back(pose.x, pose.y);
    radii.push_back(item->radius);
    add(std::move(item), pose);
  }
  spawn_agents(config);
}

std::vector<const RecyclableItem*> RecyclingWorld::items() const {
  std::vector<const RecyclableItem*> out;
  for (const auto& object : objects()) {
    if (const auto* item = dynamic_cast<const RecyclableItem*>(object.get())) {
      out.push_back(item);
    }
  }
  return out;
}

const BinSpec* RecyclingWorld::bin_containing(const RecyclableItem& item) const {
  if (item.held || item.on_robot) {
    return nullptr;  // 그리퍼에서 완전히 분리돼야 한다
  }
  for (const auto& bin : layout_.bins) {
    if (bin.inner().contains(item.pose.x, item.pose.y, 0.01) && item.z < bin.height) {
      return &bin;
    }
  }
  return nullptr;
}

std::string RecyclingWorld::truth_json(double time) const {
  std::ostringstream out;
  int correct = 0, wrong = 0, dropped = 0, remaining = 0, held = 0;
  out << "{\"time\":" << fixed(time, 3) << ",\"seed\":" << scene_.seed << ",\"scene\":\"" << scene_name(scene_.scene)
      << "\",\"target\":" << (scene_.target ? std::string("\"") + category_name(*scene_.target) + "\"" : "null")
      << ",\"items\":[";
  bool first = true;
  for (const auto* item : items()) {
    const BinSpec* bin = bin_containing(*item);
    const std::string state = item->held ? "held" : item->on_robot ? "on_robot" : bin ? "bin"
                              : layout_.work_area.contains(item->pose.x, item->pose.y) ? "work_area" : "floor";
    if (bin) {
      (bin->category == item->category ? correct : wrong) += 1;
    } else if (item->held || item->on_robot) {
      ++held;
    } else {
      ++remaining;
    }
    dropped += item->dropped ? 1 : 0;
    out << (first ? "" : ",") << "{\"id\":" << item->id << ",\"category\":\"" << category_name(item->category)
        << "\",\"variant\":\"" << item->variant << "\",\"x\":" << fixed(item->pose.x, 4)
        << ",\"y\":" << fixed(item->pose.y, 4) << ",\"z\":" << fixed(item->z, 4)
        << ",\"yaw\":" << fixed(item->pose.theta, 3) << ",\"lying\":" << (item->lying ? "true" : "false")
        << ",\"state\":\"" << state << "\",\"bin\":"
        << (bin ? std::string("\"") + category_name(bin->category) + "\"" : "null") << ",\"correct\":"
        << (bin ? (bin->category == item->category ? "true" : "false") : "null") << ",\"bin_error_mm\":"
        << (bin ? fixed(std::hypot(item->pose.x - bin->x, item->pose.y - bin->y) * 1000.0, 1) : "null")
        << ",\"dropped\":" << (item->dropped ? "true" : "false") << ",\"releases\":" << item->releases << "}";
    first = false;
  }
  out << "],\"summary\":{\"total\":" << items().size() << ",\"correct\":" << correct << ",\"wrong\":" << wrong
      << ",\"dropped\":" << dropped << ",\"held\":" << held << ",\"remaining\":" << remaining << "}}";
  return out.str();
}

void RecyclingWorld::add_render_instances(std::vector<render::Instance>& out) const {
  out.insert(out.end(), static_instances_.begin(), static_instances_.end());
  add_conveyor_render(out);
  for (const auto& object : objects()) {
    if (const auto* visual = dynamic_cast<const VisualObject*>(object.get())) {
      visual->add_render(out, time());
    }
    // 접지 그림자 (바닥에 있는 것만)
    if (object->radius > 0.0 && object->z < 0.3 && !object->held) {
      const double s = object->radius * (object->kind() == "worker" ? 2.6 : 2.2);
      const Pose pose = Pose::translation(object->pose.x, object->pose.y, 0.0045) * Pose::yaw(object->pose.theta);
      out.push_back({render::make_quad(s, s * (object->kind() == "forklift" ? 0.6 : 1.0)), pose,
                     shadow_material(object->kind() == "recyclable" ? 0.45F : 0.55F)});
    }
  }
}

}  // namespace srf_recycling_ws::sim
