#include "world/recycling_world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <sstream>

namespace srf_recycling_ws::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;
constexpr double kDropHeight = 0.03;  // 수거함 밖에서 이보다 높이서 떨어지면 낙하

using render::Color;
using render::Pose;

Pose up(double z) { return Pose::translation(0, 0, z); }

std::string fixed(double value, int digits) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.*f", digits, value);
  return text;
}

// ---- 물품 모양 (아래면 중심이 원점, +z 위) ----

void make_can(RecyclableItem& item, std::mt19937& rng) {
  static const std::array<std::pair<const char*, Color>, 4> kLabels{{
      {"red", {0.80F, 0.10F, 0.12F}}, {"blue", {0.12F, 0.30F, 0.75F}},
      {"green", {0.10F, 0.55F, 0.25F}}, {"black", {0.12F, 0.12F, 0.14F}}}};
  const auto& label = kLabels[std::uniform_int_distribution<std::size_t>(0, kLabels.size() - 1)(rng)];
  item.shape = Shape::Can;
  item.diameter = 0.053;
  item.length = 0.100;
  item.variant = std::string("can/") + label.first;
  const double r = item.diameter / 2;
  const Color silver{0.78F, 0.79F, 0.82F};
  item.parts.push_back({render::make_cylinder(r * 0.93, 0.008), up(0.004), silver});         // 바닥 테
  item.parts.push_back({render::make_cylinder(r, 0.084), up(0.050), label.second});           // 몸통 라벨
  item.parts.push_back({render::make_frustum(r, r * 0.88, 0.008), up(0.096), silver});        // 어깨
  item.parts.push_back({render::make_cylinder(r * 0.86, 0.002), up(0.0995), silver});         // 뚜껑
}

void make_bottle(RecyclableItem& item, std::mt19937& rng) {
  static const std::array<std::pair<const char*, Color>, 3> kCaps{{
      {"white", {0.95F, 0.95F, 0.95F}}, {"blue", {0.15F, 0.35F, 0.85F}}, {"red", {0.85F, 0.15F, 0.15F}}}};
  static const std::array<Color, 3> kLabels{{{0.20F, 0.60F, 0.85F}, {0.95F, 0.80F, 0.15F}, {0.30F, 0.70F, 0.35F}}};
  const auto& cap = kCaps[std::uniform_int_distribution<std::size_t>(0, kCaps.size() - 1)(rng)];
  item.shape = Shape::Bottle;
  item.diameter = 0.056;
  item.length = 0.170;
  item.variant = std::string("bottle/") + cap.first;
  const double r = item.diameter / 2;
  const Color plastic{0.80F, 0.90F, 0.94F};  // 투명 페트 (반투명은 표현하지 않는다)
  item.parts.push_back({render::make_cylinder(r, 0.110), up(0.055), plastic});
  item.parts.push_back({render::make_cylinder(r * 1.01, 0.045),
                        up(0.060), kLabels[std::uniform_int_distribution<std::size_t>(0, 2)(rng)]});  // 라벨
  item.parts.push_back({render::make_frustum(r, 0.012, 0.035), up(0.1275), plastic});              // 어깨
  item.parts.push_back({render::make_cylinder(0.012, 0.012), up(0.151), plastic});                 // 목
  item.parts.push_back({render::make_cylinder(0.0145, 0.013), up(0.1635), cap.second});            // 뚜껑
}

void make_carton(RecyclableItem& item, std::mt19937& rng) {
  static const std::array<std::pair<const char*, Color>, 3> kBands{{
      {"milk", {0.20F, 0.45F, 0.85F}}, {"juice", {0.95F, 0.55F, 0.10F}}, {"plain", {0.85F, 0.80F, 0.70F}}}};
  const auto& band = kBands[std::uniform_int_distribution<std::size_t>(0, kBands.size() - 1)(rng)];
  item.shape = Shape::Carton;
  item.diameter = 0.055;  // 한 변
  item.length = 0.095;
  item.variant = std::string("carton/") + band.first;
  const Color white{0.94F, 0.94F, 0.92F};
  item.parts.push_back({render::make_box(0.055, 0.055, 0.075), up(0.0375), white});
  item.parts.push_back({render::make_box(0.0555, 0.0555, 0.030), up(0.040), band.second});  // 띠
  item.parts.push_back({render::make_box(0.055, 0.012, 0.020), up(0.085), white});          // 박공 지붕 (단순화)
}

void make_paper_ball(RecyclableItem& item, std::mt19937& rng) {
  static const std::array<std::pair<const char*, Color>, 3> kPapers{{
      {"white", {0.93F, 0.93F, 0.90F}}, {"news", {0.72F, 0.72F, 0.70F}}, {"kraft", {0.70F, 0.56F, 0.38F}}}};
  const auto& paper = kPapers[std::uniform_int_distribution<std::size_t>(0, kPapers.size() - 1)(rng)];
  item.shape = Shape::PaperBall;
  item.diameter = 0.060;
  item.length = 0.055;
  item.variant = std::string("paper_ball/") + paper.first;
  item.parts.push_back({render::make_lumpy_sphere(0.029, 0.28, static_cast<std::uint32_t>(rng())), up(0.028),
                        paper.second});
}

// 모호 장면용: 플라스틱 뚜껑이 달린 종이컵 (복합 재질 → 분류 불확실).
void make_cup(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.shape = Shape::Cup;
  item.diameter = 0.070;
  item.length = 0.090;
  item.variant = "cup/paper_with_lid";
  item.parts.push_back({render::make_frustum(0.024, 0.033, 0.080), up(0.040), {0.95F, 0.95F, 0.93F}});
  item.parts.push_back({render::make_frustum(0.030, 0.032, 0.030), up(0.045), {0.55F, 0.38F, 0.22F}});  // 슬리브
  item.parts.push_back({render::make_frustum(0.035, 0.028, 0.010), up(0.085), {0.85F, 0.88F, 0.90F}});  // 뚜껑
}

void shape_item(RecyclableItem& item, std::mt19937& rng) {
  switch (item.category) {
    case Category::Can: make_can(item, rng); break;
    case Category::Pet: make_bottle(item, rng); break;
    case Category::Paper:
      std::bernoulli_distribution(0.5)(rng) ? make_carton(item, rng) : make_paper_ball(item, rng);
      break;
    case Category::Unknown: make_cup(item, rng); break;
  }
}

void add_box(std::vector<render::Instance>& out, double cx, double cy, double cz, double sx, double sy, double sz,
             Color color) {
  out.push_back({render::make_box(sx, sy, sz), Pose::translation(cx, cy, cz), color});
}

}  // namespace

const char* category_name(Category category) {
  switch (category) {
    case Category::Can: return "can";
    case Category::Pet: return "pet";
    case Category::Paper: return "paper";
    case Category::Unknown: return "unknown";
  }
  return "unknown";
}

std::optional<Category> parse_category(const std::string& text) {
  for (const auto c : {Category::Can, Category::Pet, Category::Paper, Category::Unknown}) {
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
  // 떨어진다 (자유낙하). 바닥(수거함 안도 바닥 높이)에 닿으면 낙하 여부를 판정한다.
  fall_speed += kGravity * dt;
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

RecyclingWorld::RecyclingWorld(RecyclingLayout layout) : layout_(std::move(layout)) { build_static(); }

void RecyclingWorld::build_static() {
  const double t = 0.05;  // 벽 두께
  const double rx = layout_.room_x, ry = layout_.room_y, h = layout_.wall_height;
  // 방 벽 (충돌·라이다)
  add_wall({rx / 2, -t / 2, rx / 2 + t, t / 2, h});
  add_wall({rx / 2, ry + t / 2, rx / 2 + t, t / 2, h});
  add_wall({-t / 2, ry / 2, t / 2, ry / 2, h});
  add_wall({rx + t / 2, ry / 2, t / 2, ry / 2, h});
  const Color wall_color{0.90F, 0.88F, 0.84F};
  add_box(static_instances_, rx / 2, -t / 2, h / 2, rx + 2 * t, t, h, wall_color);
  add_box(static_instances_, rx / 2, ry + t / 2, h / 2, rx + 2 * t, t, h, wall_color);
  add_box(static_instances_, -t / 2, ry / 2, h / 2, t, ry, h, wall_color);
  add_box(static_instances_, rx + t / 2, ry / 2, h / 2, t, ry, h, wall_color);
  // 바닥 타일 (0.5 m 바둑판: 영상에서 거리감)
  const double tile = 0.5;
  for (int i = 0; i * tile < rx; ++i) {
    for (int j = 0; j * tile < ry; ++j) {
      const double sx = std::min(tile, rx - i * tile), sy = std::min(tile, ry - j * tile);
      const float shade = (i + j) % 2 == 0 ? 0.70F : 0.64F;
      add_box(static_instances_, i * tile + sx / 2, j * tile + sy / 2, -0.005, sx, sy, 0.01, {shade, shade, shade * 1.02F});
    }
  }
  // 작업 영역 매트
  const auto& w = layout_.work_area;
  add_box(static_instances_, w.center_x(), w.center_y(), 0.0005, w.max_x - w.min_x, w.max_y - w.min_y, 0.001,
          {0.82F, 0.76F, 0.60F});
  // 수거함: 벽 4개 (충돌·라이다) + 바닥판
  for (const auto& bin : layout_.bins) {
    const double hx = bin.inner_x / 2, hy = bin.inner_y / 2, wt = bin.wall;
    const std::array<srf::simulator::Wall, 4> walls{{
        {bin.x, bin.y - hy - wt / 2, hx + wt, wt / 2, bin.height},
        {bin.x, bin.y + hy + wt / 2, hx + wt, wt / 2, bin.height},
        {bin.x - hx - wt / 2, bin.y, wt / 2, hy, bin.height},
        {bin.x + hx + wt / 2, bin.y, wt / 2, hy, bin.height},
    }};
    for (const auto& wall : walls) {
      add_wall(wall);
      add_box(static_instances_, wall.center_x, wall.center_y, wall.height / 2, wall.half_x * 2, wall.half_y * 2,
              wall.height, bin.color);
    }
    const Color floor{bin.color[0] * 0.6F, bin.color[1] * 0.6F, bin.color[2] * 0.6F};
    add_box(static_instances_, bin.x, bin.y, 0.001, bin.inner_x, bin.inner_y, 0.002, floor);
  }
}

void RecyclingWorld::randomize(const SceneConfig& config) {
  scene_ = config;
  auto& all = objects();
  all.erase(std::remove_if(all.begin(), all.end(),
                           [](const auto& object) { return object && object->kind() == "recyclable"; }),
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
  std::shuffle(list.begin(), list.end(), rng);

  // 2) 작업 영역 안 임의 위치·방향 (겹치지 않게)
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const auto& area = layout_.work_area;
  std::vector<std::pair<double, double>> placed;
  std::vector<double> radii;
  int next_id = 1;
  for (const Category category : list) {
    auto item = std::make_unique<RecyclableItem>();
    item->id = next_id++;
    item->category = category;
    shape_item(*item, rng);
    const bool can_lie = item->shape == Shape::Can || item->shape == Shape::Bottle;
    item->lying = can_lie && unit(rng) < config.lying_ratio;
    item->radius = item->lying ? item->length / 2 : item->diameter / 2;
    item->height = item->lying ? item->diameter : item->length;
    item->mass = 0.05;
    item->friction = 0.5;
    item->movable = true;
    item->graspable = true;
    item->sides = item->shape == Shape::Carton ? 4 : 24;
    item->color = {item->parts.size() > 1 ? item->parts[1].color[0] : item->parts[0].color[0],
                   item->parts.size() > 1 ? item->parts[1].color[1] : item->parts[0].color[1],
                   item->parts.size() > 1 ? item->parts[1].color[2] : item->parts[0].color[2]};
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
      std::fprintf(stderr, "[recycling] 물품 %d를 놓을 자리가 없다 (작업 영역이 좁다)\n", item->id);
      continue;
    }
    placed.emplace_back(pose.x, pose.y);
    radii.push_back(item->radius);
    add(std::move(item), pose);
  }
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
    std::string state = item->held ? "held" : item->on_robot ? "on_robot" : bin ? "bin"
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
  for (const auto* item : items()) {
    const Pose pose = item->world_pose();
    for (const auto& part : item->parts) {
      out.push_back({part.mesh, pose * part.offset, part.color});
    }
  }
}

}  // namespace srf_recycling_ws::sim
