#include "world/recycling_world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <sstream>

#include "render/textures.hpp"
#include "world/agents.hpp"

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
    out.push_back({part.mesh, pose_world * part.offset, part.material, id});  // id: 학습 데이터 물체 번호
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
  next_item_id_ = 1;
  place_items(list, rng);
  jobs_.clear();
  current_job_.reset();
  pending_.clear();
  cell_state_ = "대기";
  mat_had_items_ = mat_has_items();
  supplies_ = 0;
  collected_.clear();
  supply_rng_.seed(config.seed * 7919U + 13U);
  spawn_agents(config);
}

void RecyclingWorld::place_items(const std::vector<Category>& list, std::mt19937& rng) {
  // 작업 매트 안 임의 위치·방향 (이미 있는 물품·서로 겹치지 않게)
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const auto& area = layout_.work_area;
  std::vector<std::pair<double, double>> placed;
  std::vector<double> radii;
  for (const auto* item : items()) {
    if (area.contains(item->pose.x, item->pose.y) && !item->held) {
      placed.emplace_back(item->pose.x, item->pose.y);
      radii.push_back(item->radius);
    }
  }
  for (const Category category : list) {
    auto item = std::make_unique<RecyclableItem>();
    item->id = next_item_id_++;
    item->category = category;
    build_item(*item, rng);
    item->lying = item->can_lie && unit(rng) < scene_.lying_ratio;
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
}

// ---- 협동 작업 ----

int RecyclingWorld::count_in(const BinSpec& bin) const {
  int n = 0;
  for (const auto* item : items()) {
    n += bin_containing(*item) == &bin ? 1 : 0;
  }
  return n;
}

bool RecyclingWorld::mat_has_items() const {
  for (const auto* item : items()) {
    if (!item->held && layout_.work_area.contains(item->pose.x, item->pose.y)) {
      return true;
    }
  }
  return false;
}

bool RecyclingWorld::request_supply() {
  if ((current_job_ && current_job_->supply) ||
      std::any_of(jobs_.begin(), jobs_.end(), [](const Job& job) { return job.supply; })) {
    return false;
  }
  jobs_.push_back({true, Category::Can});
  return true;
}

bool RecyclingWorld::request_empty(Category bin) {
  const auto same = [bin](const Job& job) { return !job.supply && job.bin == bin; };
  if ((current_job_ && same(*current_job_)) || std::any_of(jobs_.begin(), jobs_.end(), same)) {
    return false;
  }
  if (std::none_of(layout_.bins.begin(), layout_.bins.end(), [bin](const BinSpec& b) { return b.category == bin; })) {
    return false;
  }
  jobs_.push_back({false, bin});
  return true;
}

std::vector<RecyclingWorld::BinStatus> RecyclingWorld::bin_status() const {
  std::vector<BinStatus> out;
  for (const auto& bin : layout_.bins) {
    BinStatus status;
    status.category = bin.category;
    status.count = count_in(bin);
    status.capacity = scene_.bin_capacity;
    status.full = status.count >= scene_.bin_capacity;
    const auto same = [&](const Job& job) { return !job.supply && job.bin == bin.category; };
    status.requested = (current_job_ && same(*current_job_)) || std::any_of(jobs_.begin(), jobs_.end(), same);
    out.push_back(status);
  }
  return out;
}

void RecyclingWorld::start_next_job() {
  if (operator_ == nullptr || current_job_ || jobs_.empty() || !operator_->idle()) {
    return;
  }
  current_job_ = jobs_.front();
  jobs_.pop_front();
  const auto& L = layout_;
  const auto& idle = L.operator_idle;
  std::vector<Waypoint> route;
  if (current_job_->supply) {
    // 하역장 마대에서 토트에 담아 → 매트에 붓고 → 돌아온다
    operator_->carry_color = {0.2F, 0.45F, 0.85F};
    route = {{2.0, 6.5, 0.0}, {2.2, 7.7, 0.0},
             {L.supply_pickup.x, L.supply_pickup.y, 2.5, CellAction::PickSupply},
             {2.2, 7.7, 0.0}, {L.mat_drop.x, 6.5, 0.0},
             {L.mat_drop.x, L.mat_drop.y, 2.5, CellAction::DumpSupply},
             {L.mat_drop.x, 6.5, 0.0}, {2.0, 6.5, 0.0}, {idle.x, idle.y, 0.5, CellAction::Done}};
    cell_state_ = "공급: 하역장으로";
  } else {
    // 바구니 앞에서 비워 → 종류별 대형 수거함에 붓고 → 돌아온다
    const BinSpec* bin = nullptr;
    for (const auto& b : L.bins) {
      if (b.category == current_job_->bin) bin = &b;
    }
    const auto drop = L.wheelie_drop.count(bin->category) ? L.wheelie_drop.at(bin->category) : srf::msgs::Pose2D{15.65, 2.05, 0};
    operator_->carry_color = bin->color;
    const double front_x = bin->x - bin->inner_x / 2 - 0.45;
    route = {{2.0, 6.5, 0.0}, {front_x - 0.4, 6.4, 0.0},
             {front_x, bin->y, 2.5, CellAction::CollectBin, bin->category},
             {front_x, 1.6, 0.0}, {9.6, 1.6, 0.0},
             {drop.x, drop.y, 2.5, CellAction::DumpWheelie, bin->category},
             {9.6, 1.6, 0.0}, {front_x, 1.6, 0.0}, {front_x - 0.4, 6.4, 0.0}, {2.0, 6.5, 0.0},
             {idle.x, idle.y, 0.5, CellAction::Done}};
    cell_state_ = std::string("비우기(") + category_name(bin->category) + "): 바구니로";
  }
  operator_->route = std::move(route);
  operator_->next = 0;
}

void RecyclingWorld::update() {
  // 1) 작업자가 지점에서 한 일 (step 밖이라 물체를 더하고 지워도 안전하다)
  for (const auto& [action, bin] : pending_) {
    switch (action) {
      case CellAction::PickSupply: cell_state_ = "공급: 매트로 운반"; break;
      case CellAction::DumpSupply: {
        std::vector<Category> list;
        const std::array<Category, 3> kinds{Category::Can, Category::Pet, Category::Paper};
        for (int i = 0; i < scene_.supply_items; ++i) {
          list.push_back(kinds[std::uniform_int_distribution<std::size_t>(0, 2)(supply_rng_)]);
        }
        for (int i = 0; i < scene_.distractors; ++i) {
          list.push_back(Category::Other);
        }
        place_items(list, supply_rng_);
        ++supplies_;
        mat_had_items_ = true;
        cell_state_ = "공급: 돌아가는 중";
        break;
      }
      case CellAction::CollectBin: {
        const BinSpec* spec = nullptr;
        for (const auto& b : layout_.bins) {
          if (b.category == bin) spec = &b;
        }
        auto& all = objects();
        all.erase(std::remove_if(all.begin(), all.end(),
                                 [&](const auto& object) {
                                   const auto* item = dynamic_cast<const RecyclableItem*>(object.get());
                                   if (item == nullptr || spec == nullptr || bin_containing(*item) != spec) {
                                     return false;
                                   }
                                   ++collected_[item->category];
                                   return true;
                                 }),
                  all.end());
        cell_state_ = std::string("비우기(") + category_name(bin) + "): 대형 수거함으로";
        break;
      }
      case CellAction::DumpWheelie: cell_state_ = std::string("비우기(") + category_name(bin) + "): 돌아가는 중"; break;
      case CellAction::Done:
        current_job_.reset();
        cell_state_ = "대기";
        break;
      case CellAction::None: break;
    }
  }
  pending_.clear();

  // 2) 자동 요청: 바구니가 차면 비우기, 매트가 (물품이 있다가) 비면 공급
  if (scene_.auto_empty) {
    for (const auto& status : bin_status()) {
      if (status.full && !status.requested) {
        request_empty(status.category);
      }
    }
  }
  const bool has = mat_has_items();
  if (scene_.auto_supply && mat_had_items_ && !has) {
    request_supply();  // 있다가 비는 순간 한 번 (처음부터 빈 '부재' 장면은 공급하지 않는다)
  }
  mat_had_items_ = has;

  // 3) 다음 작업
  start_next_job();
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
      << ",\"dropped\":" << dropped << ",\"held\":" << held << ",\"remaining\":" << remaining << "}";
  // 협동 작업 상태: 바구니 채움, 셀 담당 작업자, 대기 작업, 수거 누계
  out << ",\"cell\":{\"operator\":\"" << cell_state_ << "\",\"queue\":" << jobs_.size() + (current_job_ ? 1 : 0)
      << ",\"supplies\":" << supplies_ << ",\"bins\":[";
  bool first_bin = true;
  for (const auto& status : bin_status()) {
    out << (first_bin ? "" : ",") << "{\"bin\":\"" << category_name(status.category) << "\",\"count\":" << status.count
        << ",\"capacity\":" << status.capacity << ",\"full\":" << (status.full ? "true" : "false")
        << ",\"requested\":" << (status.requested ? "true" : "false") << "}";
    first_bin = false;
  }
  out << "],\"collected\":{";
  bool first_kind = true;
  for (const auto& [category, n] : collected_) {
    out << (first_kind ? "" : ",") << "\"" << category_name(category) << "\":" << n;
    first_kind = false;
  }
  out << "}}}";
  return out.str();
}

srf::msgs::OccupancyGrid RecyclingWorld::truth_map(double resolution) const {
  srf::msgs::OccupancyGrid grid;
  grid.header = srf::msgs::Header{0.0, "map"};
  const double margin = 0.5;
  grid.info.resolution = resolution;
  grid.info.width = static_cast<std::uint32_t>(std::ceil((layout_.hall_x + 2 * margin) / resolution));
  grid.info.height = static_cast<std::uint32_t>(std::ceil((layout_.hall_y + 2 * margin) / resolution));
  grid.info.origin.position = {-margin, -margin, 0.0};
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, -1);
  for (std::uint32_t j = 0; j < grid.info.height; ++j) {
    for (std::uint32_t i = 0; i < grid.info.width; ++i) {
      const double x = -margin + (i + 0.5) * resolution, y = -margin + (j + 0.5) * resolution;
      bool occupied = false;
      for (const auto& w : walls()) {
        if (std::abs(x - w.center_x) <= w.half_x && std::abs(y - w.center_y) <= w.half_y && w.height > 0.05) {
          occupied = true;
          break;
        }
      }
      const bool inside = x > 0 && y > 0 && x < layout_.hall_x && y < layout_.hall_y;
      grid.data[static_cast<std::size_t>(j) * grid.info.width + i] = occupied ? 100 : inside ? 0 : -1;
    }
  }
  return grid;
}

srf::msgs::MapMarkers RecyclingWorld::default_markers() const {
  using srf::msgs::MapMarker;
  using srf::msgs::MapZone;
  srf::msgs::MapMarkers m;
  const auto marker = [&](const std::string& name, const std::string& kind, double x, double y, double z, double yaw,
                          srf::msgs::Vector3 scale, std::vector<srf::msgs::KeyValue> properties) {
    MapMarker item;
    item.name = name;
    item.kind = kind;
    item.pose.position = {x, y, z};
    item.pose.orientation = {0, 0, std::sin(yaw / 2), std::cos(yaw / 2)};
    item.scale = scale;
    item.properties = std::move(properties);
    m.markers.push_back(item);
  };
  const auto rect = [&](const std::string& name, const std::string& kind, const Rect& r,
                        std::vector<srf::msgs::KeyValue> properties) {
    MapZone zone;
    zone.name = name;
    zone.kind = kind;
    zone.polygon = {{r.min_x, r.min_y, 0}, {r.max_x, r.min_y, 0}, {r.max_x, r.max_y, 0}, {r.min_x, r.max_y, 0}};
    zone.properties = std::move(properties);
    m.zones.push_back(zone);
  };
  const auto& L = layout_;
  marker("robot_home", "home", L.robot_start.x, L.robot_start.y, 0, L.robot_start.theta, {}, {{"role", "start"}});
  const auto& w = L.work_area;
  marker("work_mat", "pick", w.center_x(), w.center_y(), 0.002, 0, {w.max_x - w.min_x, w.max_y - w.min_y, 0},
         {{"role", "supply_drop"}, {"surface", "rubber_mat"}});
  marker("work_mat_approach", "waypoint", w.min_x - 0.30, w.center_y(), 0, 0, {}, {{"for", "work_mat"}});
  for (const auto& bin : L.bins) {
    const std::string c = category_name(bin.category);
    marker("bin_" + c, "drop", bin.x, bin.y, bin.height, 0, {bin.inner_x, bin.inner_y, bin.height},
           {{"category", c}, {"capacity", std::to_string(scene_.bin_capacity)}});
    marker("bin_" + c + "_approach", "waypoint", bin.x - bin.inner_x / 2 - 0.30, bin.y, 0, 0, {}, {{"for", "bin_" + c}});
  }
  marker("operator_idle", "human", L.operator_idle.x, L.operator_idle.y, 0, 0, {}, {{"role", "cell_operator"}});
  marker("supply_pickup", "human", L.supply_pickup.x, L.supply_pickup.y, 0, 0, {}, {{"role", "receiving"}});
  rect("robot_cell", "info", L.robot_cell, {{"owner", "robot"}});
  rect("work_area_slow", "speed", {w.min_x - 0.5, w.min_y - 0.5, w.max_x + 0.5, w.max_y + 0.5}, {{"speed_limit", "0.12"}});
  rect("bins_slow", "speed", {L.bins.front().x - 0.7, L.bins.front().y - 0.5, L.bins.back().x + 0.3, L.bins.back().y + 0.5},
       {{"speed_limit", "0.10"}});
  rect("walkway", "keepout", {0.0, 7.2, L.hall_x, 8.2}, {{"reason", "보행자 통로"}});
  rect("forklift_lane", "keepout", {0.0, 8.5, L.hall_x, 10.0}, {{"reason", "지게차 통로"}});
  rect("operator_corner", "keepout", {L.operator_idle.x - 0.5, L.operator_idle.y - 0.5, L.operator_idle.x + 0.5,
                                      L.operator_idle.y + 0.5}, {{"reason", "작업자 대기"}});
  MapZone conveyor;  // 3D: 컨베이어 위 작업 공간
  conveyor.name = "conveyor_space";
  conveyor.kind = "info";
  conveyor.shape = "box";
  conveyor.pose.position = {13.0, 12.0, 1.2};
  conveyor.pose.orientation.w = 1.0;
  conveyor.size = {9.4, 0.8, 0.7};
  conveyor.properties = {{"line", "sorting"}};
  m.zones.push_back(conveyor);
  return m;
}

void RecyclingWorld::add_render_instances(std::vector<render::Instance>& out) const {
  out.insert(out.end(), static_instances_.begin(), static_instances_.end());
  add_conveyor_render(out);
  // 바구니 상태 램프 (칸막이): 초록 여유 · 주황 하나 남음 · 빨강 꽉 참 · 파랑 깜빡임 = 작업자가 비우러 옴
  static const auto lamp = render::make_cylinder(0.035, 0.03, 20);
  const auto status = bin_status();
  for (std::size_t i = 0; i < status.size() && i < layout_.bins.size(); ++i) {
    const auto& st = status[i];
    render::Color color = st.full ? render::Color{0.95F, 0.12F, 0.1F}
                          : st.count >= st.capacity - 1 ? render::Color{1.0F, 0.6F, 0.05F}
                                                        : render::Color{0.15F, 0.85F, 0.25F};
    if (st.requested && std::fmod(time(), 1.0) < 0.5) {
      color = {0.2F, 0.5F, 1.0F};
    }
    render::Material light = render::solid(color);
    light.emissive = 0.95F;
    out.push_back({lamp, Pose::translation(8.585, layout_.bins[i].y + 0.3, 0.55) * Pose::axis_angle({0, 1, 0}, kPi / 2), light});
  }
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
