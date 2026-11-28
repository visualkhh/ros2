#pragma once

// 재활용 선별장 월드 (24 × 14 m 실내).
//   구역: 로봇 분리수거 셀(미션: 작업 매트 + 종류별 수거 바구니) · 보행자 통로 · 지게차 통로 · 하역장(셔터·마대·팔레트)
//         · 선별 컨베이어 라인(움직이는 벨트·작업자) · 압축 베일 적재장 · 압축기 · 대형 수거함 구역 · 사무실
//   움직이는 것: 걷는 작업자, 지게차, 컨베이어 위 물품 — 라이다·카메라에 보인다 (장애물 시험).
//   미션 물품(캔·페트·종이, 불확실·비대상)은 작업 매트 안 임의 위치·방향 (시드 재현).
//   진실 판정: 물품마다 어디 있는지(작업 영역·들림·어느 바구니·바닥), 낙하 기록.
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "render/scene_renderer.hpp"
#include "srf/simulator/world.hpp"

namespace srf_recycling_ws::sim {

// 미션 분류: 캔·페트·종이가 대상. Unknown = 분류가 불확실한 복합 재질, Other = 대상이 아닌 것(유리·비닐·스티로폼).
enum class Category { Can, Pet, Paper, Unknown, Other };
enum class Scene { Normal, Multi, Ambiguous, Absent };

const char* category_name(Category category);  // "can" | "pet" | "paper" | "unknown" | "other"
std::optional<Category> parse_category(const std::string& text);
const char* scene_name(Scene scene);
std::optional<Scene> parse_scene(const std::string& text);

struct Rect {
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  [[nodiscard]] bool contains(double x, double y, double margin = 0.0) const {
    return x >= min_x + margin && x <= max_x - margin && y >= min_y + margin && y <= max_y - margin;
  }
  [[nodiscard]] double center_x() const { return (min_x + max_x) / 2; }
  [[nodiscard]] double center_y() const { return (min_y + max_y) / 2; }
};

struct BinSpec {
  Category category = Category::Can;
  double x = 0.0;  // 안쪽 중심
  double y = 0.0;
  double inner_x = 0.24;
  double inner_y = 0.24;
  double height = 0.10;  // 낮은 바구니 (팔이 넘어가 놓을 수 있게)
  double wall = 0.012;
  std::string texture;   // 바깥 면 (라벨)
  render::Color color{0.8F, 0.2F, 0.2F};
  [[nodiscard]] Rect inner() const { return {x - inner_x / 2, y - inner_y / 2, x + inner_x / 2, y + inner_y / 2}; }
};

// 미션 배치. 공통 맵 추첨 뒤 실제 배치로 바꾼다.
struct RecyclingLayout {
  double hall_x = 24.0;
  double hall_y = 14.0;
  double wall_height = 6.0;
  Rect robot_cell{1.0, 1.0, 9.0, 6.8};
  Rect work_area{3.6, 3.5, 4.4, 4.3};  // 고무 매트
  srf::msgs::Pose2D robot_start{2.2, 3.9, 0.0};
  std::vector<BinSpec> bins{
      {Category::Can, 7.6, 2.9, 0.24, 0.24, 0.10, 0.012, "crate_can", {0.80F, 0.18F, 0.16F}},
      {Category::Pet, 7.6, 3.9, 0.24, 0.24, 0.10, 0.012, "crate_pet", {0.16F, 0.40F, 0.80F}},
      {Category::Paper, 7.6, 4.9, 0.24, 0.24, 0.10, 0.012, "crate_paper", {0.18F, 0.58F, 0.28F}},
  };
  // 셀 담당 작업자 동선
  srf::msgs::Pose2D operator_idle{0.7, 5.5, 0.0};  // 대기 (셀 북서쪽 구석, 로봇 동선 밖)
  srf::msgs::Pose2D supply_pickup{2.4, 10.3, 0.0};   // 하역장 마대 앞 (마대 벽에서 몸 반지름 이상 떨어져야 한다)
  srf::msgs::Pose2D mat_drop{4.0, 4.75, 0.0};      // 매트 북쪽 가장자리 (여기서 붓는다)
  // 종류별 대형 수거함 앞 (바구니를 비우는 곳)
  std::map<Category, srf::msgs::Pose2D> wheelie_drop{
      {Category::Pet, {11.6, 2.05, 0.0}}, {Category::Paper, {12.95, 2.05, 0.0}}, {Category::Can, {14.3, 2.05, 0.0}}};
};

struct SceneConfig {
  Scene scene = Scene::Normal;
  std::optional<Category> target;  // 지시 대상 종류 (없으면 "전부 분리수거")
  int items = 4;                   // 미션 물품 수 (비대상 제외)
  std::uint32_t seed = 42;
  double lying_ratio = 0.5;        // 원기둥 물품이 누워 있을 확률
  int distractors = 0;             // 비대상 물품(유리병·비닐·스티로폼) 수
  int people = 4;                  // 걷는 작업자 (0~4). 컨베이어 작업자는 항상 있다
  int forklifts = 1;               // 0~1
  // 협동 작업 (셀 담당 작업자)
  bool auto_supply = true;         // 매트가 비면(물품이 있다가 다 치워지면) 하역장에서 새로 가져온다
  bool auto_empty = true;          // 바구니가 차면 비운다
  int supply_items = 4;            // 한 번에 가져오는 물품 수 (+ distractors 비율대로 비대상)
  int bin_capacity = 5;            // 바구니가 "꽉 참"이 되는 개수
};

// 셀 담당 작업자가 경로의 지점에서 하는 일. 월드가 스텝 뒤(update)에 처리한다.
enum class CellAction { None, PickSupply, DumpSupply, CollectBin, DumpWheelie, Done };
enum class Carry { None, ToteFull, ToteEmpty, Crate };

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
  double pause = 0.0;
  CellAction action = CellAction::None;
  Category bin = Category::Can;
};

class Worker;  // agents.hpp

// 그려지는 월드 물체 (물품·작업자·지게차).
class VisualObject : public srf::simulator::WorldObject {
 public:
  virtual void add_render(std::vector<render::Instance>& out, double time) const = 0;
};

// 재활용품. 물리는 원(radius)으로 단순화하고, 보이는 모양은 여러 부품 메시(텍스처 재질)로 그린다.
class RecyclableItem final : public VisualObject {
 public:
  struct Part {
    std::shared_ptr<const render::Mesh> mesh;
    render::Pose offset;  // 물품 기준 (아래면 중심이 원점, 서 있는 자세)
    render::Material material;
  };

  int id = 0;
  Category category = Category::Can;
  bool lying = false;
  bool can_lie = false;    // 원기둥 물품만 눕는다
  double length = 0.1;     // 서 있을 때 높이 (누우면 바닥 길이)
  double diameter = 0.05;  // 굵기 (상자는 한 변)
  std::string variant;     // 예: "can/cola", "pet/water", "paper/milk", "other/glass_bottle_brown"
  std::vector<Part> parts;

  // 낙하 기록 (그리퍼에서 놓인 높이·위치)
  bool was_held = false;
  int releases = 0;
  double last_release_height = 0.0;
  bool dropped = false;  // 수거함 밖에서 3 cm 넘게 떨어짐
  double fall_speed = 0.0;

  [[nodiscard]] std::string kind() const override { return "recyclable"; }
  void tick(double time, double dt) override;
  void add_render(std::vector<render::Instance>& out, double time) const override;
  [[nodiscard]] render::Pose world_pose() const;
};

// 물품 모양을 만든다 (종류에 맞는 여러 변형 중 하나). items.cpp
void build_item(RecyclableItem& item, std::mt19937& rng);

class RecyclingWorld final : public srf::simulator::World {
 public:
  explicit RecyclingWorld(RecyclingLayout layout = {});

  // 미션 물품·작업자·지게차를 지우고 장면 설정대로 새로 놓는다 (시뮬레이션 스레드에서 부른다).
  void randomize(const SceneConfig& config);

  [[nodiscard]] const RecyclingLayout& layout() const { return layout_; }
  [[nodiscard]] const SceneConfig& scene() const { return scene_; }
  [[nodiscard]] std::vector<const RecyclableItem*> items() const;
  [[nodiscard]] const BinSpec* bin_containing(const RecyclableItem& item) const;
  [[nodiscard]] std::string truth_json(double time) const;
  // 렌더링용: 시설·움직이는 장비·물품·그림자 (로봇 제외).
  void add_render_instances(std::vector<render::Instance>& out) const;

  // ---- 협동 작업 ----
  // 요청: 하역장에서 물품 공급 / 바구니 비우기. 이미 대기 중이거나 하고 있으면 false.
  bool request_supply();
  bool request_empty(Category bin);
  void set_auto(bool supply, bool empty) {
    scene_.auto_supply = supply;
    scene_.auto_empty = empty;
  }
  // 스텝마다 부른다 (시뮬레이션 스레드, World::step 뒤): 작업자 동작 처리, 자동 요청, 다음 작업 배정.
  void update();
  struct BinStatus {
    Category category = Category::Can;
    int count = 0;
    int capacity = 0;
    bool full = false;
    bool requested = false;  // 비우기 대기·진행 중
  };
  [[nodiscard]] std::vector<BinStatus> bin_status() const;
  [[nodiscard]] std::string cell_state() const { return cell_state_; }  // 셀 담당 작업자가 하는 일
  [[nodiscard]] int supplies() const { return supplies_; }
  [[nodiscard]] const std::map<Category, int>& collected() const { return collected_; }
  // 셀 담당 작업자가 지점에 도착했을 때 부른다 (step 도중이라 물체 목록은 update에서 바꾼다).
  void queue_action(CellAction action, Category bin) { pending_.emplace_back(action, bin); }

  // 시설 만들기에서 쓴다 (facility.cpp)
  void add_static(const render::Instance& instance) { static_instances_.push_back(instance); }
  void add_box(double cx, double cy, double cz, double sx, double sy, double sz, const render::Material& material,
               bool collide = false);

 private:
  void build_facility();                                                // facility.cpp
  void add_conveyor_render(std::vector<render::Instance>& out) const;  // facility.cpp
  void spawn_agents(const SceneConfig& config);                         // agents.cpp
  void place_items(const std::vector<Category>& list, std::mt19937& rng);
  [[nodiscard]] int count_in(const BinSpec& bin) const;
  [[nodiscard]] bool mat_has_items() const;
  void start_next_job();

  struct Job {
    bool supply = true;
    Category bin = Category::Can;
  };
  std::deque<Job> jobs_;
  std::optional<Job> current_job_;
  std::vector<std::pair<CellAction, Category>> pending_;
  Worker* operator_ = nullptr;
  std::string cell_state_ = "대기";
  bool mat_had_items_ = false;
  int supplies_ = 0;
  int next_item_id_ = 1;
  std::map<Category, int> collected_;
  std::mt19937 supply_rng_;

  RecyclingLayout layout_;
  SceneConfig scene_;
  std::vector<render::Instance> static_instances_;
  struct ConveyorItem {
    std::vector<RecyclableItem::Part> parts;
    double phase = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };
  std::vector<ConveyorItem> conveyor_items_;
};

}  // namespace srf_recycling_ws::sim
