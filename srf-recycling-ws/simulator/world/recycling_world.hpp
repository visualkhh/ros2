#pragma once

// 재활용품 분리수거 미션 월드.
//   - 사각 방(벽) + 작업 영역(바닥 매트) + 종류별 분리수거함(지정 위치, 벽으로 충돌·라이다에 보인다)
//   - 재활용품(캔·페트병·종이, 모호 장면용 '불확실' 물품)을 작업 영역 안 임의 위치·방향으로 놓는다
//   - 시드가 같으면 배치가 같다 (회차 재현). 장면: 정상 / 다수 후보 / 모호 / 부재
//   - 진실 판정: 물품마다 어디 있는지(작업 영역, 들림, 어느 수거함, 바닥), 낙하 기록
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "render/scene_renderer.hpp"
#include "srf/simulator/world.hpp"

namespace srf_recycling_ws::sim {

enum class Category { Can, Pet, Paper, Unknown };
enum class Shape { Can, Bottle, Carton, PaperBall, Cup };
enum class Scene { Normal, Multi, Ambiguous, Absent };

const char* category_name(Category category);  // "can" | "pet" | "paper" | "unknown"
std::optional<Category> parse_category(const std::string& text);
const char* scene_name(Scene scene);
std::optional<Scene> parse_scene(const std::string& text);

// 사각형 영역 (월드 좌표, m).
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
  double x = 0.0;           // 안쪽 중심
  double y = 0.0;
  double inner_x = 0.22;    // 안쪽 크기
  double inner_y = 0.22;
  double height = 0.10;     // 벽 높이 (팔이 넘어가 놓을 수 있게 낮게)
  double wall = 0.012;
  render::Color color{0.8F, 0.2F, 0.2F};
  [[nodiscard]] Rect inner() const { return {x - inner_x / 2, y - inner_y / 2, x + inner_x / 2, y + inner_y / 2}; }
};

// 공통 맵 추첨 전 임시 배치. 실제 맵이 정해지면 이 값을 바꾼다.
struct RecyclingLayout {
  double room_x = 3.0;  // 방 안쪽 크기 (0,0)~(room_x, room_y)
  double room_y = 2.0;
  double wall_height = 0.5;
  Rect work_area{1.25, 0.60, 2.05, 1.40};
  srf::msgs::Pose2D robot_start{0.45, 1.0, 0.0};
  std::vector<BinSpec> bins{
      {Category::Can, 2.80, 0.55, 0.22, 0.22, 0.10, 0.012, {0.85F, 0.18F, 0.15F}},    // 캔: 빨강
      {Category::Pet, 2.80, 1.00, 0.22, 0.22, 0.10, 0.012, {0.15F, 0.40F, 0.85F}},    // 페트: 파랑
      {Category::Paper, 2.80, 1.45, 0.22, 0.22, 0.10, 0.012, {0.20F, 0.62F, 0.25F}},  // 종이: 초록
  };
};

struct SceneConfig {
  Scene scene = Scene::Normal;
  std::optional<Category> target;  // 지시 대상 종류 (없으면 "전부 분리수거")
  int items = 4;
  std::uint32_t seed = 42;
  double lying_ratio = 0.5;        // 캔·병이 누워 있을 확률
};

// 재활용품. 물리는 원(radius)으로 단순화하고, 보이는 모양은 여러 부품 메시로 그린다.
class RecyclableItem final : public srf::simulator::WorldObject {
 public:
  struct Part {
    std::shared_ptr<const render::Mesh> mesh;
    render::Pose offset;  // 물품 기준 (아래면 중심이 원점, 서 있는 자세)
    render::Color color;
  };

  int id = 0;
  Category category = Category::Can;
  Shape shape = Shape::Can;
  bool lying = false;
  double length = 0.1;    // 서 있을 때 높이 (누우면 바닥 길이)
  double diameter = 0.05;
  std::string variant;    // 표시용 (예: "can/red", "carton/milk")
  std::vector<Part> parts;

  // 낙하 기록 (그리퍼에서 놓인 높이·위치)
  bool was_held = false;
  int releases = 0;
  double last_release_height = 0.0;
  bool dropped = false;   // 수거함 밖에서 3 cm 넘게 떨어짐
  double fall_speed = 0.0;

  [[nodiscard]] std::string kind() const override { return "recyclable"; }
  void tick(double time, double dt) override;
  // 월드 자세 (렌더링): 아래면 중심 기준, 누우면 옆으로 눕힌다.
  [[nodiscard]] render::Pose world_pose() const;
};

class RecyclingWorld final : public srf::simulator::World {
 public:
  explicit RecyclingWorld(RecyclingLayout layout = {});

  // 기존 물품을 지우고 장면 설정대로 새로 놓는다 (시뮬레이션 스레드에서 부른다).
  void randomize(const SceneConfig& config);

  [[nodiscard]] const RecyclingLayout& layout() const { return layout_; }
  [[nodiscard]] const SceneConfig& scene() const { return scene_; }
  [[nodiscard]] std::vector<const RecyclableItem*> items() const;
  // 물품이 들어 있는 수거함 (안쪽 + 그리퍼에서 분리 + 테두리 아래). 없으면 nullptr.
  [[nodiscard]] const BinSpec* bin_containing(const RecyclableItem& item) const;
  // 진실 판정 JSON (srf/recycling/truth 토픽).
  [[nodiscard]] std::string truth_json(double time) const;
  // 렌더링용: 방·매트·수거함·물품 (로봇 제외).
  void add_render_instances(std::vector<render::Instance>& out) const;

 private:
  void build_static();

  RecyclingLayout layout_;
  SceneConfig scene_;
  std::vector<render::Instance> static_instances_;
};

}  // namespace srf_recycling_ws::sim
