// 재활용 월드: 시드 재현, 장면 규칙, 배치, 판정(수거함·낙하), 카메라 렌더.
#include <cmath>
#include <string>

#include "render/scene_renderer.hpp"
#include "world/recycling_world.hpp"

#include "test_framework.hpp"

using namespace srf_recycling_ws::sim;

namespace {

int count_of(const RecyclingWorld& world, Category category) {
  int n = 0;
  for (const auto* item : world.items()) {
    n += item->category == category ? 1 : 0;
  }
  return n;
}

RecyclableItem* mutable_item(RecyclingWorld& world, int index) {
  int seen = 0;
  for (auto& object : world.objects()) {
    if (auto* item = dynamic_cast<RecyclableItem*>(object.get())) {
      if (seen++ == index) {
        return item;
      }
    }
  }
  return nullptr;
}

}  // namespace

SRF_TEST(recycling_same_seed_same_layout) {
  RecyclingWorld a, b, c;
  a.randomize({Scene::Normal, std::nullopt, 5, 7});
  b.randomize({Scene::Normal, std::nullopt, 5, 7});
  c.randomize({Scene::Normal, std::nullopt, 5, 8});
  CHECK_EQ(a.truth_json(0), b.truth_json(0));
  CHECK(a.truth_json(0) != c.truth_json(0));
}

SRF_TEST(recycling_scene_rules) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    RecyclingWorld world;
    world.randomize({Scene::Normal, Category::Can, 4, seed});
    CHECK_EQ(1, count_of(world, Category::Can));  // 정상: 지시 대상이 정확히 하나
    world.randomize({Scene::Normal, std::nullopt, 4, seed});
    CHECK(count_of(world, Category::Can) >= 1 && count_of(world, Category::Pet) >= 1 &&
          count_of(world, Category::Paper) >= 1);
    world.randomize({Scene::Multi, Category::Pet, 4, seed});
    CHECK(count_of(world, Category::Pet) >= 2);  // 다수 후보
    world.randomize({Scene::Ambiguous, std::nullopt, 4, seed});
    CHECK_EQ(1, count_of(world, Category::Unknown));  // 분류 불확실 물품
    world.randomize({Scene::Absent, Category::Paper, 4, seed});
    CHECK_EQ(0, count_of(world, Category::Paper));  // 부재
    CHECK_EQ(4u, world.items().size());
    world.randomize({Scene::Absent, std::nullopt, 4, seed});
    CHECK_EQ(0u, world.items().size());  // 대상 지정 없으면 빈 작업 영역
  }
}

SRF_TEST(recycling_items_inside_work_area_without_overlap) {
  for (std::uint32_t seed = 1; seed <= 30; ++seed) {
    RecyclingWorld world;
    world.randomize({Scene::Normal, std::nullopt, 6, seed});
    const auto items = world.items();
    CHECK_EQ(6u, items.size());
    for (std::size_t i = 0; i < items.size(); ++i) {
      CHECK(world.layout().work_area.contains(items[i]->pose.x, items[i]->pose.y, items[i]->radius));
      for (std::size_t j = i + 1; j < items.size(); ++j) {
        CHECK(std::hypot(items[i]->pose.x - items[j]->pose.x, items[i]->pose.y - items[j]->pose.y) >
              items[i]->radius + items[j]->radius);
      }
    }
  }
}

SRF_TEST(recycling_judges_bins_and_drops) {
  RecyclingWorld world;
  world.randomize({Scene::Normal, Category::Can, 3, 3});
  RecyclableItem* can = nullptr;
  RecyclableItem* other = nullptr;
  for (int i = 0; i < 3; ++i) {
    auto* item = mutable_item(world, i);
    (item->category == Category::Can ? can : other) = item;
  }
  CHECK(can != nullptr && other != nullptr);
  const BinSpec& can_bin = world.layout().bins[0];
  CHECK(can_bin.category == Category::Can);

  // 수거함 위 8 cm에서 놓으면 떨어져 수거함 안 → 정답, 낙하 아님
  can->held = true;
  world.step(0.01);
  can->pose.x = can_bin.x;
  can->pose.y = can_bin.y;
  can->z = 0.08;
  can->held = false;
  for (int i = 0; i < 100; ++i) world.step(0.01);
  CHECK_EQ(0.0, can->z);
  CHECK(world.bin_containing(*can) == &can_bin);
  CHECK(!can->dropped);
  CHECK_EQ(1, can->releases);
  CHECK(world.truth_json(0).find("\"correct\":1,\"wrong\":0") != std::string::npos);

  // 다른 종류를 캔 수거함에 → 오분류
  other->pose.x = can_bin.x + 0.05;
  other->pose.y = can_bin.y;
  world.step(0.01);
  CHECK(world.truth_json(0).find("\"wrong\":1") != std::string::npos);

  // 그리퍼에서 수거함 밖 20 cm 높이로 떨어뜨림 → 낙하
  other->held = true;
  world.step(0.01);
  other->pose.x = 1.0;
  other->pose.y = 1.0;
  other->z = 0.20;
  other->held = false;
  for (int i = 0; i < 100; ++i) world.step(0.01);
  CHECK(other->dropped);
  CHECK(world.bin_containing(*other) == nullptr);
}

SRF_TEST(recycling_camera_sees_items_and_bins) {
  RecyclingWorld world;
  world.randomize({Scene::Normal, std::nullopt, 4, 7});
  render::RobotMesh robot(SRF_RECYCLING_WS_URDF_PATH);
  const srf::msgs::Pose2D pose{world.layout().work_area.min_x - 0.2, world.layout().work_area.center_y(), 0.0};
  const std::map<std::string, double> joints{{"joint2", -1.0}, {"joint3", 0.3}, {"joint4", 0.7}};
  std::vector<render::Instance> instances;
  world.add_render_instances(instances);
  robot.add_instances(pose, joints, instances);
  render::Camera camera;
  camera.width = 160;
  camera.height = 120;
  camera.set_fov(69.0 * 3.14159265358979 / 180.0);
  camera.world_from_optical = robot.link_pose("camera_color_optical_frame", pose, joints);
  // 광학축 +z는 차체 +x를 본다
  CHECK(camera.world_from_optical.r[0][2] > 0.99);
  render::Frame frame;
  render::render(camera, instances, {}, frame);
  int near_pixels = 0;
  for (const auto mm : frame.depth_mm) {
    near_pixels += (mm > 0 && mm < 1000) ? 1 : 0;  // 1 m 안 (작업 영역 물품·매트)
  }
  CHECK(near_pixels > 160 * 120 / 10);
  CHECK(robot.triangle_count() > 100000);  // URDF 메시를 다 읽었다
}

SRF_TEST(recycling_distractors_and_agents) {
  RecyclingWorld world;
  SceneConfig config;
  config.items = 3;
  config.distractors = 2;
  config.seed = 4;
  world.randomize(config);
  CHECK_EQ(2, count_of(world, Category::Other));
  CHECK_EQ(5u, world.items().size());

  // 걷는 작업자 4 + 컨베이어 작업자 3 + 지게차 1
  int workers = 0, forklifts = 0;
  for (const auto& object : world.objects()) {
    workers += object->kind() == "worker" ? 1 : 0;
    forklifts += object->kind() == "forklift" ? 1 : 0;
  }
  CHECK_EQ(7, workers);
  CHECK_EQ(1, forklifts);

  // 움직인다 (10초)
  std::vector<std::pair<double, double>> before;
  for (const auto& object : world.objects()) before.emplace_back(object->pose.x, object->pose.y);
  for (int i = 0; i < 1000; ++i) world.step(0.01);
  int moved = 0;
  for (std::size_t i = 0; i < world.objects().size(); ++i) {
    const auto& o = world.objects()[i];
    if ((o->kind() == "worker" || o->kind() == "forklift") &&
        std::hypot(o->pose.x - before[i].first, o->pose.y - before[i].second) > 1.0) {
      ++moved;
    }
  }
  CHECK(moved >= 4);  // 걷는 작업자들과 지게차

  config.people = 0;
  config.forklifts = 0;
  world.randomize(config);
  workers = 0;
  for (const auto& object : world.objects()) workers += object->kind() == "worker" ? 1 : 0;
  CHECK_EQ(3, workers);  // 컨베이어 작업자만
}

SRF_TEST(recycling_worker_waits_for_robot) {
  // 통로를 걷는 작업자 앞에 로봇(같은 kind의 물체)을 세우면 멈춘다
  class FakeRobot final : public srf::simulator::WorldObject {
   public:
    [[nodiscard]] std::string kind() const override { return "waffle_pi_arm"; }
  };
  RecyclingWorld world;
  SceneConfig config;
  config.people = 1;  // 보행자 통로 왕복 (0.8,7.7) → (23,7.7)
  config.forklifts = 0;
  world.randomize(config);
  srf::simulator::WorldObject* walker = nullptr;
  for (const auto& object : world.objects()) {
    if (object->kind() == "worker" && object->pose.y > 7.5 && object->pose.x < 1.0) walker = object.get();
  }
  CHECK(walker != nullptr);
  auto robot = std::make_unique<FakeRobot>();
  robot->radius = 0.2;
  world.add(std::move(robot), srf::msgs::Pose2D{3.0, 7.7, 0.0});
  for (int i = 0; i < 800; ++i) world.step(0.01);
  CHECK(walker->pose.x < 3.0 - 0.2 - 0.25);  // 로봇 앞에서 기다린다
  CHECK(walker->pose.x > 1.5);               // 출발은 했다
}
