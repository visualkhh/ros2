// 재활용 선별장 시설 (정적): 바닥·벽·구역·설비·표지판 + 컨베이어(움직이는 벨트와 물품).
// 좌표: 홀 (0,0)~(24,14) m. 남쪽 벽 y=0, 북쪽 y=14, 서쪽 x=0, 동쪽 x=24.
#include <cmath>
#include <random>

#include "render/textures.hpp"
#include "world/recycling_world.hpp"

namespace srf_recycling_ws::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kConveyorStart = 8.3;   // 컨베이어 x 시작
constexpr double kConveyorLength = 9.4;  // 물품이 흐르는 길이
constexpr double kConveyorY = 12.0;
constexpr double kBeltTop = 0.86;
constexpr double kBeltSpeed = 0.35;      // m/s

using render::Color;
using render::Material;
using render::Pose;

Material tex(const std::string& name, float su = 1, float sv = 1, Color tint = {1, 1, 1}, float specular = 0.05F,
             float shininess = 24.0F) {
  Material m = render::solid(tint, specular, shininess);
  m.texture = textures::get(name);
  m.uv_scale = {su, sv};
  return m;
}

// 벽에 붙이는 판: 법선이 yaw 방향 (글씨가 바로 읽히게).
Pose facing(double x, double y, double z, double yaw) {
  return Pose::translation(x, y, z) * Pose::yaw(yaw + kPi / 2) * Pose::axis_angle({1, 0, 0}, kPi / 2);
}

std::string sign_name(const std::string& text, const std::string& bg, const std::string& fg) {
  return "sign/" + text + "/" + bg + "/" + fg;
}

}  // namespace

void RecyclingWorld::build_facility() {
  const double hx = layout_.hall_x, hy = layout_.hall_y, wh = layout_.wall_height;
  const auto quad = [this](double sx, double sy, const Pose& pose, const Material& m) {
    add_static({render::make_quad(sx, sy), pose, m});
  };

  // ---- 바닥·벽 ----
  add_box(hx / 2, hy / 2, -0.005, hx, hy, 0.01, tex("concrete", static_cast<float>(hx / 2), static_cast<float>(hy / 2)));
  const double t = 0.2;
  const Material wall = tex("wall_panel", static_cast<float>(hx / 1.6), static_cast<float>(wh / 1.6), {1, 1, 1}, 0.08F);
  add_box(hx / 2, -t / 2, wh / 2, hx + 2 * t, t, wh, wall, true);
  add_box(hx / 2, hy + t / 2, wh / 2, hx + 2 * t, t, wh, wall, true);
  add_box(-t / 2, hy / 2, wh / 2, t, hy, wh, wall, true);
  add_box(hx + t / 2, hy / 2, wh / 2, t, hy, wh, wall, true);
  const Material kick = tex("concrete", 8, 1, {0.8F, 0.8F, 0.8F});  // 벽 아래 콘크리트 띠
  add_box(hx / 2, 0.01, 0.15, hx, 0.02, 0.3, kick);
  add_box(hx / 2, hy - 0.01, 0.15, hx, 0.02, 0.3, kick);
  add_box(0.01, hy / 2, 0.15, 0.02, hy, 0.3, kick);
  add_box(hx - 0.01, hy / 2, 0.15, 0.02, hy, 0.3, kick);
  Material daylight = render::solid({0.86F, 0.92F, 0.97F});  // 높은 창 (밖이 밝다)
  daylight.emissive = 0.85F;
  for (double x = 2.0; x < hx - 1.0; x += 4.0) {
    quad(3.0, 0.7, facing(x, 0.02, 4.9, kPi / 2), daylight);
    quad(3.0, 0.7, facing(x, hy - 0.02, 4.9, -kPi / 2), daylight);
  }

  // ---- 바닥 구역 표시 ----
  const auto& cell = layout_.robot_cell;
  add_box(cell.center_x(), cell.center_y(), 0.0005, cell.max_x - cell.min_x, cell.max_y - cell.min_y, 0.001,
          tex("epoxy_green", 4, 3, {1, 1, 1}, 0.25F, 60));
  const Material hazard = tex("hazard_stripe", 1, 1);
  const double b = 0.10;  // 셀 경계 위험 테이프
  add_box(cell.center_x(), cell.min_y, 0.0013, cell.max_x - cell.min_x, b, 0.0005, tex("hazard_stripe", 40, 0.5F));
  add_box(cell.center_x(), cell.max_y, 0.0013, cell.max_x - cell.min_x, b, 0.0005, tex("hazard_stripe", 40, 0.5F));
  add_box(cell.min_x, cell.center_y(), 0.0013, b, cell.max_y - cell.min_y, 0.0005, tex("hazard_stripe", 0.5F, 30));
  add_box(cell.max_x, cell.center_y(), 0.0013, b, cell.max_y - cell.min_y, 0.0005, tex("hazard_stripe", 0.5F, 30));
  const auto& w = layout_.work_area;  // 고무 매트
  add_box(w.center_x(), w.center_y(), 0.001, w.max_x - w.min_x, w.max_y - w.min_y, 0.002, tex("rubber_mat", 3, 3, {1, 1, 1}, 0.03F));
  add_box(hx / 2, 7.7, 0.0006, hx, 1.0, 0.0012, tex("walkway", static_cast<float>(hx / 2), 1, {1, 1, 1}, 0.2F, 40));
  add_box(hx / 2, 8.5, 0.0013, hx, 0.1, 0.0005, tex("tape_yellow", 20, 1));  // 지게차 통로 경계
  add_box(hx / 2, 10.0, 0.0013, hx, 0.1, 0.0005, tex("tape_yellow", 20, 1));
  add_box(hx / 2, 9.25, 0.0004, hx, 1.4, 0.0008, tex("epoxy_gray", 12, 1, {1, 1, 1}, 0.2F, 40));

  // ---- 로봇 분리수거 셀 ----
  quad(4.2, 1.05, facing(5.0, 0.02, 2.6, kPi / 2), tex(sign_name("로봇 분리수거 구역  ROBOT SORTING CELL", "#1e5aa8", "#ffffff")));
  quad(1.8, 0.45, facing(2.0, 0.02, 1.5, kPi / 2), tex(sign_name("⚠ 작업 중 접근 주의", "#f2c21b", "#1a1a1a")));
  const Material partition = tex("steel_blue", 3, 1, {1, 1, 1}, 0.3F, 40);
  add_box(8.65, 3.9, 0.55, 0.1, 3.8, 1.1, partition, true);  // 바구니 뒤 칸막이
  const std::array<std::pair<double, std::string>, 3> crate_signs{{
      {2.9, sign_name("캔류  CAN", "#c62828", "#ffffff")},
      {3.9, sign_name("페트병  PET", "#1565c0", "#ffffff")},
      {4.9, sign_name("종이류  PAPER", "#2e7d32", "#ffffff")},
  }};
  for (const auto& [y, name] : crate_signs) {
    quad(0.9, 0.22, facing(8.59, y, 0.55, kPi), tex(name));
  }
  // 작업자 호출 패널 (버튼 = 진실 화면의 "작업 셀" 패널, recycling/request 서비스)
  add_box(8.57, 5.55, 0.95, 0.06, 0.32, 0.42, render::solid({0.85F, 0.85F, 0.82F}, 0.3F, 40));
  quad(0.3, 0.1, facing(8.535, 5.55, 1.1, kPi), tex(sign_name("작업자 호출", "#37474f", "#ffffff")));
  for (int i = 0; i < 3; ++i) {
    add_static({render::make_cylinder(0.025, 0.02, 16), Pose::translation(8.535, 5.45 + i * 0.1, 0.9) * Pose::axis_angle({0, 1, 0}, kPi / 2),
                render::solid(i == 0 ? Color{0.85F, 0.15F, 0.12F} : i == 1 ? Color{0.15F, 0.4F, 0.85F} : Color{0.15F, 0.6F, 0.25F}, 0.5F, 60)});
  }
  for (const double x : {1.0, 3.5, 6.0, 8.6}) {  // 셀 북쪽 볼라드
    add_static({render::make_cylinder(0.055, 0.9), Pose::translation(x, cell.max_y, 0.45), tex("steel_yellow", 1, 1, {1, 1, 1}, 0.4F, 50)});
    add_static({render::make_cylinder(0.057, 0.08), Pose::translation(x, cell.max_y, 0.7), render::solid({0.1F, 0.1F, 0.1F})});
    add_wall({x, cell.max_y, 0.06, 0.06, 0.9});
  }
  // 작업대 (서쪽 벽)
  const Material wood = tex("pallet_wood", 2, 1, {0.95F, 0.9F, 0.85F});
  add_box(0.55, 2.2, 0.88, 0.7, 1.4, 0.04, wood, true);
  for (const double dx : {-0.3, 0.3}) {
    for (const double dy : {-0.65, 0.65}) {
      add_box(0.55 + dx, 2.2 + dy, 0.43, 0.04, 0.04, 0.86, tex("steel_brushed", 1, 1, {1, 1, 1}, 0.5F, 60));
    }
  }
  add_box(0.5, 2.0, 0.98, 0.45, 0.22, 0.16, render::solid({0.75F, 0.12F, 0.1F}, 0.4F, 40));  // 공구함

  // ---- 기둥 ----
  for (const double x : {6.0, 12.0, 18.0}) {
    for (const double y : {7.0, 10.15}) {
      add_box(x, y, wh / 2, 0.35, 0.35, wh, tex("steel_yellow", 1, 6, {1, 1, 1}, 0.3F, 40), true);
      add_box(x, y, 0.5, 0.37, 0.37, 1.0, tex("hazard_stripe", 1, 2));
    }
  }
  add_static({render::make_cylinder(0.08, 0.45), Pose::translation(12.0, 6.75, 0.55), render::solid({0.8F, 0.08F, 0.06F}, 0.5F, 60)});  // 소화기
  quad(0.5, 0.18, facing(12.0, 6.82, 1.1, -kPi / 2), tex(sign_name("소화기", "#c62828", "#ffffff")));

  // ---- 하역장 (북서) ----
  for (const double x : {2.75, 5.75}) {
    quad(2.5, 3.5, facing(x, hy - 0.03, 1.75, -kPi / 2), tex("rollup_door", 1, 3, {1, 1, 1}, 0.3F, 40));
    add_box(x - 1.3, hy - 0.06, 1.8, 0.1, 0.1, 3.6, tex("hazard_stripe", 1, 6));
    add_box(x + 1.3, hy - 0.06, 1.8, 0.1, 0.1, 3.6, tex("hazard_stripe", 1, 6));
  }
  quad(3.0, 0.6, facing(4.25, hy - 0.03, 4.0, -kPi / 2), tex(sign_name("하역장  RECEIVING", "#37474f", "#ffffff")));
  const std::array<std::pair<double, double>, 6> bags{{{1.1, 11.0}, {2.25, 11.1}, {3.4, 10.95}, {1.2, 12.2}, {2.4, 12.3}, {1.15, 13.4}}};
  std::mt19937 rng(7);
  for (const auto& [x, y] : bags) {  // 혼합 재활용품 마대
    add_box(x, y, 0.45, 0.95, 0.95, 0.9, tex("jumbo_bag", 1, 1, {1, 1, 1}, 0.04F), true);
    add_static({render::make_lumpy_sphere(0.5, 0.18, static_cast<std::uint32_t>(rng())),
                Pose::translation(x, y, 0.88) * Pose::rpy({0, 0, 0}, {0, 0, 0}) * Pose::translation(0, 0, 0),
                tex("jumbo_bag", 2, 2, {0.96F, 0.96F, 0.94F})});
  }
  for (const auto& [px, py] : std::array<std::pair<double, double>, 2>{{{5.4, 12.8}, {6.4, 11.4}}}) {  // 팔레트 + 박스 더미
    add_box(px, py, 0.07, 1.2, 1.0, 0.14, tex("pallet_wood", 2, 2), true);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 2; ++j) {
        for (int k = 0; k < 2; ++k) {
          add_box(px - 0.4 + i * 0.4, py - 0.25 + j * 0.5, 0.14 + 0.16 + k * 0.32, 0.38, 0.46, 0.31,
                  tex("cardboard", 1, 1, {1, 1, 1}, 0.03F));
        }
      }
    }
  }

  // ---- 선별 컨베이어 (북쪽 가운데) ----
  const double cx = kConveyorStart + kConveyorLength / 2;
  const Material steel = tex("steel_brushed", 6, 1, {1, 1, 1}, 0.5F, 60);
  add_box(cx, kConveyorY, kBeltTop - 0.085, kConveyorLength + 0.6, 0.9, 0.15, steel);
  add_wall({cx, kConveyorY, (kConveyorLength + 0.6) / 2, 0.45, 0.95});
  for (const double y : {kConveyorY - 0.42, kConveyorY + 0.42}) {
    add_box(cx, y, kBeltTop + 0.05, kConveyorLength + 0.6, 0.04, 0.12, tex("steel_yellow", 10, 1, {1, 1, 1}, 0.3F, 40));
  }
  for (double x = kConveyorStart - 0.2; x <= kConveyorStart + kConveyorLength + 0.2; x += 1.2) {
    for (const double y : {kConveyorY - 0.38, kConveyorY + 0.38}) {
      add_box(x, y, (kBeltTop - 0.16) / 2, 0.06, 0.06, kBeltTop - 0.16, steel);
    }
  }
  add_box(7.5, kConveyorY, 0.9, 1.2, 1.2, 1.0, tex("steel_blue", 2, 2, {1, 1, 1}, 0.3F, 40), true);  // 투입 호퍼
  add_static({render::make_frustum(0.45, 0.8, 0.6, 4), Pose::translation(7.5, kConveyorY, 1.7) * Pose::yaw(kPi / 4),
              tex("steel_blue", 2, 1, {0.9F, 0.9F, 0.95F}, 0.3F, 40)});
  add_box(cx, 11.05, 0.002, kConveyorLength, 0.6, 0.004, tex("rubber_mat", 20, 1));  // 작업자 발판
  const std::array<std::string, 3> carts{"wheelie_blue", "wheelie_yellow", "wheelie_green"};
  for (int i = 0; i < 6; ++i) {  // 선별 낙하 수레 (컨베이어 반대편)
    add_box(9.0 + i * 1.5, 12.95, 0.32, 0.6, 0.5, 0.64, tex(carts[static_cast<std::size_t>(i % 3)], 1, 1, {1, 1, 1}, 0.15F), true);
  }
  quad(3.2, 0.6, facing(cx, 10.9, 3.2, -kPi / 2), tex(sign_name("선별 라인  SORTING LINE", "#37474f", "#ffffff")));

  // 컨베이어 위 물품 (흐른다)
  std::mt19937 item_rng(99);
  std::uniform_real_distribution<double> u(0, 1);
  const std::array<Category, 5> mix{Category::Can, Category::Pet, Category::Paper, Category::Paper, Category::Other};
  for (int i = 0; i < 24; ++i) {
    RecyclableItem item;
    item.category = mix[static_cast<std::size_t>(i % 5)];
    build_item(item, item_rng);
    ConveyorItem decor;
    decor.parts = item.parts;
    decor.phase = i * (kConveyorLength / 24.0) + u(item_rng) * 0.2;
    decor.y = kConveyorY + (u(item_rng) - 0.5) * 0.5;
    decor.yaw = u(item_rng) * 2 * kPi;
    if (item.can_lie) {  // 벨트 위에서는 대부분 누워 있다
      for (auto& part : decor.parts) {
        part.offset = Pose::translation(0, 0, item.diameter / 2) * Pose::axis_angle({0, 1, 0}, kPi / 2) *
                      Pose::translation(0, 0, -item.length / 2) * part.offset;
      }
    }
    conveyor_items_.push_back(std::move(decor));
  }

  // ---- 압축 베일 적재장 (북동) ----
  const std::array<std::string, 3> bales{"bale_cardboard", "bale_pet", "bale_can"};
  std::mt19937 bale_rng(3);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double x = 19.6 + i * 1.2, y = 10.95 + j * 1.2;
      const int levels = 1 + static_cast<int>(bale_rng() % 2);
      const std::string& kind = bales[static_cast<std::size_t>((i + j) % 3)];
      for (int k = 0; k < levels; ++k) {
        add_box(x, y, 0.55 + k * 1.1, 1.1, 1.1, 1.1, tex(kind, 1, 1, {1, 1, 1}, 0.06F), k == 0);
      }
    }
  }
  quad(3.0, 0.6, facing(21.4, hy - 0.03, 4.0, -kPi / 2), tex(sign_name("압축품 보관  BALES", "#37474f", "#ffffff")));

  // ---- 압축기 (동쪽 가운데) ----
  add_box(20.25, 5.75, 1.1, 2.5, 2.5, 2.2, tex("steel_blue", 2, 2, {1, 1, 1}, 0.35F, 40), true);
  add_box(20.25, 5.75, 0.125, 2.54, 2.54, 0.25, tex("hazard_stripe", 4, 0.5F));
  add_box(20.25, 5.75, 2.6, 1.6, 1.6, 0.8, tex("steel_yellow", 2, 1, {1, 1, 1}, 0.3F, 40));
  quad(1.6, 0.4, facing(18.99, 5.75, 1.6, kPi), tex(sign_name("압축기  BALER", "#f2c21b", "#1a1a1a")));
  add_box(18.75, 4.6, 0.7, 0.25, 0.4, 1.4, render::solid({0.55F, 0.57F, 0.6F}, 0.3F, 40), true);  // 조작반
  add_static({render::make_cylinder(0.03, 0.03), Pose::translation(18.61, 4.6, 1.15) * Pose::axis_angle({0, 1, 0}, kPi / 2),
              render::solid({0.9F, 0.05F, 0.05F}, 0.5F, 60)});
  add_box(18.3, 6.9, 0.55, 1.1, 1.1, 1.1, tex("bale_pet", 1, 1), true);  // 막 나온 베일

  // ---- 대형 수거함 구역 (남쪽 가운데) ----
  const std::array<std::string, 4> wheelies{"wheelie_blue", "wheelie_paper", "wheelie_yellow", "wheelie_gray"};  // 페트·종이·캔·일반 (셀 바구니를 여기에 비운다)
  for (int i = 0; i < 4; ++i) {
    const double x = 11.6 + i * 1.35, y = 1.0;
    add_box(x, y, 0.6, 1.1, 1.0, 1.1, tex(wheelies[static_cast<std::size_t>(i)], 1, 1, {1, 1, 1}, 0.15F, 30), true);
    add_box(x, y + 0.02, 1.18, 1.15, 1.05, 0.06, render::solid({0.15F, 0.15F, 0.17F}, 0.2F));  // 뚜껑
    for (const double dx : {-0.42, 0.42}) {
      add_static({render::make_cylinder(0.09, 0.05), Pose::translation(x + dx, y - 0.45, 0.09) * Pose::axis_angle({1, 0, 0}, kPi / 2),
                  render::solid({0.08F, 0.08F, 0.08F})});
    }
  }
  // 유리병 철망 케이지
  Material cage = tex("cage_mesh", 6, 8, {1, 1, 1}, 0.4F, 40);
  add_box(16.6, 4.4, 0.8, 1.2, 1.0, 1.6, cage, true);
  std::mt19937 glass_rng(5);
  for (int i = 0; i < 10; ++i) {
    RecyclableItem bottle;
    bottle.category = Category::Other;
    build_item(bottle, glass_rng);
    const Pose pose = Pose::translation(16.2 + (i % 4) * 0.25, 4.1 + (i / 4) * 0.28, 0.03 + (i % 3) * 0.06) *
                      Pose::yaw(i * 0.7) * Pose::axis_angle({0, 1, 0}, kPi / 2) * Pose::translation(0, 0, -0.09);
    for (const auto& part : bottle.parts) {
      add_static({part.mesh, pose * part.offset, part.material});
    }
  }

  // ---- 사무실 (남동, 유리벽) ----
  Material glass = tex("glass", 4, 2, {1, 1, 1}, 0.8F, 100);
  glass.blend = render::Blend::Alpha;
  add_box(20.5, 1.9, 1.3, 0.04, 3.8, 2.6, glass, true);
  add_box(21.0, 3.8, 1.3, 1.0, 0.04, 2.6, glass, true);
  add_box(23.2, 3.8, 1.3, 1.6, 0.04, 2.6, glass, true);  // 21.5~22.4 문
  for (const double y : {0.0, 1.2, 2.4, 3.8}) {
    add_box(20.5, y, 1.3, 0.06, 0.06, 2.6, tex("steel_brushed", 1, 2, {0.6F, 0.6F, 0.62F}, 0.5F, 60));
  }
  add_box(22.2, 3.8, 2.65, 3.4, 0.06, 0.1, tex("steel_brushed", 4, 1, {0.6F, 0.6F, 0.62F}));
  add_box(22.6, 1.4, 0.74, 1.6, 0.8, 0.04, wood, true);  // 책상
  add_box(22.6, 1.65, 0.97, 0.55, 0.04, 0.32, render::solid({0.05F, 0.05F, 0.06F}, 0.6F, 80));  // 모니터
  Material screen = render::solid({0.35F, 0.55F, 0.85F});
  screen.emissive = 0.9F;
  add_box(22.6, 1.628, 0.97, 0.5, 0.005, 0.27, screen);
  add_box(22.6, 0.85, 0.45, 0.5, 0.5, 0.08, render::solid({0.15F, 0.15F, 0.18F}));  // 의자
  quad(1.4, 0.35, facing(21.95, 3.83, 2.4, -kPi / 2), tex(sign_name("사무실  OFFICE", "#37474f", "#ffffff")));
  quad(1.2, 0.4, facing(hx - 0.03, 7.7, 2.4, kPi), tex(sign_name("비상구  EXIT", "#2e7d32", "#ffffff")));
  (void)hazard;
}

void RecyclingWorld::add_conveyor_render(std::vector<render::Instance>& out) const {
  const double now = time();
  // 벨트: 텍스처가 흘러간다
  static const auto belt = render::make_box(kConveyorLength + 0.6, 0.76, 0.01);
  render::Material rubber = render::solid({1, 1, 1}, 0.15F, 30);
  rubber.texture = textures::get("rubber_belt");
  rubber.uv_scale = {static_cast<float>((kConveyorLength + 0.6) / 0.6), 1.0F};
  rubber.uv_offset = {static_cast<float>(-now * kBeltSpeed / 0.6), 0.0F};
  out.push_back({belt, Pose::translation(kConveyorStart + kConveyorLength / 2, kConveyorY, kBeltTop - 0.005), rubber});
  for (const auto& item : conveyor_items_) {
    const double x = kConveyorStart + std::fmod(item.phase + now * kBeltSpeed, kConveyorLength);
    const Pose pose = Pose::translation(x, item.y, kBeltTop) * Pose::yaw(item.yaw);
    for (const auto& part : item.parts) {
      out.push_back({part.mesh, pose * part.offset, part.material});
    }
  }
}

}  // namespace srf_recycling_ws::sim
