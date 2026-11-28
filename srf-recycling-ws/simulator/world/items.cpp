// 재활용품 카탈로그: 종류별 여러 변형 (크기는 OpenMANIPULATOR-X 그리퍼로 집을 수 있는 범위).
#include <cmath>

#include "render/textures.hpp"
#include "world/recycling_world.hpp"

namespace srf_recycling_ws::sim {

namespace {

constexpr double kPi = 3.14159265358979323846;
using render::Color;
using render::Material;
using render::Pose;

Pose up(double z) { return Pose::translation(0, 0, z); }

Material textured(const std::string& texture, Color tint = {1, 1, 1}, float specular = 0.05F, float shininess = 24.0F) {
  Material m = render::solid(tint, specular, shininess);
  m.texture = textures::get(texture);
  return m;
}

Material metal(Color color) { return render::solid(color, 0.65F, 60.0F); }

Material clear_plastic(Color tint, float opacity) {
  Material m = render::solid(tint, 0.7F, 80.0F);
  m.blend = render::Blend::Alpha;
  m.opacity = opacity;
  return m;
}

template <class T>
const T& pick(const std::vector<T>& list, std::mt19937& rng) {
  return list[std::uniform_int_distribution<std::size_t>(0, list.size() - 1)(rng)];
}

void can(RecyclableItem& item, std::mt19937& rng) {
  static const std::vector<std::string> brands{"cola", "cider", "coffee", "beer", "energy"};
  const std::string brand = pick(brands, rng);
  const bool crushed = std::bernoulli_distribution(0.25)(rng);
  item.diameter = 0.053;
  item.length = crushed ? 0.095 : 0.115;
  item.can_lie = true;
  item.variant = std::string(crushed ? "can/crushed_" : "can/") + brand;
  const double r = item.diameter / 2;
  const Color alu{0.80F, 0.81F, 0.84F};
  const double body = item.length - 0.018;
  auto body_mesh = crushed ? render::make_crushed_cylinder(r, body, 0.28, static_cast<std::uint32_t>(rng()))
                           : render::make_cylinder(r, body);
  item.parts.push_back({render::make_frustum(r * 0.85, r, 0.008), up(0.004), metal(alu)});          // 바닥 테
  item.parts.push_back({body_mesh, up(0.008 + body / 2), textured("label/can/" + brand, {1, 1, 1}, 0.55F, 50)});
  item.parts.push_back({render::make_frustum(r, r * 0.84, 0.009), up(item.length - 0.0055), metal(alu)});  // 어깨
  item.parts.push_back({render::make_cylinder(r * 0.82, 0.002), up(item.length - 0.001), metal(alu)});     // 뚜껑
  item.parts.push_back({render::make_box(0.014, 0.008, 0.0015), Pose::translation(0.006, 0, item.length), metal({0.7F, 0.7F, 0.72F})});  // 따개
}

void food_can(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.diameter = 0.066;
  item.length = 0.042;
  item.can_lie = false;
  item.variant = "can/tuna";
  const double r = item.diameter / 2;
  item.parts.push_back({render::make_cylinder(r, item.length - 0.004), up(item.length / 2),
                        textured("label/can/tuna", {1, 1, 1}, 0.4F, 40)});
  item.parts.push_back({render::make_cylinder(r * 0.97, 0.004), up(item.length - 0.002), metal({0.78F, 0.79F, 0.80F})});
}

void bottle(RecyclableItem& item, std::mt19937& rng) {
  static const std::vector<std::string> brands{"water", "soda", "tea", "juice"};
  const std::string brand = pick(brands, rng);
  const bool small = std::bernoulli_distribution(0.35)(rng);
  item.diameter = small ? 0.050 : 0.058;
  item.length = small ? 0.135 : 0.175;
  item.can_lie = true;
  item.variant = std::string(small ? "pet/small_" : "pet/") + brand;
  const double r = item.diameter / 2;
  const double body = item.length * 0.66;
  const Color tint = brand == "soda" ? Color{0.75F, 0.92F, 0.80F} : Color{0.85F, 0.93F, 0.97F};
  const Material plastic = clear_plastic(tint, 0.42F);
  const Color caps[4] = {{0.95F, 0.95F, 0.95F}, {0.15F, 0.35F, 0.85F}, {0.85F, 0.15F, 0.15F}, {0.2F, 0.6F, 0.25F}};
  const Color cap = caps[std::uniform_int_distribution<int>(0, 3)(rng)];
  // 투명 몸통 안쪽에 물 (마시다 남은 양) — 투명해도 형태가 보이게
  const double water = body * std::uniform_real_distribution<double>(0.0, 0.7)(rng);
  if (water > 0.01) {
    item.parts.push_back({render::make_cylinder(r * 0.92, water), up(0.004 + water / 2), clear_plastic({0.6F, 0.75F, 0.85F}, 0.35F)});
  }
  item.parts.push_back({render::make_frustum(r * 0.85, r, 0.012), up(0.006), plastic});                 // 바닥 굴곡
  item.parts.push_back({render::make_cylinder(r, body - 0.012), up(0.012 + (body - 0.012) / 2), plastic});
  item.parts.push_back({render::make_cylinder(r * 1.005, body * 0.38), up(body * 0.52), textured("label/pet/" + brand)});  // 라벨 띠
  const double shoulder = item.length * 0.22;
  item.parts.push_back({render::make_frustum(r, 0.0125, shoulder, 32, false), up(body + shoulder / 2), plastic});
  item.parts.push_back({render::make_cylinder(0.0125, 0.012), up(body + shoulder + 0.006), plastic});      // 목
  item.parts.push_back({render::make_cylinder(0.0145, 0.016), up(item.length - 0.008), render::solid(cap, 0.2F)});  // 뚜껑
}

void milk_carton(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.diameter = 0.058;
  item.length = 0.105;
  item.variant = "paper/milk";
  const double s = item.diameter;
  item.parts.push_back({render::make_box(s, s, 0.080), up(0.040), textured("label/milk")});
  // 박공 지붕 대신 사각뿔대(모서리 반지름 = 한 변/√2) + 위 접합 날개
  item.parts.push_back({render::make_frustum(s * 0.7071, s * 0.18, 0.022, 4),
                        up(0.091) * Pose::yaw(kPi / 4), textured("label/milk")});
  item.parts.push_back({render::make_box(s * 0.5, 0.004, 0.012), up(0.106), render::solid({0.95F, 0.95F, 0.93F})});
}

void parcel_box(RecyclableItem& item, std::mt19937& rng) {
  item.diameter = 0.07;
  item.length = std::uniform_real_distribution<double>(0.045, 0.07)(rng);
  item.variant = "paper/box";
  item.parts.push_back({render::make_box(0.07, 0.07, item.length), up(item.length / 2), textured("label/box")});
}

void paper_cup(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.diameter = 0.07;
  item.length = 0.09;
  item.can_lie = true;
  item.variant = "paper/cup";
  item.parts.push_back({render::make_frustum(0.024, 0.034, 0.088, 32, true), up(0.044), textured("label/cup")});
  item.parts.push_back({render::make_frustum(0.035, 0.036, 0.004, 32, false), up(0.089), render::solid({0.97F, 0.97F, 0.96F})});  // 말린 입구
}

void paper_ball(RecyclableItem& item, std::mt19937& rng) {
  const bool news = std::bernoulli_distribution(0.5)(rng);
  item.diameter = 0.062;
  item.length = 0.056;
  item.variant = news ? "paper/crumpled_newspaper" : "paper/crumpled";
  item.parts.push_back({render::make_lumpy_sphere(0.029, 0.30, static_cast<std::uint32_t>(rng())), up(0.028),
                        news ? textured("newspaper") : render::solid({0.94F, 0.94F, 0.91F}, 0.02F)});
}

void rolled_newspaper(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.diameter = 0.045;
  item.length = 0.16;
  item.can_lie = true;
  item.variant = "paper/newspaper_roll";
  item.parts.push_back({render::make_cylinder(0.0225, 0.16), up(0.08), textured("newspaper")});
}

// 불확실: 플라스틱 뚜껑 달린 종이컵, 알루미늄 포함 멸균팩 (분리 기준이 애매하다)
void cup_with_lid(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.diameter = 0.075;
  item.length = 0.10;
  item.variant = "unknown/cup_with_lid";
  item.parts.push_back({render::make_frustum(0.025, 0.034, 0.088), up(0.044), textured("label/cup")});
  item.parts.push_back({render::make_frustum(0.037, 0.030, 0.012), up(0.094), clear_plastic({0.92F, 0.94F, 0.95F}, 0.6F)});
  item.parts.push_back({render::make_cylinder(0.003, 0.05), Pose::translation(0.01, 0, 0.11) * Pose::axis_angle({0, 1, 0}, 0.25),
                        render::solid({0.15F, 0.6F, 0.3F})});  // 빨대
}

void tetra_pack(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.diameter = 0.045;
  item.length = 0.095;
  item.variant = "unknown/tetra_pack";
  item.parts.push_back({render::make_box(0.062, 0.040, 0.095), up(0.0475), textured("label/tetra")});
}

// 비대상
void glass_bottle(RecyclableItem& item, std::mt19937& rng) {
  const bool brown = std::bernoulli_distribution(0.5)(rng);
  item.diameter = 0.056;
  item.length = 0.18;
  item.can_lie = true;
  item.variant = brown ? "other/glass_bottle_brown" : "other/glass_bottle_green";
  const Color tint = brown ? Color{0.45F, 0.25F, 0.10F} : Color{0.20F, 0.45F, 0.20F};
  Material glass = render::solid(tint, 0.9F, 120.0F);
  glass.blend = render::Blend::Alpha;
  glass.opacity = 0.75F;
  item.parts.push_back({render::make_cylinder(0.028, 0.11), up(0.055), glass});
  item.parts.push_back({render::make_frustum(0.028, 0.011, 0.04, 32, false), up(0.13), glass});
  item.parts.push_back({render::make_cylinder(0.011, 0.03), up(0.165), glass});
  item.parts.push_back({render::make_cylinder(0.0125, 0.006), up(0.177), metal({0.75F, 0.65F, 0.2F})});  // 왕관 뚜껑
}

void vinyl_bag(RecyclableItem& item, std::mt19937& rng) {
  item.diameter = 0.08;
  item.length = 0.05;
  item.variant = "other/vinyl";
  Material m = textured("vinyl", {1, 1, 1}, 0.5F, 40);
  m.blend = render::Blend::Alpha;
  item.parts.push_back({render::make_lumpy_sphere(0.04, 0.4, static_cast<std::uint32_t>(rng())), up(0.025) * Pose::rpy({0, 0, 0}, {0, 0, 0}), m});
}

void styrofoam(RecyclableItem& item, std::mt19937& rng) {
  (void)rng;
  item.diameter = 0.09;
  item.length = 0.025;
  item.variant = "other/styrofoam_tray";
  item.parts.push_back({render::make_box(0.12, 0.085, 0.022), up(0.011), textured("styrofoam", {1, 1, 1}, 0.02F)});
}

}  // namespace

void build_item(RecyclableItem& item, std::mt19937& rng) {
  std::uniform_real_distribution<double> u(0, 1);
  switch (item.category) {
    case Category::Can: (u(rng) < 0.8 ? can : food_can)(item, rng); break;
    case Category::Pet: bottle(item, rng); break;
    case Category::Paper: {
      const double p = u(rng);
      (p < 0.3 ? milk_carton : p < 0.5 ? parcel_box : p < 0.7 ? paper_cup : p < 0.9 ? paper_ball : rolled_newspaper)(item, rng);
      break;
    }
    case Category::Unknown: (u(rng) < 0.6 ? cup_with_lid : tetra_pack)(item, rng); break;
    case Category::Other: {
      const double p = u(rng);
      (p < 0.45 ? glass_bottle : p < 0.75 ? vinyl_bag : styrofoam)(item, rng);
      break;
    }
  }
  (void)kPi;
}

}  // namespace srf_recycling_ws::sim
