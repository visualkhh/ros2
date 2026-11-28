#pragma once

// 절차 텍스처: Qt(QImage/QPainter)로 노이즈 재질과 글씨 들어간 라벨·표지판을 만든다.
//   - 외부 이미지 없음 (라이선스 걱정 없음), 같은 이름이면 같은 그림 (재현).
//   - 글씨는 QGuiApplication이 있을 때만 그린다 (테스트처럼 없으면 무늬만).
//   - 이름으로 한 번 만들고 캐시한다 (스레드 안전).
#include <memory>
#include <string>

#include "render/scene_renderer.hpp"

class QImage;

namespace srf_recycling_ws::sim::textures {

// 재질: concrete, epoxy_gray, epoxy_green, wall_panel, steel_brushed, aluminum, rubber_belt, pallet_wood,
//       cardboard, rubber_mat, hazard_stripe, walkway, jumbo_bag, rollup_door, bale_cardboard, bale_pet,
//       bale_can, hi_vis_vest, denim, shirt_blue, shirt_gray, forklift_yellow, glass, shadow_blob,
//       crate_can, crate_pet, crate_paper, wheelie_blue, wheelie_green, wheelie_paper, wheelie_yellow, wheelie_gray,
//       cage_mesh, newspaper, kraft, styrofoam, vinyl
// 라벨(물품): label/can/<brand>, label/pet/<brand>, label/milk, label/cup, label/tetra, label/box
// 표지판: sign/<글씨>/<배경 #rrggbb>/<글씨 #rrggbb>   (예: sign/로봇 분리수거 구역/#1e5aa8/#ffffff)
std::shared_ptr<const render::Texture> get(const std::string& name);

// QImage → Texture (RGBA8).
std::shared_ptr<render::Texture> from_image(const QImage& image);

}  // namespace srf_recycling_ws::sim::textures
