#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "srf/hardware/hardware.hpp"

namespace srf_recycling_ws {

// 카메라 프레임 한 장. encoding/step 규약은 srf::msgs::Image와 같다.
struct RealSenseImage {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string encoding;
  std::uint32_t step = 0;
  std::vector<std::uint8_t> data;
};

struct RealSenseStats {
  bool running = false;
  std::uint64_t frames = 0;
  std::uint64_t color_frames = 0;
  std::uint64_t depth_frames = 0;
  double color_age = 0.0;
  double depth_age = 0.0;
  std::string serial;
  std::string firmware;
  std::string product;
};

// 실기 RealSenseDevice와 시뮬레이터 SimulatorRealSense가 공유하는 장치 인터페이스.
// Hardware는 물리 계층만 담당하고, 토픽 발행은 Sensor 레이어가 담당한다.
class RealSenseCamera : public srf::hardware::Hardware {
 public:
  // 새 프레임이 있을 때만 반환한다. 없으면 nullopt.
  // 구현체 워커가 미리 가공해 둔 캐시를 복사만 해야 한다 (소비자 스레드에서 인코딩 금지).
  // color: JPEG 또는 RGB8/BGR8, depth: 16UC1 little-endian mm.
  [[nodiscard]] virtual std::optional<RealSenseImage> take_color() = 0;
  [[nodiscard]] virtual std::optional<RealSenseImage> take_depth() = 0;
  [[nodiscard]] virtual RealSenseStats stats() const = 0;
};

}  // namespace srf_recycling_ws
