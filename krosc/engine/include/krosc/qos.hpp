#pragma once

#include <cstddef>
#include <optional>

namespace krosc {

enum class Durability {
  Volatile,        // 지금 구독 중인 쪽에만 전달한다 (기본)
  TransientLocal,  // 마지막 메시지를 기억해 뒀다가 나중에 구독한 쪽에게도 전달한다 (latch)
};

struct QoS {
  // 구독: 처리를 기다리는 메시지 최대 개수. 넘치면 가장 오래된 것을 버린다.
  // 발행 + TransientLocal: 나중에 구독한 쪽을 위해 기억해 둘 마지막 메시지 수.
  std::optional<std::size_t> depth{};
  Durability durability = Durability::Volatile;

  constexpr QoS() = default;
  constexpr QoS(std::optional<std::size_t> depth_value, Durability durability_value = Durability::Volatile)
      : depth(depth_value), durability(durability_value) {}

  [[nodiscard]] constexpr bool latched() const { return durability == Durability::TransientLocal; }
};

inline constexpr QoS QOS_DEFAULT{};                                             // 무제한 큐, 휘발성
inline constexpr QoS QOS_LATCHED{1, Durability::TransientLocal};                // 마지막 하나를 기억
inline constexpr QoS QOS_SENSOR_DATA{5, Durability::Volatile};                  // 센서: 느린 구독자는 오래된 것을 건너뛴다
inline constexpr QoS QOS_COMMAND{1, Durability::Volatile};                      // 명령: 최신만
inline constexpr QoS QOS_TF{100, Durability::Volatile};
inline constexpr QoS QOS_TF_STATIC{100, Durability::TransientLocal};

}  // namespace krosc
