#pragma once

#include <cstddef>
#include <optional>

namespace krosc {

enum class Durability {
  Volatile,        // 지금 구독 중인 쪽에만 전달한다 (기본)
  TransientLocal,  // 마지막 메시지를 기억해 뒀다가 나중에 구독한 쪽에게도 전달한다 (latch)
};

struct QoS {
  // 구독: 처리를 기다리는 메시지 최대 개수. 넘치면 가장 오래된 것을 버린다 (KEEP_LAST). 비우면 KEEP_ALL.
  // 발행 + TransientLocal: 나중에 구독한 쪽을 위해 기억해 둘 마지막 메시지 수.
  std::optional<std::size_t> depth{};
  Durability durability = Durability::Volatile;

  constexpr QoS() = default;
  constexpr QoS(std::optional<std::size_t> depth_value, Durability durability_value = Durability::Volatile)
      : depth(depth_value), durability(durability_value) {}

  [[nodiscard]] constexpr bool latched() const { return durability == Durability::TransientLocal; }
};

// ROS 2 기본 프로필과 같은 depth를 쓴다 (rmw_qos_profile_*). Reliability는 두지 않는다:
// 프로세스 안은 큐 전달, 프로세스 사이는 TCP라서 BEST_EFFORT를 구분할 곳이 없다.
inline constexpr QoS QOS_DEFAULT{10};                                // rmw_qos_profile_default: KEEP_LAST 10
inline constexpr QoS QOS_KEEP_ALL{std::nullopt};                     // KEEP_ALL: 버리지 않는다 (메모리 주의)
inline constexpr QoS QOS_LATCHED{1, Durability::TransientLocal};     // 마지막 하나를 기억
inline constexpr QoS QOS_SENSOR_DATA{5, Durability::Volatile};       // rmw_qos_profile_sensor_data: KEEP_LAST 5
inline constexpr QoS QOS_PARAMETER_EVENTS{1000, Durability::Volatile};  // rmw_qos_profile_parameter_events
inline constexpr QoS QOS_COMMAND{1, Durability::Volatile};           // krosc 전용: 명령은 최신만
inline constexpr QoS QOS_TF{100, Durability::Volatile};              // tf2_ros DynamicBroadcasterQoS
// ROS는 static broadcaster마다 자기 변환을 모아 depth 1로 내지만, krosc의 latched 기록은 토픽 단위라
// 발행자 여럿의 메시지가 한 기록을 나눠 쓴다. 그래서 StaticListenerQoS와 같은 100을 쓴다.
inline constexpr QoS QOS_TF_STATIC{100, Durability::TransientLocal};

}  // namespace krosc
