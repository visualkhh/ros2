#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <typeindex>
#include <vector>

#include "krosc/net/net_protocol.hpp"
#include "krosc/net/rpc_codec.hpp"
#include "krosc/qos.hpp"
#include "krosc/topic.hpp"

namespace krosc::net {

// 토픽 메시지 타입 지원 (ROS의 type support에 해당).
// 메시지 타입 하나가 표의 한 줄이고, 전송 계층(PeerNetwork/웹 게이트웨이)은 타입을 모른 채
// 이 표만 보고 인코딩·디코딩·로컬 발행·구독을 한다. 새 타입 = topic_codec.cpp에 한 줄.
// Kind는 전송용 타입 ID다 (피어와 브라우저가 같은 숫자를 쓴다).
struct TopicType {
  Kind kind;
  std::type_index type;
  std::string_view name;  // msgs::message_type_name<T>()
  void (*encode)(RpcWriter& out, const void* message) = nullptr;  // out 뒤에 이어 쓴다
  std::shared_ptr<const void> (*decode)(const std::uint8_t* data, std::size_t size) = nullptr;
  void (*publish)(TopicRegistry& topics, const std::string& topic, std::shared_ptr<const void> message,
                  QoS qos) = nullptr;
  std::shared_ptr<krosc::detail::SubscriptionBase> (*subscribe)(TopicRegistry& topics, const std::string& topic,
                                                         std::function<void(const void*)> callback,
                                                         QoS qos) = nullptr;
};

// 등록되지 않은 타입이면 nullptr.
[[nodiscard]] const TopicType* find_topic_type(Kind kind);
[[nodiscard]] const TopicType* find_topic_type(std::type_index type);

}  // namespace krosc::net
