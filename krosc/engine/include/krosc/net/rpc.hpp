#pragma once

// krosc RPC: PeerNetwork 위에서 서비스/액션을 타입 소거로 주고받는 코덱.
//
// - 서비스 요청/응답은 msgs 타입 이름과 바이트 payload로 오간다.
// - 액션은 기존 로컬 API(ActionServer/ActionClient)를 그대로 쓰고,
//   send_goal/cancel_goal 서비스 + /_action/event 토픽만 여기서 인코딩한다.
// - 등록되지 않은 타입은 원격 전송하지 않는다 (로컬에서는 그대로 동작).

#include <any>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <typeindex>
#include <vector>

#include "krosc/action_types.hpp"
#include "krosc/net/rpc_codec.hpp"

namespace krosc::net {

constexpr std::uint8_t kFrameRpcRequest = 5;
constexpr std::uint8_t kFrameRpcResponse = 6;
constexpr std::uint8_t kFrameActionEvent = 7;

constexpr std::uint8_t kRpcRequest = 0;
constexpr std::uint8_t kRpcOk = 1;
constexpr std::uint8_t kRpcError = 2;

struct RpcEnvelope {
  std::uint32_t request_id = 0;
  std::uint8_t status = kRpcRequest;
  std::string request_type;
  std::string response_type;
  std::vector<std::uint8_t> payload;
  std::string error;
};

[[nodiscard]] std::vector<std::uint8_t> encode_rpc_envelope(const RpcEnvelope& envelope);
[[nodiscard]] std::optional<RpcEnvelope> decode_rpc_envelope(
    const std::vector<std::uint8_t>& payload);

// ---- 타입 코덱 레지스트리 (도메인 코덱은 각 도메인 파일에서 등록) ----
[[nodiscard]] std::optional<std::string> rpc_type_name(std::type_index type);
[[nodiscard]] std::vector<std::uint8_t> encode_rpc_value(std::type_index type, const void* value);
// 서비스 응답 타입이 코덱에 등록됐는지.
[[nodiscard]] bool rpc_type_registered(std::type_index type);
[[nodiscard]] std::any decode_rpc_value(const std::string& type_name, const std::uint8_t* data,
                                        std::size_t size);

// send_goal/cancel_goal처럼 액션 이름에 따라 payload가 달라지는 서비스.
[[nodiscard]] std::vector<std::uint8_t> encode_rpc_request(const std::string& service,
                                                          std::type_index type,
                                                          const void* request);
[[nodiscard]] std::any decode_rpc_request(const std::string& service,
                                          const std::string& type_name, const std::uint8_t* data,
                                          std::size_t size);
[[nodiscard]] std::vector<std::uint8_t> encode_rpc_response(const std::string& service,
                                                           std::type_index type,
                                                           const void* response);
[[nodiscard]] std::any decode_rpc_response(const std::string& service,
                                           const std::string& type_name,
                                           const std::uint8_t* data, std::size_t size);

// ---- 액션 코덱 레지스트리 (도메인 액션은 각 도메인 파일에서 등록) ----
struct ActionCodec {
  std::string name;
  std::function<std::vector<std::uint8_t>(const std::any&)> encode_goal;
  std::function<std::any(const std::uint8_t*, std::size_t)> decode_goal;
  std::function<std::vector<std::uint8_t>(const std::any&)> encode_feedback;
  std::function<std::any(const std::uint8_t*, std::size_t)> decode_feedback;
  std::function<std::vector<std::uint8_t>(const std::any&)> encode_result;
  std::function<std::any(const std::uint8_t*, std::size_t)> decode_result;
};

void register_action_codec(ActionCodec codec);
[[nodiscard]] const ActionCodec* find_action_codec(const std::string& action_name);
[[nodiscard]] std::string action_name_from_service(const std::string& service);
[[nodiscard]] std::string action_name_from_event_topic(const std::string& topic);

[[nodiscard]] std::vector<std::uint8_t> encode_action_event(const std::string& topic,
                                                            const ActionEvent& event);
[[nodiscard]] std::optional<ActionEvent> decode_action_event(const std::string& topic,
                                                             const std::uint8_t* data,
                                                             std::size_t size);

}  // namespace krosc::net
