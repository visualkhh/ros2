#include "krosc/net/rpc.hpp"

#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <typeinfo>
#include <utility>

namespace krosc::net {

// 도메인 코덱 등록 파일 (engine/core/msgs/rpc_builtin.cpp).
void register_builtin_rpc_codecs();
void register_builtin_action_codecs();

namespace {

struct CodecTable {
  std::map<std::type_index, RpcCodec> by_type;
  std::map<std::string, std::type_index> by_name;
};

std::mutex& codec_mutex() {
  static std::mutex mutex;
  return mutex;
}

CodecTable& codec_table() {
  static CodecTable table;
  return table;
}

std::mutex& action_mutex() {
  static std::mutex mutex;
  return mutex;
}

std::map<std::string, ActionCodec>& action_table() {
  static std::map<std::string, ActionCodec> table;
  return table;
}

void ensure_builtin_rpc_codecs() {
  static std::once_flag once;
  std::call_once(once, [] { register_builtin_rpc_codecs(); });
}

void ensure_builtin_action_codecs() {
  static std::once_flag once;
  std::call_once(once, [] { register_builtin_action_codecs(); });
}

bool matches_action(const std::string& name, const std::string& codec_name) {
  if (name == codec_name) {
    return true;
  }
  if (name.size() <= codec_name.size()) {
    return false;
  }
  const std::size_t offset = name.size() - codec_name.size();
  return name[offset - 1] == '/' && name.compare(offset, codec_name.size(), codec_name) == 0;
}

const ActionCodec* find_action_codec_locked(const std::string& action_name) {
  for (const auto& [name, codec] : action_table()) {
    (void)name;
    if (matches_action(action_name, codec.name)) {
      return &codec;
    }
  }
  return nullptr;
}

}  // namespace

std::vector<std::uint8_t> encode_rpc_envelope(const RpcEnvelope& envelope) {
  RpcWriter writer;
  writer.u32(envelope.request_id);
  writer.u8(envelope.status);
  writer.str(envelope.request_type);
  writer.str(envelope.response_type);
  writer.u32(static_cast<std::uint32_t>(envelope.payload.size()));
  writer.bytes(envelope.payload);
  writer.str(envelope.error);
  return std::move(writer.data);
}

std::optional<RpcEnvelope> decode_rpc_envelope(const std::vector<std::uint8_t>& payload) {
  RpcReader reader(payload);
  RpcEnvelope envelope;
  envelope.request_id = reader.u32();
  envelope.status = reader.u8();
  envelope.request_type = reader.str();
  envelope.response_type = reader.str();
  const std::uint32_t length = reader.u32();
  if (!reader.ok() || length > reader.remaining()) {
    return std::nullopt;
  }
  envelope.payload.assign(reader.rest(), reader.rest() + length);
  reader.skip(length);
  envelope.error = reader.str();
  if (!reader.ok()) {
    return std::nullopt;
  }
  return envelope;
}

void register_rpc_codec(std::type_index type, RpcCodec codec) {
  std::lock_guard<std::mutex> lock(codec_mutex());
  CodecTable& table = codec_table();
  const std::string name = codec.name;
  table.by_type[type] = std::move(codec);
  table.by_name.insert_or_assign(name, type);
}

std::optional<std::string> rpc_type_name(std::type_index type) {
  ensure_builtin_rpc_codecs();
  std::lock_guard<std::mutex> lock(codec_mutex());
  const auto found = codec_table().by_type.find(type);
  if (found == codec_table().by_type.end()) {
    return std::nullopt;
  }
  return found->second.name;
}

std::vector<std::uint8_t> encode_rpc_value(std::type_index type, const void* value) {
  ensure_builtin_rpc_codecs();
  std::lock_guard<std::mutex> lock(codec_mutex());
  const auto found = codec_table().by_type.find(type);
  if (found == codec_table().by_type.end() || value == nullptr) {
    return {};
  }
  return found->second.encode(value);
}

bool rpc_type_registered(std::type_index type) {
  ensure_builtin_rpc_codecs();
  std::lock_guard<std::mutex> lock(codec_mutex());
  return codec_table().by_type.count(type) != 0;
}

std::any decode_rpc_value(const std::string& type_name, const std::uint8_t* data,
                          std::size_t size) {
  ensure_builtin_rpc_codecs();
  std::lock_guard<std::mutex> lock(codec_mutex());
  const auto name_it = codec_table().by_name.find(type_name);
  if (name_it == codec_table().by_name.end()) {
    return {};
  }
  const auto type_it = codec_table().by_type.find(name_it->second);
  if (type_it == codec_table().by_type.end()) {
    return {};
  }
  return type_it->second.decode(data, size);
}

void register_action_codec(ActionCodec codec) {
  std::lock_guard<std::mutex> lock(action_mutex());
  action_table()[codec.name] = std::move(codec);
}

const ActionCodec* find_action_codec(const std::string& action_name) {
  ensure_builtin_action_codecs();
  std::lock_guard<std::mutex> lock(action_mutex());
  return find_action_codec_locked(action_name);
}

std::string action_name_from_service(const std::string& service) {
  for (const char* suffix : {"/send_goal", "/cancel_goal"}) {
    const std::string ending(suffix);
    if (service.size() > ending.size() &&
        service.compare(service.size() - ending.size(), ending.size(), ending) == 0) {
      return service.substr(0, service.size() - ending.size());
    }
  }
  return service;
}

std::string action_name_from_event_topic(const std::string& topic) {
  const std::string suffix = "/_action/event";
  if (topic.size() > suffix.size() &&
      topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0) {
    return topic.substr(0, topic.size() - suffix.size());
  }
  return topic;
}

std::vector<std::uint8_t> encode_rpc_request(const std::string& service, std::type_index type,
                                             const void* request) {
  if (type == typeid(SendGoalRequest)) {
    const auto* goal_request = static_cast<const SendGoalRequest*>(request);
    const ActionCodec* codec = find_action_codec(action_name_from_service(service));
    if (goal_request == nullptr || codec == nullptr) {
      return {};
    }
    const std::vector<std::uint8_t> goal = codec->encode_goal(goal_request->goal);
    RpcWriter writer;
    writer.str(goal_request->goal_id);
    writer.u32(static_cast<std::uint32_t>(goal.size()));
    writer.bytes(goal);
    return std::move(writer.data);
  }
  if (type == typeid(CancelGoalRequest)) {
    const auto* cancel_request = static_cast<const CancelGoalRequest*>(request);
    if (cancel_request == nullptr) {
      return {};
    }
    RpcWriter writer;
    writer.str(cancel_request->goal_id);
    return std::move(writer.data);
  }
  return encode_rpc_value(type, request);
}

std::any decode_rpc_request(const std::string& service, const std::string& type_name,
                            const std::uint8_t* data, std::size_t size) {
  if (type_name == "krosc/action/SendGoalRequest") {
    const ActionCodec* codec = find_action_codec(action_name_from_service(service));
    if (codec == nullptr) {
      return {};
    }
    RpcReader reader(data, size);
    SendGoalRequest request;
    request.goal_id = reader.str();
    const std::uint32_t length = reader.u32();
    if (!reader.ok() || length > reader.remaining()) {
      return {};
    }
    request.goal = codec->decode_goal(reader.rest(), length);
    if (!request.goal.has_value()) {
      return {};
    }
    return std::any(std::move(request));
  }
  if (type_name == "krosc/action/CancelGoalRequest") {
    RpcReader reader(data, size);
    CancelGoalRequest request;
    request.goal_id = reader.str();
    if (!reader.ok()) {
      return {};
    }
    return std::any(std::move(request));
  }
  return decode_rpc_value(type_name, data, size);
}

std::vector<std::uint8_t> encode_rpc_response(const std::string& service, std::type_index type,
                                              const void* response) {
  (void)service;
  return encode_rpc_value(type, response);
}

std::any decode_rpc_response(const std::string& service, const std::string& type_name,
                             const std::uint8_t* data, std::size_t size) {
  (void)service;
  return decode_rpc_value(type_name, data, size);
}

std::vector<std::uint8_t> encode_action_event(const std::string& topic,
                                              const ActionEvent& event) {
  const ActionCodec* codec = find_action_codec(action_name_from_event_topic(topic));
  if (codec == nullptr) {
    return {};
  }
  std::vector<std::uint8_t> feedback;
  std::vector<std::uint8_t> result;
  if (event.has_feedback) {
    feedback = codec->encode_feedback(event.feedback);
    if (feedback.empty()) {
      return {};
    }
  }
  if (event.has_result) {
    result = codec->encode_result(event.result);
    if (result.empty()) {
      return {};
    }
  }
  RpcWriter writer;
  writer.str(event.goal_id);
  writer.u8(event.has_feedback ? 1U : 0U);
  writer.u8(event.has_result ? 1U : 0U);
  writer.u32(static_cast<std::uint32_t>(feedback.size()));
  writer.bytes(feedback);
  writer.u32(static_cast<std::uint32_t>(result.size()));
  writer.bytes(result);
  writer.u8(static_cast<std::uint8_t>(event.status));
  return std::move(writer.data);
}

std::optional<ActionEvent> decode_action_event(const std::string& topic,
                                               const std::uint8_t* data, std::size_t size) {
  const ActionCodec* codec = find_action_codec(action_name_from_event_topic(topic));
  if (codec == nullptr) {
    return std::nullopt;
  }
  RpcReader reader(data, size);
  ActionEvent event;
  event.goal_id = reader.str();
  event.has_feedback = reader.u8() != 0;
  event.has_result = reader.u8() != 0;
  const std::uint32_t feedback_length = reader.u32();
  if (!reader.ok() || feedback_length > reader.remaining()) {
    return std::nullopt;
  }
  const std::uint8_t* feedback = reader.rest();
  reader.skip(feedback_length);
  const std::uint32_t result_length = reader.u32();
  if (!reader.ok() || result_length > reader.remaining()) {
    return std::nullopt;
  }
  const std::uint8_t* result = reader.rest();
  reader.skip(result_length);
  if (event.has_feedback) {
    event.feedback = codec->decode_feedback(feedback, feedback_length);
    if (!event.feedback.has_value()) {
      return std::nullopt;
    }
  }
  if (event.has_result) {
    event.result = codec->decode_result(result, result_length);
    if (!event.result.has_value()) {
      return std::nullopt;
    }
  }
  event.status = static_cast<GoalStatus>(reader.u8());
  if (!reader.ok()) {
    return std::nullopt;
  }
  return event;
}

}  // namespace krosc::net
