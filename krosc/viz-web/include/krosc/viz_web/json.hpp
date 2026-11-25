#pragma once

// 의존성 없는 최소 JSON. 웹 RPC(브라우저 <-> 게이트웨이)에서만 쓴다.
// 토픽 데이터는 기존 바이너리 프레임을 그대로 쓰고, RPC는 저빈도라 JSON이 단순하다.

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace krosc::viz_web {

class Json {
 public:
  using Array = std::vector<Json>;
  using Object = std::map<std::string, Json>;
  enum class Type { Null, Bool, Number, String, Array, Object };

  Json() = default;
  Json(std::nullptr_t) {}
  Json(bool value) : value_(value) {}
  Json(int value) : value_(static_cast<double>(value)) {}
  Json(std::int64_t value) : value_(static_cast<double>(value)) {}
  Json(double value) : value_(value) {}
  Json(const char* value) : value_(std::string(value == nullptr ? "" : value)) {}
  Json(std::string value) : value_(std::move(value)) {}

  static Json object();
  static Json array();
  static std::optional<Json> parse(std::string_view text);

  [[nodiscard]] Type type() const;
  [[nodiscard]] bool is_null() const { return type() == Type::Null; }
  [[nodiscard]] bool is_bool() const { return type() == Type::Bool; }
  [[nodiscard]] bool is_number() const { return type() == Type::Number; }
  [[nodiscard]] bool is_string() const { return type() == Type::String; }
  [[nodiscard]] bool is_array() const { return type() == Type::Array; }
  [[nodiscard]] bool is_object() const { return type() == Type::Object; }

  [[nodiscard]] bool as_bool(bool fallback = false) const;
  [[nodiscard]] double as_double(double fallback = 0.0) const;
  [[nodiscard]] std::int64_t as_int(std::int64_t fallback = 0) const;
  [[nodiscard]] std::string as_string(std::string fallback = {}) const;
  [[nodiscard]] const Array& as_array() const;
  [[nodiscard]] const Object& as_object() const;

  [[nodiscard]] bool contains(const std::string& key) const;
  [[nodiscard]] const Json& operator[](const std::string& key) const;
  Json& operator[](const std::string& key);
  void push(Json value);

  [[nodiscard]] std::string dump() const;

 private:
  std::variant<std::nullptr_t, bool, double, std::string, Array, Object> value_{nullptr};
};

}  // namespace krosc::viz_web
