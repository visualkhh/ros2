#include "krosc/viz_web/json.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace krosc::viz_web {

namespace {

void append_utf8(std::string& out, std::uint32_t code) {
  if (code <= 0x7F) {
    out.push_back(static_cast<char>(code));
  } else if (code <= 0x7FF) {
    out.push_back(static_cast<char>(0xC0 | (code >> 6)));
    out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
  } else if (code <= 0xFFFF) {
    out.push_back(static_cast<char>(0xE0 | (code >> 12)));
    out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (code >> 18)));
    out.push_back(static_cast<char>(0x80 | ((code >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
  }
}

class Parser {
 public:
  explicit Parser(std::string_view text) : text_(text) {}

  std::optional<Json> parse() {
    skip();
    auto value = parse_value(0);
    skip();
    if (!value || position_ != text_.size()) {
      return std::nullopt;
    }
    return value;
  }

 private:
  void skip() {
    while (position_ < text_.size()) {
      const char c = text_[position_];
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        ++position_;
      } else {
        break;
      }
    }
  }

  bool consume(char expected) {
    if (position_ < text_.size() && text_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  std::optional<Json> parse_value(int depth) {
    if (depth > 32 || position_ >= text_.size()) {
      return std::nullopt;
    }
    const char c = text_[position_];
    if (c == '{') {
      return parse_object(depth + 1);
    }
    if (c == '[') {
      return parse_array(depth + 1);
    }
    if (c == '"') {
      auto value = parse_string();
      if (!value) {
        return std::nullopt;
      }
      return Json(std::move(*value));
    }
    if (c == 't' && text_.substr(position_, 4) == "true") {
      position_ += 4;
      return Json(true);
    }
    if (c == 'f' && text_.substr(position_, 5) == "false") {
      position_ += 5;
      return Json(false);
    }
    if (c == 'n' && text_.substr(position_, 4) == "null") {
      position_ += 4;
      return Json();
    }
    return parse_number();
  }

  std::optional<Json> parse_object(int depth) {
    if (!consume('{')) {
      return std::nullopt;
    }
    Json object = Json::object();
    skip();
    if (consume('}')) {
      return object;
    }
    for (;;) {
      skip();
      auto key = parse_string();
      if (!key) {
        return std::nullopt;
      }
      skip();
      if (!consume(':')) {
        return std::nullopt;
      }
      skip();
      auto value = parse_value(depth);
      if (!value) {
        return std::nullopt;
      }
      object[*key] = std::move(*value);
      skip();
      if (consume('}')) {
        return object;
      }
      if (!consume(',')) {
        return std::nullopt;
      }
    }
  }

  std::optional<Json> parse_array(int depth) {
    if (!consume('[')) {
      return std::nullopt;
    }
    Json array = Json::array();
    skip();
    if (consume(']')) {
      return array;
    }
    for (;;) {
      skip();
      auto value = parse_value(depth);
      if (!value) {
        return std::nullopt;
      }
      array.push(std::move(*value));
      skip();
      if (consume(']')) {
        return array;
      }
      if (!consume(',')) {
        return std::nullopt;
      }
    }
  }

  std::optional<std::string> parse_string() {
    if (!consume('"')) {
      return std::nullopt;
    }
    std::string out;
    while (position_ < text_.size()) {
      const char c = text_[position_++];
      if (c == '"') {
        return out;
      }
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (position_ >= text_.size()) {
        return std::nullopt;
      }
      const char escape = text_[position_++];
      switch (escape) {
        case '"':
          out.push_back('"');
          break;
        case '\\':
          out.push_back('\\');
          break;
        case '/':
          out.push_back('/');
          break;
        case 'b':
          out.push_back('\b');
          break;
        case 'f':
          out.push_back('\f');
          break;
        case 'n':
          out.push_back('\n');
          break;
        case 'r':
          out.push_back('\r');
          break;
        case 't':
          out.push_back('\t');
          break;
        case 'u': {
          if (position_ + 4 > text_.size()) {
            return std::nullopt;
          }
          std::uint32_t code = 0;
          for (int i = 0; i < 4; ++i) {
            const char digit = text_[position_++];
            code <<= 4;
            if (digit >= '0' && digit <= '9') {
              code |= static_cast<std::uint32_t>(digit - '0');
            } else if (digit >= 'a' && digit <= 'f') {
              code |= static_cast<std::uint32_t>(digit - 'a' + 10);
            } else if (digit >= 'A' && digit <= 'F') {
              code |= static_cast<std::uint32_t>(digit - 'A' + 10);
            } else {
              return std::nullopt;
            }
          }
          append_utf8(out, code);
          break;
        }
        default:
          return std::nullopt;
      }
    }
    return std::nullopt;
  }

  std::optional<Json> parse_number() {
    const std::size_t start = position_;
    if (position_ < text_.size() && text_[position_] == '-') {
      ++position_;
    }
    while (position_ < text_.size() &&
           ((text_[position_] >= '0' && text_[position_] <= '9') || text_[position_] == '.' ||
            text_[position_] == 'e' || text_[position_] == 'E' || text_[position_] == '+' ||
            text_[position_] == '-')) {
      ++position_;
    }
    if (start == position_) {
      return std::nullopt;
    }
    const std::string number(text_.substr(start, position_ - start));
    char* end = nullptr;
    const double value = std::strtod(number.c_str(), &end);
    if (end == nullptr || *end != '\0' || !std::isfinite(value)) {
      return std::nullopt;
    }
    return Json(value);
  }

  std::string_view text_;
  std::size_t position_ = 0;
};

void dump_string(std::ostringstream& out, const std::string& text) {
  out << '"';
  for (const unsigned char c : text) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec;
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  out << '"';
}

void dump_value(std::ostringstream& out, const Json& value) {
  switch (value.type()) {
    case Json::Type::Null:
      out << "null";
      break;
    case Json::Type::Bool:
      out << (value.as_bool() ? "true" : "false");
      break;
    case Json::Type::Number: {
      const double number = value.as_double();
      if (std::floor(number) == number && std::abs(number) < 1e15) {
        out << static_cast<long long>(number);
      } else {
        out << std::setprecision(15) << number;
      }
      break;
    }
    case Json::Type::String:
      dump_string(out, value.as_string());
      break;
    case Json::Type::Array: {
      out << '[';
      bool first = true;
      for (const auto& item : value.as_array()) {
        if (!first) {
          out << ',';
        }
        first = false;
        dump_value(out, item);
      }
      out << ']';
      break;
    }
    case Json::Type::Object: {
      out << '{';
      bool first = true;
      for (const auto& [key, item] : value.as_object()) {
        if (!first) {
          out << ',';
        }
        first = false;
        dump_string(out, key);
        out << ':';
        dump_value(out, item);
      }
      out << '}';
      break;
    }
  }
}

}  // namespace

Json Json::object() {
  Json value;
  value.value_ = Object{};
  return value;
}

Json Json::array() {
  Json value;
  value.value_ = Array{};
  return value;
}

std::optional<Json> Json::parse(std::string_view text) { return Parser(text).parse(); }

Json::Type Json::type() const {
  switch (value_.index()) {
    case 0:
      return Type::Null;
    case 1:
      return Type::Bool;
    case 2:
      return Type::Number;
    case 3:
      return Type::String;
    case 4:
      return Type::Array;
    default:
      return Type::Object;
  }
}

bool Json::as_bool(bool fallback) const {
  return std::holds_alternative<bool>(value_) ? std::get<bool>(value_) : fallback;
}

double Json::as_double(double fallback) const {
  return std::holds_alternative<double>(value_) ? std::get<double>(value_) : fallback;
}

std::int64_t Json::as_int(std::int64_t fallback) const {
  if (!std::holds_alternative<double>(value_)) {
    return fallback;
  }
  return static_cast<std::int64_t>(std::llround(std::get<double>(value_)));
}

std::string Json::as_string(std::string fallback) const {
  return std::holds_alternative<std::string>(value_) ? std::get<std::string>(value_)
                                                     : std::move(fallback);
}

const Json::Array& Json::as_array() const {
  static const Array empty;
  return std::holds_alternative<Array>(value_) ? std::get<Array>(value_) : empty;
}

const Json::Object& Json::as_object() const {
  static const Object empty;
  return std::holds_alternative<Object>(value_) ? std::get<Object>(value_) : empty;
}

bool Json::contains(const std::string& key) const {
  return is_object() && std::get<Object>(value_).count(key) != 0;
}

const Json& Json::operator[](const std::string& key) const {
  static const Json null;
  if (!is_object()) {
    return null;
  }
  const auto& object = std::get<Object>(value_);
  const auto found = object.find(key);
  return found == object.end() ? null : found->second;
}

Json& Json::operator[](const std::string& key) {
  if (!is_object()) {
    value_ = Object{};
  }
  return std::get<Object>(value_)[key];
}

void Json::push(Json value) {
  if (!is_array()) {
    value_ = Array{};
  }
  std::get<Array>(value_).push_back(std::move(value));
}

std::string Json::dump() const {
  std::ostringstream out;
  dump_value(out, *this);
  return out.str();
}

}  // namespace krosc::viz_web
