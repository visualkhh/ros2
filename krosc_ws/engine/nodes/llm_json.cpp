#include "llm_json.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace krosc_ws::llm {

namespace {

std::string_view trim_view(std::string_view text) {
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())) != 0) {
    text.remove_prefix(1);
  }
  while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())) != 0) {
    text.remove_suffix(1);
  }
  return text;
}

int hex_value(char character) {
  if (character >= '0' && character <= '9') {
    return character - '0';
  }
  if (character >= 'a' && character <= 'f') {
    return character - 'a' + 10;
  }
  if (character >= 'A' && character <= 'F') {
    return character - 'A' + 10;
  }
  return -1;
}

void append_utf8(std::string& out, unsigned code_point) {
  if (code_point <= 0x7F) {
    out.push_back(static_cast<char>(code_point));
  } else if (code_point <= 0x7FF) {
    out.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
    out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
  } else if (code_point <= 0xFFFF) {
    out.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
    out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (code_point >> 18)));
    out.push_back(static_cast<char>(0x80 | ((code_point >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
  }
}

bool parse_string_at(std::string_view text, std::size_t quote, std::string& out,
                     std::size_t& end) {
  if (quote >= text.size() || text[quote] != '"') {
    return false;
  }
  out.clear();
  for (std::size_t i = quote + 1; i < text.size(); ++i) {
    const char character = text[i];
    if (character == '"') {
      end = i + 1;
      return true;
    }
    if (character != '\\') {
      out.push_back(character);
      continue;
    }
    if (++i >= text.size()) {
      return false;
    }
    switch (text[i]) {
      case '"': out.push_back('"'); break;
      case '\\': out.push_back('\\'); break;
      case '/': out.push_back('/'); break;
      case 'b': out.push_back('\b'); break;
      case 'f': out.push_back('\f'); break;
      case 'n': out.push_back('\n'); break;
      case 'r': out.push_back('\r'); break;
      case 't': out.push_back('\t'); break;
      case 'u': {
        if (i + 4 >= text.size()) {
          return false;
        }
        unsigned code_point = 0;
        for (int digit = 1; digit <= 4; ++digit) {
          const int value = hex_value(text[i + static_cast<std::size_t>(digit)]);
          if (value < 0) {
            return false;
          }
          code_point = code_point * 16U + static_cast<unsigned>(value);
        }
        i += 4;
        // UTF-16 surrogate pair
        if (code_point >= 0xD800 && code_point <= 0xDBFF && i + 6 < text.size() &&
            text[i + 1] == '\\' && text[i + 2] == 'u') {
          unsigned low = 0;
          bool valid = true;
          for (int digit = 3; digit <= 6; ++digit) {
            const int value = hex_value(text[i + static_cast<std::size_t>(digit)]);
            if (value < 0) {
              valid = false;
              break;
            }
            low = low * 16U + static_cast<unsigned>(value);
          }
          if (valid && low >= 0xDC00 && low <= 0xDFFF) {
            code_point = 0x10000U + ((code_point - 0xD800U) << 10U) + (low - 0xDC00U);
            i += 6;
          }
        }
        append_utf8(out, code_point);
        break;
      }
      default:
        return false;
    }
  }
  return false;
}

std::optional<std::size_t> value_start(std::string_view json, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\"";
  std::size_t position = 0;
  while ((position = json.find(needle, position)) != std::string_view::npos) {
    std::size_t index = position + needle.size();
    while (index < json.size() &&
           std::isspace(static_cast<unsigned char>(json[index])) != 0) {
      ++index;
    }
    if (index < json.size() && json[index] == ':') {
      ++index;
      while (index < json.size() &&
             std::isspace(static_cast<unsigned char>(json[index])) != 0) {
        ++index;
      }
      return index;
    }
    position += needle.size();
  }
  return std::nullopt;
}

}  // namespace

std::string escape_json(std::string_view text) {
  std::string out;
  out.reserve(text.size() + 8);
  for (const unsigned char character : text) {
    switch (character) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (character < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(character));
          out += buffer;
        } else {
          out.push_back(static_cast<char>(character));
        }
        break;
    }
  }
  return out;
}

std::string strip_code_fence(std::string text) {
  std::string_view view = trim_view(text);
  if (view.rfind("```", 0) == 0) {
    const std::size_t newline = view.find('\n');
    view = newline == std::string_view::npos ? std::string_view{} : view.substr(newline + 1);
    const std::size_t fence = view.rfind("```");
    if (fence != std::string_view::npos) {
      view = view.substr(0, fence);
    }
    view = trim_view(view);
  }
  return std::string(view);
}

std::optional<std::string> string_field(std::string_view json, std::string_view key) {
  const auto start = value_start(json, key);
  if (!start) {
    return std::nullopt;
  }
  std::string out;
  std::size_t end = 0;
  if (!parse_string_at(json, *start, out, end)) {
    return std::nullopt;
  }
  return out;
}

std::optional<double> number_field(std::string_view json, std::string_view key) {
  const auto start = value_start(json, key);
  if (!start) {
    return std::nullopt;
  }
  const std::string rest(json.substr(*start));
  char* end = nullptr;
  const double value = std::strtod(rest.c_str(), &end);
  if (end == rest.c_str() || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

std::optional<std::string> ollama_content(std::string_view response) {
  auto content = string_field(response, "content");
  if (!content) {
    return std::nullopt;
  }
  return strip_code_fence(std::move(*content));
}

std::optional<Plan> parse_plan(std::string_view content) {
  const std::string text = strip_code_fence(std::string(content));
  auto intent = string_field(text, "intent");
  if (!intent || intent->empty()) {
    return std::nullopt;
  }
  Plan plan;
  plan.intent = std::move(*intent);
  if (const auto meters = number_field(text, "meters")) {
    plan.meters = *meters;
  }
  if (const auto degrees = number_field(text, "degrees")) {
    plan.degrees = *degrees;
  }
  if (const auto direction = string_field(text, "direction")) {
    plan.direction = *direction;
  }
  if (const auto preset = string_field(text, "preset")) {
    plan.preset = *preset;
  }
  if (const auto question = string_field(text, "question")) {
    plan.question = *question;
  }
  if (const auto target = string_field(text, "target")) {
    plan.target = *target;
  }
  if (const auto melody = string_field(text, "melody")) {
    plan.melody = *melody;
  }
  if (const auto style = string_field(text, "style")) {
    plan.style = *style;
  }
  return plan;
}

}  // namespace krosc_ws::llm
