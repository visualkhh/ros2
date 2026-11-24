#pragma once

#include <cstdint>
#include <cstdio>
#include <functional>
#include <optional>
#include <source_location>
#include <string>
#include <string_view>
#include <utility>

#include "krosc/msgs.hpp"

namespace krosc {

enum class LogLevel : std::int8_t {
  Debug = 10,
  Info = 20,
  Warn = 30,
  Error = 40,
  Fatal = 50,
};

[[nodiscard]] std::string_view log_level_name(LogLevel level);
[[nodiscard]] std::optional<LogLevel> parse_log_level(std::string_view text);

inline constexpr const char* ROSOUT_TOPIC_NAME = "rosout";

// 노드의 로거: 콘솔(stderr)과 rosout 토픽으로 같이 내보낸다. 수준 미만은 어디에도 나가지 않는다.
// 메시지는 printf 형식이다: logger.info("v=%.2f", v). (std::string을 넘길 때는 .c_str()을 쓴다)
class NodeLogger {
 public:
  using Publish = std::function<void(const msgs::Log&)>;
  using Now = std::function<double()>;
  using LevelFn = std::function<LogLevel()>;

  NodeLogger(std::string name, Publish publish, Now now, LevelFn level);

  template <class... Args>
  void debug(const char* format, Args&&... args) {
    if (!enabled_for(LogLevel::Debug)) {
      return;
    }
    write(LogLevel::Debug, format_string(format, std::forward<Args>(args)...), std::source_location::current());
  }

  template <class... Args>
  void info(const char* format, Args&&... args) {
    if (!enabled_for(LogLevel::Info)) {
      return;
    }
    write(LogLevel::Info, format_string(format, std::forward<Args>(args)...), std::source_location::current());
  }

  template <class... Args>
  void warn(const char* format, Args&&... args) {
    if (!enabled_for(LogLevel::Warn)) {
      return;
    }
    write(LogLevel::Warn, format_string(format, std::forward<Args>(args)...), std::source_location::current());
  }

  template <class... Args>
  void error(const char* format, Args&&... args) {
    if (!enabled_for(LogLevel::Error)) {
      return;
    }
    write(LogLevel::Error, format_string(format, std::forward<Args>(args)...), std::source_location::current());
  }

  template <class... Args>
  void fatal(const char* format, Args&&... args) {
    write(LogLevel::Fatal, format_string(format, std::forward<Args>(args)...), std::source_location::current());
  }

  [[nodiscard]] bool enabled_for(LogLevel level) const { return level >= level_(); }
  [[nodiscard]] const std::string& name() const { return name_; }

 private:
  template <class... Args>
  static std::string format_string(const char* format, Args&&... args) {
    if constexpr (sizeof...(Args) == 0) {
      return std::string(format ? format : "");
    } else {
      const int size = std::snprintf(nullptr, 0, format, args...);
      if (size <= 0) {
        return std::string(format ? format : "");
      }
      std::string result(static_cast<std::size_t>(size), '\0');
      std::snprintf(result.data(), result.size() + 1, format, args...);
      return result;
    }
  }

  void write(LogLevel level, std::string message, const std::source_location& location);

  std::string name_;
  Publish publish_;
  Now now_;
  LevelFn level_;
};

}  // namespace krosc
