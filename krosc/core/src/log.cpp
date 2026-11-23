#include "krosc/log.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <utility>

namespace krosc {

std::string_view log_level_name(LogLevel level) {
  switch (level) {
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Error:
      return "ERROR";
    case LogLevel::Fatal:
      return "FATAL";
  }
  return "INFO";
}

std::optional<LogLevel> parse_log_level(std::string_view text) {
  std::string upper(text);
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  if (upper == "DEBUG") {
    return LogLevel::Debug;
  }
  if (upper == "INFO") {
    return LogLevel::Info;
  }
  if (upper == "WARN" || upper == "WARNING") {
    return LogLevel::Warn;
  }
  if (upper == "ERROR") {
    return LogLevel::Error;
  }
  if (upper == "FATAL" || upper == "CRITICAL") {
    return LogLevel::Fatal;
  }
  return std::nullopt;
}

NodeLogger::NodeLogger(std::string name, Publish publish, Now now, LevelFn level)
    : name_(std::move(name)), publish_(std::move(publish)), now_(std::move(now)), level_(std::move(level)) {}

void NodeLogger::write(LogLevel level, std::string message, const std::source_location& location) {
  if (!enabled_for(level)) {
    return;
  }
  std::fprintf(stderr, "[%s] [%s]: %s\n", std::string(log_level_name(level)).c_str(), name_.c_str(), message.c_str());
  if (publish_) {
    msgs::Log log;
    log.stamp = now_ ? now_() : 0.0;
    log.level = static_cast<std::int8_t>(level);
    log.name = name_;
    log.msg = std::move(message);
    log.file = location.file_name();
    log.function = location.function_name();
    log.line = location.line();
    publish_(log);
  }
}

}  // namespace krosc
