#include "krosc/viz_web/viz_web_cli.hpp"

#include <cstdio>
#include <stdexcept>
#include <string>

namespace krosc::viz_web {

namespace {

void print_usage(const std::string& program) {
  std::printf(
      "usage: %s [--robot HOST:PORT] [--port PORT] [--bind ADDRESS] [--www DIR]\n",
      program.c_str());
}

bool value_for(int argc, char** argv, int& index, const char* flag, std::string& value,
               std::string& error) {
  if (index + 1 >= argc) {
    error = std::string(flag) + " 값이 필요하다";
    return false;
  }
  value = argv[++index];
  return true;
}

bool port_for(int argc, char** argv, int& index, const char* flag, std::uint16_t& port,
              std::string& error) {
  std::string text;
  if (!value_for(argc, argv, index, flag, text, error)) {
    return false;
  }
  try {
    const int value = std::stoi(text);
    if (value <= 0 || value > 65535) {
      throw std::out_of_range("port");
    }
    port = static_cast<std::uint16_t>(value);
  } catch (...) {
    error = std::string(flag) + " 값이 포트 범위(1..65535)가 아니다: " + text;
    return false;
  }
  return true;
}

void parse_robot(const std::string& text, VizWebOptions& options) {
  const std::size_t colon = text.rfind(':');
  if (colon == std::string::npos) {
    options.robot_host = text;
    return;
  }
  options.robot_host = text.substr(0, colon);
  try {
    const int value = std::stoi(text.substr(colon + 1));
    options.robot_port = value > 0 && value <= 65535 ? static_cast<std::uint16_t>(value) : 9000;
  } catch (...) {
    options.robot_port = 9000;
  }
}

}  // namespace

bool parse_viz_web_options(int argc, char** argv, VizWebOptions& options, bool& help,
                           std::string& error) {
  help = false;
  error.clear();

  std::string program = argc > 0 && argv[0] != nullptr ? argv[0] : "krosc_viz_web";
  const std::size_t slash = program.find_last_of('/');
  if (slash != std::string::npos) {
    program = program.substr(slash + 1);
  }

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--robot") {
      std::string text;
      if (!value_for(argc, argv, index, "--robot", text, error)) {
        return false;
      }
      parse_robot(text, options);
    } else if (argument == "--port") {
      if (!port_for(argc, argv, index, "--port", options.port, error)) {
        return false;
      }
    } else if (argument == "--bind") {
      if (!value_for(argc, argv, index, "--bind", options.bind_address, error)) {
        return false;
      }
    } else if (argument == "--www") {
      if (!value_for(argc, argv, index, "--www", options.www_root, error)) {
        return false;
      }
    } else if (argument == "--help" || argument == "-h") {
      print_usage(program);
      help = true;
      return true;
    } else {
      error = "알 수 없는 인자: " + argument;
      return false;
    }
  }
  return true;
}

}  // namespace krosc::viz_web
