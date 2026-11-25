#include "krosc/viz/viz_cli.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>
#include <vector>

namespace krosc::viz {

namespace {

std::vector<std::string> split_list(const std::string& text) {
  std::vector<std::string> result;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const std::size_t begin = item.find_first_not_of(" \t");
    const std::size_t end = item.find_last_not_of(" \t");
    if (begin != std::string::npos) {
      result.push_back(item.substr(begin, end - begin + 1));
    }
  }
  return result;
}

void ensure_plugin(std::vector<std::string>& plugins, const std::string& id) {
  if (std::find(plugins.begin(), plugins.end(), id) == plugins.end()) {
    plugins.push_back(id);
  }
}

void remove_plugin(std::vector<std::string>& plugins, const std::string& id) {
  plugins.erase(std::remove(plugins.begin(), plugins.end(), id), plugins.end());
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

bool number_for(int argc, char** argv, int& index, const char* flag, double& value,
                std::string& error) {
  std::string text;
  if (!value_for(argc, argv, index, flag, text, error)) {
    return false;
  }
  try {
    value = std::stod(text);
  } catch (...) {
    error = std::string(flag) + " 값이 숫자가 아니다: " + text;
    return false;
  }
  return true;
}

void print_usage(const std::string& program, const VizCliDefaults& defaults) {
  std::printf(
      "usage: %s [--server HOST[:PORT]] [--fixed-frame FRAME] [--tab 3d|map|moveit]\n"
      "       [--plugins a,b,...] [--manual|--teleop|--manipulator|--no-manual] [--no-plugins]\n"
      "       [--distance M] [--screenshot PATH] [--duration SEC]%s\n",
      program.c_str(),
      defaults.usage_extra.empty() ? "" : ("\n       " + defaults.usage_extra).c_str());
}

}  // namespace

bool parse_viz_options(int argc, char** argv, const VizCliDefaults& defaults, VizOptions& options,
                       bool& help, std::string& error) {
  options = VizOptions{};
  options.fixed_frame = defaults.fixed_frame;
  options.tab = defaults.tab;
  std::string plugins_text = defaults.plugins;
  bool teleop = false;
  bool manipulator = false;
  bool no_manual = false;
  help = false;
  error.clear();

  std::string program = argc > 0 && argv[0] != nullptr ? argv[0] : "viz";
  const std::size_t slash = program.find_last_of('/');
  if (slash != std::string::npos) {
    program = program.substr(slash + 1);
  }

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--screenshot") {
      if (!value_for(argc, argv, index, "--screenshot", options.screenshot, error)) {
        return false;
      }
    } else if (argument == "--duration") {
      if (!number_for(argc, argv, index, "--duration", options.duration, error)) {
        return false;
      }
    } else if (argument == "--server") {
      if (!value_for(argc, argv, index, "--server", options.server, error)) {
        return false;
      }
    } else if (argument == "--fixed-frame") {
      if (!value_for(argc, argv, index, "--fixed-frame", options.fixed_frame, error)) {
        return false;
      }
    } else if (argument == "--tab") {
      if (!value_for(argc, argv, index, "--tab", options.tab, error)) {
        return false;
      }
    } else if (argument == "--plugins") {
      if (!value_for(argc, argv, index, "--plugins", plugins_text, error)) {
        return false;
      }
    } else if (argument == "--distance") {
      if (!number_for(argc, argv, index, "--distance", options.distance, error)) {
        return false;
      }
    } else if (argument == "--manual") {
      teleop = true;
      manipulator = true;
    } else if (argument == "--teleop") {
      teleop = true;
    } else if (argument == "--manipulator") {
      manipulator = true;
    } else if (argument == "--no-manual") {
      no_manual = true;
    } else if (argument == "--no-plugins") {
      plugins_text.clear();
    } else if (argument == "--help") {
      print_usage(program, defaults);
      help = true;
      return true;
    }
    // 앱 전용 플래그는 각 앱이 따로 처리한다.
  }

  options.plugins = split_list(plugins_text);
  if (teleop) {
    ensure_plugin(options.plugins, "teleop");
  }
  if (manipulator) {
    ensure_plugin(options.plugins, "manipulator");
  }
  if (no_manual) {
    remove_plugin(options.plugins, "teleop");
    remove_plugin(options.plugins, "manipulator");
  }
  return true;
}

}  // namespace krosc::viz
