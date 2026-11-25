#include "krosc/viz_web/viz_web_plugins.hpp"

#include <cstdio>

namespace krosc::viz_web {

namespace {

std::string json_escape(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 8);
  for (const unsigned char character : text) {
    switch (character) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (character < 0x20) {
          char escaped[7];
          std::snprintf(escaped, sizeof(escaped), "\\u%04x", character);
          out += escaped;
        } else {
          out += static_cast<char>(character);
        }
        break;
    }
  }
  return out;
}

}  // namespace

std::string VizWebPluginRegistry::manifest_json() const {
  if (plugins_.empty()) {
    return "[]";
  }
  std::string out = "[\n";
  for (std::size_t index = 0; index < plugins_.size(); ++index) {
    const auto& plugin = plugins_[index];
    out += "  { \"name\": \"" + json_escape(plugin.name) + "\", \"title\": \"" +
           json_escape(plugin.title) + "\", \"module\": \"" + json_escape(plugin.module) + "\" }";
    out += index + 1 == plugins_.size() ? "\n" : ",\n";
  }
  out += "]\n";
  return out;
}

void register_core_web_plugins(VizWebPluginRegistry& registry,
                               const std::vector<std::string>& ids) {
  for (const auto& id : ids) {
    if (id == "slam") {
      registry.add(id, "SLAM / Map", "/plugins/slam.js");
    } else if (id == "navigation") {
      registry.add(id, "Navigation", "/plugins/navigation.js");
    } else if (id == "teleop") {
      registry.add(id, "Teleop", "/plugins/teleop.js");
    } else if (id == "diagnostics") {
      registry.add(id, "Diagnostics", "/plugins/diagnostics.js");
    }
  }
}

}  // namespace krosc::viz_web
