#include "krosc/viz/plugins/registry.hpp"

#include <memory>

#include "krosc/viz/plugins/manipulator_plugin.hpp"
#include "krosc/viz/plugins/navigation_plugin.hpp"
#include "krosc/viz/plugins/slam_plugin.hpp"
#include "krosc/viz/plugins/teleop_plugin.hpp"

namespace krosc::viz {

void register_core_plugins(VizPluginRegistry& registry, const std::vector<std::string>& ids) {
  for (const auto& id : ids) {
    if (id == "teleop") {
      registry.add(id, [] { return std::make_unique<TeleopPlugin>(); });
    } else if (id == "slam") {
      registry.add(id, [] { return std::make_unique<SlamPlugin>(); });
    } else if (id == "navigation") {
      registry.add(id, [] { return std::make_unique<NavigationPlugin>(); });
    } else if (id == "manipulator") {
      registry.add(id, [] { return std::make_unique<ManipulatorPlugin>(); });
    }
  }
}

}  // namespace krosc::viz
