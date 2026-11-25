#include "krosc/viz/plugins/registry.hpp"

#include <memory>

#include "krosc/viz/plugins/manipulator_plugin.hpp"
#include "krosc/viz/plugins/navigation_plugin.hpp"
#include "krosc/viz/plugins/slam_plugin.hpp"
#include "krosc/viz/plugins/teleop_plugin.hpp"

namespace krosc::viz {

void register_core_plugins(VizPluginRegistry& registry) {
  registry.add("teleop", [] { return std::make_unique<TeleopPlugin>(); });
  registry.add("slam", [] { return std::make_unique<SlamPlugin>(); });
  registry.add("navigation", [] { return std::make_unique<NavigationPlugin>(); });
  registry.add("manipulator", [] { return std::make_unique<ManipulatorPlugin>(); });
}

}  // namespace krosc::viz
