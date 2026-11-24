#include "krosc/nodes/nav2/controllers/controller_plugins.hpp"

#include <memory>

#include "krosc/nodes/nav2/controllers/dwb_local_planner.hpp"
#include "krosc/nodes/nav2/controllers/mppi_controller.hpp"
#include "krosc/nodes/nav2/controllers/regulated_pure_pursuit_controller.hpp"
#include "krosc/nodes/nav2/controllers/rotation_shim_controller.hpp"

namespace krosc::nodes::nav2 {

ControllerRegistry builtin_controllers() {
  ControllerRegistry registry;
  registry.add("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
               [] { return std::make_unique<RegulatedPurePursuitController>(); });
  registry.add("dwb_core::DWBLocalPlanner", [] { return std::make_unique<DWBLocalPlanner>(); });
  registry.add("nav2_mppi_controller::MPPIController", [] { return std::make_unique<MPPIController>(); });
  registry.add("nav2_rotation_shim_controller::RotationShimController",
               [] { return std::make_unique<RotationShimController>(); });
  return registry;
}

}  // namespace krosc::nodes::nav2
