#include <memory>

#include "krosc/moveit/controllers/controller_manager.hpp"
#include "krosc/moveit/controllers/simple_controller_manager.hpp"

namespace krosc::moveit {

ControllerManagerRegistry builtin_controller_managers() {
  ControllerManagerRegistry registry;
  registry.add("krosc_moveit/SimpleControllerManager",
               [] { return std::make_unique<SimpleControllerManager>(); });
  return registry;
}

}  // namespace krosc::moveit
