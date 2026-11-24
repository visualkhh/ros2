#include <memory>

#include "krosc/moveit/controller_manager.hpp"
#include "krosc/moveit/simple_controller_manager.hpp"

namespace krosc::moveit {

ControllerManagerRegistry builtin_controller_managers() {
  ControllerManagerRegistry registry;
  registry.add("krosc_moveit/SimpleControllerManager",
               [] { return std::make_unique<SimpleControllerManager>(); });
  return registry;
}

}  // namespace krosc::moveit
