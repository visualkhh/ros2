#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/nodes/nav2/controller_server.hpp"
#include "krosc/nodes/nav2/controllers/controller.hpp"
#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"

#include "nodes/rl_nav_controller.hpp"

#include "test_framework.hpp"

KROSC_TEST(rl_nav_plugin_publishes_status_and_diagnostics) {
  krosc::Context context;
  krosc::Node monitor(context, "rl_monitor");
  bool got_status = false;
  bool got_diagnostics = false;
  std::string status;
  krosc::msgs::DiagnosticArray diagnostics;
  monitor.subscribe<krosc::msgs::String>("rl_nav_status",
                                         [&](const krosc::msgs::String& message) {
                                           status = message.data;
                                           got_status = true;
                                         });
  monitor.subscribe<krosc::msgs::DiagnosticArray>(
      "diagnostics", [&](const krosc::msgs::DiagnosticArray& message) {
        diagnostics = message;
        got_diagnostics = true;
      });

  krosc::nodes::nav2::Costmap2DROS local(
      context, "local_costmap",
      {{"global_frame", "odom"},
       {"robot_base_frame", "base_footprint"},
       {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
       {"rolling_window", true},
       {"width", 2.0},
       {"height", 2.0},
       {"resolution", 0.05}});

  krosc::nodes::nav2::ControllerRegistry registry = krosc::nodes::nav2::builtin_controllers();
  registry.add("rl_nav_controller::RlNavController",
               [] { return std::make_unique<krosc_ws::RlNavController>(); });
  krosc::nodes::nav2::ControllerServer server(
      context, local, "controller_server",
      {{"controller_plugins", std::vector<std::string>{"FollowPath"}},
       {"FollowPath.plugin", std::string("rl_nav_controller::RlNavController")},
       {"FollowPath.enable", false},
       {"FollowPath.fallback_controller", std::string("mppi")}},
      registry);

  CHECK(got_status);
  CHECK(got_diagnostics);
  CHECK(status.find("provider=") != std::string::npos);
  CHECK(status.find("onnxruntime=") != std::string::npos);
  CHECK(status.find("fallback=mppi") != std::string::npos);

  bool found = false;
  for (const auto& status_entry : diagnostics.status) {
    if (status_entry.name != "controller_server: RL Navigation") {
      continue;
    }
    found = true;
    std::string provider;
    std::string onnxruntime;
    for (const auto& value : status_entry.values) {
      if (value.key == "provider") {
        provider = value.value;
      } else if (value.key == "onnxruntime") {
        onnxruntime = value.value;
      }
    }
    CHECK(!provider.empty());
#ifdef KROSC_WS_HAS_ONNXRUNTIME
    CHECK_EQ(std::string("true"), onnxruntime);
    CHECK(provider != "-");
#else
    CHECK_EQ(std::string("false"), onnxruntime);
    CHECK_EQ(std::string("-"), provider);
#endif
  }
  CHECK(found);
}
