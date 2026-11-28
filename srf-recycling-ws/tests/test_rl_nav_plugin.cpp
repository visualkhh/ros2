#include <string>
#include <vector>

#include "srf/msgs.hpp"
#include "srf/nodes/nav2/controller_server.hpp"
#include "srf/nodes/nav2/controllers/controller.hpp"
#include "srf/nodes/nav2/costmap/costmap_2d_ros.hpp"

#include "nodes/rl_nav/rl_nav_controller.hpp"

#include "test_framework.hpp"

SRF_TEST(rl_nav_plugin_publishes_status_and_diagnostics) {
  srf::Context context;
  srf::Node monitor(context, "rl_monitor");
  bool got_status = false;
  bool got_diagnostics = false;
  std::string status;
  srf::msgs::DiagnosticArray diagnostics;
  monitor.subscribe<srf::msgs::String>("rl_nav_status",
                                         [&](const srf::msgs::String& message) {
                                           status = message.data;
                                           got_status = true;
                                         });
  monitor.subscribe<srf::msgs::DiagnosticArray>(
      "diagnostics", [&](const srf::msgs::DiagnosticArray& message) {
        diagnostics = message;
        got_diagnostics = true;
      });

  srf::nodes::nav2::Costmap2DROS local(
      context, "local_costmap",
      {{"global_frame", "odom"},
       {"robot_base_frame", "base_footprint"},
       {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
       {"rolling_window", true},
       {"width", 2.0},
       {"height", 2.0},
       {"resolution", 0.05}});

  srf::nodes::nav2::ControllerRegistry registry = srf::nodes::nav2::builtin_controllers();
  registry.add("rl_nav_controller::RlNavController",
               [] { return std::make_unique<srf_recycling_ws::RlNavController>(); });
  srf::nodes::nav2::ControllerServer server(
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
#ifdef SRF_RECYCLING_WS_HAS_ONNXRUNTIME
    CHECK_EQ(std::string("true"), onnxruntime);
    CHECK(provider != "-");
#else
    CHECK_EQ(std::string("false"), onnxruntime);
    CHECK_EQ(std::string("-"), provider);
#endif
  }
  CHECK(found);
}
