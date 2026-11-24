#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/math.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/nodes/nav2/behavior_server.hpp"
#include "krosc/nodes/nav2/bt_navigator.hpp"
#include "krosc/nodes/nav2/controllers/controller_plugins.hpp"
#include "krosc/nodes/nav2/controllers/rotation_shim_controller.hpp"
#include "krosc/nodes/nav2/controller_server.hpp"
#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"
#include "krosc/nodes/nav2/planner_server.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf_ros.hpp"

#include "test_framework.hpp"

namespace {

using krosc::ParamValue;

constexpr int kMapWidth = 80;
constexpr int kMapHeight = 60;
constexpr double kMapResolution = 0.05;

krosc::msgs::OccupancyGrid make_map() {
  krosc::msgs::OccupancyGrid map;
  map.header = krosc::msgs::Header{0.0, "map"};
  map.info.resolution = kMapResolution;
  map.info.width = kMapWidth;
  map.info.height = kMapHeight;
  map.info.origin.position = {0.0, 0.0, 0.0};
  map.data.assign(static_cast<std::size_t>(kMapWidth) * kMapHeight, 0);
  // x = 1.0m에 벽. y = 0.9~1.5m는 통로.
  const int wall_x = static_cast<int>(1.0 / kMapResolution);
  for (int y = 0; y < kMapHeight; ++y) {
    if (y >= 18 && y < 30) {
      continue;
    }
    map.data[static_cast<std::size_t>(y) * kMapWidth + wall_x] = 100;
  }
  return map;
}

template <class Predicate>
bool wait_until(Predicate predicate, double seconds = 3.0) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                            std::chrono::duration<double>(seconds));
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

krosc::msgs::TransformStamped transform(const std::string& parent, const std::string& child, double x,
                                        double y, double yaw) {
  krosc::msgs::TransformStamped message;
  message.header = krosc::msgs::Header{1.0, parent};
  message.child_frame_id = child;
  message.transform.translation = {x, y, 0.0};
  message.transform.rotation = krosc::tfmath::from_yaw(yaw);
  return message;
}

}  // namespace

KROSC_TEST(nav2_costmap_layers_and_planner_plugins) {
  krosc::Context context;
  krosc::Node map_source(context, "map_source");
  map_source.publish("map", make_map(), krosc::QOS_LATCHED);

  krosc::nodes::nav2::Costmap2DROS global(
      context, "global_costmap",
      {{"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
       {"robot_radius", 0.1},
       {"inflation_radius", 0.3},
       {"cost_scaling_factor", 3.0}});
  CHECK(wait_until([&] { return global.costmap() != nullptr; }));

  const auto costmap = global.costmap();
  CHECK(costmap != nullptr);
  CHECK_EQ(kMapWidth, costmap->width);
  CHECK_EQ(kMapHeight, costmap->height);
  CHECK_NEAR(kMapResolution, costmap->resolution, 1e-9);
  int lethal = 0;
  for (const std::uint8_t value : costmap->costs) {
    if (value == krosc::nodes::nav2::LETHAL_OBSTACLE) {
      ++lethal;
    }
  }
  CHECK(lethal > 0);
  CHECK_EQ(std::int8_t{100}, krosc::nodes::nav2::translate_cost(krosc::nodes::nav2::LETHAL_OBSTACLE));
  CHECK_EQ(std::int8_t{-1}, krosc::nodes::nav2::translate_cost(krosc::nodes::nav2::NO_INFORMATION));

  krosc::nodes::nav2::PlannerServer planner(
      context, global, "planner_server",
      {{"planner_plugins", std::vector<std::string>{"GridBased", "Theta", "Smac2D", "SmacHybrid"}},
       {"GridBased.plugin", "nav2_navfn_planner::NavfnPlanner"},
       {"Theta.plugin", "nav2_theta_star_planner::ThetaStarPlanner"},
       {"Smac2D.plugin", "nav2_smac_planner::SmacPlanner2D"},
       {"SmacHybrid.plugin", "nav2_smac_planner::SmacPlannerHybrid"},
       {"SmacHybrid.minimum_turning_radius", 0.2}});

  krosc::msgs::nav2::ComputePathToPoseGoal request;
  request.goal.header = krosc::msgs::Header{1.0, "map"};
  request.goal.pose.position = {2.5, 1.2, 0.0};
  request.use_start = true;
  request.start.header = krosc::msgs::Header{1.0, "map"};
  request.start.pose.position = {0.3, 1.2, 0.0};
  for (const auto& id : {"GridBased", "Theta", "Smac2D", "SmacHybrid"}) {
    request.planner_id = id;
    const auto result = planner.compute_path(request);
    if (result.error_code != krosc::msgs::nav2::ComputePathToPoseResult::NONE) {
      std::cerr << "planner " << id << " failed: " << result.error_code << " " << result.error_msg << "\n";
    }
    CHECK_EQ(krosc::msgs::nav2::ComputePathToPoseResult::NONE, result.error_code);
    CHECK(result.path.poses.size() > 2);
    CHECK_NEAR(2.5, result.path.poses.back().pose.position.x, 1e-6);
    CHECK_NEAR(1.2, result.path.poses.back().pose.position.y, 1e-6);
  }
}

KROSC_TEST(nav2_controller_plugins_choose_motion) {
  krosc::Context context;
  krosc::Node node(context, "plugin_host");

  krosc::nodes::nav2::Costmap costmap(0.05, 100, 100, 0.0, 0.0);
  std::vector<krosc::nodes::nav2::Pose2> path;
  for (int index = 0; index <= 20; ++index) {
    path.push_back(krosc::nodes::nav2::Pose2{0.1 + 0.05 * index, 0.5, 0.0});
  }

  krosc::nodes::nav2::RegulatedPurePursuitController rpp;
  rpp.configure(node, "FollowPath");
  const auto rpp_command =
      rpp.compute_velocity_commands(krosc::nodes::nav2::Pose2{0.1, 0.5, 0.0},
                                    krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  CHECK(rpp_command.linear > 0.0);
  CHECK_NEAR(0.0, rpp_command.angular, 1e-6);
  CHECK(!rpp.local_plan().empty());

  krosc::nodes::nav2::DWBLocalPlanner dwb;
  dwb.configure(node, "DWB");
  const auto dwb_command =
      dwb.compute_velocity_commands(krosc::nodes::nav2::Pose2{0.1, 0.5, 0.0},
                                    krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  CHECK(dwb_command.linear >= 0.0);
  CHECK(!dwb.local_plan().empty());

  krosc::nodes::nav2::MPPIController mppi;
  mppi.configure(node, "MPPI");
  const auto mppi_command =
      mppi.compute_velocity_commands(krosc::nodes::nav2::Pose2{0.1, 0.5, 0.0},
                                     krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  CHECK(mppi_command.linear <= 0.22 + 1e-9);
  CHECK(std::abs(mppi_command.angular) <= 1.5 + 1e-9);
  CHECK(!mppi.local_plan().empty());
}

KROSC_TEST(nav2_rotation_shim_rotates_then_delegates) {
  krosc::Context context;
  krosc::Node node(context, "shim_host");
  krosc::nodes::nav2::Costmap costmap(0.05, 100, 100, 0.0, 0.0);

  constexpr double kPi = 3.14159265358979323846;
  std::vector<krosc::nodes::nav2::Pose2> path;
  for (int index = 0; index <= 10; ++index) {
    path.push_back(krosc::nodes::nav2::Pose2{0.0, 0.1 * index, kPi / 2.0});
  }

  krosc::nodes::nav2::RotationShimController shim;
  shim.configure(node, "FollowPath");
  const auto rotate = shim.compute_velocity_commands(krosc::nodes::nav2::Pose2{0.0, 0.0, 0.0},
                                                     krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  CHECK_NEAR(0.0, rotate.linear, 1e-12);
  CHECK(rotate.angular > 0.0);
  CHECK(rotate.angular <= 3.2 * 0.05 + 1e-9);

  const auto delegated = shim.compute_velocity_commands(
      krosc::nodes::nav2::Pose2{0.0, 0.0, kPi / 2.0}, krosc::nodes::nav2::Velocity{}, path,
      costmap, 0.05);
  CHECK(delegated.linear > 0.0);

  // 회전에 들어간 뒤에는 angular_disengage_threshold(22.5도)까지만 기다린다.
  shim.compute_velocity_commands(krosc::nodes::nav2::Pose2{0.0, 0.0, 0.0},
                                 krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  const auto hysteresis = shim.compute_velocity_commands(
      krosc::nodes::nav2::Pose2{0.0, 0.0, kPi / 2.0 - 0.5}, krosc::nodes::nav2::Velocity{}, path,
      costmap, 0.05);
  CHECK_NEAR(0.0, hysteresis.linear, 1e-12);
  CHECK(hysteresis.angular > 0.0);
}

KROSC_TEST(nav2_bt_navigator_drives_and_cancels) {
  krosc::Context context;
  krosc::nodes::nav2::Costmap2DROS global(
      context, "global_costmap",
      {{"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
       {"robot_radius", 0.1},
       {"inflation_radius", 0.3},
       {"cost_scaling_factor", 3.0}});
  krosc::nodes::nav2::Costmap2DROS local(
      context, "local_costmap",
      {{"global_frame", "odom"},
       {"robot_base_frame", "base_footprint"},
       {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
       {"rolling_window", true},
       {"width", 2.0},
       {"height", 2.0},
       {"resolution", kMapResolution},
       {"obstacle_max_range", 2.5},
       {"raytrace_max_range", 3.0}});
  krosc::nodes::nav2::PlannerServer planner(context, global, "planner_server",
                                            {{"planner_plugins", std::vector<std::string>{"GridBased"}}});
  krosc::nodes::nav2::ControllerServer controller(
      context, local, "controller_server",
      {{"controller_plugins", std::vector<std::string>{"FollowPath"}},
       {"cmd_vel_topic", "cmd_vel_nav"},
       {"controller_frequency", 20.0},
       {"max_linear_accel", 0.0},
       {"max_angular_accel", 0.0}});
  krosc::nodes::nav2::BehaviorServer behavior(context);
  krosc::nodes::nav2::BtNavigator bt(
      context, "bt_navigator",
      {{"replanning_period", 10.0}, {"number_of_recoveries", 1}});

  krosc::Node monitor(context, "monitor");
  std::atomic<double> linear{0.0};
  std::atomic<double> angular{0.0};
  std::atomic<bool> got_plan{false};
  std::atomic<int> plan_poses{0};
  std::atomic<int> local_plan_poses{0};
  std::atomic<bool> got_costmap{false};
  std::atomic<bool> got_local_costmap{false};
  std::mutex status_mutex;
  std::string last_status;

  monitor.subscribe<krosc::msgs::Twist>(
      "cmd_vel_nav",
      [&](const krosc::msgs::Twist& message) {
        linear.store(message.linear.x);
        angular.store(message.angular.z);
      },
      krosc::QOS_COMMAND);
  monitor.subscribe<krosc::msgs::Path>("plan", [&](const krosc::msgs::Path& message) {
    got_plan.store(true);
    plan_poses.store(static_cast<int>(message.poses.size()));
  });
  monitor.subscribe<krosc::msgs::Path>("local_plan", [&](const krosc::msgs::Path& message) {
    local_plan_poses.store(static_cast<int>(message.poses.size()));
  });
  monitor.subscribe<krosc::msgs::OccupancyGrid>("global_costmap/costmap",
                                                [&](const krosc::msgs::OccupancyGrid& message) {
                                                  got_costmap.store(message.info.width > 0);
                                                });
  monitor.subscribe<krosc::msgs::OccupancyGrid>("local_costmap/costmap",
                                                [&](const krosc::msgs::OccupancyGrid& message) {
                                                  got_local_costmap.store(message.info.width > 0);
                                                });
  monitor.subscribe<krosc::msgs::String>("goal_status", [&](const krosc::msgs::String& message) {
    std::lock_guard<std::mutex> lock(status_mutex);
    last_status = message.data;
  });

  krosc::Node sim(context, "sim");
  krosc::StaticTransformBroadcaster static_tf(sim);
  krosc::TransformBroadcaster dynamic_tf(sim);
  sim.publish("map", make_map(), krosc::QOS_LATCHED);
  static_tf.send_transform(transform("map", "odom", 0.0, 0.0, 0.0));
  dynamic_tf.send_transform(transform("odom", "base_footprint", 0.3, 1.2, 0.0));

  CHECK(wait_until([&] { return got_costmap.load(); }, 2.0));
  CHECK(wait_until([&] { return got_local_costmap.load(); }, 2.0));
  CHECK_EQ(0, local_plan_poses.load());

  krosc::msgs::PoseStamped goal;
  goal.header = krosc::msgs::Header{sim.now(), "map"};
  goal.pose.position = {2.5, 1.2, 0.0};
  sim.publish("goal_pose", goal);
  CHECK(wait_until([&] { return std::abs(linear.load()) > 0.01; }, 3.0));
  CHECK(std::abs(angular.load()) < 2.0);
  CHECK(wait_until([&] { return got_plan.load() && plan_poses.load() > 2; }, 2.0));
  CHECK(wait_until([&] { return local_plan_poses.load() > 0; }, 2.0));

  sim.publish("cancel_goal", krosc::msgs::Bool{true});
  CHECK(wait_until(
      [&] {
        std::lock_guard<std::mutex> lock(status_mutex);
        return last_status == "canceled";
      },
      2.0));
  CHECK(wait_until([&] { return std::abs(linear.load()) < 1e-9; }, 2.0));
}
