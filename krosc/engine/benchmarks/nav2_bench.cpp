#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/node.hpp"
#include "krosc/nodes/nav2/controllers/controller_plugins.hpp"
#include "krosc/nodes/nav2/costmap/costmap.hpp"
#include "krosc/nodes/nav2/planners/planner_plugins.hpp"

namespace {

constexpr int kSize = 200;
constexpr double kResolution = 0.05;

krosc::msgs::OccupancyGrid make_map() {
  krosc::msgs::OccupancyGrid grid;
  grid.header = krosc::msgs::Header{0.0, "map"};
  grid.info.resolution = kResolution;
  grid.info.width = kSize;
  grid.info.height = kSize;
  grid.info.origin.position = {0.0, 0.0, 0.0};
  grid.data.assign(static_cast<std::size_t>(kSize) * kSize, 0);
  const auto wall = [&](int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        grid.data[static_cast<std::size_t>(y) * kSize + x] = 100;
      }
    }
  };
  wall(0, 0, kSize - 1, 2);
  wall(0, kSize - 3, kSize - 1, kSize - 1);
  wall(0, 0, 2, kSize - 1);
  wall(kSize - 3, 0, kSize - 1, kSize - 1);
  for (int y = 20; y < 170; y += 30) {
    wall(20, y, 170, y + 1);
  }
  wall(100, 20, 101, 110);
  wall(100, 140, 101, 180);
  return grid;
}

template <class Function>
void measure(const std::string& label, int iterations, Function function) {
  const auto start = std::chrono::steady_clock::now();
  for (int index = 0; index < iterations; ++index) {
    function();
  }
  const double total =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  std::printf("%-24s %8.3f ms/op  (%d ops, %.1f ms)\n", label.c_str(), total / iterations,
              iterations, total);
}

}  // namespace

int main() {
  krosc::Context context;
  krosc::nodes::nav2::Costmap costmap =
      krosc::nodes::nav2::Costmap::from_occupancy_grid(make_map(), false);
  krosc::Node node(context, "nav2_bench", {{"GridBased.use_astar", true}});
  const krosc::nodes::nav2::Pose2 start{0.5, 0.5, 0.0};
  const krosc::nodes::nav2::Pose2 goal{9.5, 9.5, 0.0};

  measure("costmap inflate", 50, [&] {
    auto copy = krosc::nodes::nav2::Costmap::from_occupancy_grid(make_map(), false);
    copy.inflate(0.2, 0.55, 3.0);
  });

  krosc::nodes::nav2::NavfnPlanner navfn;
  navfn.configure(node, "GridBased");
  navfn.create_plan(costmap, start, goal);
  measure("Navfn (A*)", 100, [&] { navfn.create_plan(costmap, start, goal); });

  krosc::nodes::nav2::ThetaStarPlanner theta;
  theta.configure(node, "Theta");
  theta.create_plan(costmap, start, goal);
  measure("Theta*", 50, [&] { theta.create_plan(costmap, start, goal); });

  krosc::nodes::nav2::SmacPlanner2D smac2d;
  smac2d.configure(node, "Smac2D");
  smac2d.create_plan(costmap, start, goal);
  measure("Smac2D", 50, [&] { smac2d.create_plan(costmap, start, goal); });

  krosc::nodes::nav2::SmacPlannerHybrid smac_hybrid;
  smac_hybrid.configure(node, "SmacHybrid");
  smac_hybrid.create_plan(costmap, start, goal);
  measure("SmacHybrid", 10, [&] { smac_hybrid.create_plan(costmap, start, goal); });

  const auto points = navfn.create_plan(costmap, start, goal);
  std::vector<krosc::nodes::nav2::Pose2> path;
  path.reserve(points.size());
  for (const auto& point : points) {
    path.push_back({point.first, point.second, 0.0});
  }

  krosc::nodes::nav2::RegulatedPurePursuitController rpp;
  rpp.configure(node, "FollowPath");
  measure("RPP", 2000, [&] {
    rpp.compute_velocity_commands(start, krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  });

  krosc::nodes::nav2::DWBLocalPlanner dwb;
  dwb.configure(node, "DWB");
  measure("DWB", 100, [&] {
    dwb.compute_velocity_commands(start, krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  });

  krosc::nodes::nav2::MPPIController mppi;
  mppi.configure(node, "MPPI");
  measure("MPPI", 100, [&] {
    mppi.compute_velocity_commands(start, krosc::nodes::nav2::Velocity{}, path, costmap, 0.05);
  });
  return 0;
}
