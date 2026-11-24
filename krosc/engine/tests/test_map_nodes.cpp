#include <filesystem>
#include <memory>
#include <string>

#include "krosc/context.hpp"
#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/node.hpp"
#include "krosc/nodes/map_io.hpp"
#include "krosc/nodes/map_saver.hpp"
#include "krosc/nodes/map_server.hpp"
#include "krosc/qos.hpp"

#include "test_framework.hpp"

namespace {

namespace fs = std::filesystem;

krosc::msgs::OccupancyGrid make_grid(int width, int height) {
  krosc::msgs::OccupancyGrid grid;
  grid.header = krosc::msgs::Header{0.0, "map"};
  grid.info.resolution = 0.05;
  grid.info.width = static_cast<std::uint32_t>(width);
  grid.info.height = static_cast<std::uint32_t>(height);
  grid.info.origin.position = {0.0, 0.0, 0.0};
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * height, 0);
  for (int y = 0; y < height; ++y) {
    grid.data[static_cast<std::size_t>(y) * width + width / 2] = 100;
  }
  return grid;
}

fs::path temp_dir(const std::string& tag) {
  const fs::path path = fs::temp_directory_path() / ("krosc_map_nodes_" + tag);
  fs::remove_all(path);
  fs::create_directories(path);
  return path;
}

}  // namespace

KROSC_TEST(map_server_loads_and_unloads) {
  const fs::path directory = temp_dir("server");
  const krosc::msgs::OccupancyGrid grid = make_grid(20, 10);
  krosc::nodes::save_map(grid, (directory / "src" / "map").string());

  krosc::Context context;
  krosc::nodes::MapServer server(context, "map_server",
                                 {{"maps_directory", directory.string()},
                                  {"topic_name", std::string("map")},
                                  {"frame_id", std::string("map")}});
  krosc::Node monitor(context, "monitor");
  krosc::msgs::OccupancyGrid latest;
  monitor.subscribe<krosc::msgs::OccupancyGrid>(
      "map", [&](const krosc::msgs::OccupancyGrid& message) { latest = message; },
      krosc::QOS_LATCHED);

  krosc::msgs::nav2::LoadMapRequest request;
  request.map_url = "src/map.yaml";
  const auto response =
      monitor.call_service<krosc::msgs::nav2::LoadMapRequest, krosc::msgs::nav2::LoadMapResponse>(
          "map_server/load_map", request);
  CHECK_EQ(krosc::msgs::nav2::LoadMapResponse::RESULT_SUCCESS, response.result);
  CHECK_EQ(20u, latest.info.width);
  CHECK_EQ(10u, latest.info.height);

  monitor.call_service<krosc::msgs::Empty, krosc::msgs::Empty>("map_server/unload_map",
                                                              krosc::msgs::Empty{});
  CHECK_EQ(0u, latest.info.width);
  fs::remove_all(directory);
}

KROSC_TEST(map_saver_saves_latched_map) {
  const fs::path directory = temp_dir("saver");
  krosc::Context context;
  krosc::nodes::MapSaver saver(context, "map_saver", {{"maps_directory", directory.string()}});
  krosc::Node source(context, "source");
  source.publish("map", make_grid(16, 12), krosc::QOS_LATCHED);

  krosc::msgs::nav2::SaveMapRequest request;
  request.map_url = "saved/map";
  request.map_topic = "map";
  const auto response =
      source.call_service<krosc::msgs::nav2::SaveMapRequest, krosc::msgs::nav2::SaveMapResponse>(
          "map_saver/save_map", request);
  CHECK(response.result);
  CHECK(fs::exists(directory / "saved" / "map.yaml"));
  CHECK(fs::exists(directory / "saved" / "map.pgm"));
  const auto loaded = krosc::nodes::load_map((directory / "saved" / "map.yaml").string());
  CHECK_EQ(16u, loaded.info.width);
  CHECK_EQ(12u, loaded.info.height);
  fs::remove_all(directory);
}
