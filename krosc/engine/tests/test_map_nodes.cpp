#include <filesystem>
#include <memory>
#include <string>

#include "krosc/context.hpp"
#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/node.hpp"
#include "krosc/nodes/map_catalog.hpp"
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

KROSC_TEST(map_catalog_loads_map_by_name) {
  const fs::path directory = temp_dir("catalog");
  const krosc::msgs::OccupancyGrid grid = make_grid(20, 10);
  krosc::nodes::save_map(grid, (directory / "src" / "map").string());

  krosc::Context context;
  krosc::nodes::MapServer server(context, "map_server",
                                 {{"maps_directory", directory.string()},
                                  {"topic_name", std::string("map")},
                                  {"frame_id", std::string("map")}});
  krosc::nodes::MapCatalog catalog(context, "map_catalog",
                                   {{"maps_directory", directory.string()},
                                    {"map_frame", std::string("map")},
                                    {"map_server", std::string("map_server")},
                                    {"map_saver", std::string("map_saver")},
                                    {"list_period", 10.0}});
  krosc::Node monitor(context, "monitor");
  krosc::msgs::OccupancyGrid latest;
  monitor.subscribe<krosc::msgs::OccupancyGrid>(
      "map", [&](const krosc::msgs::OccupancyGrid& message) { latest = message; },
      krosc::QOS_LATCHED);

  // MapCatalog는 서비스로 이름을 받고, map_server에는 <이름>/map.yaml로 넘겨야 한다.
  const auto loaded = monitor.call_service<krosc::msgs::nav2::LoadMapRequest,
                                           krosc::msgs::nav2::LoadMapResponse>(
      "map_catalog/load_map", krosc::msgs::nav2::LoadMapRequest{"src"});
  CHECK_EQ(krosc::msgs::nav2::LoadMapResponse::RESULT_SUCCESS, loaded.result);
  CHECK_EQ(20u, latest.info.width);
  CHECK_EQ(10u, latest.info.height);
  fs::remove_all(directory);
}

KROSC_TEST(map_catalog_clears_map_name_on_unload) {
  const fs::path directory = temp_dir("catalog_unload");
  const krosc::msgs::OccupancyGrid grid = make_grid(20, 10);
  krosc::nodes::save_map(grid, (directory / "src" / "map").string());

  krosc::Context context;
  krosc::nodes::MapServer server(context, "map_server",
                                 {{"maps_directory", directory.string()},
                                  {"topic_name", std::string("map")},
                                  {"frame_id", std::string("map")}});
  krosc::nodes::MapCatalog catalog(context, "map_catalog",
                                   {{"maps_directory", directory.string()},
                                    {"map_frame", std::string("map")},
                                    {"map_server", std::string("map_server")},
                                    {"map_saver", std::string("map_saver")},
                                    {"list_period", 10.0}});
  krosc::Node monitor(context, "monitor");
  krosc::msgs::String latest;
  monitor.subscribe<krosc::msgs::String>(
      "map_name", [&](const krosc::msgs::String& message) { latest = message; },
      krosc::QOS_LATCHED);

  const auto loaded = monitor.call_service<krosc::msgs::nav2::LoadMapRequest,
                                           krosc::msgs::nav2::LoadMapResponse>(
      "map_catalog/load_map", krosc::msgs::nav2::LoadMapRequest{"src"});
  CHECK_EQ(krosc::msgs::nav2::LoadMapResponse::RESULT_SUCCESS, loaded.result);
  CHECK_EQ(std::string("src"), latest.data);
  // unload 뒤에는 웹/Qt가 옛 지도 이름을 계속 보여주지 않도록 map_name을 비워 발행해야 한다.
  monitor.call_service<krosc::msgs::Empty, krosc::msgs::Empty>(
      "map_catalog/unload_map", krosc::msgs::Empty{});
  CHECK(latest.data.empty());
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

KROSC_TEST(map_catalog_switches_cartographer_mode_parameter) {
  const fs::path directory = temp_dir("mode");
  krosc::Context context;
  krosc::Node cartographer(context, "cartographer_node");
  auto mode = cartographer.declare_parameter<std::string>(
      "mode", std::string("idle"),
      [](const std::string& value) {
        return value == "idle" || value == "mapping" || value == "localization";
      });
  krosc::Node monitor(context, "monitor");
  krosc::nodes::MapCatalog catalog(context, "map_catalog",
                                   {{"maps_directory", directory.string()}});
  const auto response =
      monitor.call_service<krosc::msgs::String, krosc::msgs::Bool>(
          "map_catalog/set_slam_mode", krosc::msgs::String{"mapping"});
  CHECK(response.data);
  CHECK_EQ(std::string("mapping"), mode.value());
  fs::remove_all(directory);
}
