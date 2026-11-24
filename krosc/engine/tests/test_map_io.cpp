#include <filesystem>
#include <string>

#include "krosc/nodes/map_io.hpp"

#include "test_framework.hpp"

KROSC_TEST(map_io_save_and_load_roundtrip) {
  krosc::msgs::OccupancyGrid grid;
  grid.header = krosc::msgs::Header{1.0, "map"};
  grid.info.resolution = 0.05;
  grid.info.width = 3;
  grid.info.height = 2;
  grid.info.origin.position = {-1.0, 2.0, 0.0};
  grid.info.origin.orientation.w = 1.0;
  grid.data = {0, 100, -1, 50, 0, -1};
  const auto directory = std::filesystem::temp_directory_path() / "krosc_map_io_test";
  std::filesystem::remove_all(directory);
  const std::string yaml = krosc::nodes::save_map(grid, (directory / "map").string());
  CHECK(std::filesystem::exists(yaml));
  CHECK(std::filesystem::exists(directory / "map.pgm"));
  const auto loaded = krosc::nodes::load_map(yaml, "map", 2.0);
  CHECK_EQ(loaded.info.width, 3U);
  CHECK_EQ(loaded.info.height, 2U);
  CHECK_NEAR(loaded.info.resolution, 0.05, 1e-9);
  CHECK_NEAR(loaded.info.origin.position.x, -1.0, 1e-9);
  CHECK_NEAR(loaded.info.origin.position.y, 2.0, 1e-9);
  CHECK_EQ(loaded.data.size(), static_cast<std::size_t>(6));
  CHECK(loaded.data[1] >= 50);   // 벽
  CHECK(loaded.data[0] == 0);    // 빈 곳
  CHECK(loaded.data[2] < 0);     // 모름
  std::filesystem::remove_all(directory);
}
