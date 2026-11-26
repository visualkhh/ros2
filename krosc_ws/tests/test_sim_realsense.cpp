#include <cstdint>
#include <memory>
#include <optional>

#include "hardwares/realsense/simulator_realsense.hpp"
#include "krosc/simulator/worlds/maze_world.hpp"
#include "robot/waffle_pi_arm.hpp"

#include "test_framework.hpp"

KROSC_TEST(sim_realsense_renders_color_and_depth) {
  krosc::simulator::MazeWorld world(3, 42);
  auto robot = std::make_unique<krosc_ws::sim::WafflePiArm>(false, 42, KROSC_WS_URDF_PATH);
  auto* model = static_cast<krosc_ws::sim::WafflePiArm*>(world.add(std::move(robot)));
  krosc_ws::sim::SimulatorRealSense camera(*model, false, 42, 64, 48);
  camera.start();
  for (int step = 0; step < 4; ++step) {
    world.step(0.05);
  }

  const auto color = camera.take_color();
  const auto depth = camera.take_depth();
  CHECK(color.has_value());
  CHECK(depth.has_value());
  if (color && depth) {
    CHECK_EQ(64u, color->width);
    CHECK_EQ(48u, color->height);
    CHECK_EQ(std::string("rgb8"), color->encoding);
    CHECK_EQ(64u * 3u, color->step);
    CHECK_EQ(64u * 48u * 3u, color->data.size());
    CHECK_EQ(std::string("16UC1"), depth->encoding);
    CHECK_EQ(64u * 2u, depth->step);
    CHECK_EQ(64u * 48u * 2u, depth->data.size());
    bool has_depth = false;
    for (std::size_t index = 0; index < depth->data.size(); index += 2) {
      const std::uint16_t millimeters =
          static_cast<std::uint16_t>(depth->data[index]) |
          static_cast<std::uint16_t>(depth->data[index + 1] << 8);
      if (millimeters > 0) {
        has_depth = true;
        break;
      }
    }
    CHECK(has_depth);
  }
  CHECK(!camera.take_color().has_value());
  CHECK(!camera.take_depth().has_value());

  const auto stats = camera.stats();
  CHECK(stats.running);
  CHECK(stats.frames > 0);
  CHECK_EQ(std::string("sim"), stats.serial);
}
