#include <chrono>
#include <cmath>
#include <thread>

#include "krosc/tf/math.hpp"
#include "krosc/tf/tf.hpp"

#include "test_framework.hpp"

using namespace std::chrono_literals;

namespace {
krosc::msgs::Transform translation(double x, double y = 0.0, double z = 0.0) {
  krosc::msgs::Transform transform;
  transform.translation = {x, y, z};
  return transform;
}
}  // namespace

KROSC_TEST(tf_math_roundtrip_and_compose) {
  const auto q = krosc::tfmath::from_rpy(0.2, -0.3, 0.7);
  const auto rpy = krosc::tfmath::to_rpy(q);
  CHECK_NEAR(0.2, rpy.roll, 1e-9);
  CHECK_NEAR(-0.3, rpy.pitch, 1e-9);
  CHECK_NEAR(0.7, rpy.yaw, 1e-9);
  const auto point = krosc::tfmath::apply(translation(1.0, 2.0, 3.0), krosc::msgs::Vector3{0.5, -0.5, 0.0});
  CHECK_NEAR(1.5, point.x, 1e-12);
  CHECK_NEAR(1.5, point.y, 1e-12);
  const auto inverse = krosc::tfmath::inverse(translation(1.0, 2.0, 3.0));
  const auto roundtrip = krosc::tfmath::compose(translation(1.0, 2.0, 3.0), inverse);
  CHECK_NEAR(0.0, roundtrip.translation.x, 1e-12);
  CHECK_NEAR(0.0, roundtrip.translation.y, 1e-12);
}

KROSC_TEST(tf_static_and_dynamic_interpolation) {
  krosc::Buffer buffer;
  buffer.set_transform("odom", "base", 0.0, translation(0.0), false);
  buffer.set_transform("odom", "base", 1.0, translation(1.0), false);
  const auto at_half = buffer.lookup_transform("odom", "base", 0.5);
  CHECK_NEAR(0.5, at_half.transform.translation.x, 1e-9);
  CHECK_EQ(std::string("odom"), at_half.header.frame_id);
  CHECK_EQ(std::string("base"), at_half.child_frame_id);
  buffer.set_transform("base", "laser", 0.0, translation(0.1), true);
  const auto laser = buffer.lookup_transform("odom", "laser", 0.25);
  CHECK_NEAR(0.35, laser.transform.translation.x, 1e-9);
  const auto point = buffer.transform_point("odom", "laser", krosc::msgs::Vector3{0.0, 0.0, 0.0}, 0.5);
  CHECK_NEAR(0.6, point.x, 1e-9);
  CHECK(buffer.can_transform("odom", "base"));
  CHECK(!buffer.can_transform("odom", "unknown"));
  CHECK_THROWS_AS(buffer.lookup_transform("odom", "unknown"), krosc::LookupException);
  CHECK_THROWS_AS(buffer.lookup_transform("odom", "base", 3.0), krosc::ExtrapolationException);
}

KROSC_TEST(tf_connectivity_and_cycles) {
  krosc::Buffer buffer;
  buffer.set_transform("map", "odom", 0.0, translation(0.0), true);
  buffer.set_transform("odom", "base", 0.0, translation(0.0), true);
  buffer.set_transform("other", "frame", 0.0, translation(0.0), true);
  CHECK_THROWS_AS(buffer.lookup_transform("map", "frame"), krosc::ConnectivityException);
  CHECK_THROWS_AS(buffer.set_transform("base", "map", 0.0, translation(0.0), true), krosc::ConnectivityException);
  CHECK_THROWS_AS(buffer.set_transform("self", "self", 0.0, translation(0.0), true), std::invalid_argument);
  const auto frames = buffer.all_frames();
  CHECK_EQ(5u, frames.size());
  CHECK(buffer.parent_of("odom").has_value());
  CHECK(!buffer.parent_of("map").has_value());
  CHECK(buffer.all_frames_as_string().find("Frame base exists") != std::string::npos);
  buffer.clear();
  CHECK(buffer.all_frames().empty());
}

KROSC_TEST(tf_timeout_waits_for_transform) {
  krosc::Buffer buffer;
  std::thread publisher([&] {
    std::this_thread::sleep_for(30ms);
    buffer.set_transform("odom", "base", 0.0, translation(0.7), true);
  });
  const auto stamped = buffer.lookup_transform("odom", "base", std::nullopt, 1.0);
  CHECK_NEAR(0.7, stamped.transform.translation.x, 1e-9);
  publisher.join();
}
