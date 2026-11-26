#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "krosc/context.hpp"
#include "krosc/tf/math.hpp"
#include "krosc/nodes/diff_drive_odometry.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/simulator/simulator.hpp"
#include "krosc/simulator/worlds/maze_world.hpp"
#include "krosc/simulator/runner.hpp"
#include "krosc/simulator/objects/turtlebot3.hpp"
#include "krosc/tf/tf_ros.hpp"
#include "krosc/time/time.hpp"

#include "test_framework.hpp"

namespace {

class Bridge final : public krosc::Node {
 public:
  Bridge(krosc::Context& context, krosc::simulator::TurtleBot3& robot)
      : Node(context, "TestBridge"), robot_(robot) {
    robot_.set_scan_callback([this](const krosc::msgs::LaserScan& scan) {
      publish("scan", scan, krosc::QOS_SENSOR_DATA);
    });
    robot_.set_imu_callback([this](const krosc::msgs::Imu& imu) {
      publish("imu", imu, krosc::QOS_SENSOR_DATA);
    });
    timer_ = create_timer(0.05, [this] {
      const auto positions = robot_.joint_positions();
      krosc::msgs::JointState state;
      state.header.stamp = now();
      for (const auto& [name, position] : positions) {
        state.name.push_back(name);
        state.position.push_back(position);
      }
      publish(krosc::JOINT_STATES_TOPIC_NAME, std::move(state));
    });
  }

 private:
  krosc::simulator::TurtleBot3& robot_;
  krosc::Timer timer_;
};

class Recorder final : public krosc::Node {
 public:
  explicit Recorder(krosc::Context& context) : Node(context, "Recorder") {
    subscribe<krosc::msgs::Odometry>("odom", [this](const krosc::msgs::Odometry& message) {
      std::lock_guard<std::mutex> lock(mutex_);
      odom = message;
    });
    subscribe<krosc::msgs::Odometry>("odometry/filtered", [this](const krosc::msgs::Odometry& message) {
      std::lock_guard<std::mutex> lock(mutex_);
      filtered = message;
    });
    subscribe<krosc::msgs::LaserScan>("scan", [this](const krosc::msgs::LaserScan& message) {
      std::lock_guard<std::mutex> lock(mutex_);
      scan = message;
    });
    subscribe<krosc::msgs::Imu>("imu", [this](const krosc::msgs::Imu& message) {
      std::lock_guard<std::mutex> lock(mutex_);
      imu = message;
    });
  }

  std::optional<krosc::msgs::Odometry> odom;
  std::optional<krosc::msgs::Odometry> filtered;
  std::optional<krosc::msgs::LaserScan> scan;
  std::optional<krosc::msgs::Imu> imu;

 private:
  std::mutex mutex_;
};

}  // namespace

KROSC_TEST(world_rays_and_straight_motion) {
  krosc::simulator::MazeWorld world(0);
  auto robot_holder = std::make_unique<krosc::simulator::TurtleBot3>();
  auto* actual = static_cast<krosc::simulator::TurtleBot3*>(world.add(std::move(robot_holder)));
  const double ahead = world.ray_distance(0.0, 0.0, 0.0);
  CHECK(std::isfinite(ahead));
  CHECK(ahead > 0.5);
  CHECK(ahead < 2.3);
  actual->set_command(krosc::msgs::Twist{{0.2, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  for (int i = 0; i < 120; ++i) {
    world.step(1.0 / 60.0);  // 2초
  }
  CHECK_NEAR(0.4, actual->pose.x, 0.02);
  CHECK_NEAR(0.0, actual->pose.y, 0.02);
  CHECK_NEAR(0.0, actual->pose.theta, 0.02);
}

KROSC_TEST(simulation_publishes_scan_imu_odom_and_filtered) {
  auto time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context::Options options;
  options.time_source = time_source;
  krosc::Context context(options);

  krosc::simulator::MazeWorld world(0, 7);
  auto robot_holder = std::make_unique<krosc::simulator::TurtleBot3>(true, 7);
  auto* robot = static_cast<krosc::simulator::TurtleBot3*>(world.add(std::move(robot_holder)));

  Bridge bridge(context, *robot);
  krosc::nodes::DiffDriveOdometry odometry(context, "odom_node", {{"publish_tf", false}});
  krosc::nodes::Ekf ekf(context, "ekf",
                        {{"odom0", std::string("odom")},
                         {"odom0_config", [] {
                            std::vector<bool> config(15, false);
                            config[6] = true;
                            config[11] = true;
                            return config;
                          }()},
                         {"imu0", std::string("imu")},
                         {"imu0_config", [] {
                            std::vector<bool> config(15, false);
                            config[11] = true;
                            return config;
                          }()},
                         {"publish_tf", true}});
  Recorder recorder(context);

  robot->set_command(krosc::msgs::Twist{{0.15, 0.0, 0.0}, {0.0, 0.0, 0.4}});
  for (int i = 0; i < 300; ++i) {  // 5초
    world.step(1.0 / 60.0);
    time_source->set(world.time());
  }

  CHECK(recorder.odom.has_value());
  CHECK(recorder.filtered.has_value());
  CHECK(recorder.scan.has_value());
  CHECK(recorder.imu.has_value());
  if (recorder.scan) {
    CHECK_EQ(360u, recorder.scan->ranges.size());
    bool finite = false;
    for (float range : recorder.scan->ranges) {
      finite = finite || std::isfinite(range);
    }
    CHECK(finite);
  }
  if (recorder.odom && recorder.filtered) {
    const double odom_error = std::hypot(recorder.odom->pose.pose.position.x - robot->pose.x,
                                         recorder.odom->pose.pose.position.y - robot->pose.y);
    const double filtered_error = std::hypot(recorder.filtered->pose.pose.position.x - robot->pose.x,
                                             recorder.filtered->pose.pose.position.y - robot->pose.y);
    CHECK(odom_error < 2.0);
    CHECK(filtered_error < 1.5);
    CHECK_EQ(0u, odometry.jumps());
  }
  time_source->set(world.time() + 0.01);
  context.shutdown();
}

KROSC_TEST(simulation_runner_steps_world) {
  krosc::simulator::MazeWorld world(0, 3);
  krosc::simulator::SimulationRunner runner(world, 120.0);
  int steps = 0;
  runner.add_listener([&](krosc::simulator::World&) { ++steps; });
  runner.step_once();
  CHECK_EQ(1, steps);
  CHECK_NEAR(1.0 / 120.0, world.time(), 1e-12);
  runner.start();
  while (steps < 5) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  runner.stop();
  CHECK(steps >= 5);
}

KROSC_TEST(simulator_uses_injected_world_and_robot) {
  krosc::Context context;
  krosc::simulator::MazeWorld world(0, 42);
  auto robot_holder = std::make_unique<krosc::simulator::TurtleBot3>(false, 42);
  auto* robot = static_cast<krosc::simulator::TurtleBot3*>(world.add(std::move(robot_holder)));
  krosc::simulator::Simulator simulator(context, world, *robot,
                                        krosc::simulator::SimulatorOptions{100.0});
  robot->set_command(krosc::msgs::Twist{{0.16, 0.0, 0.0}, {0.0, 0.0, 0.28}});
  CHECK_NEAR(0.16, simulator.robot().state().command.linear.x, 1e-9);
  CHECK_NEAR(0.28, simulator.robot().state().command.angular.z, 1e-9);
  simulator.runner().step_once();
  const auto snapshot = simulator.snapshots().latest();
  CHECK(snapshot != nullptr);
  if (snapshot) {
    CHECK_NEAR(world.time(), snapshot->time, 1e-12);
    CHECK_NEAR(robot->pose.x, snapshot->robot_pose.x, 1e-12);
  }
  context.shutdown();
}
