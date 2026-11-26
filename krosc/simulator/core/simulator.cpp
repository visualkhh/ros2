#include "krosc/simulator/simulator.hpp"

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "krosc/context.hpp"
#include "krosc/qos.hpp"
#include "krosc/nodes/diff_drive_odometry.hpp"
#include "krosc/nodes/ekf.hpp"
#include "krosc/nodes/robot_state_publisher.hpp"
#include "krosc/simulator/maze_world.hpp"
#include "krosc/simulator/runner.hpp"
#include "krosc/simulator/snapshot.hpp"
#include "krosc/simulator/turtlebot3.hpp"
#include "krosc/time/time.hpp"

#include "map_publisher.hpp"
#include "robot_bridge.hpp"

namespace krosc::simulator {

namespace {

std::string read_text(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    return {};
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::vector<bool> ekf_config(std::initializer_list<int> enabled) {
  std::vector<bool> config(15, false);
  for (const int index : enabled) {
    if (index >= 0 && index < 15) {
      config[static_cast<std::size_t>(index)] = true;
    }
  }
  return config;
}

}  // namespace

struct Simulator::Impl {
  Impl(krosc::Context& context_value, const SimulatorOptions& options_value)
      : options(options_value) {
    world = std::make_unique<MazeWorld>(options.pucks, options.seed);
    auto robot_holder = std::make_unique<TurtleBot3>(options.noisy, options.seed);
    robot = static_cast<TurtleBot3*>(world->add(std::move(robot_holder)));
    urdf = read_text(options.urdf_path);
    if (urdf.empty()) {
      throw std::runtime_error("URDF를 읽지 못했다: " + options.urdf_path);
    }
    if (options.bridge) {
      bridge = std::make_unique<RobotBridge>(context_value, *robot);
    }
    map_publisher = std::make_unique<MapPublisher>(context_value, world->walls());
    robot_state = std::make_unique<krosc::nodes::RobotStatePublisher>(
        context_value, "robot_state_publisher",
        std::map<std::string, krosc::ParamValue>{{"robot_description", urdf}});
    if (options.odometry) {
      odometry = std::make_unique<krosc::nodes::DiffDriveOdometry>(
          context_value, "diff_drive_odometry",
          std::map<std::string, krosc::ParamValue>{
              {"publish_tf", false},
              {"wheel_radius", TurtleBot3::kWheelRadius},
              {"wheel_separation", TurtleBot3::kWheelSeparation}});
    }
    if (options.ekf) {
      ekf = std::make_unique<krosc::nodes::Ekf>(
          context_value, "ekf_filter_node",
          std::map<std::string, krosc::ParamValue>{
              {"odom0", std::string("odom")},
              {"odom0_config", ekf_config({6, 11})},
              {"imu0", std::string("imu")},
              {"imu0_config", ekf_config({11})},
              {"publish_tf", true}});
    }
    recorder = std::make_unique<SnapshotRecorder>(*world, *robot);
    runner = std::make_unique<SimulationRunner>(*world, options.step_hz);
    auto* time_source = dynamic_cast<SimulatedTimeSource*>(&context_value.time_source());
    runner->add_listener([this, time_source](World& current) {
      if (time_source != nullptr) {
        time_source->set(current.time());
      }
      recorder->record();
    });
  }

  ~Impl() {
    if (runner) {
      runner->stop();
    }
  }

  SimulatorOptions options;
  std::unique_ptr<MazeWorld> world;
  TurtleBot3* robot = nullptr;
  std::unique_ptr<RobotBridge> bridge;
  std::unique_ptr<MapPublisher> map_publisher;
  std::unique_ptr<krosc::nodes::RobotStatePublisher> robot_state;
  std::unique_ptr<krosc::nodes::DiffDriveOdometry> odometry;
  std::unique_ptr<krosc::nodes::Ekf> ekf;
  std::unique_ptr<SnapshotRecorder> recorder;
  std::unique_ptr<SimulationRunner> runner;
  std::string urdf;
  bool started = false;
};

Simulator::Simulator(krosc::Context& context, SimulatorOptions options)
    : context_(context), impl_(std::make_unique<Impl>(context, options)) {}

Simulator::~Simulator() { stop(); }

void Simulator::start() {
  if (impl_->started) {
    return;
  }
  impl_->started = true;
  impl_->runner->start();
}

void Simulator::stop() {
  if (!impl_ || !impl_->started) {
    return;
  }
  impl_->started = false;
  impl_->runner->stop();
}

void Simulator::set_demo_command() {
  if (!impl_->bridge) {
    return;
  }
  impl_->bridge->publish("cmd_vel",
                         krosc::msgs::Twist{{0.16, 0.0, 0.0}, {0.0, 0.0, 0.28}},
                         krosc::QOS_COMMAND);
}

void Simulator::set_stop_command() {
  if (!impl_->bridge) {
    return;
  }
  impl_->bridge->publish("cmd_vel",
                         krosc::msgs::Twist{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
                         krosc::QOS_COMMAND);
}

World& Simulator::world() { return *impl_->world; }
MazeWorld& Simulator::maze_world() { return *impl_->world; }
TurtleBot3& Simulator::robot() { return *impl_->robot; }
SimulationRunner& Simulator::runner() { return *impl_->runner; }
SnapshotStore& Simulator::snapshots() { return impl_->recorder->store(); }
const SnapshotStore& Simulator::snapshots() const { return impl_->recorder->store(); }
krosc::Node& Simulator::bridge_node() { return *impl_->bridge; }
const std::string& Simulator::urdf() const { return impl_->urdf; }

}  // namespace krosc::simulator
