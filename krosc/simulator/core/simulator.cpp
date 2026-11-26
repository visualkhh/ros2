#include "krosc/simulator/simulator.hpp"

#include <stdexcept>
#include <utility>

#include "krosc/context.hpp"
#include "krosc/simulator/runner.hpp"
#include "krosc/time/time.hpp"

namespace krosc::simulator {

struct Simulator::Impl {
  Impl(krosc::Context& context_value, World& world_value, SimulatorRobot& robot_value,
       const SimulatorOptions& options)
      : world(world_value), robot(robot_value), recorder(world_value, robot_value),
        runner(world_value, options.step_hz) {
    if (robot.world != &world) {
      throw std::invalid_argument("SimulatorRobot을 먼저 World::add()로 월드에 넣어야 한다");
    }
    auto* time_source = dynamic_cast<SimulatedTimeSource*>(&context_value.time_source());
    runner.add_listener([this, time_source](World& current) {
      if (time_source != nullptr) {
        time_source->set(current.time());
      }
      recorder.record();
    });
  }

  ~Impl() { runner.stop(); }

  World& world;
  SimulatorRobot& robot;
  SnapshotRecorder recorder;
  SimulationRunner runner;
  bool started = false;
};

Simulator::Simulator(krosc::Context& context, World& world, SimulatorRobot& robot,
                     SimulatorOptions options)
    : impl_(std::make_unique<Impl>(context, world, robot, options)) {}

Simulator::~Simulator() { stop(); }

void Simulator::start() {
  if (impl_->started) {
    return;
  }
  impl_->started = true;
  impl_->runner.start();
}

void Simulator::stop() {
  if (!impl_ || !impl_->started) {
    return;
  }
  impl_->started = false;
  impl_->runner.stop();
}

World& Simulator::world() { return impl_->world; }
SimulatorRobot& Simulator::robot() { return impl_->robot; }
SimulationRunner& Simulator::runner() { return impl_->runner; }
SnapshotStore& Simulator::snapshots() { return impl_->recorder.store(); }
const SnapshotStore& Simulator::snapshots() const { return impl_->recorder.store(); }

}  // namespace krosc::simulator
