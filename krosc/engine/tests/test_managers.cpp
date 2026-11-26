#include <memory>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/control/control.hpp"
#include "krosc/control/control_manager.hpp"
#include "krosc/sensor/sensor.hpp"
#include "krosc/sensor/sensor_manager.hpp"

#include "test_framework.hpp"

namespace {

class FakeSensor final : public krosc::sensor::Sensor {
 public:
  explicit FakeSensor(krosc::Context& context) : Sensor(context) {}
  bool fail = false;
  int reads = 0;
  double last_period = 0.0;

  bool read(double, double period) override {
    ++reads;
    last_period = period;
    return !fail;
  }
};

class FakeControl final : public krosc::control::Control {
 public:
  bool fail_read = false;
  bool fail_write = false;
  int reads = 0;
  int writes = 0;
  std::vector<std::string> events;

  std::vector<std::shared_ptr<krosc::control::StateInterface>> export_state_interfaces() override {
    return {};
  }
  std::vector<std::shared_ptr<krosc::control::CommandInterface>> export_command_interfaces() override {
    return {};
  }
  bool read(double, double) override {
    ++reads;
    events.push_back("read");
    return !fail_read;
  }
  bool write(double, double) override {
    ++writes;
    events.push_back("write");
    return !fail_write;
  }
};

}  // namespace

KROSC_TEST(sensor_manager_schedules_period) {
  krosc::Context context;
  FakeSensor sensor(context);
  krosc::sensor::SensorManager manager;
  CHECK(manager.add(sensor, "sensor", 0.1));
  CHECK(!manager.add(sensor, "sensor", 0.1));
  CHECK(!manager.add(sensor, "zero", 0.0));
  CHECK(manager.start(false));

  CHECK_EQ(0u, manager.poll(0.0));
  CHECK_EQ(1, sensor.reads);
  CHECK_EQ(0u, manager.poll(0.05));
  CHECK_EQ(1, sensor.reads);
  CHECK_EQ(0u, manager.poll(0.1));
  CHECK_EQ(2, sensor.reads);
  CHECK_EQ(0u, manager.poll(0.2));
  CHECK_EQ(3, sensor.reads);
  CHECK_NEAR(0.1, sensor.last_period, 1e-9);

  sensor.fail = true;
  CHECK_EQ(1u, manager.poll(0.3));
  CHECK_EQ(1u, manager.failures());
  manager.stop();
}

KROSC_TEST(control_manager_read_hook_write_order) {
  FakeControl control;
  krosc::control::ControlManager manager;
  CHECK(manager.add(control, "control", 0.1, 0.25));
  CHECK(!manager.add(control, "control", 0.1, 0.25));

  std::vector<std::string> order;
  manager.set_control_hook([&](double, double) { order.push_back("hook"); }, 0.2);
  CHECK(manager.start(false));

  CHECK_EQ(0u, manager.poll(0.0));
  CHECK_EQ(2u, control.events.size());
  CHECK_EQ(std::string("read"), control.events[0]);
  CHECK_EQ(std::string("write"), control.events[1]);
  CHECK_EQ(1u, order.size());

  CHECK_EQ(0u, manager.poll(0.05));
  CHECK_EQ(0u, manager.poll(0.1));
  CHECK_EQ(2, control.reads);
  CHECK_EQ(0u, manager.poll(0.2));
  CHECK_EQ(3, control.reads);
  CHECK_EQ(2u, order.size());
  CHECK_EQ(0u, manager.poll(0.3));
  CHECK_EQ(4, control.reads);
  CHECK_EQ(2, control.writes);

  control.fail_write = true;
  CHECK_EQ(1u, manager.poll(0.5));
  CHECK_EQ(1u, manager.write_failures());
  manager.stop();
}
