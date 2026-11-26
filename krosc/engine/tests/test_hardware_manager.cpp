#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/control/control.hpp"
#include "krosc/hardware/hardware.hpp"
#include "krosc/hardware/hardware_manager.hpp"
#include "krosc/sensor/sensor.hpp"

#include "test_framework.hpp"

namespace {

using krosc::hardware::Hardware;
using krosc::hardware::HardwareManager;
using krosc::sensor::Sensor;

class FakeHardware final : public Hardware {
 public:
  int starts = 0;
  int stops = 0;
  void start() override { ++starts; }
  void stop() override { ++stops; }
  [[nodiscard]] bool connected() const override { return true; }
};

class FakeSensor final : public Sensor {
 public:
  explicit FakeSensor(krosc::Context& context) : Sensor(context) {}
  bool fail_read = false;
  int reads = 0;
  double last_period = 0.0;
  std::vector<std::string> events;

  bool read(double, double period) override {
    ++reads;
    last_period = period;
    events.push_back("sensor:read");
    return !fail_read;
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
    events.push_back("control:read");
    return !fail_read;
  }
  bool write(double, double) override {
    ++writes;
    events.push_back("control:write");
    return !fail_write;
  }
};

}  // namespace

KROSC_TEST(hardware_manager_add_rules) {
  krosc::Context context;
  FakeHardware hardware;
  FakeSensor sensor(context);
  FakeControl control;
  HardwareManager manager;
  CHECK(manager.add_hardware(hardware, {"hw"}));
  CHECK(!manager.add_hardware(hardware, {"hw"}));
  CHECK(!manager.add_hardware(hardware, {""}));
  CHECK(manager.add_sensor(sensor, {"sensor", 0.1}));
  CHECK(!manager.add_sensor(sensor, {"sensor", 0.1}));
  CHECK(!manager.add_sensor(sensor, {"zero", 0.0}));
  CHECK(manager.add_control(control, {"control", 0.1, 0.2}));
  CHECK(!manager.add_control(control, {"control", 0.1, 0.2}));
  CHECK(!manager.add_control(control, {"bad", 0.0, 0.2}));
}

KROSC_TEST(hardware_manager_read_control_write_order) {
  krosc::Context context;
  FakeHardware hardware;
  FakeSensor sensor(context);
  FakeControl control;
  HardwareManager manager;
  CHECK(manager.add_hardware(hardware, {"hw"}));
  CHECK(manager.add_sensor(sensor, {"sensor", 0.1}));
  CHECK(manager.add_control(control, {"control", 0.1, 0.25}));

  std::vector<std::string> order;
  manager.set_control_hook([&](double, double) { order.push_back("hook"); }, 0.2);

  CHECK(manager.start(false));
  CHECK_EQ(1, hardware.starts);

  CHECK_EQ(0u, manager.poll(0.0));
  CHECK_EQ(1, sensor.reads);
  CHECK_EQ(1, control.reads);
  CHECK_EQ(1, control.writes);
  CHECK_EQ(3u, control.events.size() + sensor.events.size());
  CHECK_EQ(std::string("sensor:read"), sensor.events[0]);
  CHECK_EQ(std::string("control:read"), control.events[0]);
  CHECK_EQ(std::string("control:write"), control.events[1]);
  CHECK_EQ(1u, order.size());

  CHECK_EQ(0u, manager.poll(0.05));
  CHECK_EQ(1, sensor.reads);
  CHECK_EQ(1, control.reads);

  CHECK_EQ(0u, manager.poll(0.1));
  CHECK_EQ(2, sensor.reads);
  CHECK_EQ(2, control.reads);

  CHECK_EQ(0u, manager.poll(0.2));
  CHECK_EQ(3, sensor.reads);
  CHECK_EQ(3, control.reads);
  CHECK_EQ(2u, order.size());

  CHECK_EQ(0u, manager.poll(0.3));
  CHECK_EQ(4, sensor.reads);
  CHECK_EQ(4, control.reads);
  CHECK_EQ(2, control.writes);
  CHECK_NEAR(0.1, sensor.last_period, 1e-9);

  manager.stop();
  CHECK_EQ(1, hardware.stops);
}

KROSC_TEST(hardware_manager_counts_failures) {
  krosc::Context context;
  FakeHardware hardware;
  FakeSensor sensor(context);
  FakeControl control;
  HardwareManager manager;
  CHECK(manager.add_hardware(hardware, {"hw"}));
  CHECK(manager.add_sensor(sensor, {"sensor", 0.1}));
  CHECK(manager.add_control(control, {"control", 0.1, 0.2}));
  CHECK(manager.start(false));

  sensor.fail_read = true;
  CHECK_EQ(1u, manager.poll(0.0));  // sensor 실패
  CHECK_EQ(1u, manager.sensor_failures());

  sensor.fail_read = false;
  control.fail_write = true;
  CHECK_EQ(1u, manager.poll(0.2));  // control write 실패
  CHECK_EQ(1u, manager.control_write_failures());
}
