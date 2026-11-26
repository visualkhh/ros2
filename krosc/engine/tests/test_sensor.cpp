#include <memory>
#include <string>
#include <vector>

#include "krosc/sensor/sensor.hpp"
#include "krosc/sensor/sensor_manager.hpp"

#include "test_framework.hpp"

namespace {

using krosc::sensor::Sensor;
using krosc::sensor::SensorInfo;
using krosc::sensor::SensorManager;

class FakeSensor final : public Sensor {
 public:
  bool fail_init = false;
  bool fail_activate = false;
  bool fail_read = false;
  int reads = 0;
  std::vector<std::string> events;

  bool on_init(const SensorInfo& info) override {
    events.push_back("init:" + info.name);
    return !fail_init;
  }
  bool on_activate() override {
    events.push_back("activate");
    return !fail_activate;
  }
  bool on_deactivate() override {
    events.push_back("deactivate");
    return true;
  }
  bool read(double, double) override {
    ++reads;
    events.push_back("read");
    return !fail_read;
  }
};

}  // namespace

KROSC_TEST(sensor_manager_add_rules) {
  SensorManager manager;
  CHECK(manager.add("lidar", "lds03", std::make_unique<FakeSensor>()));
  CHECK(!manager.add("lidar", "other", std::make_unique<FakeSensor>()));
  CHECK(!manager.add("", "empty", std::make_unique<FakeSensor>()));
  CHECK(!manager.add("null", "null", nullptr));
  CHECK_EQ(1u, manager.size());
  CHECK(manager.find("lidar") != nullptr);
  CHECK(manager.find("nope") == nullptr);
  CHECK_EQ(0u, manager.active_count());
}

KROSC_TEST(sensor_manager_lifecycle_and_read) {
  SensorManager manager;
  auto first = std::make_unique<FakeSensor>();
  auto second = std::make_unique<FakeSensor>();
  FakeSensor* first_ptr = first.get();
  FakeSensor* second_ptr = second.get();
  CHECK(manager.add("b", "type_b", std::move(second)));
  CHECK(manager.add("a", "type_a", std::move(first)));

  CHECK(manager.start());
  CHECK_EQ(2u, manager.active_count());
  CHECK_EQ(2u, second_ptr->events.size());
  CHECK_EQ(std::string("init:b"), second_ptr->events[0]);
  CHECK_EQ(std::string("activate"), second_ptr->events[1]);
  CHECK_EQ(std::string("init:a"), first_ptr->events[0]);
  CHECK_EQ(std::string("activate"), first_ptr->events[1]);
  CHECK_EQ(0u, manager.read(1.0, 0.01));

  first_ptr->fail_read = true;
  CHECK_EQ(1u, manager.read(1.1, 0.01));
  CHECK_EQ(1u, manager.read_failures());
  first_ptr->fail_read = false;
  CHECK_EQ(0u, manager.read(1.2, 0.01));
  CHECK_EQ(3, first_ptr->reads);
  CHECK_EQ(3, second_ptr->reads);

  manager.stop();
  CHECK_EQ(0u, manager.active_count());
  CHECK_EQ(std::string("deactivate"), first_ptr->events.back());
  CHECK_EQ(std::string("deactivate"), second_ptr->events.back());

  // stop 뒤에는 read가 호출되지 않는다.
  CHECK_EQ(0u, manager.read(2.0, 0.01));
  CHECK_EQ(3, first_ptr->reads);
}

KROSC_TEST(sensor_manager_rolls_back_on_failure) {
  SensorManager manager;
  auto first = std::make_unique<FakeSensor>();
  auto second = std::make_unique<FakeSensor>();
  FakeSensor* first_ptr = first.get();
  FakeSensor* second_ptr = second.get();
  second_ptr->fail_activate = true;
  CHECK(manager.add("a", "type_a", std::move(first)));
  CHECK(manager.add("b", "type_b", std::move(second)));

  CHECK(!manager.start());
  CHECK_EQ(0u, manager.active_count());
  CHECK_EQ(0, first_ptr->reads);
  CHECK_EQ(std::string("deactivate"), first_ptr->events.back());

  second_ptr->fail_activate = false;
  CHECK(manager.start());
  CHECK_EQ(2u, manager.active_count());
}
