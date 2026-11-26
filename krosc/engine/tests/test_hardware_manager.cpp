#include <memory>
#include <string>
#include <vector>

#include "krosc/hardware/hardware.hpp"
#include "krosc/hardware/hardware_manager.hpp"

#include "test_framework.hpp"

namespace {

using krosc::hardware::Hardware;
using krosc::hardware::HardwareConfig;
using krosc::hardware::HardwareManager;

class FakeHardware final : public Hardware {
 public:
  explicit FakeHardware(std::string name) : name_(std::move(name)) {}

  bool fail_read = false;
  bool fail_write = false;
  int reads = 0;
  int writes = 0;
  double last_read_period = 0.0;
  double last_write_period = 0.0;
  int starts = 0;
  int stops = 0;
  std::vector<std::string> events;

  void start() override { ++starts; }
  void stop() override { ++stops; }
  [[nodiscard]] bool connected() const override { return true; }

  bool read(double, double period) override {
    ++reads;
    last_read_period = period;
    events.push_back("read:" + name_);
    return !fail_read;
  }
  bool write(double, double period) override {
    ++writes;
    last_write_period = period;
    events.push_back("write:" + name_);
    return !fail_write;
  }

 private:
  std::string name_;
};

HardwareConfig config(const std::string& name, double read_period, double write_period) {
  HardwareConfig result;
  result.name = name;
  result.read_period = read_period;
  result.write_period = write_period;
  return result;
}

}  // namespace

KROSC_TEST(hardware_manager_add_rules) {
  HardwareManager manager;
  CHECK(manager.add(std::make_unique<FakeHardware>("a"), config("a", 0.1, 0.2)));
  CHECK(!manager.add(std::make_unique<FakeHardware>("a"), config("a", 0.1, 0.2)));
  CHECK(!manager.add(std::make_unique<FakeHardware>(""), config("", 0.1, 0.2)));
  CHECK(!manager.add(std::make_unique<FakeHardware>("zero"), config("zero", 0.0, 0.0)));
  CHECK(!manager.add(std::make_unique<FakeHardware>("neg"), config("neg", 0.1, -0.1)));
  CHECK(!manager.add(nullptr, config("null", 0.1, 0.2)));
  CHECK_EQ(1u, manager.size());
  CHECK(manager.find("a") != nullptr);
  CHECK(manager.find("nope") == nullptr);
}

KROSC_TEST(hardware_manager_read_control_write_order) {
  HardwareManager manager;
  auto hardware = std::make_unique<FakeHardware>("opencr");
  FakeHardware* ptr = hardware.get();
  CHECK(manager.add(std::move(hardware), config("opencr", 0.1, 0.25)));

  std::vector<std::string> order;
  manager.set_control_hook([&](double, double period) {
    order.push_back("control");
    CHECK_NEAR(0.2, period, 1e-9);
  }, 0.2);

  CHECK(manager.start(false));
  CHECK_EQ(1, ptr->starts);

  CHECK_EQ(0u, manager.poll(0.0));
  CHECK_EQ(1, ptr->reads);
  CHECK_EQ(1, ptr->writes);
  CHECK_EQ(2u, ptr->events.size());
  CHECK_EQ(std::string("read:opencr"), ptr->events[0]);
  CHECK_EQ(std::string("write:opencr"), ptr->events[1]);
  CHECK_EQ(1u, order.size());
  CHECK_EQ(std::string("control"), order[0]);

  CHECK_EQ(0u, manager.poll(0.05));
  CHECK_EQ(1, ptr->reads);
  CHECK_EQ(1, ptr->writes);

  CHECK_EQ(0u, manager.poll(0.1));
  CHECK_EQ(2, ptr->reads);
  CHECK_EQ(1, ptr->writes);

  CHECK_EQ(0u, manager.poll(0.2));
  CHECK_EQ(3, ptr->reads);
  CHECK_EQ(1, ptr->writes);
  CHECK_EQ(2u, order.size());

  CHECK_EQ(0u, manager.poll(0.3));
  CHECK_EQ(4, ptr->reads);
  CHECK_EQ(2, ptr->writes);
  CHECK_NEAR(0.1, ptr->last_read_period, 1e-9);
  CHECK_NEAR(0.25, ptr->last_write_period, 1e-9);

  manager.stop();
  CHECK_EQ(1, ptr->stops);
}

KROSC_TEST(hardware_manager_counts_failures) {
  HardwareManager manager;
  auto hardware = std::make_unique<FakeHardware>("hw");
  FakeHardware* ptr = hardware.get();
  CHECK(manager.add(std::move(hardware), config("hw", 0.1, 0.2)));
  CHECK(manager.start(false));

  ptr->fail_read = true;
  CHECK_EQ(1u, manager.poll(0.0));  // read 실패, write는 성공
  CHECK_EQ(1u, manager.read_failures());
  CHECK_EQ(0u, manager.write_failures());

  ptr->fail_read = false;
  ptr->fail_write = true;
  CHECK_EQ(1u, manager.poll(0.2));  // read 성공, write 실패
  CHECK_EQ(1u, manager.read_failures());
  CHECK_EQ(1u, manager.write_failures());
}
