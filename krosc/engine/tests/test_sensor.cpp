#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/sensor/sensor.hpp"
#include "krosc/sensor/sensor_manager.hpp"

#include "test_framework.hpp"

namespace {

using krosc::sensor::Sensor;
using krosc::sensor::SensorError;
using krosc::sensor::SensorManager;

class FakeSensor final : public Sensor {
 public:
  explicit FakeSensor(krosc::Context& context, std::string name = "fake")
      : Sensor(context), name_(std::move(name)) {}

  bool fail_init = false;
  bool fail_activate = false;
  bool fail_read = false;
  int reads = 0;
  int finishes = 0;
  double last_period = 0.0;
  std::string name_;
  std::vector<std::string> events;
  std::vector<SensorError> errors;

  bool on_init() override {
    events.push_back("init:" + name_);
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
  void on_finish() override { ++finishes; }
  void on_error(const SensorError& error) override { errors.push_back(error); }
  bool on_update(double, double period) override {
    ++reads;
    last_period = period;
    events.push_back("read");
    return !fail_read;
  }
};

}  // namespace

KROSC_TEST(sensor_manager_add_rules) {
  krosc::Context context;
  SensorManager manager;
  CHECK(manager.add("lidar", 0.1, std::make_unique<FakeSensor>(context, "lidar")));
  CHECK(!manager.add("lidar", 0.1, std::make_unique<FakeSensor>(context, "lidar2")));
  CHECK(!manager.add("", 0.1, std::make_unique<FakeSensor>(context, "empty")));
  CHECK(!manager.add("zero", 0.0, std::make_unique<FakeSensor>(context, "zero")));
  CHECK(!manager.add("null", 0.1, nullptr));
  CHECK_EQ(1u, manager.size());
  CHECK(manager.find("lidar") != nullptr);
  CHECK(manager.find("nope") == nullptr);
  CHECK_EQ(0u, manager.active_count());
}

KROSC_TEST(sensor_manager_lifecycle_and_read) {
  krosc::Context context;
  SensorManager manager;
  auto first = std::make_unique<FakeSensor>(context, "a");
  auto second = std::make_unique<FakeSensor>(context, "b");
  FakeSensor* first_ptr = first.get();
  FakeSensor* second_ptr = second.get();
  CHECK(manager.add("b", 0.1, std::move(second)));
  CHECK(manager.add("a", 0.2, std::move(first)));

  CHECK(manager.start(false));
  CHECK_EQ(2u, manager.active_count());
  CHECK_EQ(2u, second_ptr->events.size());
  CHECK_EQ(std::string("init:b"), second_ptr->events[0]);
  CHECK_EQ(std::string("activate"), second_ptr->events[1]);
  CHECK_EQ(std::string("init:a"), first_ptr->events[0]);
  CHECK_EQ(std::string("activate"), first_ptr->events[1]);

  CHECK_EQ(0u, manager.poll(1.0));
  CHECK_EQ(1, first_ptr->reads);
  CHECK_EQ(1, second_ptr->reads);
  CHECK_EQ(0u, manager.poll(1.0));  // 아직 다음 주기가 아니다

  second_ptr->fail_read = true;
  CHECK_EQ(1u, manager.poll(1.1));
  CHECK_EQ(1u, manager.read_failures());
  CHECK_NEAR(0.2, first_ptr->last_period, 1e-9);
  CHECK_NEAR(0.1, second_ptr->last_period, 1e-9);
  CHECK_EQ(1u, second_ptr->errors.size());
  CHECK(second_ptr->errors[0].stage == SensorError::Stage::Read);
  CHECK_EQ(1u, second_ptr->errors[0].consecutive_failures);
  second_ptr->fail_read = false;

  manager.stop();
  CHECK_EQ(0u, manager.active_count());
  CHECK_EQ(std::string("deactivate"), first_ptr->events.back());
  CHECK_EQ(std::string("deactivate"), second_ptr->events.back());
  CHECK_EQ(0, first_ptr->finishes);  // stop은 finish가 아니다

  // stop 뒤에는 on_update가 호출되지 않는다.
  CHECK_EQ(0u, manager.poll(2.0));
  CHECK_EQ(1, first_ptr->reads);
  CHECK_EQ(2, second_ptr->reads);

  manager.finish();
  CHECK_EQ(1, first_ptr->finishes);
  CHECK_EQ(1, second_ptr->finishes);
  manager.finish();
  CHECK_EQ(1, first_ptr->finishes);  // finish는 한 번만
  CHECK(!manager.start(false));      // finish 뒤 재시작 불가
}

KROSC_TEST(sensor_manager_reports_read_errors) {
  krosc::Context context;
  SensorManager manager;
  auto fake = std::make_unique<FakeSensor>(context, "imu");
  FakeSensor* fake_ptr = fake.get();
  CHECK(manager.add("imu", 0.1, std::move(fake)));
  CHECK(manager.start(false));

  fake_ptr->fail_read = true;
  CHECK_EQ(1u, manager.poll(0.0));
  CHECK_EQ(1u, fake_ptr->errors.size());
  CHECK_EQ(1u, fake_ptr->errors[0].consecutive_failures);
  CHECK_EQ(1u, manager.poll(0.1));
  CHECK_EQ(2u, fake_ptr->errors.size());
  CHECK_EQ(2u, fake_ptr->errors[1].consecutive_failures);

  fake_ptr->fail_read = false;
  CHECK_EQ(0u, manager.poll(0.2));
  CHECK_EQ(2u, fake_ptr->errors.size());  // 성공하면 on_error는 늘지 않는다

  fake_ptr->fail_read = true;
  CHECK_EQ(1u, manager.poll(0.3));
  CHECK_EQ(3u, fake_ptr->errors.size());
  CHECK_EQ(1u, fake_ptr->errors[2].consecutive_failures);  // 성공 후 리셋
}

KROSC_TEST(sensor_manager_schedules_by_period) {
  krosc::Context context;
  SensorManager manager;
  auto fast = std::make_unique<FakeSensor>(context, "fast");
  auto slow = std::make_unique<FakeSensor>(context, "slow");
  FakeSensor* fast_ptr = fast.get();
  FakeSensor* slow_ptr = slow.get();
  CHECK(manager.add("fast", 0.1, std::move(fast)));
  CHECK(manager.add("slow", 0.25, std::move(slow)));
  CHECK(manager.start(false));

  CHECK_EQ(0u, manager.poll(0.0));
  CHECK_EQ(1, fast_ptr->reads);
  CHECK_EQ(1, slow_ptr->reads);

  CHECK_EQ(0u, manager.poll(0.05));
  CHECK_EQ(1, fast_ptr->reads);
  CHECK_EQ(1, slow_ptr->reads);

  CHECK_EQ(0u, manager.poll(0.1));
  CHECK_EQ(2, fast_ptr->reads);
  CHECK_EQ(1, slow_ptr->reads);

  CHECK_EQ(0u, manager.poll(0.25));
  CHECK_EQ(3, fast_ptr->reads);  // 0.2에 밀린 주기를 한 번만 따라잡는다
  CHECK_EQ(2, slow_ptr->reads);

  CHECK_EQ(0u, manager.poll(0.29));
  CHECK_EQ(3, fast_ptr->reads);
  CHECK_EQ(2, slow_ptr->reads);
}

KROSC_TEST(sensor_manager_rolls_back_on_failure) {
  krosc::Context context;
  SensorManager manager;
  auto first = std::make_unique<FakeSensor>(context, "a");
  auto second = std::make_unique<FakeSensor>(context, "b");
  FakeSensor* first_ptr = first.get();
  FakeSensor* second_ptr = second.get();
  second_ptr->fail_activate = true;
  CHECK(manager.add("a", 0.1, std::move(first)));
  CHECK(manager.add("b", 0.1, std::move(second)));

  CHECK(!manager.start(false));
  CHECK_EQ(0u, manager.active_count());
  CHECK_EQ(0, first_ptr->reads);
  CHECK_EQ(std::string("deactivate"), first_ptr->events.back());
  CHECK_EQ(1u, second_ptr->errors.size());
  CHECK(second_ptr->errors[0].stage == SensorError::Stage::Activate);

  second_ptr->fail_activate = false;
  CHECK(manager.start(false));
  CHECK_EQ(2u, manager.active_count());
}

KROSC_TEST(sensor_manager_thread_loop) {
  krosc::Context context;
  SensorManager manager;
  auto fake = std::make_unique<FakeSensor>(context, "imu");
  FakeSensor* fake_ptr = fake.get();
  CHECK(manager.add("imu", 0.02, std::move(fake)));
  CHECK(manager.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  manager.stop();
  CHECK(fake_ptr->reads >= 2);
  CHECK_EQ(0u, manager.active_count());
}
