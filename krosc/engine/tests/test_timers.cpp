#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "krosc/context.hpp"
#include "krosc/node.hpp"
#include "krosc/time/time.hpp"
#include "krosc/time/timer.hpp"

#include "test_framework.hpp"

using namespace std::chrono_literals;

KROSC_TEST(timer_simulated_period_and_missed) {
  auto time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context::Options options;
  options.time_source = time_source;
  krosc::Context context(options);
  krosc::Node node(context, "timer");
  int calls = 0;
  auto timer = node.create_timer(0.1, [&] { ++calls; });
  time_source->set(0.05);
  CHECK_EQ(0, calls);
  time_source->set(0.1);
  CHECK_EQ(1, calls);
  time_source->set(0.35);
  CHECK_EQ(2, calls);
  CHECK_EQ(1u, timer.missed());
  time_source->set(0.4);
  CHECK_EQ(3, calls);
  timer.cancel();
  time_source->set(0.8);
  CHECK_EQ(3, calls);
  CHECK_EQ(3u, timer.calls());
}

KROSC_TEST(timer_cancel_from_callback) {
  auto time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context::Options options;
  options.time_source = time_source;
  krosc::Context context(options);
  krosc::Node node(context, "cancel_timer");
  int calls = 0;
  krosc::Timer timer;
  timer = node.create_timer(0.1, [&, timer_ptr = &timer] {
    ++calls;
    timer_ptr->cancel();
  });
  time_source->set(0.1);
  time_source->set(0.2);
  CHECK_EQ(1, calls);
  CHECK(timer.canceled());
}

KROSC_TEST(timer_realtime_runs) {
  auto time_source = std::make_shared<krosc::SystemTimeSource>();
  krosc::Context::Options options;
  options.time_source = time_source;
  options.executor_threads = 1;
  krosc::Context context(options);
  krosc::Node node(context, "realtime_timer");
  std::atomic<int> calls{0};
  auto timer = node.create_timer(0.005, [&] { calls.fetch_add(1, std::memory_order_relaxed); });
  std::this_thread::sleep_for(80ms);
  timer.cancel();
  (void)context.timers().join();
  CHECK(calls.load() >= 1);
  CHECK_EQ(0u, context.timers().timer_count());
}

KROSC_TEST(timer_manager_stats) {
  auto time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context::Options options;
  options.time_source = time_source;
  krosc::Context context(options);
  krosc::Node node(context, "stats");
  auto timer = node.create_timer(0.1, [] {});
  time_source->set(0.1);
  const auto stats = context.timers().stats();
  CHECK_EQ(1u, stats.size());
  if (!stats.empty()) {
    CHECK_EQ(std::string("stats"), stats[0].name);
    CHECK_NEAR(0.1, stats[0].period, 1e-12);
    CHECK_EQ(1u, stats[0].calls);
  }
}
