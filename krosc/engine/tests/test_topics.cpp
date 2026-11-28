#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/executor.hpp"
#include "krosc/node.hpp"
#include "krosc/time/time.hpp"

#include "test_framework.hpp"

using namespace std::chrono_literals;

KROSC_TEST(topic_inline_publish_and_unsubscribe) {
  krosc::Context context;
  krosc::Node node(context, "topics");
  int received = 0;
  auto subscription = node.subscribe<krosc::msgs::String>("chat", [&](const krosc::msgs::String& message) {
    ++received;
    CHECK_EQ(std::string("hello"), message.data);
  });
  node.publish("chat", krosc::msgs::String{"hello"});
  CHECK_EQ(1, received);
  CHECK_EQ(1u, subscription.delivered());
  subscription.cancel();
  node.publish("chat", krosc::msgs::String{"again"});
  CHECK_EQ(1, received);
}

KROSC_TEST(topic_type_is_locked) {
  krosc::Context context;
  krosc::Node node(context, "types");
  node.publish("mixed", 1);
  CHECK_THROWS_AS(node.subscribe<double>("mixed", [](double) {}), std::runtime_error);
  CHECK_THROWS_AS(node.publish("mixed", 1.5), std::runtime_error);
}

KROSC_TEST(topic_latched_history) {
  krosc::Context context;
  krosc::Node node(context, "latch");
  node.publish("config", krosc::msgs::String{"one"}, krosc::QOS_LATCHED);
  node.publish("config", krosc::msgs::String{"two"}, krosc::QOS_LATCHED);
  std::string received;
  node.subscribe<krosc::msgs::String>("config", [&](const krosc::msgs::String& message) { received = message.data; },
                                      krosc::QOS_LATCHED);
  CHECK_EQ(std::string("two"), received);
}

KROSC_TEST(topic_depth_drops_oldest) {
  krosc::Context::Options options;
  options.executor_threads = 1;
  options.time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context context(options);
  krosc::Node node(context, "depth");
  std::mutex mutex;
  std::condition_variable cv;
  bool started = false;
  bool release = false;
  std::vector<int> received;
  auto subscription = node.subscribe<int>(
      "numbers",
      [&](const int& value) {
        std::unique_lock<std::mutex> lock(mutex);
        received.push_back(value);
        if (received.size() == 1) {
          started = true;
          cv.notify_all();
          cv.wait(lock, [&] { return release; });
        }
      },
      krosc::QOS_COMMAND);
  node.publish<int>("numbers", 1);
  {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock, [&] { return started; });
  }
  node.publish<int>("numbers", 2);
  node.publish<int>("numbers", 3);
  node.publish<int>("numbers", 4);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  cv.notify_all();
  context.executor().join();
  CHECK_EQ(2u, received.size());
  if (received.size() == 2) {
    CHECK_EQ(1, received[0]);
    CHECK_EQ(4, received[1]);
  }
  CHECK_EQ(2u, subscription.dropped());
}

KROSC_TEST(executor_pool_join) {
  krosc::Executor executor(4);
  std::atomic<int> count{0};
  for (int i = 0; i < 1000; ++i) {
    executor.post([&] { count.fetch_add(1, std::memory_order_relaxed); });
  }
  executor.join();
  CHECK_EQ(1000, count.load());
  executor.shutdown();
}

KROSC_TEST(node_names_are_unique) {
  krosc::Context context;
  krosc::Node first(context, "same");
  CHECK_THROWS_AS(krosc::Node(context, "same"), std::runtime_error);
}

KROSC_TEST(publish_after_shutdown_is_dropped) {
  krosc::Context::Options options;
  options.time_source = std::make_shared<krosc::SimulatedTimeSource>();
  auto context = std::make_unique<krosc::Context>(options);
  auto node = std::make_unique<krosc::Node>(*context, "shutdown");
  context->shutdown();
  CHECK_NOTHROW(node->publish("late", 1));
}

KROSC_TEST(topic_publisher_handle) {
  krosc::Context context;
  krosc::Node node(context, "publisher");
  std::vector<std::string> received;
  node.subscribe<krosc::msgs::String>("chat", [&](const krosc::msgs::String& message) { received.push_back(message.data); });
  auto publisher = node.create_publisher<krosc::msgs::String>("chat");
  CHECK(publisher.valid());
  publisher.publish(krosc::msgs::String{"a"});
  node.publish("chat", krosc::msgs::String{"b"});  // 이름으로 발행해도 같은 채널
  CHECK_EQ(2u, received.size());
  CHECK_EQ(std::string("a"), received[0]);
  // 타입이 다른 채널이면 만들 때 바로 실패한다.
  CHECK_THROWS_AS(node.create_publisher<int>("chat"), std::runtime_error);
  // latched 발행자는 나중 구독자에게도 전달된다.
  auto latched = node.create_publisher<krosc::msgs::String>("config", krosc::QOS_LATCHED);
  latched.publish(krosc::msgs::String{"late"});
  std::string late;
  node.subscribe<krosc::msgs::String>("config", [&](const krosc::msgs::String& message) { late = message.data; },
                                      krosc::QOS_LATCHED);
  CHECK_EQ(std::string("late"), late);
  context.shutdown();
  publisher.publish(krosc::msgs::String{"after shutdown"});  // 종료 뒤 발행은 조용히 무시
  CHECK_EQ(2u, received.size());
}

KROSC_TEST(topic_default_qos_matches_ros2) {
  CHECK(krosc::QOS_DEFAULT.depth.has_value());
  CHECK_EQ(10u, *krosc::QOS_DEFAULT.depth);
  CHECK_EQ(5u, *krosc::QOS_SENSOR_DATA.depth);
  CHECK_EQ(1000u, *krosc::QOS_PARAMETER_EVENTS.depth);
  CHECK(!krosc::QOS_KEEP_ALL.depth.has_value());
}
