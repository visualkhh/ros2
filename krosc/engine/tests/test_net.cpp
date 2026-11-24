#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "krosc/context.hpp"
#include "krosc/net.hpp"
#include "krosc/qos.hpp"

#include "test_framework.hpp"

using namespace std::chrono_literals;

KROSC_TEST(net_roundtrip_topics_and_teleop) {
  krosc::Context server_context;
  krosc::net::Server server(server_context, 0);
  CHECK(server.port() != 0);
  CHECK(server.running());

  // latched: 클라이언트가 나중에 붙어도 받아야 한다.
  server_context.topics().publish("robot_description", krosc::msgs::String{"<robot name='net_test'/>"},
                                  krosc::QOS_LATCHED);
  server_context.topics().publish(
      "tf_static", krosc::msgs::TFMessage{{
                       krosc::msgs::TransformStamped{
                           krosc::msgs::Header{0.0, "base_link"}, "base_scan",
                           krosc::msgs::Transform{{0.06, 0.0, 0.02}, krosc::msgs::Quaternion{}}},
                   }},
      krosc::QOS_TF_STATIC);

  krosc::msgs::OccupancyGrid map;
  map.header = krosc::msgs::Header{0.0, "map"};
  map.info.resolution = 0.05;
  map.info.width = 2;
  map.info.height = 2;
  map.info.origin.orientation.w = 1.0;
  map.data = {0, 100, -1, 50};
  server_context.topics().publish("map", map, krosc::QOS_LATCHED);

  krosc::Context client_context;
  std::atomic<bool> got_description{false};
  std::atomic<bool> got_map{false};
  std::atomic<bool> got_scan{false};
  std::mutex mutex;
  std::condition_variable cv;
  client_context.topics().subscribe<krosc::msgs::String>(
      "robot_description",
      [&](const krosc::msgs::String& message) {
        if (message.data.find("net_test") != std::string::npos) {
          got_description.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_LATCHED);
  client_context.topics().subscribe<krosc::msgs::OccupancyGrid>(
      "map",
      [&](const krosc::msgs::OccupancyGrid& message) {
        if (message.info.width == 2 && message.info.height == 2 && message.data.size() == 4 && message.data[1] == 100) {
          got_map.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_LATCHED);
  client_context.topics().subscribe<krosc::msgs::LaserScan>(
      "scan",
      [&](const krosc::msgs::LaserScan& message) {
        if (message.ranges.size() == 3) {
          got_scan.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_SENSOR_DATA);

  auto client = std::make_unique<krosc::net::Client>(client_context, "127.0.0.1", server.port());

  std::unique_lock<std::mutex> lock(mutex);
  const bool connected = cv.wait_for(lock, 3s, [&] {
    return client->connected() && got_description.load();
  });
  CHECK(connected);
  CHECK(got_description.load());
  CHECK(cv.wait_for(lock, 3s, [&] { return got_map.load(); }));

  krosc::msgs::LaserScan scan;
  scan.header.stamp = 1.0;
  scan.header.frame_id = "base_scan";
  scan.angle_min = 0.0;
  scan.angle_max = 2.0;
  scan.angle_increment = 1.0;
  scan.range_min = 0.12;
  scan.range_max = 3.5;
  scan.ranges = {1.0F, 2.0F, 3.0F};
  server_context.topics().publish("scan", scan, krosc::QOS_SENSOR_DATA);
  CHECK(cv.wait_for(lock, 3s, [&] { return got_scan.load(); }));

  std::atomic<bool> got_command{false};
  std::atomic<double> command_linear{0.0};
  server_context.topics().subscribe<krosc::msgs::Twist>(
      "cmd_vel_teleop",
      [&](const krosc::msgs::Twist& twist) {
        command_linear.store(twist.linear.x);
        got_command.store(true);
        cv.notify_all();
      },
      krosc::QOS_COMMAND);
  client_context.topics().publish("cmd_vel", krosc::msgs::Twist{{0.17, 0.0, 0.0}, {0.0, 0.0, -0.4}},
                                  krosc::QOS_COMMAND);
  CHECK(cv.wait_for(lock, 3s, [&] { return got_command.load(); }));
  CHECK_NEAR(0.17, command_linear.load(), 1e-9);

  lock.unlock();
  client->shutdown();
  client.reset();
  server.shutdown();
}

KROSC_TEST(net_large_latched_map_roundtrip) {
  krosc::Context server_context;
  krosc::net::Server server(server_context, 0);
  krosc::msgs::OccupancyGrid map;
  map.header = krosc::msgs::Header{2.0, "map"};
  map.info.resolution = 0.05;
  map.info.width = 163;
  map.info.height = 262;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(163U * 262U, static_cast<std::int8_t>(-1));
  map.data[0] = 100;
  map.data[163U * 262U - 1] = 50;
  server_context.topics().publish("map", map, krosc::QOS_LATCHED);

  krosc::Context client_context;
  std::atomic<bool> got_map{false};
  std::mutex mutex;
  std::condition_variable cv;
  client_context.topics().subscribe<krosc::msgs::OccupancyGrid>(
      "map",
      [&](const krosc::msgs::OccupancyGrid& message) {
        if (message.info.width == 163 && message.info.height == 262 && message.data.size() == 163U * 262U) {
          got_map.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_LATCHED);
  auto client = std::make_unique<krosc::net::Client>(client_context, "127.0.0.1", server.port());
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return got_map.load(); }));
}

KROSC_TEST(net_latched_map_replays_for_late_subscriber) {
  krosc::Context server_context;
  krosc::net::Server server(server_context, 0);
  krosc::msgs::OccupancyGrid map;
  map.header = krosc::msgs::Header{3.0, "map"};
  map.info.resolution = 0.05;
  map.info.width = 4;
  map.info.height = 4;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(16, static_cast<std::int8_t>(0));
  map.data[0] = 100;
  server_context.topics().publish("map", map, krosc::QOS_LATCHED);

  krosc::Context client_context;
  auto client = std::make_unique<krosc::net::Client>(client_context, "127.0.0.1", server.port());
  std::this_thread::sleep_for(500ms);  // viz처럼 연결이 먼저 되고 구독은 나중에
  std::atomic<bool> got_map{false};
  std::mutex mutex;
  std::condition_variable cv;
  client_context.topics().subscribe<krosc::msgs::OccupancyGrid>(
      "map",
      [&](const krosc::msgs::OccupancyGrid& message) {
        if (message.info.width == 4 && message.info.height == 4 && message.data.size() == 16) {
          got_map.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_LATCHED);
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 2s, [&] { return got_map.load(); }));
}
