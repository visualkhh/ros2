#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/node.hpp"
#include "krosc/simulator/worlds/maze_world.hpp"
#include "krosc/tf/tf.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

int main() {
  {
    krosc::Executor executor(0);
    krosc::TopicRegistry topics(executor);
    std::atomic<std::uint64_t> received{0};
    topics.subscribe<krosc::msgs::String>("bench", [&](const krosc::msgs::String&) {
      received.fetch_add(1, std::memory_order_relaxed);
    });
    constexpr int kCount = 1'000'000;
    const auto start = Clock::now();
    for (int i = 0; i < kCount; ++i) {
      topics.publish("bench", krosc::msgs::String{"hello"});
    }
    const double elapsed = seconds_since(start);
    std::printf("pub/sub inline      : %8.1f ns/op  (%llu received)\n", elapsed / kCount * 1e9,
                static_cast<unsigned long long>(received.load()));
  }
  {
    krosc::Executor executor(4);
    krosc::TopicRegistry topics(executor);
    std::atomic<std::uint64_t> received{0};
    topics.subscribe<krosc::msgs::String>("bench", [&](const krosc::msgs::String&) {
      received.fetch_add(1, std::memory_order_relaxed);
    }, krosc::QOS_DEFAULT);
    constexpr int kCount = 200'000;
    const auto start = Clock::now();
    for (int i = 0; i < kCount; ++i) {
      topics.publish("bench", krosc::msgs::String{"hello"});
    }
    executor.join();
    const double elapsed = seconds_since(start);
    std::printf("pub/sub pool(4)     : %8.1f ns/op  (%llu received, all delivered)\n", elapsed / kCount * 1e9,
                static_cast<unsigned long long>(received.load()));
  }
  {
    krosc::Buffer buffer;
    for (int i = 0; i < 300; ++i) {
      krosc::msgs::Transform transform;
      transform.translation = {static_cast<double>(i) * 0.01, 0.0, 0.0};
      buffer.set_transform("odom", "base", static_cast<double>(i) * 0.01, transform);
    }
    constexpr int kCount = 1'000'000;
    const auto start = Clock::now();
    double sum = 0.0;
    for (int i = 0; i < kCount; ++i) {
      sum += buffer.lookup_transform("odom", "base").transform.translation.x;
    }
    const double elapsed = seconds_since(start);
    std::printf("tf lookup           : %8.1f ns/op  (sum %.3f)\n", elapsed / kCount * 1e9, sum);
  }
  {
    krosc::simulator::MazeWorld world(0);
    std::vector<double> angles(360);
    for (int i = 0; i < 360; ++i) {
      angles[static_cast<std::size_t>(i)] = static_cast<double>(i) * 0.017453292519943295;
    }
    std::vector<float> ranges;
    constexpr int kCount = 10'000;
    const auto start = Clock::now();
    for (int i = 0; i < kCount; ++i) {
      world.ray_distances(0.0, 0.0, angles, 0.132, ranges);
    }
    const double elapsed = seconds_since(start);
    std::printf("lidar 360 rays      : %8.1f us/scan\n", elapsed / kCount * 1e6);
  }
  return 0;
}
