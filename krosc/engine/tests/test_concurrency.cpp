#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/nodes/nav2/behavior_server.hpp"
#include "krosc/nodes/nav2/bt_navigator.hpp"
#include "krosc/nodes/nav2/controller_server.hpp"
#include "krosc/nodes/nav2/costmap/costmap_2d_ros.hpp"
#include "krosc/nodes/nav2/planner_server.hpp"
#include "krosc/qos.hpp"
#include "krosc/tf/math.hpp"
#include "krosc/tf/tf_ros.hpp"

#include "test_framework.hpp"

namespace {

constexpr int kMapWidth = 80;
constexpr int kMapHeight = 60;
constexpr double kMapResolution = 0.05;

krosc::msgs::OccupancyGrid make_map() {
  krosc::msgs::OccupancyGrid map;
  map.header = krosc::msgs::Header{0.0, "map"};
  map.info.resolution = kMapResolution;
  map.info.width = kMapWidth;
  map.info.height = kMapHeight;
  map.info.origin.position = {0.0, 0.0, 0.0};
  map.data.assign(static_cast<std::size_t>(kMapWidth) * kMapHeight, 0);
  const int wall_x = static_cast<int>(1.0 / kMapResolution);
  for (int y = 0; y < kMapHeight; ++y) {
    if (y >= 18 && y < 30) {
      continue;
    }
    map.data[static_cast<std::size_t>(y) * kMapWidth + wall_x] = 100;
  }
  return map;
}

krosc::msgs::LaserScan make_scan(double stamp) {
  krosc::msgs::LaserScan scan;
  scan.header = krosc::msgs::Header{stamp, "base_scan"};
  scan.angle_min = 0.0;
  scan.angle_max = 2.0 * 3.14159265358979323846;
  scan.angle_increment = scan.angle_max / 360.0;
  scan.range_min = 0.05;
  scan.range_max = 12.0;
  scan.ranges.assign(360, 5.0F);
  for (int i = 0; i < 360; ++i) {
    if (i % 37 == 0) {
      scan.ranges[static_cast<std::size_t>(i)] = 0.8F;
    }
  }
  return scan;
}

template <class Predicate>
bool wait_until(Predicate predicate, double seconds = 3.0) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(seconds));
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

krosc::msgs::TransformStamped transform(const std::string& parent, const std::string& child, double x,
                                        double y, double yaw) {
  krosc::msgs::TransformStamped message;
  message.header = krosc::msgs::Header{1.0, parent};
  message.child_frame_id = child;
  message.transform.translation = {x, y, 0.0};
  message.transform.rotation = krosc::tfmath::from_yaw(yaw);
  return message;
}

}  // namespace

// TSan/동시성 스트레스 전용. 일반 테스트에서는 KROSC_STRESS가 없으면 건너뛴다.
//   KROSC_STRESS=1 TSAN_OPTIONS=halt_on_error=0 \
//     ./krosc_core_tests
KROSC_TEST(concurrency_nav2_stack_executor_10) {
  if (std::getenv("KROSC_STRESS") == nullptr) {
    return;
  }

  krosc::Context::Options options;
  options.executor_threads = 10;
  krosc::Context context(options);

  krosc::Node sim(context, "stress_sim");
  krosc::Node monitor(context, "stress_monitor");
  sim.publish("map", make_map(), krosc::QOS_LATCHED);
  krosc::StaticTransformBroadcaster static_tf(sim);
  static_tf.send_transform(transform("map", "odom", 0.0, 0.0, 0.0));

  krosc::nodes::nav2::Costmap2DROS global(
      context, "global_costmap",
      {{"plugins", std::vector<std::string>{"static_layer", "inflation_layer"}},
       {"robot_radius", 0.1},
       {"inflation_radius", 0.3},
       {"cost_scaling_factor", 3.0}});
  krosc::nodes::nav2::Costmap2DROS local(
      context, "local_costmap",
      {{"global_frame", "odom"},
       {"robot_base_frame", "base_footprint"},
       {"plugins", std::vector<std::string>{"obstacle_layer", "inflation_layer"}},
       {"rolling_window", true},
       {"width", 2.0},
       {"height", 2.0},
       {"resolution", kMapResolution},
       {"obstacle_max_range", 2.5},
       {"raytrace_max_range", 3.0}});
  krosc::nodes::nav2::PlannerServer planner(context, global, "planner_server",
                                            {{"planner_plugins", std::vector<std::string>{"GridBased"}}});
  krosc::nodes::nav2::ControllerServer controller(
      context, local, "controller_server",
      {{"controller_plugins", std::vector<std::string>{"FollowPath"}},
       {"cmd_vel_topic", "cmd_vel_nav"},
       {"controller_frequency", 50.0},
       {"max_linear_accel", 0.0},
       {"max_angular_accel", 0.0}});
  krosc::nodes::nav2::BehaviorServer behavior(context);
  krosc::nodes::nav2::BtNavigator bt(
      context, "bt_navigator",
      {{"replanning_period", 0.5}, {"number_of_recoveries", 1}});

  std::atomic<bool> stop{false};
  std::atomic<int> scans{0};
  std::atomic<int> odoms{0};
  std::atomic<int> plans{0};
  std::atomic<int> commands{0};

  monitor.subscribe<krosc::msgs::Path>("plan", [&](const krosc::msgs::Path&) { plans.fetch_add(1); });
  monitor.subscribe<krosc::msgs::Twist>("cmd_vel_nav",
                                       [&](const krosc::msgs::Twist&) { commands.fetch_add(1); });

  std::thread scan_thread([&] {
    krosc::Node node(context, "stress_scan");
    while (!stop.load()) {
      node.publish("scan", make_scan(node.now()), krosc::QOS_SENSOR_DATA);
      scans.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
  std::thread odom_thread([&] {
    krosc::Node node(context, "stress_odom");
    krosc::TransformBroadcaster broadcaster(node);
    int step = 0;
    while (!stop.load()) {
      const double x = 0.3 + 0.001 * static_cast<double>(step++ % 100);
      broadcaster.send_transform(transform("odom", "base_footprint", x, 1.2, 0.0));
      odoms.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  CHECK(wait_until([&] { return global.costmap() != nullptr; }, 3.0));
  CHECK(wait_until([&] { return scans.load() > 2 && odoms.load() > 5; }, 2.0));

  krosc::msgs::PoseStamped goal;
  goal.header = krosc::msgs::Header{sim.now(), "map"};
  goal.pose.position = {2.5, 1.2, 0.0};
  krosc::ActionClient<krosc::msgs::nav2::NavigateToPoseGoal,
                      krosc::msgs::nav2::NavigateToPoseFeedback,
                      krosc::msgs::nav2::NavigateToPoseResult>
      nav_client(context, krosc::msgs::nav2::NAVIGATE_TO_POSE_ACTION);
  const auto handle = nav_client.send_goal(krosc::msgs::nav2::NavigateToPoseGoal{goal, ""});
  CHECK(handle->accepted());
  CHECK(wait_until([&] { return plans.load() > 0; }, 3.0));
  CHECK(wait_until([&] { return commands.load() > 0; }, 3.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  CHECK(scans.load() > 10);
  CHECK(odoms.load() > 20);
  (void)handle->cancel();

  stop.store(true);
  scan_thread.join();
  odom_thread.join();
  CHECK(wait_until([&] { return global.costmap() != nullptr; }, 1.0));
  context.shutdown();
}

namespace {

// 콜백 안에 머무는 동안 같은 노드의 다른 콜백이 들어오는지 센다.
struct Overlap {
  std::atomic<int> inside{0};
  std::atomic<int> peak{0};
  std::atomic<int> calls{0};
  void enter() {
    const int now = inside.fetch_add(1) + 1;
    int previous = peak.load();
    while (now > previous && !peak.compare_exchange_weak(previous, now)) {
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
    inside.fetch_sub(1);
    calls.fetch_add(1);
  }
};

}  // namespace

KROSC_TEST(strand_node_callbacks_never_overlap_but_nodes_run_in_parallel) {
  krosc::Context::Options options;
  options.executor_threads = 4;
  krosc::Context context(options);
  krosc::Node node_a(context, "strand_a");
  krosc::Node node_b(context, "strand_b");
  Overlap a;
  Overlap b;
  Overlap both;  // 두 노드를 합쳐 동시에 몇 개가 돌았는지
  const auto track = [&both](Overlap& own) {
    const int now = both.inside.fetch_add(1) + 1;
    int previous = both.peak.load();
    while (now > previous && !both.peak.compare_exchange_weak(previous, now)) {
    }
    own.enter();
    both.inside.fetch_sub(1);
  };
  // 노드마다 서로 다른 토픽 구독 2개 + 타이머 1개: 기본(MutuallyExclusive)이면 한 줄로 돈다.
  node_a.subscribe<int>("a1", [&](const int&) { track(a); }, krosc::QOS_KEEP_ALL);
  node_a.subscribe<int>("a2", [&](const int&) { track(a); }, krosc::QOS_KEEP_ALL);
  node_a.create_timer(0.001, [&] { track(a); });
  node_b.subscribe<int>("b1", [&](const int&) { track(b); }, krosc::QOS_KEEP_ALL);
  node_b.subscribe<int>("b2", [&](const int&) { track(b); }, krosc::QOS_KEEP_ALL);
  krosc::Node publisher(context, "strand_pub");
  for (int i = 0; i < 100; ++i) {
    publisher.publish<int>("a1", i, krosc::QOS_KEEP_ALL);
    publisher.publish<int>("a2", i, krosc::QOS_KEEP_ALL);
    publisher.publish<int>("b1", i, krosc::QOS_KEEP_ALL);
    publisher.publish<int>("b2", i, krosc::QOS_KEEP_ALL);
  }
  context.executor().join();
  CHECK_EQ(1, a.peak.load());
  CHECK_EQ(1, b.peak.load());
  CHECK(a.calls.load() >= 200);
  CHECK_EQ(200, b.calls.load());
  CHECK(both.peak.load() >= 2);  // 다른 노드끼리는 같은 워커 풀에서 동시에 돈다
  context.shutdown();
}

KROSC_TEST(strand_reentrant_group_can_overlap) {
  krosc::Context::Options options;
  options.executor_threads = 4;
  krosc::Context context(options);
  krosc::Node node(context, "reentrant");
  Overlap overlap;
  node.subscribe<int>("r1", [&](const int&) { overlap.enter(); }, krosc::QOS_KEEP_ALL, krosc::CallbackGroup::Reentrant);
  node.subscribe<int>("r2", [&](const int&) { overlap.enter(); }, krosc::QOS_KEEP_ALL, krosc::CallbackGroup::Reentrant);
  for (int i = 0; i < 100; ++i) {
    node.publish<int>("r1", i);
    node.publish<int>("r2", i);
  }
  context.executor().join();
  CHECK_EQ(200, overlap.calls.load());
  CHECK(overlap.peak.load() >= 2);
  context.shutdown();
}

KROSC_TEST(strand_inline_publish_to_self_runs_after_current_callback) {
  // inline 실행기: 콜백 안에서 자기 노드로 발행해도 재진입하지 않고, 지금 콜백이 끝난 뒤 이어서 돈다.
  krosc::Context context;
  krosc::Node node(context, "inline_strand");
  std::vector<std::string> order;
  node.subscribe<int>("first", [&](const int&) {
    order.push_back("first:begin");
    node.publish<int>("second", 1);
    order.push_back("first:end");
  });
  node.subscribe<int>("second", [&](const int&) { order.push_back("second"); });
  node.publish<int>("first", 1);
  CHECK_EQ(3u, order.size());
  CHECK_EQ(std::string("first:begin"), order[0]);
  CHECK_EQ(std::string("first:end"), order[1]);
  CHECK_EQ(std::string("second"), order[2]);
}
