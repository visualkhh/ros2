#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/math.hpp"
#include "krosc/nodes/diff_drive_odometry.hpp"
#include "krosc/nodes/robot_state_publisher.hpp"
#include "krosc/nodes/twist_mux.hpp"
#include "krosc/service.hpp"
#include "krosc/tf_ros.hpp"
#include "krosc/time.hpp"

#include "test_framework.hpp"

namespace {

const char* kUrdf = R"(
<robot name="test_bot">
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.01" rpy="0 0 0"/>
  </joint>
  <link name="base_link"/>
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_left_link"/>
    <origin xyz="0 0.144 0.023"/><axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_left_link"/>
  <joint name="scan_joint" type="fixed">
    <parent link="base_link"/><child link="base_scan"/>
    <origin xyz="-0.064 0 0.122"/>
  </joint>
  <link name="base_scan"/>
</robot>
)";

struct AddRequest {
  int a = 0;
  int b = 0;
};
struct AddResponse {
  int sum = 0;
};

}  // namespace

KROSC_TEST(service_roundtrip_and_errors) {
  krosc::Context context;
  krosc::Node server_node(context, "server");
  krosc::Node client_node(context, "client");
  auto server = server_node.create_service<AddRequest, AddResponse>(
      "add", [](const AddRequest& request) { return AddResponse{request.a + request.b}; });
  const AddResponse response = client_node.call_service<AddRequest, AddResponse>("add", AddRequest{2, 3});
  CHECK_EQ(5, response.sum);
  CHECK(context.services().has_service("add"));
  server.destroy();
  CHECK(!context.services().has_service("add"));
  CHECK_THROWS_AS((client_node.call_service<AddRequest, AddResponse>("add", AddRequest{1, 2})),
                  krosc::ServiceNotFoundError);
  auto first = server_node.create_service<AddRequest, AddResponse>("dup", [](const AddRequest&) { return AddResponse{}; });
  (void)first;
  CHECK_THROWS_AS((server_node.create_service<AddRequest, AddResponse>(
                      "dup", [](const AddRequest&) { return AddResponse{}; })),
                  std::runtime_error);
}

KROSC_TEST(robot_state_publisher_static_and_joint_tf) {
  krosc::Context::Options options;
  options.time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context context(options);
  krosc::nodes::RobotStatePublisher rsp(context, "rsp", {{"robot_description", std::string(kUrdf)}});
  krosc::Buffer buffer;
  krosc::Node listener_node(context, "listener");
  krosc::TransformListener listener(buffer, listener_node);
  CHECK(buffer.can_transform("base_link", "base_scan"));
  const auto static_transform = buffer.lookup_transform("base_link", "base_scan");
  CHECK_NEAR(-0.064, static_transform.transform.translation.x, 1e-9);
  CHECK_NEAR(0.122, static_transform.transform.translation.z, 1e-9);

  krosc::msgs::JointState joint_state;
  joint_state.header.stamp = 1.0;
  joint_state.name = {"wheel_left_joint"};
  joint_state.position = {0.5};
  listener_node.publish(krosc::JOINT_STATES_TOPIC_NAME, joint_state);
  const auto wheel = buffer.lookup_transform("base_link", "wheel_left_link", 1.0);
  const auto yaw = krosc::tfmath::to_rpy(wheel.transform.rotation).yaw;
  CHECK_NEAR(0.5, yaw, 1e-6);
}

KROSC_TEST(diff_drive_odometry_integrates_wheels) {
  krosc::Context::Options options;
  options.time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context context(options);
  krosc::nodes::DiffDriveOdometry odometry(context, "odom", {{"publish_tf", false}});
  krosc::msgs::Odometry latest;
  bool received = false;
  krosc::Node monitor(context, "monitor");
  monitor.subscribe<krosc::msgs::Odometry>("odom", [&](const krosc::msgs::Odometry& message) {
    latest = message;
    received = true;
  });
  krosc::msgs::JointState state;
  state.header.stamp = 0.0;
  state.name = {"wheel_left_joint", "wheel_right_joint"};
  state.position = {0.0, 0.0};
  monitor.publish(krosc::JOINT_STATES_TOPIC_NAME, state);
  state.header.stamp = 1.0;
  state.position = {1.0, 1.0};
  monitor.publish(krosc::JOINT_STATES_TOPIC_NAME, state);
  CHECK(received);
  CHECK_NEAR(0.033, latest.pose.pose.position.x, 1e-6);
  CHECK_NEAR(0.0, latest.pose.pose.position.y, 1e-6);
  CHECK_NEAR(0.033, latest.twist.twist.linear.x, 1e-6);
  CHECK_EQ(0u, odometry.jumps());
}

KROSC_TEST(twist_mux_priority_timeout_and_forwarding) {
  krosc::Context::Options options;
  options.time_source = std::make_shared<krosc::SimulatedTimeSource>();
  krosc::Context context(options);
  krosc::nodes::TwistMux mux(context, "mux",
                             {{"topics", std::vector<std::string>{"teleop"}},
                              {"cmd_vel_out", std::string("cmd_vel")}});
  krosc::msgs::Twist received;
  bool got = false;
  krosc::Node monitor(context, "mux_monitor");
  monitor.subscribe<krosc::msgs::Twist>("cmd_vel", [&](const krosc::msgs::Twist& twist) {
    received = twist;
    got = true;
  });
  monitor.publish("cmd_vel_teleop", krosc::msgs::Twist{{0.2, 0.0, 0.0}, {0.0, 0.0, 0.5}});
  CHECK(got);
  CHECK_NEAR(0.2, received.linear.x, 1e-12);
  CHECK_NEAR(0.5, received.angular.z, 1e-12);
}
