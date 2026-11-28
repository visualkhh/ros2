#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <cstring>
#include <set>
#include <thread>

#include "krosc/action.hpp"
#include "krosc/context.hpp"
#include "krosc/msgs/nav2_msgs.hpp"
#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/net/topic_codec.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter_rpc.hpp"
#include "krosc/net/rpc.hpp"
#include "krosc/qos.hpp"

#include "test_framework.hpp"

using namespace std::chrono_literals;

KROSC_TEST(net_protocol_frame_roundtrip) {
  // 웹 게이트웨이와 브라우저가 같은 body 포맷을 쓴다.
  krosc::net::Frame frame;
  frame.type = krosc::net::kFrameTopic;
  frame.topic = "moveit_target";
  frame.kind = krosc::net::Kind::String;
  frame.payload = {1, 2, 3, 4};

  const auto body = krosc::net::encode_body(frame);
  CHECK_EQ(body.size(), 3U + frame.topic.size() + 1U + frame.payload.size());
  const auto decoded = krosc::net::decode_body(body);
  CHECK(decoded.has_value());
  CHECK_EQ(frame.type, decoded->type);
  CHECK_EQ(frame.topic, decoded->topic);
  CHECK(frame.kind == decoded->kind);
  CHECK(frame.payload == decoded->payload);
  CHECK_EQ(frame.topic, krosc::net::frame_topic(body).value());

  const auto encoded = krosc::net::encode_frame(frame);
  CHECK_EQ(encoded.size(), body.size() + 4U);
  CHECK_EQ(static_cast<std::uint32_t>(body.size()),
           static_cast<std::uint32_t>(encoded[0]) |
               (static_cast<std::uint32_t>(encoded[1]) << 8) |
               (static_cast<std::uint32_t>(encoded[2]) << 16) |
               (static_cast<std::uint32_t>(encoded[3]) << 24));
}

KROSC_TEST(net_message_codec_roundtrip) {
  krosc::msgs::Path path;
  path.header = krosc::msgs::Header{12.5, "map"};
  krosc::msgs::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position = {1.0, 2.0, 0.0};
  path.poses.push_back(pose);

  const auto* codec = krosc::net::find_topic_type(std::type_index(typeid(krosc::msgs::Path)));
  CHECK(codec != nullptr);
  CHECK(codec == krosc::net::find_topic_type(krosc::net::Kind::Path));
  krosc::net::RpcWriter writer;
  codec->encode(writer, &path);
  CHECK(!writer.data.empty());
  const auto decoded = codec->decode(writer.data.data(), writer.data.size());
  CHECK(codec->decode(writer.data.data(), writer.data.size() - 1) == nullptr);  // 잘린 데이터는 거부
  CHECK(decoded != nullptr);
  const auto typed = std::static_pointer_cast<const krosc::msgs::Path>(decoded);
  CHECK_EQ(1U, typed->poses.size());
  CHECK_NEAR(1.0, typed->poses[0].pose.position.x, 1e-9);
  CHECK_EQ(std::string("map"), typed->header.frame_id);
}

namespace {

krosc::Context::Options peer_server_options(const std::string& name) {
  krosc::Context::Options options;
  options.network.name = name;
  options.network.host = "127.0.0.1";
  options.network.port = 0;
  return options;
}

krosc::Context::Options peer_client_options(const std::string& name, std::uint16_t port) {
  krosc::Context::Options options;
  options.network.name = name;
  options.network.servers = {"127.0.0.1:" + std::to_string(port)};
  return options;
}

}  // namespace

KROSC_TEST(net_peer_roundtrip_topics_and_teleop) {
  krosc::Context server_context(peer_server_options("robot"));

  // latched: 클라이언트가 나중에 붙어도 받아야 한다.
  server_context.topics().publish("robot_description", krosc::msgs::String{"<robot name='net_test'/>"},
                                  krosc::QOS_LATCHED);
  krosc::msgs::OccupancyGrid map;
  map.header = krosc::msgs::Header{0.0, "map"};
  map.info.resolution = 0.05;
  map.info.width = 2;
  map.info.height = 2;
  map.info.origin.orientation.w = 1.0;
  map.data = {0, 100, -1, 50};
  server_context.topics().publish("map", map, krosc::QOS_LATCHED);

  krosc::Context client_context(peer_client_options("viz", server_context.peer_network()->port()));
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
        if (message.info.width == 2 && message.data.size() == 4 && message.data[1] == 100) {
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

  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return got_description.load() && got_map.load(); }));

  krosc::msgs::LaserScan scan;
  scan.header = krosc::msgs::Header{1.0, "base_scan"};
  scan.ranges = {1.0F, 2.0F, 3.0F};
  for (int i = 0; i < 20 && !got_scan.load(); ++i) {
    server_context.topics().publish("scan", scan, krosc::QOS_SENSOR_DATA);
    cv.wait_for(lock, 50ms);
  }
  CHECK(got_scan.load());

  lock.unlock();
  client_context.shutdown();
  server_context.shutdown();
}

KROSC_TEST(net_peer_teleop_receive_remap) {
  // 로봇은 원격 cmd_vel을 cmd_vel_teleop으로 받는다 (krosc_ws/engine/main.cpp와 같은 설정).
  auto server_options = peer_server_options("robot");
  server_options.network.receive_remap["cmd_vel"] = "cmd_vel_teleop";
  krosc::Context server_context(server_options);
  krosc::Context client_context(peer_client_options("viz", server_context.peer_network()->port()));

  std::atomic<bool> got_command{false};
  std::atomic<double> command_linear{0.0};
  std::mutex mutex;
  std::condition_variable cv;
  server_context.topics().subscribe<krosc::msgs::Twist>(
      "cmd_vel_teleop",
      [&](const krosc::msgs::Twist& twist) {
        command_linear.store(twist.linear.x);
        got_command.store(true);
        cv.notify_all();
      },
      krosc::QOS_COMMAND);
  std::unique_lock<std::mutex> lock(mutex);
  for (int i = 0; i < 40 && !got_command.load(); ++i) {
    client_context.topics().publish("cmd_vel", krosc::msgs::Twist{{0.17, 0.0, 0.0}, {0.0, 0.0, -0.4}},
                                    krosc::QOS_COMMAND);
    cv.wait_for(lock, 50ms);
  }
  CHECK(got_command.load());
  CHECK_NEAR(0.17, command_linear.load(), 1e-9);
  lock.unlock();
  client_context.shutdown();
  server_context.shutdown();
}

KROSC_TEST(net_peer_large_latched_map_roundtrip) {
  krosc::Context server_context(peer_server_options("robot"));
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

  krosc::Context client_context(peer_client_options("viz", server_context.peer_network()->port()));
  std::atomic<bool> got_map{false};
  std::mutex mutex;
  std::condition_variable cv;
  client_context.topics().subscribe<krosc::msgs::OccupancyGrid>(
      "map",
      [&](const krosc::msgs::OccupancyGrid& message) {
        if (message.info.width == 163 && message.data.size() == 163U * 262U && message.data.back() == 50) {
          got_map.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_LATCHED);
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return got_map.load(); }));
  lock.unlock();
  client_context.shutdown();
  server_context.shutdown();
}

KROSC_TEST(net_peer_tf_static_keeps_every_broadcaster) {
  // tf_static은 발행자마다 메시지가 따로다. 발행자 QoS(depth)가 그대로 전달되어
  // 늦게 붙은 피어도 모든 고정 변환을 받아야 한다 (이름 추측으로 depth 1이 되면 하나만 남는다).
  krosc::Context server_context(peer_server_options("robot"));
  const auto stamped = [](const std::string& parent, const std::string& child) {
    return krosc::msgs::TFMessage{{krosc::msgs::TransformStamped{
        krosc::msgs::Header{0.0, parent}, child, krosc::msgs::Transform{{0.1, 0.0, 0.0}, krosc::msgs::Quaternion{}}}}};
  };
  server_context.topics().publish("tf_static", stamped("base_link", "base_scan"), krosc::QOS_TF_STATIC);
  server_context.topics().publish("tf_static", stamped("base_link", "imu_link"), krosc::QOS_TF_STATIC);
  server_context.topics().publish("tf_static", stamped("base_link", "camera_link"), krosc::QOS_TF_STATIC);

  krosc::Context client_context(peer_client_options("viz", server_context.peer_network()->port()));
  std::mutex mutex;
  std::condition_variable cv;
  std::set<std::string> children;
  client_context.topics().subscribe<krosc::msgs::TFMessage>(
      "tf_static",
      [&](const krosc::msgs::TFMessage& message) {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& transform : message.transforms) {
          children.insert(transform.child_frame_id);
        }
        cv.notify_all();
      },
      krosc::QOS_TF_STATIC);
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return children.size() == 3; }));
  lock.unlock();

  // 원격에서 다시 발행된 tf_static도 depth를 유지해, 그 프로세스의 늦은 구독자도 셋 다 받는다.
  std::set<std::string> late_children;
  client_context.topics().subscribe<krosc::msgs::TFMessage>(
      "tf_static",
      [&](const krosc::msgs::TFMessage& message) {
        std::lock_guard<std::mutex> late_lock(mutex);
        for (const auto& transform : message.transforms) {
          late_children.insert(transform.child_frame_id);
        }
        cv.notify_all();
      },
      krosc::QOS_TF_STATIC);
  lock.lock();
  CHECK(cv.wait_for(lock, 3s, [&] { return late_children.size() == 3; }));
  lock.unlock();
  client_context.shutdown();
  server_context.shutdown();
}

KROSC_TEST(net_peer_local_only_and_host) {
  krosc::Context local;
  CHECK(local.peer_network() == nullptr);

  krosc::Context::Options host_options;
  host_options.network.host = "127.0.0.1";
  host_options.network.port = 0;  // 임시 포트
  krosc::Context host(host_options);
  CHECK(host.peer_network() != nullptr);
  CHECK(host.peer_network()->port() != 0);
  CHECK(host.peer_network()->peer_endpoints().empty());
}

KROSC_TEST(net_peer_topic_mesh_and_latched) {
  krosc::Context::Options server_options;
  server_options.network.name = "peer_server";
  server_options.network.host = "127.0.0.1";
  server_options.network.port = 0;
  krosc::Context server(server_options);
  CHECK(server.peer_network() != nullptr);
  CHECK(server.peer_network()->port() != 0);

  krosc::Context::Options client_options;
  client_options.network.name = "peer_client";
  client_options.network.servers = {"127.0.0.1:" + std::to_string(server.peer_network()->port())};
  krosc::Context client(client_options);
  CHECK(client.peer_network() != nullptr);

  std::atomic<bool> got{false};
  std::mutex mutex;
  std::condition_variable cv;
  client.topics().subscribe<krosc::msgs::String>(
      "chatter",
      [&](const krosc::msgs::String& message) {
        if (message.data == "hello peer") {
          got.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_DEFAULT);

  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return !client.peer_network()->peer_endpoints().empty(); }));
  // discover/subscribe가 오갈 시간을 준다.
  std::this_thread::sleep_for(200ms);
  for (int i = 0; i < 10 && !got.load(); ++i) {
    server.topics().publish("chatter", krosc::msgs::String{"hello peer"}, krosc::QOS_DEFAULT);
    cv.wait_for(lock, 50ms);
  }
  CHECK(got.load());

  // latched: 구독 전에 발행된 값도 새 구독자에게 재전송되어야 한다.
  server.topics().publish("map_status", krosc::msgs::String{"ready"}, krosc::QOS_LATCHED);
  std::atomic<bool> latched{false};
  client.topics().subscribe<krosc::msgs::String>(
      "map_status",
      [&](const krosc::msgs::String& message) {
        if (message.data == "ready") {
          latched.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_LATCHED);
  CHECK(cv.wait_for(lock, 3s, [&] { return latched.load(); }));

  lock.unlock();
  client.shutdown();
  server.shutdown();
}

KROSC_TEST(net_peer_replays_latched_published_before_connect) {
  // 로봇이 먼저 뜨고 Viz가 나중에 붙는 실제 순서. map/robot_description은 연결 전에
  // 한 번만 발행되므로, 새 peer가 구독할 때 latched 캐시에서 재전송되어야 한다.
  krosc::Context::Options server_options;
  server_options.network.name = "late_server";
  server_options.network.host = "127.0.0.1";
  server_options.network.port = 0;
  krosc::Context server(server_options);
  server.topics().publish("map_status", krosc::msgs::String{"ready-before"}, krosc::QOS_LATCHED);

  krosc::Context::Options client_options;
  client_options.network.name = "late_client";
  client_options.network.servers = {"127.0.0.1:" + std::to_string(server.peer_network()->port())};
  krosc::Context client(client_options);

  std::atomic<bool> got{false};
  std::mutex mutex;
  std::condition_variable cv;
  client.topics().subscribe<krosc::msgs::String>(
      "map_status",
      [&](const krosc::msgs::String& message) {
        if (message.data == "ready-before") {
          got.store(true);
        }
        cv.notify_all();
      },
      krosc::QOS_LATCHED);
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return got.load(); }));
  lock.unlock();
  client.shutdown();
  server.shutdown();
}

KROSC_TEST(net_peer_receive_remap_cmd_vel) {
  krosc::Context::Options robot_options;
  robot_options.network.name = "remap_robot";
  robot_options.network.host = "127.0.0.1";
  robot_options.network.port = 0;
  robot_options.network.receive_remap["cmd_vel"] = "cmd_vel_teleop";
  krosc::Context robot(robot_options);
  CHECK(robot.peer_network() != nullptr);

  krosc::Context::Options viz_options;
  viz_options.network.name = "remap_viz";
  viz_options.network.add_server("127.0.0.1", robot.peer_network()->port());
  krosc::Context viz(viz_options);
  CHECK(viz.peer_network() != nullptr);

  std::atomic<double> teleop_linear{0.0};
  std::mutex mutex;
  std::condition_variable cv;
  robot.topics().subscribe<krosc::msgs::Twist>(
      "cmd_vel_teleop",
      [&](const krosc::msgs::Twist& message) {
        teleop_linear.store(message.linear.x);
        cv.notify_all();
      },
      krosc::QOS_COMMAND);

  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return !viz.peer_network()->peer_endpoints().empty(); }));
  std::this_thread::sleep_for(300ms);
  krosc::msgs::Twist twist;
  twist.linear.x = 0.23;
  for (int i = 0; i < 20 && std::abs(teleop_linear.load() - 0.23) > 1e-9; ++i) {
    viz.topics().publish("cmd_vel", twist, krosc::QOS_COMMAND);
    cv.wait_for(lock, 100ms);
  }
  CHECK_NEAR(0.23, teleop_linear.load(), 1e-9);

  lock.unlock();
  viz.shutdown();
  robot.shutdown();
}


namespace {

void register_test_action_codec() {
  auto encode_int = [](const std::any& value) {
    const int number = std::any_cast<int>(value);
    std::vector<std::uint8_t> bytes(4);
    std::memcpy(bytes.data(), &number, sizeof(number));
    return bytes;
  };
  auto decode_int = [](const std::uint8_t* data, std::size_t size) -> std::any {
    if (size != sizeof(int)) {
      return {};
    }
    int number = 0;
    std::memcpy(&number, data, sizeof(number));
    return std::any(number);
  };
  krosc::net::ActionCodec codec;
  codec.name = "test_action";
  codec.encode_goal = encode_int;
  codec.decode_goal = decode_int;
  codec.encode_feedback = encode_int;
  codec.decode_feedback = decode_int;
  codec.encode_result = encode_int;
  codec.decode_result = decode_int;
  krosc::net::register_action_codec(std::move(codec));
}

}  // namespace

KROSC_TEST(net_rpc_service_and_parameter_services) {
  register_test_action_codec();

  krosc::Context::Options server_options;
  server_options.network.name = "rpc_server";
  server_options.network.host = "127.0.0.1";
  server_options.network.port = 0;
  krosc::Context server(server_options);
  krosc::Node server_node(server, "rpc_param_node");
  auto answer = server_node.declare_parameter<std::int64_t>("answer", 1);
  auto add_service = server_node.create_service<krosc::msgs::SetBoolRequest,
                                                krosc::msgs::SetBoolResponse>(
      "test/add", [](const krosc::msgs::SetBoolRequest& request) {
        return krosc::msgs::SetBoolResponse{request.data, request.data ? "yes" : "no"};
      });

  krosc::Context::Options client_options;
  client_options.network.name = "rpc_client";
  client_options.network.rpc_timeout_sec = 0.4;
  client_options.network.servers = {"127.0.0.1:" + std::to_string(server.peer_network()->port())};
  krosc::Context client(client_options);

  std::mutex mutex;
  std::condition_variable cv;
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return !client.peer_network()->peer_endpoints().empty(); }));
  std::this_thread::sleep_for(100ms);

  const auto response = client.services().call<krosc::msgs::SetBoolRequest,
                                               krosc::msgs::SetBoolResponse>(
      "test/add", krosc::msgs::SetBoolRequest{true});
  CHECK(response.success);
  CHECK_EQ(std::string("yes"), response.message);

  krosc::msgs::rcl_interfaces::SetParametersRequest update;
  krosc::msgs::rcl_interfaces::Parameter parameter;
  parameter.name = "answer";
  parameter.value.type = static_cast<std::uint8_t>(
      krosc::msgs::rcl_interfaces::ParameterType::Integer);
  parameter.value.integer_value = 42;
  update.parameters.push_back(parameter);
  const auto set_response = client.services().call<
      krosc::msgs::rcl_interfaces::SetParametersRequest,
      krosc::msgs::rcl_interfaces::SetParametersResponse>(
      "rpc_param_node/set_parameters", update);
  CHECK_EQ(1U, set_response.results.size());
  CHECK(set_response.results[0].successful);
  CHECK_EQ(std::int64_t{42}, answer.value());

  krosc::msgs::rcl_interfaces::GetParametersRequest query;
  query.names = {"answer"};
  const auto get_response = client.services().call<
      krosc::msgs::rcl_interfaces::GetParametersRequest,
      krosc::msgs::rcl_interfaces::GetParametersResponse>(
      "rpc_param_node/get_parameters", query);
  CHECK_EQ(1U, get_response.values.size());
  CHECK_EQ(std::int64_t{42}, get_response.values[0].integer_value);

  bool missing_failed = false;
  try {
    (void)client.services().call<krosc::msgs::Empty, krosc::msgs::Empty>(
        "missing_service", krosc::msgs::Empty{});
  } catch (const std::exception&) {
    missing_failed = true;
  }
  CHECK(missing_failed);

  lock.unlock();
  client.shutdown();
  server.shutdown();
}

KROSC_TEST(net_rpc_nav2_actions_roundtrip) {
  krosc::Context::Options server_options;
  server_options.network.name = "nav2_action_server";
  server_options.network.host = "127.0.0.1";
  server_options.network.port = 0;
  krosc::Context server(server_options);

  using NavigateServer = krosc::ActionServer<krosc::msgs::nav2::NavigateToPoseGoal,
                                             krosc::msgs::nav2::NavigateToPoseFeedback,
                                             krosc::msgs::nav2::NavigateToPoseResult>;
  NavigateServer navigate_server(
      server, krosc::msgs::nav2::NAVIGATE_TO_POSE_ACTION,
      [](krosc::ServerGoalHandle<krosc::msgs::nav2::NavigateToPoseGoal,
                                krosc::msgs::nav2::NavigateToPoseFeedback>& handle) {
        krosc::msgs::nav2::NavigateToPoseFeedback feedback;
        feedback.current_pose = handle.goal().pose;
        feedback.distance_remaining = 1.5;
        handle.publish_feedback(feedback);
        krosc::msgs::nav2::NavigateToPoseResult result;
        result.error_code = krosc::msgs::nav2::NavigateToPoseResult::NONE;
        return result;
      });
  using ThroughServer = krosc::ActionServer<krosc::msgs::nav2::NavigateThroughPosesGoal,
                                            krosc::msgs::nav2::NavigateThroughPosesFeedback,
                                            krosc::msgs::nav2::NavigateThroughPosesResult>;
  ThroughServer through_server(
      server, krosc::msgs::nav2::NAVIGATE_THROUGH_POSES_ACTION,
      [](krosc::ServerGoalHandle<krosc::msgs::nav2::NavigateThroughPosesGoal,
                                krosc::msgs::nav2::NavigateThroughPosesFeedback>& handle) {
        krosc::msgs::nav2::NavigateThroughPosesFeedback feedback;
        feedback.current_waypoint = static_cast<std::uint32_t>(handle.goal().poses.size() - 1);
        handle.publish_feedback(feedback);
        krosc::msgs::nav2::NavigateThroughPosesResult result;
        result.error_code = krosc::msgs::nav2::NavigateThroughPosesResult::NONE;
        return result;
      });

  krosc::Context::Options client_options;
  client_options.network.name = "nav2_action_client";
  client_options.network.rpc_timeout_sec = 1.0;
  client_options.network.servers = {"127.0.0.1:" + std::to_string(server.peer_network()->port())};
  krosc::Context client(client_options);
  krosc::ActionClient<krosc::msgs::nav2::NavigateToPoseGoal,
                      krosc::msgs::nav2::NavigateToPoseFeedback,
                      krosc::msgs::nav2::NavigateToPoseResult>
      navigate_client(client, krosc::msgs::nav2::NAVIGATE_TO_POSE_ACTION);
  krosc::ActionClient<krosc::msgs::nav2::NavigateThroughPosesGoal,
                      krosc::msgs::nav2::NavigateThroughPosesFeedback,
                      krosc::msgs::nav2::NavigateThroughPosesResult>
      through_client(client, krosc::msgs::nav2::NAVIGATE_THROUGH_POSES_ACTION);

  std::mutex mutex;
  std::condition_variable cv;
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return !client.peer_network()->peer_endpoints().empty(); }));
  std::this_thread::sleep_for(100ms);

  krosc::msgs::nav2::NavigateToPoseGoal goal;
  goal.pose.header = krosc::msgs::Header{1.0, "map"};
  goal.pose.pose.position = {1.0, 2.0, 0.0};
  std::atomic<double> feedback_distance{0.0};
  auto navigate = navigate_client.send_goal(goal, [&](const krosc::msgs::nav2::NavigateToPoseFeedback& feedback) {
    feedback_distance.store(feedback.distance_remaining);
    cv.notify_all();
  });
  const auto navigate_result = navigate->result(3.0);
  CHECK(navigate_result.status == krosc::GoalStatus::Succeeded);
  CHECK_EQ(1.5, feedback_distance.load());

  krosc::msgs::nav2::NavigateThroughPosesGoal through_goal;
  through_goal.poses.push_back(goal.pose);
  krosc::msgs::PoseStamped second;
  second.header = goal.pose.header;
  second.pose.position = {3.0, 4.0, 0.0};
  through_goal.poses.push_back(second);
  std::atomic<int> current_waypoint{-1};
  auto through = through_client.send_goal(
      through_goal, [&](const krosc::msgs::nav2::NavigateThroughPosesFeedback& feedback) {
        current_waypoint.store(static_cast<int>(feedback.current_waypoint));
        cv.notify_all();
      });
  const auto through_result = through->result(3.0);
  CHECK(through_result.status == krosc::GoalStatus::Succeeded);
  CHECK_EQ(1, current_waypoint.load());

  lock.unlock();
  navigate_client.shutdown();
  through_client.shutdown();
  navigate_server.shutdown();
  through_server.shutdown();
  client.shutdown();
  server.shutdown();
}

KROSC_TEST(net_rpc_action_roundtrip) {
  register_test_action_codec();

  krosc::Context::Options server_options;
  server_options.network.name = "action_server";
  server_options.network.host = "127.0.0.1";
  server_options.network.port = 0;
  krosc::Context server(server_options);
  krosc::ActionServer<int, int, int> action_server(
      server, "test_action", [](krosc::ServerGoalHandle<int, int>& handle) {
        handle.publish_feedback(handle.goal() * 2);
        return handle.goal() + 1;
      });

  krosc::Context::Options client_options;
  client_options.network.name = "action_client";
  client_options.network.rpc_timeout_sec = 1.0;
  client_options.network.servers = {"127.0.0.1:" + std::to_string(server.peer_network()->port())};
  krosc::Context client(client_options);
  std::atomic<int> feedback{0};
  krosc::ActionClient<int, int, int> action_client(client, "test_action");

  std::mutex mutex;
  std::condition_variable cv;
  std::unique_lock<std::mutex> lock(mutex);
  CHECK(cv.wait_for(lock, 3s, [&] { return !client.peer_network()->peer_endpoints().empty(); }));
  std::this_thread::sleep_for(100ms);

  auto handle = action_client.send_goal(7, [&](const int& value) {
    feedback.store(value);
    cv.notify_all();
  });
  const auto result = handle->result(3.0);
  CHECK(result.status == krosc::GoalStatus::Succeeded);
  CHECK(result.result.has_value());
  CHECK_EQ(8, std::any_cast<int>(result.result));
  CHECK_EQ(14, feedback.load());

  lock.unlock();
  action_client.shutdown();
  action_server.shutdown();
  client.shutdown();
  server.shutdown();
}
