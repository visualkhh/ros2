#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/context.hpp"
#include "krosc/nodes/moveit/move_group.hpp"
#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/ros2_control/controller_manager.hpp"
#include "krosc/urdf.hpp"

#include "test_framework.hpp"

#ifndef KROSC_WS_URDF_PATH
#define KROSC_WS_URDF_PATH "robot/description/urdf/turtlebot3_real.urdf"
#endif

namespace {

using krosc::ParamValue;

const std::vector<std::string>& arm_joints() {
  static const std::vector<std::string> joints{"joint1", "joint2", "joint3", "joint4",
                                               "gripper_left_joint"};
  return joints;
}

class MockArmSystem final : public krosc::ros2_control::SystemInterface {
 public:
  MockArmSystem() {
    for (const auto& joint : arm_joints()) {
      state_[joint] = std::make_shared<krosc::ros2_control::StateInterface>(joint + "/position");
      command_[joint] = std::make_shared<krosc::ros2_control::CommandInterface>(joint + "/position");
    }
    wheel_left_ = std::make_shared<krosc::ros2_control::StateInterface>("wheel_left_joint/position");
    wheel_right_ = std::make_shared<krosc::ros2_control::StateInterface>("wheel_right_joint/position");
  }

  std::vector<std::shared_ptr<krosc::ros2_control::StateInterface>> export_state_interfaces() override {
    std::vector<std::shared_ptr<krosc::ros2_control::StateInterface>> result;
    for (const auto& [joint, state] : state_) {
      (void)joint;
      result.push_back(state);
    }
    result.push_back(wheel_left_);
    result.push_back(wheel_right_);
    return result;
  }

  std::vector<std::shared_ptr<krosc::ros2_control::CommandInterface>> export_command_interfaces() override {
    std::vector<std::shared_ptr<krosc::ros2_control::CommandInterface>> result;
    for (const auto& [joint, command] : command_) {
      (void)joint;
      result.push_back(command);
    }
    return result;
  }

  bool read(double time, double period) override {
    (void)time;
    (void)period;
    for (const auto& [joint, command] : command_) {
      state_[joint]->set(command->get());
    }
    return true;
  }

  bool write(double time, double period) override {
    (void)time;
    (void)period;
    return true;
  }

  [[nodiscard]] double command(const std::string& joint) const {
    const auto it = command_.find(joint);
    return it == command_.end() ? 0.0 : it->second->get();
  }

 private:
  std::map<std::string, std::shared_ptr<krosc::ros2_control::StateInterface>> state_;
  std::map<std::string, std::shared_ptr<krosc::ros2_control::CommandInterface>> command_;
  std::shared_ptr<krosc::ros2_control::StateInterface> wheel_left_;
  std::shared_ptr<krosc::ros2_control::StateInterface> wheel_right_;
};

std::string read_text(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    return {};
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

template <class Predicate>
bool wait_until(Predicate predicate, double seconds = 5.0) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                            std::chrono::duration<double>(seconds));
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

}  // namespace

KROSC_TEST(moveit_ros2_control_real_arm_pipeline) {
  const std::string urdf = read_text(KROSC_WS_URDF_PATH);
  CHECK(!urdf.empty());

  std::string chain_error;
  const auto robot = krosc::parse_urdf(urdf);
  auto chain = krosc::manipulation::ArmChain::from_urdf(
      robot, "base_link", "end_effector_link",
      {"joint1", "joint2", "joint3", "joint4"}, chain_error);
  CHECK(chain.has_value());
  if (!chain) {
    return;
  }
  const std::array<double, 4> target_joints{0.3, -0.5, 0.4, 0.2};
  const krosc::msgs::Vector3 target = chain->tcp(target_joints);
  const double pitch_deg = chain->pitch_down(target_joints) * 180.0 / 3.14159265358979323846;

  krosc::Context context;
  MockArmSystem* hardware = nullptr;
  krosc::ros2_control::HardwareRegistry hardware_registry;
  hardware_registry.add("test/MockArmSystem", [&hardware] {
    auto system = std::make_unique<MockArmSystem>();
    hardware = system.get();
    return system;
  });
  krosc::ros2_control::ControllerManager manager(
      context, "controller_manager",
      {{"update_rate", 50.0},
       {"hardware.plugin", std::string("test/MockArmSystem")},
       {"controller_plugins",
        std::vector<std::string>{"joint_state_broadcaster", "joint_trajectory_controller"}},
       {"joint_state_broadcaster.type",
        std::string("joint_state_broadcaster/JointStateBroadcaster")},
       {"joint_trajectory_controller.type",
        std::string("joint_trajectory_controller/JointTrajectoryController")},
       {"joint_trajectory_controller.joints",
        std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"}}},
      hardware_registry);
  krosc::moveit::MoveGroup move_group(
      context, "move_group",
      {{"base_link", std::string("base_link")},
       {"tip_link", std::string("end_effector_link")},
       {"joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4"}},
       {"lower_limits", std::vector<double>{-3.141592653589793, -2.20, -2.00, -2.00}},
       {"upper_limits", std::vector<double>{3.141592653589793, 2.20, 2.00, 2.20}},
       {"max_joint_speed", 0.35},
       {"max_joint_accel", 1.0},
       {"cartesian_speed", 0.08},
       {"cartesian_accel", 0.25},
       {"sample_time", 0.05},
       {"kinematics.plugin", std::string("krosc_moveit/ArmChainKinematics")},
       {"planner.plugin", std::string("krosc_moveit/CartesianPlanner")},
       {"controller_manager.plugin", std::string("krosc_moveit/SimpleControllerManager")},
       {"controller_manager.action",
        std::string("joint_trajectory_controller/follow_joint_trajectory")}});

  krosc::Node monitor(context, "monitor");
  std::mutex status_mutex;
  std::string status;
  std::atomic<int> joint_states{0};
  monitor.subscribe<krosc::msgs::String>(
      "moveit_status",
      [&](const krosc::msgs::String& message) {
        std::lock_guard<std::mutex> lock(status_mutex);
        status = message.data;
      },
      krosc::QOS_DEFAULT);
  monitor.subscribe<krosc::msgs::JointState>(
      "joint_states",
      [&](const krosc::msgs::JointState& message) {
        joint_states.store(static_cast<int>(message.name.size()));
      },
      krosc::QOS_DEFAULT);
  const auto status_contains = [&](const std::string& text) {
    std::lock_guard<std::mutex> lock(status_mutex);
    return status.find(text) != std::string::npos;
  };

  monitor.publish("robot_description", krosc::msgs::String{urdf}, krosc::QOS_LATCHED);
  CHECK(wait_until([&] { return joint_states.load() >= 4; }, 2.0));

  std::ostringstream target_text;
  target_text << target.x << "," << target.y << "," << target.z << "," << pitch_deg << ",0,1,1";
  monitor.publish("moveit_target", krosc::msgs::String{target_text.str()}, krosc::QOS_DEFAULT);
  CHECK(wait_until([&] { return status_contains("Preview: IK ok"); }, 3.0));

  monitor.publish("moveit_plan", krosc::msgs::Empty{}, krosc::QOS_DEFAULT);
  CHECK(wait_until([&] { return status_contains("Plan OK"); }, 5.0));

  const double before = hardware->command("joint2");
  monitor.publish("moveit_execute", krosc::msgs::Empty{}, krosc::QOS_DEFAULT);
  CHECK(wait_until([&] { return status_contains("Execute 완료"); }, 20.0));
  CHECK(std::abs(hardware->command("joint2") - before) > 1e-3);
  for (const auto& joint : {"joint1", "joint2", "joint3", "joint4"}) {
    CHECK(std::isfinite(hardware->command(joint)));
  }

  // move_action 액션도 같은 스택으로 동작한다 (joint goal, joint-space).
  krosc::ActionClient<krosc::msgs::moveit::MoveGroupGoal, krosc::msgs::moveit::MoveGroupFeedback,
                      krosc::msgs::moveit::MoveGroupResult>
      client(context, krosc::msgs::moveit::MOVE_ACTION);
  krosc::msgs::moveit::MoveGroupGoal action_goal;
  action_goal.joint_goal = {0.0, 0.0, 0.0, 0.0};
  action_goal.cartesian = false;
  const auto handle = client.send_goal(action_goal);
  const auto action_result = handle->result(20.0);
  CHECK(action_result.status == krosc::GoalStatus::Succeeded);
  CHECK(action_result.result.has_value());
  CHECK_EQ(krosc::msgs::moveit::MoveGroupResult::NONE,
           std::any_cast<krosc::msgs::moveit::MoveGroupResult>(action_result.result).error_code);
  CHECK(wait_until([&] { return std::abs(hardware->command("joint1")) < 1e-6; }, 2.0));
}
