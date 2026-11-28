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

#include "srf/action.hpp"
#include "srf/manipulation/arm_chain.hpp"
#include "srf/tf/math.hpp"
#include "srf/context.hpp"
#include "srf/nodes/moveit/move_group.hpp"
#include "srf/msgs.hpp"
#include "srf/node.hpp"
#include "srf/control/controller_manager.hpp"
#include "srf/urdf.hpp"

#include "test_framework.hpp"

#ifndef SRF_DEFAULT_WS_URDF_PATH
#define SRF_DEFAULT_WS_URDF_PATH "engine/description/urdf/turtlebot3_real.urdf"
#endif

namespace {

using srf::ParamValue;

const std::vector<std::string>& arm_joints() {
  static const std::vector<std::string> joints{"joint1", "joint2", "joint3", "joint4",
                                               "gripper_left_joint"};
  return joints;
}

class MockArmSystem final : public srf::control::Control {
 public:
  MockArmSystem() {
    for (const auto& joint : arm_joints()) {
      state_[joint] = std::make_shared<srf::control::StateInterface>(joint + "/position");
      command_[joint] = std::make_shared<srf::control::CommandInterface>(joint + "/position");
    }
    wheel_left_ = std::make_shared<srf::control::StateInterface>("wheel_left_joint/position");
    wheel_right_ = std::make_shared<srf::control::StateInterface>("wheel_right_joint/position");
  }

  std::vector<std::shared_ptr<srf::control::StateInterface>> export_state_interfaces() override {
    std::vector<std::shared_ptr<srf::control::StateInterface>> result;
    for (const auto& [joint, state] : state_) {
      (void)joint;
      result.push_back(state);
    }
    result.push_back(wheel_left_);
    result.push_back(wheel_right_);
    return result;
  }

  std::vector<std::shared_ptr<srf::control::CommandInterface>> export_command_interfaces() override {
    std::vector<std::shared_ptr<srf::control::CommandInterface>> result;
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
  std::map<std::string, std::shared_ptr<srf::control::StateInterface>> state_;
  std::map<std::string, std::shared_ptr<srf::control::CommandInterface>> command_;
  std::shared_ptr<srf::control::StateInterface> wheel_left_;
  std::shared_ptr<srf::control::StateInterface> wheel_right_;
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

SRF_TEST(moveit_control_real_arm_pipeline) {
  const std::string urdf = read_text(SRF_DEFAULT_WS_URDF_PATH);
  CHECK(!urdf.empty());

  std::string chain_error;
  const auto robot = srf::parse_urdf(urdf);
  auto chain = srf::manipulation::ArmChain::from_urdf(
      robot, "base_link", "end_effector_link",
      {"joint1", "joint2", "joint3", "joint4"}, chain_error);
  CHECK(chain.has_value());
  if (!chain) {
    return;
  }
  const std::array<double, 4> target_joints{0.3, -0.5, 0.4, 0.2};
  const srf::msgs::Vector3 target = chain->tcp(target_joints);
  const double pitch_deg = chain->pitch_down(target_joints) * 180.0 / 3.14159265358979323846;

  srf::Context context;
  MockArmSystem* hardware = nullptr;
  srf::control::ControlRegistry hardware_registry;
  hardware_registry.add("test/MockArmSystem", [&hardware] {
    auto system = std::make_unique<MockArmSystem>();
    hardware = system.get();
    return system;
  });
  srf::control::ControllerManager manager(
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
  srf::moveit::MoveGroup move_group(
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
       {"kinematics.plugin", std::string("srf_moveit/ArmChainKinematics")},
       {"planner.plugin", std::string("srf_moveit/CartesianPlanner")},
       {"controller_manager.plugin", std::string("srf_moveit/SimpleControllerManager")},
       {"controller_manager.action",
        std::string("joint_trajectory_controller/follow_joint_trajectory")}});

  srf::Node monitor(context, "monitor");
  std::mutex status_mutex;
  std::string status;
  std::atomic<int> joint_states{0};
  monitor.subscribe<srf::msgs::String>(
      "moveit_status",
      [&](const srf::msgs::String& message) {
        std::lock_guard<std::mutex> lock(status_mutex);
        status = message.data;
      },
      srf::QOS_DEFAULT);
  monitor.subscribe<srf::msgs::JointState>(
      "joint_states",
      [&](const srf::msgs::JointState& message) {
        joint_states.store(static_cast<int>(message.name.size()));
      },
      srf::QOS_DEFAULT);
  const auto status_contains = [&](const std::string& text) {
    std::lock_guard<std::mutex> lock(status_mutex);
    return status.find(text) != std::string::npos;
  };

  monitor.publish("robot_description", srf::msgs::String{urdf}, srf::QOS_LATCHED);
  CHECK(wait_until([&] { return joint_states.load() >= 4; }, 2.0));

  const double yaw = std::atan2(target.y + 0.000230, target.x - 0.059985);
  const double pitch = pitch_deg * 3.14159265358979323846 / 180.0;
  srf::msgs::moveit::MoveGroupGoal target_goal;
  target_goal.target_pose.header = srf::msgs::Header{0.0, "base_link"};
  target_goal.target_pose.pose.position = {target.x, target.y, target.z};
  target_goal.target_pose.pose.orientation = srf::tfmath::from_rpy(0.0, -pitch, yaw);
  target_goal.cartesian = true;
  target_goal.grip = 0.0;
  target_goal.plan_only = true;
  {
    srf::ActionClient<srf::msgs::moveit::MoveGroupGoal, srf::msgs::moveit::MoveGroupFeedback,
                        srf::msgs::moveit::MoveGroupResult>
        planner(context, srf::msgs::moveit::MOVE_ACTION);
    const auto handle = planner.send_goal(target_goal);
    const auto result = handle->result(5.0);
    CHECK(result.status == srf::GoalStatus::Succeeded);
    CHECK(result.result.has_value());
    CHECK_EQ(srf::msgs::moveit::MoveGroupResult::NONE,
             std::any_cast<srf::msgs::moveit::MoveGroupResult>(result.result).error_code);
  }
  CHECK(wait_until([&] { return status_contains("Move: plan_only"); }, 5.0));

  const double before = hardware->command("joint2");
  target_goal.plan_only = false;
  {
    srf::ActionClient<srf::msgs::moveit::MoveGroupGoal, srf::msgs::moveit::MoveGroupFeedback,
                        srf::msgs::moveit::MoveGroupResult>
        executor(context, srf::msgs::moveit::MOVE_ACTION);
    const auto handle = executor.send_goal(target_goal);
    const auto result = handle->result(20.0);
    CHECK(result.status == srf::GoalStatus::Succeeded);
  }
  CHECK(wait_until([&] { return status_contains("Execute 완료"); }, 20.0));
  CHECK(std::abs(hardware->command("joint2") - before) > 1e-3);
  for (const auto& joint : {"joint1", "joint2", "joint3", "joint4"}) {
    CHECK(std::isfinite(hardware->command(joint)));
  }

  // move_action 액션도 같은 스택으로 동작한다 (joint goal, joint-space).
  srf::ActionClient<srf::msgs::moveit::MoveGroupGoal, srf::msgs::moveit::MoveGroupFeedback,
                      srf::msgs::moveit::MoveGroupResult>
      client(context, srf::msgs::moveit::MOVE_ACTION);
  srf::msgs::moveit::MoveGroupGoal action_goal;
  action_goal.joint_goal = {0.0, 0.0, 0.0, 0.0};
  action_goal.cartesian = false;
  const auto handle = client.send_goal(action_goal);
  const auto action_result = handle->result(20.0);
  CHECK(action_result.status == srf::GoalStatus::Succeeded);
  CHECK(action_result.result.has_value());
  CHECK_EQ(srf::msgs::moveit::MoveGroupResult::NONE,
           std::any_cast<srf::msgs::moveit::MoveGroupResult>(action_result.result).error_code);
  CHECK(wait_until([&] { return std::abs(hardware->command("joint1")) < 1e-6; }, 2.0));
}

SRF_TEST(moveit_cartesian_failure_falls_back_quickly) {
  // 어제 Cartesian planner가 도달 불가능한 경로에서 모든 IK seed를 매 스텝
  // 전체 탐색해 수 분간 블록되던 문제의 회귀 테스트.
  const std::string urdf = read_text(SRF_DEFAULT_WS_URDF_PATH);
  CHECK(!urdf.empty());
  std::string chain_error;
  const auto robot = srf::parse_urdf(urdf);
  auto chain = srf::manipulation::ArmChain::from_urdf(
      robot, "base_link", "end_effector_link",
      {"joint1", "joint2", "joint3", "joint4"}, chain_error);
  CHECK(chain.has_value());
  if (!chain) {
    return;
  }
  const std::array<double, 4> start{0.0, 0.0, 0.0, 0.0};
  const std::array<double, 4> stow{0.0, -1.9024, 1.3440, 0.9023};
  srf::manipulation::PlanOptions options;
  std::string error;
  const auto began = std::chrono::steady_clock::now();
  const auto trajectory = srf::manipulation::plan_trajectory(*chain, start, stow, options, error);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - began)
                                .count();
  CHECK(trajectory.has_value());
  CHECK(!trajectory->points.empty());
  CHECK(trajectory->fallback);
  CHECK(elapsed_ms < 2000.0);
}
