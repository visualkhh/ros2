#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "krosc/action.hpp"
#include "krosc/context.hpp"
#include "krosc/nodes/moveit/move_group.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/control/controller_manager.hpp"
#include "krosc/urdf.hpp"

#include "test_framework.hpp"

namespace {

using krosc::ParamValue;

const char* kArmUrdf = R"(
<robot name="test_arm">
  <link name="base_link"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <link name="link1"/>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.1 0 0" rpy="0 1.5707963 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <link name="link2"/>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0.1" rpy="0 -1.5707963 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <link name="link3"/>
  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="end_effector_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <link name="end_effector_link"/>
</robot>
)";

const std::vector<std::string>& arm_joints() {
  static const std::vector<std::string> joints{"joint1", "joint2", "joint3", "joint4",
                                               "gripper_left_joint"};
  return joints;
}

class MockArmSystem final : public krosc::control::SystemInterface {
 public:
  MockArmSystem() {
    for (const auto& joint : arm_joints()) {
      state_[joint] = std::make_shared<krosc::control::StateInterface>(joint + "/position");
      command_[joint] = std::make_shared<krosc::control::CommandInterface>(joint + "/position");
    }
    wheel_left_ = std::make_shared<krosc::control::StateInterface>("wheel_left_joint/position");
    wheel_right_ = std::make_shared<krosc::control::StateInterface>("wheel_right_joint/position");
    wheel_left_velocity_ =
        std::make_shared<krosc::control::StateInterface>("wheel_left_joint/velocity");
    wheel_right_velocity_ =
        std::make_shared<krosc::control::StateInterface>("wheel_right_joint/velocity");
    wheel_left_command_ =
        std::make_shared<krosc::control::CommandInterface>("wheel_left_joint/velocity");
    wheel_right_command_ =
        std::make_shared<krosc::control::CommandInterface>("wheel_right_joint/velocity");
  }

  std::vector<std::shared_ptr<krosc::control::StateInterface>> export_state_interfaces() override {
    std::vector<std::shared_ptr<krosc::control::StateInterface>> result;
    for (const auto& [joint, state] : state_) {
      (void)joint;
      result.push_back(state);
    }
    result.push_back(wheel_left_);
    result.push_back(wheel_right_);
    result.push_back(wheel_left_velocity_);
    result.push_back(wheel_right_velocity_);
    return result;
  }

  std::vector<std::shared_ptr<krosc::control::CommandInterface>> export_command_interfaces() override {
    std::vector<std::shared_ptr<krosc::control::CommandInterface>> result;
    for (const auto& [joint, command] : command_) {
      (void)joint;
      result.push_back(command);
    }
    result.push_back(wheel_left_command_);
    result.push_back(wheel_right_command_);
    return result;
  }

  bool read(double time, double period) override {
    (void)time;
    left_angle_ += wheel_left_command_->get() * period;
    right_angle_ += wheel_right_command_->get() * period;
    wheel_left_->set(left_angle_);
    wheel_right_->set(right_angle_);
    wheel_left_velocity_->set(wheel_left_command_->get());
    wheel_right_velocity_->set(wheel_right_command_->get());
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
  [[nodiscard]] double wheel_command(const std::string& joint) const {
    if (joint == "left") return wheel_left_command_->get();
    if (joint == "right") return wheel_right_command_->get();
    return 0.0;
  }

 private:
  std::map<std::string, std::shared_ptr<krosc::control::StateInterface>> state_;
  std::map<std::string, std::shared_ptr<krosc::control::CommandInterface>> command_;
  std::shared_ptr<krosc::control::StateInterface> wheel_left_;
  std::shared_ptr<krosc::control::StateInterface> wheel_right_;
  std::shared_ptr<krosc::control::StateInterface> wheel_left_velocity_;
  std::shared_ptr<krosc::control::StateInterface> wheel_right_velocity_;
  std::shared_ptr<krosc::control::CommandInterface> wheel_left_command_;
  std::shared_ptr<krosc::control::CommandInterface> wheel_right_command_;
  double left_angle_ = 0.0;
  double right_angle_ = 0.0;
};

class FakeKinematics final : public krosc::moveit::MoveItKinematicsPlugin {
 public:
  krosc::moveit::KinematicsResult search_position_ik(
      const krosc::moveit::KinematicsQuery& query) const override {
    (void)query;
    krosc::moveit::KinematicsResult result;
    result.success = true;
    result.solution = {0.1, 0.2, 0.3, 0.4};
    return result;
  }
  krosc::msgs::Vector3 get_position_fk(const std::array<double, 4>& joints) const override {
    (void)joints;
    return {};
  }
  [[nodiscard]] std::string type() const override { return "test/FakeKinematics"; }
};

class FakePlanner final : public krosc::moveit::MoveItPlannerPlugin {
 public:
  krosc::moveit::PlanningResult plan(const krosc::moveit::PlanningRequest& request) override {
    krosc::moveit::PlanningResult result;
    result.success = true;
    constexpr int kPoints = 5;
    for (int index = 0; index < kPoints; ++index) {
      const double t = static_cast<double>(index) / (kPoints - 1);
      krosc::manipulation::TrajectoryPoint point;
      for (std::size_t joint = 0; joint < 4; ++joint) {
        point.position[joint] = request.start[joint] + (request.goal[joint] - request.start[joint]) * t;
      }
      point.time = 0.2 * t;
      result.trajectory.points.push_back(point);
    }
    result.trajectory.duration = 0.2;
    return result;
  }
  [[nodiscard]] std::string type() const override { return "test/FakePlanner"; }
};

template <class Predicate>
bool wait_until(Predicate predicate, double seconds = 3.0) {
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

KROSC_TEST(control_manager_jsb_jtc_and_switch) {
  krosc::Context context;
  MockArmSystem* hardware = nullptr;
  krosc::control::HardwareRegistry hardware_registry;
  hardware_registry.add("test/MockArmSystem", [&hardware] {
    auto system = std::make_unique<MockArmSystem>();
    hardware = system.get();
    return system;
  });

  krosc::control::ControllerManager manager(
      context, "controller_manager",
      {{"update_rate", 50.0},
       {"hardware.plugin", std::string("test/MockArmSystem")},
       {"controller_plugins",
        std::vector<std::string>{"joint_state_broadcaster", "joint_trajectory_controller",
                                 "diff_drive_controller", "arm_command_controller"}},
       {"joint_state_broadcaster.type",
        std::string("joint_state_broadcaster/JointStateBroadcaster")},
       {"joint_trajectory_controller.type",
        std::string("joint_trajectory_controller/JointTrajectoryController")},
       {"joint_trajectory_controller.joints",
        std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"}},
       {"diff_drive_controller.type", std::string("diff_drive_controller/DiffDriveController")},
       {"diff_drive_controller.wheel_separation", 0.2},
       {"diff_drive_controller.wheel_radius", 0.05},
       {"diff_drive_controller.cmd_vel_topic", std::string("cmd_vel")},
       {"diff_drive_controller.odom_topic", std::string("odom")},
       {"diff_drive_controller.publish_tf", false},
       {"arm_command_controller.type",
        std::string("forward_command_controller/ForwardCommandController")},
       {"arm_command_controller.joints",
        std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"}},
       {"arm_command_controller.command_topic", std::string("arm_joint_command")},
       {"arm_command_controller.auto_activate", false}},
      hardware_registry);
  CHECK(hardware != nullptr);
  CHECK(manager.state_of("joint_state_broadcaster") ==
        krosc::control::ControllerManager::State::Active);
  CHECK(manager.state_of("joint_trajectory_controller") ==
        krosc::control::ControllerManager::State::Active);
  CHECK(manager.state_of("diff_drive_controller") ==
        krosc::control::ControllerManager::State::Active);
  CHECK(manager.state_of("arm_command_controller") ==
        krosc::control::ControllerManager::State::Inactive);

  krosc::Node monitor(context, "monitor");
  std::atomic<int> joint_states{0};
  monitor.subscribe<krosc::msgs::JointState>(
      "joint_states",
      [&](const krosc::msgs::JointState& message) { joint_states.store(static_cast<int>(message.name.size())); });
  CHECK(wait_until([&] { return joint_states.load() >= 6; }, 2.0));
  krosc::msgs::Odometry odom;
  monitor.subscribe<krosc::msgs::Odometry>(
      "odom", [&](const krosc::msgs::Odometry& message) { odom = message; });
  krosc::msgs::Twist drive;
  drive.linear.x = 0.2;
  monitor.publish("cmd_vel", drive, krosc::QOS_COMMAND);
  CHECK(wait_until(
      [&] {
        return std::abs(hardware->wheel_command("left") - 4.0) < 1e-6 &&
               std::abs(hardware->wheel_command("right") - 4.0) < 1e-6;
      },
      2.0));
  CHECK(wait_until([&] { return odom.pose.pose.position.x > 0.01; }, 2.0));

  const auto list = context.services().call<krosc::control::ListControllersRequest,
                                             krosc::control::ListControllersResponse>(
      "controller_manager/list_controllers", {});
  CHECK_EQ(4u, list.name.size());
  CHECK_EQ(4u, list.type.size());
  CHECK_EQ(4u, list.state.size());

  krosc::ActionClient<krosc::msgs::control::FollowJointTrajectoryGoal,
                              krosc::msgs::control::FollowJointTrajectoryFeedback,
                              krosc::msgs::control::FollowJointTrajectoryResult>
      client(context, "joint_trajectory_controller/follow_joint_trajectory");
  krosc::msgs::control::FollowJointTrajectoryGoal goal;
  goal.trajectory.header = krosc::msgs::Header{context.now(), "base_link"};
  goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"};
  for (int index = 0; index <= 4; ++index) {
    krosc::msgs::control::JointTrajectoryPoint point;
    const double t = static_cast<double>(index) / 4.0;
    point.positions = {0.4 * t, 0.2 * t, -0.2 * t, 0.1 * t, 0.0};
    point.time_from_start = 0.2 * t;
    goal.trajectory.points.push_back(point);
  }
  goal.goal_time_tolerance = 0.2;
  const auto handle = client.send_goal(goal);
  const auto result = handle->result(5.0);
  CHECK(result.status == krosc::GoalStatus::Succeeded);
  CHECK(result.result.has_value());
  CHECK_EQ(krosc::msgs::control::FollowJointTrajectoryResult::SUCCESSFUL,
           std::any_cast<krosc::msgs::control::FollowJointTrajectoryResult>(result.result).error_code);
  CHECK(wait_until([&] { return std::abs(hardware->command("joint1") - 0.4) < 1e-6; }, 2.0));

  // JTC가 command interface를 잡고 있는 동안 forward controller를 켜면 충돌한다.
  CHECK(!manager.switch_controller({"arm_command_controller"}, {}, 1));
  CHECK(manager.state_of("arm_command_controller") ==
        krosc::control::ControllerManager::State::Inactive);

  // JTC를 내리고 forward controller를 켜면 수동 토픽이 하드웨어로 간다.
  CHECK(manager.switch_controller({"arm_command_controller"}, {"joint_trajectory_controller"}, 1));
  krosc::msgs::JointState command;
  command.name = {"joint1", "joint2", "joint3", "joint4"};
  command.position = {0.11, 0.22, 0.33, 0.44};
  monitor.publish("arm_joint_command", command);
  CHECK(wait_until([&] { return std::abs(hardware->command("joint1") - 0.11) < 1e-6; }, 2.0));
  CHECK(wait_until([&] { return std::abs(hardware->command("joint4") - 0.44) < 1e-6; }, 2.0));
}

KROSC_TEST(moveit_move_group_action_with_plugins) {
  krosc::Context context;
  MockArmSystem* hardware = nullptr;
  krosc::control::HardwareRegistry hardware_registry;
  hardware_registry.add("test/MockArmSystem", [&hardware] {
    auto system = std::make_unique<MockArmSystem>();
    hardware = system.get();
    return system;
  });
  krosc::control::ControllerManager manager(
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

  krosc::moveit::KinematicsRegistry kinematics;
  kinematics.add("test/FakeKinematics", [] { return std::make_unique<FakeKinematics>(); });
  krosc::moveit::PlannerRegistry planners;
  planners.add("test/FakePlanner", [] { return std::make_unique<FakePlanner>(); });
  krosc::moveit::MoveGroup move(
      context, "move_group",
      {{"base_link", std::string("base_link")},
       {"tip_link", std::string("end_effector_link")},
       {"joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4"}},
       {"lower_limits", std::vector<double>{-3.14, -3.14, -3.14, -3.14}},
       {"upper_limits", std::vector<double>{3.14, 3.14, 3.14, 3.14}},
       {"kinematics.plugin", std::string("test/FakeKinematics")},
       {"planner.plugin", std::string("test/FakePlanner")},
       {"controller_manager.plugin", std::string("krosc_moveit/SimpleControllerManager")},
       {"controller_manager.action",
        std::string("joint_trajectory_controller/follow_joint_trajectory")}},
      kinematics, planners, krosc::moveit::builtin_controller_managers());

  krosc::Node sim(context, "sim");
  std::atomic<int> state_messages{0};
  std::atomic<int> joint_states{0};
  sim.subscribe<krosc::msgs::String>(
      "moveit_state", [&](const krosc::msgs::String&) { state_messages.fetch_add(1); });
  sim.subscribe<krosc::msgs::JointState>(
      "joint_states", [&](const krosc::msgs::JointState& message) {
        joint_states.store(static_cast<int>(message.name.size()));
      });
  sim.publish("robot_description", krosc::msgs::String{kArmUrdf}, krosc::QOS_LATCHED);
  CHECK(wait_until([&] { return state_messages.load() > 0 && joint_states.load() >= 4; }, 2.0));

  krosc::ActionClient<krosc::msgs::moveit::MoveGroupGoal, krosc::msgs::moveit::MoveGroupFeedback,
                      krosc::msgs::moveit::MoveGroupResult>
      client(context, krosc::msgs::moveit::MOVE_ACTION);
  krosc::msgs::moveit::MoveGroupGoal goal;
  goal.joint_goal = {0.1, 0.2, 0.3, 0.4};
  goal.cartesian = false;
  const auto handle = client.send_goal(goal);
  const auto action_result = handle->result(5.0);
  CHECK(action_result.status == krosc::GoalStatus::Succeeded);
  CHECK(action_result.result.has_value());
  const auto move_result = std::any_cast<krosc::msgs::moveit::MoveGroupResult>(action_result.result);
  CHECK_EQ(krosc::msgs::moveit::MoveGroupResult::NONE, move_result.error_code);
  CHECK_EQ(5u, move_result.trajectory.points.size());
  CHECK(wait_until([&] { return std::abs(hardware->command("joint4") - 0.4) < 1e-6; }, 2.0));

  // plan_only는 컨트롤러를 움직이지 않고 궤적만 돌려준다.
  krosc::msgs::moveit::MoveGroupGoal plan_only;
  plan_only.joint_goal = {0.0, 0.0, 0.0, 0.0};
  plan_only.plan_only = true;
  const auto plan_handle = client.send_goal(plan_only);
  const auto plan_result = plan_handle->result(5.0);
  CHECK(plan_result.status == krosc::GoalStatus::Succeeded);
  CHECK_EQ(krosc::msgs::moveit::MoveGroupResult::NONE,
           std::any_cast<krosc::msgs::moveit::MoveGroupResult>(plan_result.result).error_code);
}
