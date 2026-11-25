#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "krosc/action.hpp"
#include "krosc/krosc.hpp"
#include "krosc/tf/math.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/viz_web/viz_web.hpp"
#include "krosc/viz_web/viz_web_cli.hpp"
#include "krosc/viz_web/viz_web_plugins.hpp"

#ifndef KROSC_WS_VIZ_WEB_ROOT
#define KROSC_WS_VIZ_WEB_ROOT "viz-web/www"
#endif

namespace {

using MoveClient = krosc::ActionClient<krosc::msgs::moveit::MoveGroupGoal,
                                       krosc::msgs::moveit::MoveGroupFeedback,
                                       krosc::msgs::moveit::MoveGroupResult>;
using MoveHandle = krosc::ClientGoalHandle<krosc::msgs::moveit::MoveGroupGoal,
                                           krosc::msgs::moveit::MoveGroupFeedback,
                                           krosc::msgs::moveit::MoveGroupResult>;

struct MoveState {
  explicit MoveState(krosc::Context& context)
      : client(std::make_shared<MoveClient>(context, krosc::msgs::moveit::MOVE_ACTION)) {}
  std::shared_ptr<MoveClient> client;
  std::shared_ptr<MoveHandle> handle;
  std::mutex mutex;
};

void register_move_rpc(krosc::viz_web::WebRpc& rpc, krosc::Context& context) {
  auto state = std::make_shared<MoveState>(context);
  rpc.add("moveit/send_goal",
          [state, &context](const krosc::viz_web::Json& params,
                            const krosc::viz_web::WebRpc::Notify& notify) {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (state->handle && state->handle->status() == krosc::GoalStatus::Accepted) {
              krosc::viz_web::Json result = krosc::viz_web::Json::object();
              result["accepted"] = false;
              result["error"] = "busy";
              return result;
            }
            krosc::msgs::moveit::MoveGroupGoal goal;
            goal.preset = params["preset"].as_string();
            goal.plan_only = params["plan_only"].as_bool(false);
            goal.cartesian = params["cartesian"].as_bool(true);
            const double grip_mm = params["grip"].as_double(-1.0);
            goal.grip = grip_mm < 0.0 ? -1.0 : grip_mm / 1000.0;
            if (goal.preset.empty()) {
              const double x = params["x"].as_double();
              const double y = params["y"].as_double();
              const double z = params["z"].as_double(0.32);
              const double pitch = params["pitch"].as_double() * 3.14159265358979323846 / 180.0;
              const double yaw = std::atan2(y + 0.000230, x - 0.059985);
              goal.target_pose.header = krosc::msgs::Header{context.now(), "base_link"};
              goal.target_pose.pose.position = {x, y, z};
              goal.target_pose.pose.orientation = krosc::tfmath::from_rpy(0.0, -pitch, yaw);
            }
            state->handle = state->client->send_goal(
                goal, [notify](const krosc::msgs::moveit::MoveGroupFeedback& feedback) {
                  krosc::viz_web::Json event = krosc::viz_web::Json::object();
                  event["progress"] = feedback.progress;
                  event["state"] = feedback.state;
                  notify("moveit/feedback", event);
                });
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            result["goal_id"] = state->handle->goal_id();
            result["accepted"] = state->handle->status() != krosc::GoalStatus::Rejected;
            return result;
          });

  rpc.add("moveit/cancel",
          [state](const krosc::viz_web::Json& params, const krosc::viz_web::WebRpc::Notify&) {
            std::lock_guard<std::mutex> lock(state->mutex);
            std::string goal_id = params["goal_id"].as_string();
            if (goal_id.empty() && state->handle) {
              goal_id = state->handle->goal_id();
            }
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            result["ok"] = !goal_id.empty() && state->client->cancel(goal_id);
            return result;
          });

  rpc.add("moveit/scrub",
          [&context](const krosc::viz_web::Json& params, const krosc::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<krosc::msgs::String, krosc::msgs::Bool>(
                "move_group/scrub",
                krosc::msgs::String{std::to_string(params["index"].as_int())});
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            result["ok"] = response.data;
            return result;
          });
}

}  // namespace

int main(int argc, char** argv) {
  krosc::viz_web::VizWebOptions options;
  options.overlay_root = KROSC_WS_VIZ_WEB_ROOT;

  bool help = false;
  std::string error;
  if (!krosc::viz_web::parse_viz_web_options(argc, argv, options, help, error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  if (help) {
    return 0;
  }

  // 코어 플러그인 + krosc_ws 프로젝트 플러그인.
  krosc::viz_web::VizWebPluginRegistry registry;
  krosc::viz_web::register_core_web_plugins(registry,
                                            {"slam", "navigation", "teleop", "diagnostics"});
  registry.add("opencr", "OpenCR", "/plugins/opencr.js");
  registry.add("moveit", "MoveIt", "/plugins/moveit.js");
  registry.add("rl_nav", "RL Nav", "/plugins/rl_nav.js");
  registry.add("manipulator", "Manipulator", "/plugins/manipulator.js");

  // 프로젝트 플러그인이 쓰는 토픽. 코어 웹 게이트웨이는 기본 토픽만 안다.
  options.forward_topics = {
      {"arm_status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"map_name", krosc::net::Kind::String, krosc::QOS_LATCHED},
      {"moveit_status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"moveit_state", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"moveit_target_pose", krosc::net::Kind::Pose, krosc::QOS_DEFAULT},
      {"moveit_arm", krosc::net::Kind::Path, krosc::QOS_DEFAULT},
      {"moveit_path", krosc::net::Kind::Path, krosc::QOS_DEFAULT},
      {"rl_nav_status", krosc::net::Kind::String, krosc::QOS_LATCHED},
      {"rl_nav_model_list", krosc::net::Kind::String, krosc::QOS_LATCHED},
  };

  krosc::net::PeerNetworkOptions network;
  network.name = "krosc_ws_viz_web";
  if (!options.robot_host.empty()) {
    network.add_server(options.robot_host, options.robot_port);
  }
  krosc::Krosc app(nullptr, network);
  krosc::viz_web::VizWeb web(std::move(options), std::move(registry), app.context());
  register_move_rpc(web.rpc(), app.context());
  if (!web.initialize(error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  return web.run();
}
