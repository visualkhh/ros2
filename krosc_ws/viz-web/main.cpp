#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "krosc/action.hpp"
#include "krosc/krosc.hpp"
#include "krosc/tf/math.hpp"
#include "krosc/msgs.hpp"
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
  bool active = false;
  std::chrono::steady_clock::time_point active_since;
  std::mutex mutex;
};

void register_move_rpc(krosc::viz_web::WebRpc& rpc, krosc::Context& context) {
  auto state = std::make_shared<MoveState>(context);
  rpc.add("moveit/send_goal",
          [state, &context](const krosc::viz_web::Json& params,
                            const krosc::viz_web::WebRpc::Notify& notify) {
            std::lock_guard<std::mutex> lock(state->mutex);
            // result 이벤트가 늦게 와도 다음 요청에서 상태를 정리한다.
            if (state->active &&
                (!state->handle || state->handle->status() != krosc::GoalStatus::Accepted ||
                 std::chrono::steady_clock::now() - state->active_since >
                     std::chrono::seconds(60))) {
              state->active = false;
            }
            if (state->active) {
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
            state->active = state->handle->status() == krosc::GoalStatus::Accepted;
            state->active_since = std::chrono::steady_clock::now();
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
            if (result["ok"].as_bool()) {
              state->active = false;
            }
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

void register_arm_rpc(krosc::viz_web::WebRpc& rpc, krosc::Context& context) {
  rpc.add("arm/set_torque",
          [&context](const krosc::viz_web::Json& params, const krosc::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<krosc::msgs::SetBoolRequest,
                                                          krosc::msgs::SetBoolResponse>(
                "arm/set_torque", krosc::msgs::SetBoolRequest{params["on"].as_bool()});
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });

  rpc.add("arm/stop",
          [&context](const krosc::viz_web::Json&, const krosc::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<krosc::msgs::TriggerRequest,
                                                          krosc::msgs::TriggerResponse>(
                "arm/stop", krosc::msgs::TriggerRequest{});
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });

  rpc.add("arm/reset",
          [&context](const krosc::viz_web::Json& params, const krosc::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<krosc::msgs::ArmResetRequest,
                                                          krosc::msgs::ArmResetResponse>(
                "arm/reset",
                krosc::msgs::ArmResetRequest{params["target"].as_string("all")});
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });

  rpc.add("buzzer",
          [&context](const krosc::viz_web::Json& params, const krosc::viz_web::WebRpc::Notify&) {
            krosc::msgs::BuzzerRequest request;
            request.frequency = static_cast<std::int32_t>(params["frequency"].as_int(2000));
            request.duration = static_cast<std::int32_t>(params["duration"].as_int(150));
            const auto response = context.services().call<krosc::msgs::BuzzerRequest,
                                                          krosc::msgs::BuzzerResponse>(
                "buzzer", request);
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });
}

void register_llm_rpc(krosc::viz_web::WebRpc& rpc, krosc::Context& context) {
  rpc.add("llm/command",
          [&context](const krosc::viz_web::Json& params, const krosc::viz_web::WebRpc::Notify&) {
            krosc::viz_web::Json result = krosc::viz_web::Json::object();
            try {
              const auto response = context.services().call<krosc::msgs::String,
                                                            krosc::msgs::String>(
                  "llm/command", krosc::msgs::String{params["text"].as_string()});
              result["raw"] = response.data;
            } catch (const std::exception& error) {
              result["accepted"] = false;
              result["error"] = error.what();
            }
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
  registry.add("camera", "Camera", "/plugins/camera.js");
  registry.add("ai", "AI", "/plugins/ai.js");

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
      {"llm/status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"vla/status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"pick/status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"nav/status", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"sim_graspable", krosc::net::Kind::Path, krosc::QOS_DEFAULT},
      {"grasp_info", krosc::net::Kind::String, krosc::QOS_DEFAULT},
      {"camera/color/image_raw", krosc::net::Kind::Image, krosc::QOS_SENSOR_DATA},
      {"camera/depth/image_rect_raw", krosc::net::Kind::Image, krosc::QOS_SENSOR_DATA},
  };

  krosc::net::PeerNetworkOptions network;
  network.name = "krosc_ws_viz_web";
  if (!options.robot_host.empty()) {
    network.add_server(options.robot_host, options.robot_port);
  }
  krosc::Krosc app(nullptr, network);
  krosc::viz_web::VizWeb web(std::move(options), std::move(registry), app.context());
  register_move_rpc(web.rpc(), app.context());
  register_arm_rpc(web.rpc(), app.context());
  register_llm_rpc(web.rpc(), app.context());
  if (!web.initialize(error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  return web.run();
}
