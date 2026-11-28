#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "srf/action.hpp"
#include "srf/app.hpp"
#include "srf/tf/math.hpp"
#include "srf/msgs.hpp"
#include "srf/msgs/moveit_msgs.hpp"
#include "srf/viz_web/viz_web.hpp"
#include "srf/viz_web/viz_web_cli.hpp"
#include "srf/viz_web/viz_web_plugins.hpp"

#ifndef SRF_RECYCLING_WS_VIZ_WEB_ROOT
#define SRF_RECYCLING_WS_VIZ_WEB_ROOT "viz/web/www"
#endif

namespace {

using MoveClient = srf::ActionClient<srf::msgs::moveit::MoveGroupGoal,
                                       srf::msgs::moveit::MoveGroupFeedback,
                                       srf::msgs::moveit::MoveGroupResult>;
using MoveHandle = srf::ClientGoalHandle<srf::msgs::moveit::MoveGroupGoal,
                                           srf::msgs::moveit::MoveGroupFeedback,
                                           srf::msgs::moveit::MoveGroupResult>;

struct MoveState {
  explicit MoveState(srf::Context& context)
      : client(std::make_shared<MoveClient>(context, srf::msgs::moveit::MOVE_ACTION)) {}
  std::shared_ptr<MoveClient> client;
  std::shared_ptr<MoveHandle> handle;
  bool active = false;
  std::chrono::steady_clock::time_point active_since;
  std::mutex mutex;
};

void register_move_rpc(srf::viz_web::WebRpc& rpc, srf::Context& context) {
  auto state = std::make_shared<MoveState>(context);
  rpc.add("moveit/send_goal",
          [state, &context](const srf::viz_web::Json& params,
                            const srf::viz_web::WebRpc::Notify& notify) {
            std::lock_guard<std::mutex> lock(state->mutex);
            // result 이벤트가 늦게 와도 다음 요청에서 상태를 정리한다.
            if (state->active &&
                (!state->handle || state->handle->status() != srf::GoalStatus::Accepted ||
                 std::chrono::steady_clock::now() - state->active_since >
                     std::chrono::seconds(60))) {
              state->active = false;
            }
            if (state->active) {
              srf::viz_web::Json result = srf::viz_web::Json::object();
              result["accepted"] = false;
              result["error"] = "busy";
              return result;
            }
            srf::msgs::moveit::MoveGroupGoal goal;
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
              goal.target_pose.header = srf::msgs::Header{context.now(), "base_link"};
              goal.target_pose.pose.position = {x, y, z};
              goal.target_pose.pose.orientation = srf::tfmath::from_rpy(0.0, -pitch, yaw);
            }
            state->handle = state->client->send_goal(
                goal, [notify](const srf::msgs::moveit::MoveGroupFeedback& feedback) {
                  srf::viz_web::Json event = srf::viz_web::Json::object();
                  event["progress"] = feedback.progress;
                  event["state"] = feedback.state;
                  notify("moveit/feedback", event);
                });
            srf::viz_web::Json result = srf::viz_web::Json::object();
            result["goal_id"] = state->handle->goal_id();
            result["accepted"] = state->handle->status() != srf::GoalStatus::Rejected;
            state->active = state->handle->status() == srf::GoalStatus::Accepted;
            state->active_since = std::chrono::steady_clock::now();
            return result;
          });

  rpc.add("moveit/cancel",
          [state](const srf::viz_web::Json& params, const srf::viz_web::WebRpc::Notify&) {
            std::lock_guard<std::mutex> lock(state->mutex);
            std::string goal_id = params["goal_id"].as_string();
            if (goal_id.empty() && state->handle) {
              goal_id = state->handle->goal_id();
            }
            srf::viz_web::Json result = srf::viz_web::Json::object();
            result["ok"] = !goal_id.empty() && state->client->cancel(goal_id);
            if (result["ok"].as_bool()) {
              state->active = false;
            }
            return result;
          });

  rpc.add("moveit/scrub",
          [&context](const srf::viz_web::Json& params, const srf::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<srf::msgs::String, srf::msgs::Bool>(
                "move_group/scrub",
                srf::msgs::String{std::to_string(params["index"].as_int())});
            srf::viz_web::Json result = srf::viz_web::Json::object();
            result["ok"] = response.data;
            return result;
          });
}

void register_arm_rpc(srf::viz_web::WebRpc& rpc, srf::Context& context) {
  rpc.add("arm/set_torque",
          [&context](const srf::viz_web::Json& params, const srf::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<srf::msgs::SetBoolRequest,
                                                          srf::msgs::SetBoolResponse>(
                "arm/set_torque", srf::msgs::SetBoolRequest{params["on"].as_bool()});
            srf::viz_web::Json result = srf::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });

  rpc.add("arm/stop",
          [&context](const srf::viz_web::Json&, const srf::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<srf::msgs::TriggerRequest,
                                                          srf::msgs::TriggerResponse>(
                "arm/stop", srf::msgs::TriggerRequest{});
            srf::viz_web::Json result = srf::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });

  rpc.add("arm/reset",
          [&context](const srf::viz_web::Json& params, const srf::viz_web::WebRpc::Notify&) {
            const auto response = context.services().call<srf::msgs::ArmResetRequest,
                                                          srf::msgs::ArmResetResponse>(
                "arm/reset",
                srf::msgs::ArmResetRequest{params["target"].as_string("all")});
            srf::viz_web::Json result = srf::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });

  rpc.add("buzzer",
          [&context](const srf::viz_web::Json& params, const srf::viz_web::WebRpc::Notify&) {
            srf::msgs::BuzzerRequest request;
            request.frequency = static_cast<std::int32_t>(params["frequency"].as_int(2000));
            request.duration = static_cast<std::int32_t>(params["duration"].as_int(150));
            const auto response = context.services().call<srf::msgs::BuzzerRequest,
                                                          srf::msgs::BuzzerResponse>(
                "buzzer", request);
            srf::viz_web::Json result = srf::viz_web::Json::object();
            result["success"] = response.success;
            result["message"] = response.message;
            return result;
          });
}

void register_llm_rpc(srf::viz_web::WebRpc& rpc, srf::Context& context) {
  rpc.add("llm/command",
          [&context](const srf::viz_web::Json& params, const srf::viz_web::WebRpc::Notify&) {
            srf::viz_web::Json result = srf::viz_web::Json::object();
            try {
              const auto response = context.services().call<srf::msgs::String,
                                                            srf::msgs::String>(
                  "llm/command", srf::msgs::String{params["text"].as_string()});
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
  srf::viz_web::VizWebOptions options;
  options.overlay_root = SRF_RECYCLING_WS_VIZ_WEB_ROOT;

  bool help = false;
  std::string error;
  if (!srf::viz_web::parse_viz_web_options(argc, argv, options, help, error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  if (help) {
    return 0;
  }

  // 코어 플러그인 + srf_recycling_ws 프로젝트 플러그인.
  srf::viz_web::VizWebPluginRegistry registry;
  srf::viz_web::register_core_web_plugins(registry,
                                            {"slam", "navigation", "teleop", "diagnostics", "markers"});
  registry.add("opencr", "OpenCR", "/plugins/opencr.js");
  registry.add("moveit", "MoveIt", "/plugins/moveit.js");
  registry.add("rl_nav", "RL Nav", "/plugins/rl_nav.js");
  registry.add("manipulator", "Manipulator", "/plugins/manipulator.js");
  registry.add("camera", "Camera", "/plugins/camera.js");
  registry.add("ai", "AI", "/plugins/ai.js");

  // 프로젝트 플러그인이 쓰는 토픽. 코어 웹 게이트웨이는 기본 토픽만 안다.
  options.forward_topics = {
      {"arm_status", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"map_name", srf::net::Kind::String, srf::QOS_LATCHED},
      {"moveit_status", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"moveit_state", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"moveit_target_pose", srf::net::Kind::Pose, srf::QOS_DEFAULT},
      {"moveit_arm", srf::net::Kind::Path, srf::QOS_DEFAULT},
      {"moveit_path", srf::net::Kind::Path, srf::QOS_DEFAULT},
      {"rl_nav_status", srf::net::Kind::String, srf::QOS_LATCHED},
      {"rl_nav_model_list", srf::net::Kind::String, srf::QOS_LATCHED},
      {"llm/status", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"recycling/truth", srf::net::Kind::String, srf::QOS_DEFAULT},  // 시뮬레이터 진실 판정
      {"vlm/status", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"vla/status", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"pick/status", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"nav/status", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"sim_graspable", srf::net::Kind::Path, srf::QOS_DEFAULT},
      {"grasp_info", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"ai_info", srf::net::Kind::String, srf::QOS_LATCHED},
      {"robot_context", srf::net::Kind::String, srf::QOS_DEFAULT},
      {"camera/color/image_raw", srf::net::Kind::Image, srf::QOS_SENSOR_DATA},
      {"camera/depth/image_rect_raw", srf::net::Kind::Image, srf::QOS_SENSOR_DATA},
  };

  srf::net::PeerNetworkOptions network;
  network.name = "srf_recycling_ws_viz_web";
  if (!options.robot_host.empty()) {
    network.add_server(options.robot_host, options.robot_port);
  }
  network.token = options.token;
  srf::App app(nullptr, network);
  srf::viz_web::VizWeb web(std::move(options), std::move(registry), app.context());
  register_move_rpc(web.rpc(), app.context());
  register_arm_rpc(web.rpc(), app.context());
  register_llm_rpc(web.rpc(), app.context());
  if (!web.initialize(error)) {
    std::fprintf(stderr, "[viz-web] %s\n", error.c_str());
    return 1;
  }
  return web.run();
}
