// coordinator 흐름 시험: prompt_input → coordinator → llm / vlm / vla / Nav.
// 대역은 진짜 노드와 1:1 (같은 노드 이름·서비스·액션 이름, 각자 따로 교체 가능).
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "nodes/coordinator/coordinator.hpp"
#include "nodes/prompt_input/prompt_input.hpp"
#include "srf/action.hpp"
#include "srf/context.hpp"
#include "srf/msgs/nav2_msgs.hpp"
#include "srf/node.hpp"
#include "test_framework.hpp"

namespace {

using StringServer = srf::ActionServer<srf::msgs::String, srf::msgs::String, srf::msgs::String>;
using NavServer = srf::ActionServer<srf::msgs::nav2::NavigateToPoseGoal, srf::msgs::nav2::NavigateToPoseFeedback,
                                    srf::msgs::nav2::NavigateToPoseResult>;

// llm 노드 대역: llm/parse 가 정해진 plan 을 돌려준다
struct FakeLlm : srf::Node {
  std::string plan;
  explicit FakeLlm(srf::Context& c, std::string p) : Node(c, "llm"), plan(std::move(p)) {
    create_service<srf::msgs::String, srf::msgs::String>("llm/parse", [this](const srf::msgs::String&) {
      return srf::msgs::String{"{\"ok\":true,\"source\":\"rule\",\"via\":\"rule\",\"plan\":" + plan + "}"};
    });
  }
};

// vlm 노드 대역: vlm/select 가 준비된 답을 차례로 돌려준다
struct FakeVlm : srf::Node {
  std::mutex mutex;
  std::deque<std::string> answers;
  explicit FakeVlm(srf::Context& c, std::deque<std::string> a) : Node(c, "vlm"), answers(std::move(a)) {
    create_service<srf::msgs::String, srf::msgs::String>("vlm/select", [this](const srf::msgs::String&) {
      std::lock_guard<std::mutex> lock(mutex);
      std::string answer = answers.empty() ? R"({"decision":"hold","reason":"absent"})" : answers.front();
      if (!answers.empty()) answers.pop_front();
      return srf::msgs::String{answer};
    });
  }
};

// vla 노드 대역: vla/execute goal 을 기록하고 성공
struct FakeVla : srf::Node {
  std::mutex mutex;
  std::vector<std::string> goals;
  std::unique_ptr<StringServer> server;
  explicit FakeVla(srf::Context& c) : Node(c, "vla") {
    server = std::make_unique<StringServer>(c, "vla/execute", [this](auto& handle) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        goals.push_back(handle.goal().data);
      }
      handle.publish_feedback(srf::msgs::String{R"({"step":0})"});
      return srf::msgs::String{R"({"ok":true})"};
    });
  }
};

// bt_navigator 대역: navigate_to_pose 를 기록하고 성공
struct FakeNav : srf::Node {
  std::mutex mutex;
  std::vector<double> xs;
  std::unique_ptr<NavServer> server;
  explicit FakeNav(srf::Context& c) : Node(c, "bt_navigator") {
    server = std::make_unique<NavServer>(c, srf::msgs::nav2::NAVIGATE_TO_POSE_ACTION, [this](auto& handle) {
      std::lock_guard<std::mutex> lock(mutex);
      xs.push_back(handle.goal().pose.pose.position.x);
      return srf::msgs::nav2::NavigateToPoseResult{};
    });
  }
};

srf::msgs::MapMarkers markers() {
  srf::msgs::MapMarkers m;
  m.header.frame_id = "map";
  for (const auto& [name, x] : std::vector<std::pair<std::string, double>>{
           {"work_mat_approach", 1.0}, {"bin_can_approach", 2.0}, {"bin_pet_approach", 3.0}}) {
    srf::msgs::MapMarker marker;
    marker.name = name;
    marker.kind = "waypoint";
    marker.pose.position.x = x;
    marker.pose.orientation.w = 1.0;
    m.markers.push_back(marker);
  }
  return m;
}

// prompt/status 에서 끝(done/error/hold) 이 올 때까지
std::string run_prompt(srf::Context& context, const std::string& text) {
  srf::Node client(context, "test_client");
  std::mutex mutex;
  std::string final_status;
  auto sub = client.subscribe<srf::msgs::String>("prompt/status", [&](const srf::msgs::String& m) {
    if (m.data.find("\"stage\":\"done\"") != std::string::npos || m.data.find("\"stage\":\"error\"") != std::string::npos ||
        m.data.find("\"stage\":\"hold\"") != std::string::npos) {
      std::lock_guard<std::mutex> lock(mutex);
      final_status = m.data;
    }
  });
  const auto accepted = context.services().call<srf::msgs::String, srf::msgs::String>("prompt/submit", srf::msgs::String{text});
  CHECK((accepted.data.find("\"accepted\":true") != std::string::npos));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!final_status.empty()) return final_status;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return {};
}

}  // namespace

SRF_TEST(coordinator_sort_until_absent) {
  srf::Context context;
  srf::Node map(context, "map_marker");
  map.publish<srf::msgs::MapMarkers>("map_markers", markers(), srf::QOS_LATCHED);
  FakeLlm llm(context, R"({"intent":"sort","target":"can"})");
  FakeVlm vlm(context, {R"({"decision":"select","reason":"ok","target":{"category":"can","label":"cola","bbox":[1,2,3,4]},"target_xyz":[0.2,0.0,0.03]})",
                        R"({"decision":"select","reason":"ok","target":{"category":"can","label":"tuna","bbox":[1,2,3,4]},"target_xyz":[0.25,0.05,0.02]})",
                        R"({"decision":"hold","reason":"absent"})"});
  FakeVla vla(context);
  FakeNav nav(context);
  srf_recycling_ws::Coordinator coordinator(context);
  srf_recycling_ws::PromptInput prompt(context);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const std::string status = run_prompt(context, "캔 넣어");
  CHECK((status.find("\"stage\":\"done\"") != std::string::npos));
  CHECK((status.find("\"placed\":{\"can\":2}") != std::string::npos));
  CHECK((status.find("\"finished\":\"absent\"") != std::string::npos));
  // 순서: 매트 → 집기 → 바구니 → 넣기 (×2) → 매트 (없음)
  CHECK_EQ(std::size_t{4}, vla.goals.size());
  CHECK((vla.goals[0].find("\"kind\":\"pick\"") != std::string::npos && vla.goals[0].find("[0.2,0.0,0.03]") != std::string::npos));
  CHECK((vla.goals[1].find("\"kind\":\"place\"") != std::string::npos && vla.goals[1].find("bin_can") != std::string::npos));
  const std::vector<double> expected{1.0, 2.0, 1.0, 2.0, 1.0};
  CHECK((nav.xs == expected));
}

SRF_TEST(coordinator_hold_blocks_vla) {
  srf::Context context;
  srf::Node map(context, "map_marker");
  map.publish<srf::msgs::MapMarkers>("map_markers", markers(), srf::QOS_LATCHED);
  FakeLlm llm(context, R"({"intent":"sort","target":"pet"})");
  FakeVlm vlm(context, {R"({"decision":"hold","reason":"ambiguous","answer":"..."})"});
  FakeVla vla(context);
  FakeNav nav(context);
  srf_recycling_ws::Coordinator coordinator(context);
  srf_recycling_ws::PromptInput prompt(context);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const std::string status = run_prompt(context, "페트병 치워");
  CHECK((status.find("\"stage\":\"hold\"") != std::string::npos));
  CHECK((status.find("\"reason\":\"ambiguous\"") != std::string::npos));
  CHECK(vla.goals.empty());  // 보류면 뒤 동작(집기)을 하지 않는다
}
