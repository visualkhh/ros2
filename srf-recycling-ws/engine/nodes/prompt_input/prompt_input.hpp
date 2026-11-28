#pragma once

#include <memory>
#include <string>

#include "srf/action.hpp"
#include "srf/msgs.hpp"
#include "srf/node.hpp"

namespace srf_recycling_ws {

// 사람이 하는 말의 입구 (viz web 입력창, 나중에 음성·CLI). 해석·실행은 하지 않는다.
//   받은 문장을 coordinator/task goal 로 보내고, action feedback/result 를 prompt/status 로 내보낸다 (화면용).
//   "정지/멈춰/stop" 은 큐를 기다리지 않고 바로 coordinator/stop.
// service: prompt/submit (String 문장 -> String JSON {"accepted","goal_id"})
// topic:   prompt/status (String JSON: {"goal_id","text","stage",...coordinator feedback/result})
class PromptInput final : public srf::Node {
 public:
  explicit PromptInput(srf::Context& context);
  ~PromptInput() override;

  PromptInput(const PromptInput&) = delete;
  PromptInput& operator=(const PromptInput&) = delete;

 private:
  using TaskClient = srf::ActionClient<srf::msgs::String, srf::msgs::String, srf::msgs::String>;

  srf::msgs::String submit(const srf::msgs::String& request);
  void status(const std::string& goal_id, const std::string& text, const std::string& json);

  std::unique_ptr<TaskClient> task_;
};

}  // namespace srf_recycling_ws
