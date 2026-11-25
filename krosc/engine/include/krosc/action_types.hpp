#pragma once

// 액션의 전송 타입. action.hpp(로컬 API)와 net RPC 코덱이 함께 쓴다.
// 여기에는 Context/Node 의존성이 없어야 한다 (net 계층에서 include 가능).

#include <any>
#include <cstdint>
#include <string>

namespace krosc {

enum class GoalStatus {
  Accepted,
  Succeeded,
  Canceled,
  Aborted,
  Rejected,
};

struct SendGoalRequest {
  std::string goal_id;
  std::any goal;
};
struct SendGoalResponse {
  bool accepted = false;
};
struct CancelGoalRequest {
  std::string goal_id;
};
struct CancelGoalResponse {
  bool canceled = false;
};

struct ActionEvent {
  std::string goal_id;
  bool has_feedback = false;
  bool has_result = false;
  std::any feedback;
  std::any result;
  GoalStatus status = GoalStatus::Accepted;
};

}  // namespace krosc
