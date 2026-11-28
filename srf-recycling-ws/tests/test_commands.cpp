#include <cstdio>
#include <string>
#include <vector>

#include "nodes/llm/command_rules.hpp"

#include "test_framework.hpp"

namespace {

struct CommandCase {
  const char* text;
  const char* intent;
  const char* direction = "";
  double meters = -1.0;
  double degrees = -1.0;
  const char* preset = "";
  const char* target = "";
  const char* melody = "";
  const char* style = "";
};

void check_case(const CommandCase& item) {
  const auto plan = srf_recycling_ws::llm::parse_command(item.text);
  if (!plan) {
    std::fprintf(stderr, "no plan: %s\n", item.text);
  }
  CHECK(plan.has_value());
  if (!plan) {
    return;
  }
  CHECK_EQ(std::string(item.intent), plan->intent);
  if (item.direction[0] != '\0') {
    CHECK_EQ(std::string(item.direction), plan->direction);
  }
  if (item.meters >= 0.0) {
    CHECK_NEAR(item.meters, plan->meters, 1e-9);
  }
  if (item.degrees >= 0.0) {
    CHECK_NEAR(item.degrees, plan->degrees, 1e-9);
  }
  if (item.preset[0] != '\0') {
    CHECK_EQ(std::string(item.preset), plan->preset);
  }
  if (item.target[0] != '\0') {
    CHECK_EQ(std::string(item.target), plan->target);
  }
  if (item.melody[0] != '\0') {
    CHECK_EQ(std::string(item.melody), plan->melody);
  }
  if (item.style[0] != '\0') {
    CHECK_EQ(std::string(item.style), plan->style);
  }
}

}  // namespace

SRF_TEST(commands_manipulator_20) {
  const std::vector<CommandCase> cases = {
      {"앞에 있는 노란색 퍽을 집어 올려", "pick", "", -1, -1, "", "yellow"},
      {"노란 퍽 집어", "pick", "", -1, -1, "", "yellow"},
      {"퍽을 들어 올려", "pick", "", -1, -1, "", "yellow"},
      {"물건 집어", "pick", "", -1, -1, "", "yellow"},
      {"그리퍼로 잡아", "pick", "", -1, -1, "", "yellow"},
      {"앞의 노란 공 잡아", "pick", "", -1, -1, "", "yellow"},
      {"pick up the yellow puck", "pick", "", -1, -1, "", "yellow"},
      {"grab the puck", "pick", "", -1, -1, "", "yellow"},
      {"노란색 원기둥 집어", "pick", "", -1, -1, "", "yellow"},
      {"퍽 들어", "pick", "", -1, -1, "", "yellow"},
      {"바닥에 내려놔", "place", "", -1, -1, "", "floor"},
      {"내려놔", "place", "", -1, -1, "", "floor"},
      {"다시 내려놔", "place", "", -1, -1, "", "floor"},
      {"place on floor", "place", "", -1, -1, "", "floor"},
      {"put it down", "place", "", -1, -1, "", "floor"},
      {"내 몸에 올려놔", "place", "", -1, -1, "", "body"},
      {"잡은 물건을 내 몸에 올려놔", "place", "", -1, -1, "", "body"},
      {"상판에 올려놔", "place", "", -1, -1, "", "body"},
      {"팔 홈으로", "arm_preset", "", -1, -1, "home"},
      {"팔 접어", "arm_preset", "", -1, -1, "stow"},
  };
  for (const auto& item : cases) {
    check_case(item);
  }
}

SRF_TEST(commands_driving_20) {
  const std::vector<CommandCase> cases = {
      {"앞으로 1m", "drive", "forward", 1.0},
      {"전진 1미터", "drive", "forward", 1.0},
      {"앞으로 가", "drive", "forward", 0.5},
      {"직진", "drive", "forward", 0.5},
      {"1m 앞으로", "drive", "forward", 1.0},
      {"go forward 1 meter", "drive", "forward", 1.0},
      {"move forward 50cm", "drive", "forward", 0.5},
      {"앞으로 0.5m 이동", "drive", "forward", 0.5},
      {"전진해", "drive", "forward", 0.5},
      {"뒤로 0.5m", "drive", "backward", 0.5},
      {"후진", "drive", "backward", 0.5},
      {"뒤로 가", "drive", "backward", 0.5},
      {"back up 30cm", "drive", "backward", 0.3},
      {"왼쪽으로 90도", "drive", "left", 0.0, 90.0},
      {"좌회전", "drive", "left", 0.0, 90.0},
      {"오른쪽으로 45도", "drive", "right", 0.0, 45.0},
      {"우회전", "drive", "right", 0.0, 90.0},
      {"turn left 90", "drive", "left", 0.0, 90.0},
      {"turn right", "drive", "right", 0.0, 90.0},
      {"멈춰", "stop"},
  };
  for (const auto& item : cases) {
    check_case(item);
  }
}

SRF_TEST(commands_entertainment_30) {
  const std::vector<CommandCase> cases = {
      {"춤춰", "dance", "", -1, -1, "", "", "", "dance"},
      {"춤춰봐", "dance", "", -1, -1, "", "", "", "dance"},
      {"댄스", "dance", "", -1, -1, "", "", "", "dance"},
      {"dance", "dance", "", -1, -1, "", "", "", "dance"},
      {"춤 한번 춰", "dance", "", -1, -1, "", "", "", "dance"},
      {"신나게 춤춰", "dance", "", -1, -1, "", "", "", "dance"},
      {"파티 타임", "dance", "", -1, -1, "", "", "", "show"},
      {"celebrate", "dance", "", -1, -1, "", "", "", "celebrate"},
      {"축하해", "dance", "", -1, -1, "", "", "", "celebrate"},
      {"기뻐", "dance", "", -1, -1, "", "", "", "dance"},
      {"빙글빙글 돌아", "dance", "", -1, -1, "", "", "", "spin"},
      {"한 바퀴 돌아", "dance", "", -1, -1, "", "", "", "spin"},
      {"제자리에서 돌아", "dance", "", -1, -1, "", "", "", "spin"},
      {"spin", "dance", "", -1, -1, "", "", "", "spin"},
      {"흔들어", "dance", "", -1, -1, "", "", "", "wiggle"},
      {"몸을 흔들어", "dance", "", -1, -1, "", "", "", "wiggle"},
      {"좌우로 흔들어", "dance", "", -1, -1, "", "", "", "wiggle"},
      {"wiggle", "dance", "", -1, -1, "", "", "", "wiggle"},
      {"인사해", "dance", "", -1, -1, "", "", "", "bow"},
      {"고개 숙여", "dance", "", -1, -1, "", "", "", "bow"},
      {"노래부르면서 춤춰", "dance", "", -1, -1, "", "", "", "show"},
      {"음악 틀고 춤춰", "dance", "", -1, -1, "", "", "", "show"},
      {"노래하면서 춤춰", "dance", "", -1, -1, "", "", "", "show"},
      {"음악과 함께 춤춰", "dance", "", -1, -1, "", "", "", "show"},
      {"파티해", "dance", "", -1, -1, "", "", "", "show"},
      {"쇼 해봐", "dance", "", -1, -1, "", "", "", "show"},
      {"웨이브해", "dance", "", -1, -1, "", "", "", "wave"},
      {"팔 흔들어", "dance", "", -1, -1, "", "", "", "wave"},
      {"손 흔들어", "dance", "", -1, -1, "", "", "", "wave"},
      {"신나게 놀아봐", "dance", "", -1, -1, "", "", "", "show"},
  };
  for (const auto& item : cases) {
    check_case(item);
  }
}

SRF_TEST(commands_beep_20) {
  const std::vector<CommandCase> cases = {
      {"마리오 죽을 때 효과음", "buzzer", "", -1, -1, "", "", "mario_death"},
      {"마리오 죽는 소리", "buzzer", "", -1, -1, "", "", "mario_death"},
      {"게임오버 소리", "buzzer", "", -1, -1, "", "", "mario_death"},
      {"mario death", "buzzer", "", -1, -1, "", "", "mario_death"},
      {"죽을 때 효과음", "buzzer", "", -1, -1, "", "", "mario_death"},
      {"성공음", "buzzer", "", -1, -1, "", "", "success"},
      {"성공 소리", "buzzer", "", -1, -1, "", "", "success"},
      {"딩동댕", "buzzer", "", -1, -1, "", "", "success"},
      {"축하음", "buzzer", "", -1, -1, "", "", "success"},
      {"success sound", "buzzer", "", -1, -1, "", "", "success"},
      {"정답 소리", "buzzer", "", -1, -1, "", "", "success"},
      {"경고음", "buzzer", "", -1, -1, "", "", "alarm"},
      {"경고 알람", "buzzer", "", -1, -1, "", "", "alarm"},
      {"위험 알림", "buzzer", "", -1, -1, "", "", "alarm"},
      {"alarm", "buzzer", "", -1, -1, "", "", "alarm"},
      {"삐삐", "buzzer", "", -1, -1, "", "", "alarm"},
      {"삐 소리", "buzzer", "", -1, -1, "", "", "beep"},
      {"비프음", "buzzer", "", -1, -1, "", "", "beep"},
      {"beep", "buzzer", "", -1, -1, "", "", "beep"},
      {"짧게 삐", "buzzer", "", -1, -1, "", "", "beep"},
  };
  for (const auto& item : cases) {
    check_case(item);
  }
}
