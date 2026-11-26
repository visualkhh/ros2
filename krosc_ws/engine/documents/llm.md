# LLM: Ollama intent parser / router

> LLM은 **모터를 제어하지 않는다.** 문장을 `{intent, 파라미터}` JSON으로 바꾸고,
> C++ 라우터가 기존 서비스/액션으로 넘긴다. Ollama 호출은 워커 스레드에서 한다.

## 1. 소스

| 파일 | 역할 |
|---|---|
| `krosc_ws/engine/nodes/llm_input.cpp` | schema, prompt, worker, intent 라우팅, drive/turn, 상태 |
| `krosc_ws/engine/nodes/llm_json.cpp` | Ollama 응답/plan 파서, JSON escape |
| `krosc_ws/engine/nodes/llm_client.cpp` | `/api/chat` 요청 본문 + 응답 content |
| `krosc_ws/engine/nodes/http_client.cpp` | 의존성 없는 HTTP/1.1 POST (POSIX 소켓) |
| `krosc_ws/viz-web/main.cpp` | `llm/command` RPC 브리지 |
| `krosc_ws/viz-web/www/plugins/ai.js` | AI 패널, 버튼, `llm/status` 표시 |

## 2. Schema

`llm_input.cpp`의 `kSchema`:

```json
{
  "type": "object",
  "properties": {
    "intent": {"type": "string", "enum": [
      "drive","arm_preset","stop","buzzer","pick","place",
      "hide","approach","vlm_query","vla_pick","unsupported"]},
    "meters": {"type": "number"},
    "degrees": {"type": "number"},
    "direction": {"type": "string", "enum": ["forward","backward","left","right"]},
    "preset": {"type": "string", "enum": ["home","stow"]},
    "melody": {"type": "string", "enum": ["mario_death","success","alarm","beep"]},
    "question": {"type": "string"},
    "target": {"type": "string"}
  },
  "required": ["intent","meters"]
}
```

- `meters`가 **필수**다. drive가 아니면 0. 초기 버전에서 gemma3:4b가 `meters`를 빼먹어
  `drive에 meters가 없다`가 났고, required + "Always include meters"로 고쳤다.
- `place`의 `target`은 `floor`(기본) 또는 `body`(로봇 상판)다.
  `내려놔/바닥/다시 내려놔` = floor, `내 몸/몸 위` = body.
- `vla_pick`은 학습 VLA(red/blue) 전용, `pick`은 결정적 매니퓰레이터다.

## 3. 호출

```text
browser ai.js
  host.rpc('llm/command', {text})
    → viz-web RPC
      context.services().call<String,String>('llm/command', text)
        → llm_input::handle_command
          → queue + worker
            → Ollama POST http://127.0.0.1:11434/api/chat
              {model, stream:false, format:schema, temperature:0, messages}
            → message.content = JSON plan
          → parse + apply_direction_hint
          → dispatch
```

- 서비스는 **즉시 accepted + request_id**를 돌려주고, 실제 처리는 워커가 한다.
- 진행/결과는 `llm/status` String JSON으로 나간다.
- `llm/command`는 서비스 이름일 뿐, `llm_input` 노드 이름과 자동으로 연결되지 않는다.
  krosc 서비스는 전역 이름이고 ROS 2 namespace 자동 해석이 없다(README 참고).

## 4. plan 예시

```json
{"intent":"drive","meters":1.0,"direction":"forward"}
{"intent":"drive","meters":0.5,"direction":"backward"}
{"intent":"drive","meters":0,"direction":"left","degrees":90}
{"intent":"arm_preset","meters":0,"preset":"home"}
{"intent":"buzzer","meters":0,"melody":"mario_death"}
{"intent":"pick","meters":0,"target":"yellow"}
{"intent":"place","meters":0,"target":"floor"}
{"intent":"place","meters":0,"target":"body"}
{"intent":"hide","meters":0}
{"intent":"approach","meters":0}
{"intent":"vlm_query","meters":0,"question":"what do you see?"}
{"intent":"vla_pick","meters":0,"target":"red block"}
```

`apply_direction_hint()`는 모델이 "오른쪽으로 45도"를 `arm_preset`으로 오분류하는 것을
막기 위해 좌/우/뒤/회전/도 문구를 결정적으로 `drive`로 보정한다.
(예: `오른쪽으로 45도` → `direction=right, degrees=45`)

## 5. 라우팅 (dispatch)

```cpp
if (intent == "drive") { ... start_drive / start_turn ... }
if (intent == "arm_preset") { ... send_arm_preset ... }
if (intent == "stop") { emergency_stop(); }
if (intent == "buzzer") { play_melody(name); }
if (intent == "pick" || intent == "place") {
  call_optional("pick/command", {action, color, target});
}
if (intent == "hide" || intent == "approach") {
  call_optional(intent == "hide" ? "nav/hide" : "nav/approach", {});
}
if (intent == "vlm_query") { call_optional("vlm/query", {question}); }
if (intent == "vla_pick")  { call_optional("vla/command", {target}); }
```

## 6. drive / turn

- `start_drive(id, meters, sign)`:
  - odom 시작점 저장, `cmd_vel_nav` linear `0.15 * sign` (후진이면 음수).
  - 거리 = hypot(odom - start). 목표 도달 시 zero.
  - 2초 동안 1cm도 못 가면 `drive stalled (collision?)`.
  - 30초 타임아웃.
- `start_turn(id, left/right, degrees)`:
  - odom yaw 시작점, `angular = ±0.8 rad/s`.
  - `|wrap(yaw-start)| >= target`이면 정지. 2초 정체/30초 타임아웃.
- 두 경우 모두 `cmd_vel_nav` → twist_mux → velocity_smoother → collision_monitor → `cmd_vel`.

## 7. arm_preset

- MoveIt `move_action`에 `preset=home|stow` goal.
- 이동 전 `controller_manager/switch_controller`로 `joint_trajectory_controller`를 켠다.
  strictness 0(best effort) — `arm_command_controller`가 비활성인 상태에서 strictness 1이면
  스위치가 실패해 팔이 안 움직이는 문제가 있었다.
- 결과는 feedback(`moveit`)과 최종 done/error로 `llm/status`에 남는다.

## 8. 에러/상태

| 상황 | status |
|---|---|
| 빈 명령 | `{"accepted":false,"error":"빈 명령"}` |
| Ollama 연결/타임아웃 | `stage:"error", error:"연결 실패..."` |
| plan 파싱 실패 | `stage:"error", error:"모델 응답 파싱 실패..."` |
| pick 서비스 없음 | `stage:"error", error:"pick node 없음..."` |
| nav 서비스 없음 | `stage:"error", error:"nav behavior 없음..."` |
| stop | `vla/stop`, `pick/stop`, `nav/stop` + drive zero |

## 9. 설정

- Ollama URL: `EngineOptions::llm_ollama_url` (`http://127.0.0.1:11434`)
- 모델: `gemma3:4b`
- timeout: 10초
- `krosc_ws/engine/engine.cpp`에서 `LlmInput` 생성.

## 10. 테스트

```bash
# 모델 단독 확인
curl -s http://127.0.0.1:11434/api/tags

# 웹
open http://localhost:8080
# AI 탭 → "앞으로 1m", "왼쪽으로 90도", "마리오 ..."
```

`llm/status`, `pick/status`, `vla/status`, `nav/status`가 AI 패널의 `action`/`plan` 줄에 표시된다.
