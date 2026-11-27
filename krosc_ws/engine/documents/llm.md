# LLM: intent parser / router (LLM-first, 규칙 fallback)

> LLM은 **모터를 제어하지 않는다.** 문장을 `{intent, 파라미터}` JSON으로 바꾸고,
> C++ 라우터가 기존 서비스/액션으로 넘긴다.
> 기본 `KROSC_LLM_MODE=llm`에서는 **모든 명령이 Ollama를 먼저** 타서
> `vlm_query`/`vla_pick`/`pick` 등을 결정한다. 90개 규칙은 LLM 실패/unsupported
> 시 fallback이다. `KROSC_LLM_MODE=hybrid|rule`로 바꿀 수 있다.

## 1. 소스

| 파일 | 역할 |
|---|---|
| `krosc_ws/engine/nodes/command_rules.cpp` | 90개 자연어 규칙, 결정적 파서 |
| `krosc_ws/engine/nodes/llm_input.cpp` | 규칙→Ollama 폴백, worker, dispatch, drive/turn/dance |
| `krosc_ws/engine/nodes/llm_json.cpp` | Ollama 응답/plan 파서, JSON escape |
| `krosc_ws/engine/nodes/llm_client.cpp` | `/api/chat` 요청 본문 + 응답 content |
| `krosc_ws/engine/nodes/http_client.cpp` | 의존성 없는 HTTP/1.1 POST (POSIX 소켓) |
| `krosc_ws/viz-web/main.cpp` | `llm/command` RPC 브리지 |
| `krosc_ws/viz-web/www/plugins/ai.js` | AI 패널, 버튼, `llm/status` 표시 |
| `krosc_ws/engine/documents/commands.md` | 90개 명령 표 |

## 2. 처리 흐름

```text
browser ai.js
  host.rpc('llm/command', {text})
    → viz-web RPC
      context.services().call<String,String>('llm/command', text)
        → llm_input::handle_command
          → queue + worker
            → POST /api/chat (mode=llm, 기본)
              ├─ 성공: source="llm", LLM intent 사용
              └─ 실패/unsupported: source="rule"|"llm+rule", 90개 규칙 fallback
            → parse_plan + apply_direction_hint
            → dispatch
```

- 서비스는 **즉시 accepted + request_id**를 돌려주고, 실제 처리는 워커가 한다.
- 진행/결과는 `llm/status` String JSON으로 나간다.
- `llm/status`의 `source` 필드로 규칙/LLM 중 어디서 왔는지 알 수 있다.
- `llm/command`는 서비스 이름일 뿐, `llm_input` 노드 이름과 자동으로 연결되지 않는다.
  krosc 서비스는 전역 이름이고 ROS 2 namespace 자동 해석이 없다(README 참고).

### 예시: 규칙 경로

```text
"내 몸에 올려놔"
  → parse_command
    → {"intent":"place","target":"body"}
  → llm/status {stage:"plan", source:"rule", plan:{...}}
  → pick/command {"action":"place","target":"body"}
```

### 예시: LLM 폴백

```text
"책상 위 빨간 큐브를 왼쪽으로 옮겨줘"
  → parse_command: nullopt (90개에 없음)
  → Ollama /api/chat (JSON schema)
  → {"intent":"vla_pick","target":"red cube"}
  → llm/status {stage:"plan", source:"llm", plan:{...}}
```

## 3. 90개 규칙

매니퓰레이터 20 + 주행 20 + 엔터테인먼트 30 + 비프 20 = 90개다. 전체 표는 [commands.md](commands.md).

| 카테고리 | 예 | 결과 |
|---|---|---|
| 매니퓰레이터 | `앞에 있는 노란색 퍽을 집어 올려` | `pick yellow` |
| | `내 몸에 올려놔` | `place body` |
| | `팔 접어` | `arm_preset stow` |
| 주행 | `앞으로 1m` | `drive forward 1.0` |
| | `왼쪽으로 90도` | `drive left 90` |
| | `멈춰` | `stop` |
| 엔터테인먼트 | `춤춰` | `dance dance` |
| | `빙글빙글 돌아` | `dance spin` |
| | `인사해` | `dance bow` |
| | `노래부르면서 춤춰` | `dance show` |
| | `손 흔들어` | `dance wave` |
| 비프음 | `마리오 죽을 때 효과음` | `buzzer mario_death` |
| | `성공음` | `buzzer success` |
| | `삐 소리` | `buzzer beep` |

### 매칭 순서가 중요한 이유

```text
1. stop
2. arm_preset (팔 + home/stow)
3. place body (몸/상판/deck/body)
4. place floor (바닥/내려놔/put down)
5. pick (집/잡/픽/pick/grab/lift)
6. hide (숨/안 보이/가려)
7. approach (벽/wall/붙어/가까이)
8. buzzer (마리오/성공/경고/삐)
9. dance (쇼/웨이브/춤/빙글/흔들/인사/축하)
10. drive (방향 + 숫자)
```

- `팔 숨겨`를 hide보다 먼저 잡아야 arm stow가 된다.
- `내 몸에 올려놔`를 pick보다 먼저 잡아야 body place가 된다.
- `빙글빙글 돌아`를 drive turn보다 먼저 잡아야 dance spin이 된다.
- `축하해`(dance)와 `축하음`(buzzer success)은 `음/소리` 유무로 갈린다.

자세한 경계 케이스는 [commands.md](commands.md#5-규칙-매칭-순서-중요).

## 4. Schema

`llm_input.cpp`의 `kSchema`:

```json
{
  "type": "object",
  "properties": {
    "intent": {"type": "string", "enum": [
      "drive","arm_preset","stop","buzzer","pick","place",
      "hide","approach","dance","vlm_query","vla_pick","unsupported"]},
    "meters": {"type": "number"},
    "degrees": {"type": "number"},
    "direction": {"type": "string", "enum": ["forward","backward","left","right"]},
    "preset": {"type": "string", "enum": ["home","stow"]},
    "melody": {"type": "string", "enum": ["mario_death","success","alarm","beep"]},
    "style": {"type": "string", "enum": ["dance","spin","wiggle","bow","celebrate","show","wave"]},
    "question": {"type": "string"},
    "target": {"type": "string"}
  },
  "required": ["intent","meters"]
}
```

- `meters`가 **필수**다. drive가 아니면 0. 초기 버전에서 gemma3:4b가 `meters`를 빼먹어
  `drive에 meters가 없다`가 났고, required + "Always include meters"로 고쳤다.
- `place`의 `target`은 `floor`(기본) 또는 `body`(로봇 상판)다.
- `dance`의 `style`은 `dance`(기본), `spin`, `wiggle`, `bow`, `celebrate`, `show`, `wave`.
- `vla_pick`은 학습 VLA(red/blue) 전용, `pick`은 결정적 매니퓰레이터다.

## 5. plan 예시

```json
{"intent":"drive","meters":1.0,"direction":"forward"}
{"intent":"drive","meters":0.5,"direction":"backward"}
{"intent":"drive","meters":0,"direction":"left","degrees":90}
{"intent":"dance","meters":0,"style":"spin"}
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

## 6. 라우팅 (dispatch)

```cpp
if (intent == "drive") { ... start_drive / start_turn ... }
if (intent == "dance") { ... dance(id, style) ... }
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

## 7. drive / turn

- `start_drive(id, meters, sign)`:
  - odom 시작점 저장, `cmd_vel_nav` linear `0.15 * sign` (후진이면 음수).
  - 거리 = hypot(odom - start). 목표 도달 시 zero.
  - 2초 동안 1cm도 못 가면 `drive stalled (collision?)`.
  - 30초 타임아웃.
- `start_turn(id, left/right, degrees)`:
  - odom yaw 시작점, `angular = ±0.8 rad/s`.
  - `|wrap(yaw-start)| >= target`이면 정지. 2초 정체/30초 타임아웃.
- 두 경우 모두 `cmd_vel_nav` → twist_mux → velocity_smoother → collision_monitor → `cmd_vel`.

### 예시

```text
"앞으로 1m"
  executing {tool:"drive", direction:"forward", meters:1.0}
  done      {distance:1.01}

"왼쪽으로 90도"
  executing {tool:"turn", direction:"left", degrees:90}
  done      {degrees:90.2}
```

## 8. dance

`dance`는 100ms마다 `cmd_vel_nav`를 발행하는 짧은 오픈루프 시퀀스다.
odom 목표가 아니라 **시간 기반**이다.

| style | duration | 패턴 |
|---|---|---|
| `dance` | 4s | `linear=0.07*sin(3t)`, `angular=0.7*sin(6t)` |
| `spin` | 3s | `angular=0.9` |
| `wiggle` | 3s | `angular=±0.8` (0.4s) |
| `bow` | 2s | `linear=±0.07` (0.4s) |
| `celebrate` | 3s | `success` 멜로디 + dance |
| `show` | 8s | 주행 댄스 + `party` 멜로디 + 5관절 웨이브 |
| `wave` | 3s | 제자리 + 5관절 웨이브 |

- `show`/`wave`는 `arm_command_controller`로 전환해 `arm_joint_command`를 5Hz로 낸다.
  끝나면 `joint_trajectory_controller`로 돌아간다.
- `정지`/`stop` → `emergency_stop()` → `dance_cancel_=true` → 즉시 zero.
- 댄스 중에는 워커가 점유되므로 다음 명령은 끝난 뒤 처리된다.
- collision_monitor가 벽을 보면 더 먼저 멈출 수 있다.

## 9. arm_preset

- MoveIt `move_action`에 `preset=home|stow` goal.
- 이동 전 `controller_manager/switch_controller`로 `joint_trajectory_controller`를 켠다.
  strictness 0(best effort) — `arm_command_controller`가 비활성인 상태에서 strictness 1이면
  스위치가 실패해 팔이 안 움직이는 문제가 있었다.
- 결과는 feedback(`moveit`)과 최종 done/error로 `llm/status`에 남는다.

## 10. 상태/에러

| 상황 | status |
|---|---|
| 빈 명령 | `{"accepted":false,"error":"빈 명령"}` |
| 규칙 매칭 | `stage:"plan", source:"rule"` |
| Ollama 연결/타임아웃 | `stage:"error", error:"연결 실패..."` |
| plan 파싱 실패 | `stage:"error", error:"모델 응답 파싱 실패..."` |
| pick 서비스 없음 | `stage:"error", error:"pick node 없음..."` |
| nav 서비스 없음 | `stage:"error", error:"nav behavior 없음..."` |
| stop | `vla/stop`, `pick/stop`, `nav/stop` + drive zero |

## 11. 설정

- Ollama URL: `EngineOptions::llm_ollama_url` (`http://127.0.0.1:11434`)
- 모델: `gemma3:4b`
- timeout: 10초
- `krosc_ws/engine/engine.cpp`에서 `LlmInput` 생성.
- 원격 Ollama는 `KROSC_OLLAMA_URL=http://<server>:11434`로 덮어쓴다.
  (90개 규칙은 Ollama 없이도 동작한다.)

## 12. 테스트

```bash
# 90개 규칙
KROSC_TEST_FILTER=commands_ ./krosc_ws/build-mac/krosc_ws_tests

# 모델 단독 확인
curl -s http://127.0.0.1:11434/api/tags

# 웹
open http://localhost:8080
# AI 탭 → "앞으로 1m", "춤춰", "성공음", "내 몸에 올려놔"
```

`llm/status`, `pick/status`, `vla/status`, `nav/status`가 AI 패널의 `action`/`plan` 줄에 표시된다.
