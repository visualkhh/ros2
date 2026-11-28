# 90개 자연어 명령 카탈로그

> 이 문서는 **실제로 동작하는 명령 90개**를 모은 것이다.
> 매니퓰레이터 20 + 주행 20 + 엔터테인먼트 30 + 비프음 20.
> 기본 `SRF_DEFAULT_WS_LLM_MODE=llm`에서는 Ollama가 먼저 intent를 정하고,
> `command_rules.cpp`의 90개 규칙은 LLM 실패/unsupported 시 fallback이다.
> 표의 `plan`은 AI 패널의 `plan` 줄, `llm/status`의 `plan` JSON과 같은 모양이다.

## 0. 읽는 법

```text
"앞으로 1m"
  → llm_input::process
    → llm::parse_command()  ← 90개 규칙 (Ollama 호출 없음)
      → {"intent":"drive","direction":"forward","meters":1.0}
    → dispatch
      → cmd_vel_nav 로 0.15 m/s 전진
      → 1m 도달 시 정지, llm/status done
```

- **규칙 우선**: 90개 예시는 네트워크/모델 상태와 무관하게 즉시 동작한다.
- **LLM 폴백**: 규칙에 없는 자유 문장만 Ollama가 해석한다.
- `meters`는 drive가 아니면 0, `degrees`는 회전이 아니면 0이다.
- `place`의 `target`은 `floor`(바닥) 또는 `body`(로봇 상판)다.
- `pick`은 현재 **노란 4cm × 12cm 퍽**만 대상이다. 빨강/파랑은 `vla_pick`(학습 정책)이다.

---

## 1. 매니퓰레이터 (20)

| # | 자연어 명령 | plan | 실제 동작 |
|---:|---|---|---|
| 1 | 앞에 있는 노란색 퍽을 집어 올려 | `pick target=yellow` | nav2 접근 → MoveIt approach/descend/close/lift |
| 2 | 노란 퍽 집어 | `pick target=yellow` | 위와 같음 |
| 3 | 퍽을 들어 올려 | `pick target=yellow` | 위와 같음 |
| 4 | 물건 집어 | `pick target=yellow` | 위와 같음 (기본 색 yellow) |
| 5 | 그리퍼로 잡아 | `pick target=yellow` | 위와 같음 |
| 6 | 앞의 노란 공 잡아 | `pick target=yellow` | 위와 같음 |
| 7 | pick up the yellow puck | `pick target=yellow` | 위와 같음 |
| 8 | grab the puck | `pick target=yellow` | 위와 같음 |
| 9 | 노란색 원기둥 집어 | `pick target=yellow` | 위와 같음 |
| 10 | 퍽 들어 | `pick target=yellow` | 위와 같음 |
| 11 | 바닥에 내려놔 | `place target=floor` | lower z=0.06 → open → retreat |
| 12 | 내려놔 | `place target=floor` | 위와 같음 |
| 13 | 다시 내려놔 | `place target=floor` | 위와 같음 |
| 14 | place on floor | `place target=floor` | 위와 같음 |
| 15 | put it down | `place target=floor` | 위와 같음 |
| 16 | 내 몸에 올려놔 | `place target=body` | `sim/deck_free` 빈 자리 → 상판에 내려놓기 |
| 17 | 잡은 물건을 내 몸에 올려놔 | `place target=body` | 위와 같음 |
| 18 | 상판에 올려놔 | `place target=body` | 위와 같음 |
| 19 | 팔 홈으로 | `arm_preset preset=home` | MoveIt home 자세 |
| 20 | 팔 접어 | `arm_preset preset=stow` | MoveIt stow 자세 |

### 왜 pick과 place가 별도 plan인가

- `pick`은 **물체를 찾고 접근해서 집는** 계획이다. nav2 접근, MoveIt 4단계, 그리퍼 닫기.
- `place`는 **이미 든 물체를 놓는** 계획이다. 내릴 위치만 다르다.
- 바닥/몸은 place의 **target 파라미터**이지 별도 intent가 아니다.
  - `내려놔` → `target=floor`
  - `내 몸에 올려놔` → `target=body`

### body place의 빈 자리

`pick_place`는 `sim/deck_free` 서비스를 호출한다. 시뮬레이터가
LDS/팔 베이스/이미 올린 payload를 피해서 `(x, y)`를 준다.
후보는 앞쪽 `x=0.05, y=0`부터 시작하고, 그 자리가 차 있으면 옆으로 밀린다.
서비스가 없으면 `(0.05, 0)`으로 폴백한다.

---

## 2. 주행 (20)

| # | 자연어 명령 | plan | 실제 동작 |
|---:|---|---|---|
| 1 | 앞으로 1m | `drive forward 1.0m` | 0.15 m/s로 1m 전진 |
| 2 | 전진 1미터 | `drive forward 1.0m` | 위와 같음 |
| 3 | 앞으로 가 | `drive forward 0.5m` | 숫자가 없으면 0.5m |
| 4 | 직진 | `drive forward 0.5m` | 위와 같음 |
| 5 | 1m 앞으로 | `drive forward 1.0m` | 위와 같음 |
| 6 | go forward 1 meter | `drive forward 1.0m` | 위와 같음 |
| 7 | move forward 50cm | `drive forward 0.5m` | cm→m 변환 |
| 8 | 앞으로 0.5m 이동 | `drive forward 0.5m` | 위와 같음 |
| 9 | 전진해 | `drive forward 0.5m` | 위와 같음 |
| 10 | 뒤로 0.5m | `drive backward 0.5m` | 후진 |
| 11 | 후진 | `drive backward 0.5m` | 후진 |
| 12 | 뒤로 가 | `drive backward 0.5m` | 후진 |
| 13 | back up 30cm | `drive backward 0.3m` | 후진 |
| 14 | 왼쪽으로 90도 | `drive left 90°` | 제자리 좌회전 |
| 15 | 좌회전 | `drive left 90°` | 각도가 없으면 90° |
| 16 | 오른쪽으로 45도 | `drive right 45°` | 제자리 우회전 |
| 17 | 우회전 | `drive right 90°` | 위와 같음 |
| 18 | turn left 90 | `drive left 90°` | 위와 같음 |
| 19 | turn right | `drive right 90°` | 위와 같음 |
| 20 | 멈춰 | `stop` | drive/arm/nav/VLA 즉시 정지 |

### 주행 안전 체인

```text
cmd_vel_nav → twist_mux(priority 10) → velocity_smoother → collision_monitor → cmd_vel
```

- `drive`는 **odom 거리**로 멈춘다. 라이다가 아니라 odom이다.
- 2초 동안 1cm도 못 가면 `drive stalled (collision?)`로 멈춘다.
- 실제 정지는 collision_monitor가 더 먼저 할 수 있다. 그래서 "명령은 성공인데
  생각보다 일찍 멈췄다"가 정상일 수 있다.

### 벽/숨기 (주행 behavior)

| 명령 예 | plan | 실제 |
|---|---|---|
| 앞에 보이는 벽까지 가까이 붙어 | `approach` | `scan` 전방 ±0.35rad, 0.28m까지 전진 |
| 안 보이는 곳으로 숨어 | `hide` | global costmap의 가려진 free cell → nav2 |

`approach`는 지도 없이 된다. `hide`는 **map + localization**이 필요하다.
자세한 내용은 [nav.md](nav.md).

---

## 3. 엔터테인먼트 (30)

| # | 자연어 명령 | plan | 실제 동작 |
|---:|---|---|---|
| 1 | 춤춰 | `dance style=dance` | 4초 전진/후진 + 좌우 회전 |
| 2 | 춤춰봐 | `dance style=dance` | 위와 같음 |
| 3 | 댄스 | `dance style=dance` | 위와 같음 |
| 4 | dance | `dance style=dance` | 위와 같음 |
| 5 | 춤 한번 춰 | `dance style=dance` | 위와 같음 |
| 6 | 신나게 춤춰 | `dance style=dance` | 위와 같음 |
| 7 | 파티 타임 | `dance style=celebrate` | 성공음 + 3초 댄스 |
| 8 | celebrate | `dance style=celebrate` | 위와 같음 |
| 9 | 축하해 | `dance style=celebrate` | 위와 같음 |
| 10 | 기뻐 | `dance style=dance` | 4초 댄스 |
| 11 | 빙글빙글 돌아 | `dance style=spin` | 3초 제자리 회전 |
| 12 | 한 바퀴 돌아 | `dance style=spin` | 위와 같음 |
| 13 | 제자리에서 돌아 | `dance style=spin` | 위와 같음 |
| 14 | spin | `dance style=spin` | 위와 같음 |
| 15 | 흔들어 | `dance style=wiggle` | 3초 좌우 흔들기 |
| 16 | 몸을 흔들어 | `dance style=wiggle` | 위와 같음 |
| 17 | 좌우로 흔들어 | `dance style=wiggle` | 위와 같음 |
| 18 | wiggle | `dance style=wiggle` | 위와 같음 |
| 19 | 인사해 | `dance style=bow` | 2초 앞뒤로 숙이기 |
| 20 | 고개 숙여 | `dance style=bow` | 위와 같음 |
| 21 | 노래부르면서 춤춰 | `dance style=show` | 8초 주행 댄스 + 팔 웨이브 + 파티 멜로디 |
| 22 | 음악 틀고 춤춰 | `dance style=show` | 위와 같음 |
| 23 | 노래하면서 춤춰 | `dance style=show` | 위와 같음 |
| 24 | 음악과 함께 춤춰 | `dance style=show` | 위와 같음 |
| 25 | 파티해 | `dance style=show` | 위와 같음 |
| 26 | 쇼 해봐 | `dance style=show` | 위와 같음 |
| 27 | 웨이브해 | `dance style=wave` | 3초 제자리 팔 웨이브 |
| 28 | 팔 흔들어 | `dance style=wave` | 위와 같음 |
| 29 | 손 흔들어 | `dance style=wave` | 위와 같음 |
| 30 | 신나게 놀아봐 | `dance style=show` | 8초 쇼 |

### dance는 어떻게 움직이나

`llm_input` 워커가 100ms마다 `cmd_vel_nav`를 발행한다. 4초짜리 댄스는
다음 명령을 4초간 막는다(큐잉). `정지`/`stop`을 누르면 `dance_cancel_`이 서서
즉시 0 속도로 멈춘다.

```cpp
// 개념
while (!cancel && elapsed < duration) {
  publish("cmd_vel_nav", pattern(style, elapsed));
  sleep(100ms);
}
publish("cmd_vel_nav", zero);
```

- `dance`: `linear=0.07*sin(3t)`, `angular=0.7*sin(6t)`
- `spin`: `angular=0.9`
- `wiggle`: `angular=±0.8` (0.4초마다)
- `bow`: `linear=±0.07` (0.4초마다)
- `celebrate`: `success` 멜로디 후 `dance`
- `show`: 8초. `linear=0.08*sin(0.8t)`, `angular=0.6*sin(1.3t)` +
  `party` 멜로디 + 5관절 웨이브
- `wave`: 3초. 베이스 정지 + 5관절 웨이브

`show`/`wave`는 `arm_command_controller`로 전환해 `arm_joint_command`를 5Hz로 낸다.
끝나면 `joint_trajectory_controller`로 되돌린다. `정지`를 누르면 `dance_cancel_`이 서서
베이스와 팔을 즉시 멈춘다.

---

## 4. 비프음 (20)

| # | 자연어 명령 | plan | 실제 멜로디 |
|---:|---|---|---|
| 1 | 마리오 죽을 때 효과음 | `buzzer mario_death` | 마리오 죽음 16음 |
| 2 | 마리오 죽는 소리 | `buzzer mario_death` | 위와 같음 |
| 3 | 게임오버 소리 | `buzzer mario_death` | 위와 같음 |
| 4 | mario death | `buzzer mario_death` | 위와 같음 |
| 5 | 죽을 때 효과음 | `buzzer mario_death` | 위와 같음 |
| 6 | 성공음 | `buzzer success` | 도미솔도 상승 4음 |
| 7 | 성공 소리 | `buzzer success` | 위와 같음 |
| 8 | 딩동댕 | `buzzer success` | 위와 같음 |
| 9 | 축하음 | `buzzer success` | 위와 같음 |
| 10 | success sound | `buzzer success` | 위와 같음 |
| 11 | 정답 소리 | `buzzer success` | 위와 같음 |
| 12 | 경고음 | `buzzer alarm` | 880Hz 경고 3회 |
| 13 | 경고 알람 | `buzzer alarm` | 위와 같음 |
| 14 | 위험 알림 | `buzzer alarm` | 위와 같음 |
| 15 | alarm | `buzzer alarm` | 위와 같음 |
| 16 | 삐삐 | `buzzer alarm` | 위와 같음 |
| 17 | 삐 소리 | `buzzer beep` | 2000Hz 150ms |
| 18 | 비프음 | `buzzer beep` | 위와 같음 |
| 19 | beep | `buzzer beep` | 위와 같음 |
| 20 | 짧게 삐 | `buzzer beep` | 위와 같음 |

### 버저 경로

```text
llm_input::play_melody(name)
  → service buzzer/melody {name}
    → RobotStatus: 20ms 타이머로 음을 순서대로
      → OpenCR serial "B,<Hz>,<ms>"
        → 실제: tone(BDPIN_BUZZER, Hz, ms)
        → 시뮬: Qt QAudioSink 사각파
```

- 단일 톤 서비스 `buzzer {frequency, duration}`도 있다.
- `buzzer/melody`가 진행 중이면 단일 `buzzer`가 기존 멜로디를 취소한다.
- 자세한 음표/볼륨 문제는 [buzzer.md](buzzer.md).

---

## 5. 규칙 매칭 순서 (중요)

`command_rules.cpp`는 아래 순서로 검사한다. 순서가 바뀌면
"내 몸에 올려놔"가 pick으로, "팔 숨겨"가 hide로 새는 문제가 생긴다.

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

### 경계 케이스

| 문장 | 왜 이렇게 해석되나 |
|---|---|
| `축하해` | dance celebrate (`축하음`은 buzzer success) |
| `축하음` | buzzer success (`음`이 있으므로) |
| `삐삐` | buzzer alarm (`삐 소리`는 beep) |
| `팔 숨겨` | arm_preset stow (`숨어`는 hide) |
| `몸을 흔들어` | dance wiggle (place body는 `놓/올려`가 있어야) |
| `빙글빙글 돌아` | dance spin (drive turn보다 먼저) |
| `안 보이는 곳으로 숨어` | hide (`보이`만으로 approach가 되지 않게) |
| `앞에 보이는 벽까지` | approach (`앞`이 있어도 벽이 우선) |

---

## 6. 규칙에 없는 문장은?

규칙이 없으면 `source="llm"`으로 Ollama에 간다.

```text
"책상 위에 있는 빨간 큐브를 왼쪽으로 옮겨줘"
  → 규칙 없음
  → Ollama → JSON plan
  → llm/status {source:"llm", plan:{...}}
```

Ollama가 이상한 plan을 주면 `apply_direction_hint()`가 좌/우/뒤/회전을
결정적으로 drive로 보정한다. 그래도 애매하면 `unsupported`로 끝난다.

---

## 7. 테스트

90개 전부 유닛 테스트로 검증한다.

```bash
./srf-default-ws/build.sh tests
SRF_TEST_FILTER=commands_ ./srf-default-ws/build-mac/srf_default_ws_tests
# [ OK ] commands_manipulator_20
# [ OK ] commands_driving_20
# [ OK ] commands_entertainment_30
# [ OK ] commands_beep_20
```

실제 시뮬레이터에서는 웹 AI 패널에서 같은 문장을 넣고
`llm/status`의 `plan.source`가 `rule`인지 확인한다.
