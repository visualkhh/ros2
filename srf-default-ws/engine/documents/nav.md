# Navigation behaviors: drive/turn, wall approach, hide

> 주행은 크게 셋이다.
> **drive/turn**(odom 거리/각도), **approach**(라이다로 벽 앞), **hide**(costmap+nav2로 벽 뒤).
> "앞으로 1m"는 nav2가 아니라 odom 제어다.

## 1. 소스

| 파일 | 역할 |
|---|---|
| `srf-default-ws/engine/nodes/llm_input/llm_input.cpp` | drive/turn intent, `cmd_vel_nav` |
| `srf-default-ws/engine/nodes/nav_behavior/nav_behavior_node.cpp` | `nav/hide`, `nav/approach`, `nav/status` |
| `srf-default-ws/engine/nodes/llm_input/command_rules.cpp` | 20개 주행 문장 파싱 |
| `srf-default-ws/engine/engine.cpp` | twist_mux → velocity_smoother → collision_monitor |
| `srf/engine/core/nodes/nav2/*` | planner/controller/BT navigator |
| `srf/engine/core/nodes/nav2/costmap/*` | global/local costmap |

## 2. drive / turn

```text
"앞으로 1m"
  → command_rules: {"intent":"drive","direction":"forward","meters":1.0}
  → start_drive
    odom 시작점 저장
    cmd_vel_nav linear 0.15 m/s
    hypot(odom-start) >= 1.0 → zero
  → llm/status done {distance:1.01}
```

- `start_drive(id, meters, sign)`: sign=+1 전진, -1 후진.
- `start_turn(id, "left"/"right", degrees)`: `angular=±0.8 rad/s`, odom yaw로 판정.
- 2초 동안 1cm/0.02rad도 진행이 없으면 `drive stalled (collision?)`.
- 30초 타임아웃.
- 각도가 없으면 90°, 거리가 없으면 0.5m.

토픽 체인:

```text
cmd_vel_nav → twist_mux(priority 10) → velocity_smoother → collision_monitor → cmd_vel
```

`cmd_vel_nav`는 navigation priority 10, teleop은 100이다.
충돌 모니터가 costmap/scan으로 먼저 멈출 수 있다.

### 예시 20개

| 명령 | plan |
|---|---|
| `앞으로 1m` | forward 1.0 |
| `전진 1미터` | forward 1.0 |
| `앞으로 가` | forward 0.5 |
| `직진` | forward 0.5 |
| `1m 앞으로` | forward 1.0 |
| `go forward 1 meter` | forward 1.0 |
| `move forward 50cm` | forward 0.5 |
| `앞으로 0.5m 이동` | forward 0.5 |
| `전진해` | forward 0.5 |
| `뒤로 0.5m` | backward 0.5 |
| `후진` | backward 0.5 |
| `뒤로 가` | backward 0.5 |
| `back up 30cm` | backward 0.3 |
| `왼쪽으로 90도` | left 90° |
| `좌회전` | left 90° |
| `오른쪽으로 45도` | right 45° |
| `우회전` | right 90° |
| `turn left 90` | left 90° |
| `turn right` | right 90° |
| `멈춰` | stop |

전체는 [commands.md](commands.md#2-주행-20).

## 3. wall approach (라이다)

`"앞에 보이는 벽까지 가까이 붙어"` → `nav/approach`.

```cpp
// nav_behavior_node.cpp
constexpr double kStopDistance = 0.28;  // 충돌 모니터보다 여유
constexpr double kCloseEnough = 0.35;   // 정체해도 이만큼 붙었으면 성공

// scan 전방 ±0.35rad 최소거리
// front <= 0.28 → 정지
// 3초 정체 + front <= 0.35 → 성공 (충돌 모니터가 먼저 멈춘 경우)
// 3초 정체 + front > 0.35 → error
// 20초 타임아웃
```

- `scan` 토픽의 전방 beam만 본다. 지도 없이도 된다.
- 차체/충돌 모니터 때문에 실제 정지 거리는 0.2~0.3m다.
- 예전에는 `kStopDistance=0.22`라 충돌 모니터가 0.23m에서 멈추면
  front가 줄지 않아 `approach stalled`로 실패했다. 0.28/0.35로 고쳤다.

### 예시

```text
"앞에 보이는 벽까지 가까이 붙어"
  → nav/approach accepted
  → nav/status approaching {front:0.42}
  → nav/status approaching {front:0.31}
  → nav/status done
```

## 4. hide (벽 뒤로)

`"안 보이는 곳으로 숨어"` → `nav/hide`.

```cpp
// 현재 map/base_link TF
// global_costmap free cell 후보 (cost < 50, dist 1.0~4.5m)
// line_of_sight_blocked(): 후보까지 ray가 cost >= 65(벽/인플레이션)를 지나면 가려짐
// 가장 먼 가려진 free cell을 nav2 NavigateToPose goal로
```

- **map + localization 필수** (`slam: idle` + 지도 없음 → 거부).
- nav2 planner/controller가 costmap+scan으로 충돌 회피.
- 목표는 벽 안이 아니라 free cell이다. 벽에 딱 붙는 게 아니라 가려지는 위치.
- `nav/status`: `navigating` + `remaining` → `done/error`.
- nav2 결과 대기는 90초. 그 안에 못 가면 error.
- 지도가 실제 월드와 어긋나면 replan만 반복할 수 있다. 이때는 `정지`로 취소한다.

### 예시

```text
"안 보이는 곳으로 숨어"
  → nav/hide accepted
  → nav/status navigating {goal:[...], remaining:3.2}
  → nav/status navigating {remaining:1.1}
  → nav/status done
```

## 5. map / localization

- 지도 로드: 웹 `SLAM / Map` 탭 → 지도 선택 → `load`.
- localization 모드: `slam: mode=localization localized=yes`.
- SLAM 새 mapping: `--slam` 또는 웹 SLAM start.
- nav2의 global costmap은 `global_costmap/costmap` (OccupancyGrid, latched).

지도가 없으면 hide/nav2 pick 접근은 실패한다.
벽 approach와 가까운 pick은 지도 없이도 된다.

## 6. 디버깅

| 증상 | 볼 것 |
|---|---|
| drive가 안 감 | `llm/status` `drive stalled`, collision_monitor, `cmd_vel` |
| turn이 안 돎 | odom yaw, `turn stalled`, collision_monitor |
| approach가 즉시 실패 | `전방 0.35rad 안에 벽/물체가 없다` → 로봇을 벽 쪽으로 |
| approach가 stall | `front` 값, 0.28/0.35 임계값 |
| hide가 거부 | `costmap이 없다`, `slam: idle`, 지도 로드 |
| hide가 계속 navigating | `remaining`, nav2 replanning, 지도 정합 |
| hide 취소 | AI 패널 `정지` → `nav/stop` |

## 7. 한계

- hide는 지도 품질에 민감하다. 지도가 오래됐거나 로컬라이제이션이 틀면
  "가려진 free cell"이 실제로는 벽 뒤가 아닐 수 있다.
- approach는 라이다만 본다. 유리/낮은 턱/구멍은 못 본다.
- drive/turn은 odom 기준이라 바퀴 미끄러짐이 있으면 거리/각도가 어긋난다.
