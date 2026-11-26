# Navigation behaviors: drive/turn, wall approach, hide

## 1. 소스

| 파일 | 역할 |
|---|---|
| `krosc_ws/engine/nodes/llm_input.cpp` | drive/turn intent, `cmd_vel_nav` |
| `krosc_ws/engine/nodes/nav_behavior_node.cpp` | `nav/hide`, `nav/approach`, `nav/status` |
| `krosc_ws/engine/engine.cpp` | twist_mux → velocity_smoother → collision_monitor |
| `krosc/engine/core/nodes/nav2/*` | planner/controller/BT navigator |
| `krosc/engine/core/nodes/nav2/costmap/*` | global/local costmap |

## 2. drive / turn

```cpp
// llm_input
start_drive(id, meters, sign);   // sign=+1 forward, -1 backward
start_turn(id, "left"/"right", degrees);
```

- odom 시작점/시작 yaw 저장.
- `cmd_vel_nav` linear `0.15*sign` 또는 angular `±0.8`.
- 거리/각도 도달 시 zero.
- 2초 동안 진행이 없으면 `drive stalled (collision?)`.
- 30초 타임아웃.

토픽 체인:

```text
cmd_vel_nav → twist_mux → velocity_smoother → collision_monitor → cmd_vel
```

`cmd_vel_nav`는 navigation priority 10, teleop은 100이다.
충돌 모니터가 costmap/scan으로 먼저 멈출 수 있다.

## 3. wall approach (라이다)

`"앞에 보이는 벽까지 가까이 붙어"` → `nav/approach`.

```cpp
// nav_behavior_node.cpp
constexpr double kStopDistance = 0.22;
// scan ±0.35 rad 전방 최소거리
// 0.22 m 이하면 정지
// 3초 정체 / 20초 타임아웃
```

- `scan` 토픽의 전방 beam만 본다. 지도 없이도 된다.
- 차체/충돌 모니터 때문에 실제 정지 거리는 0.2~0.3 m다.
- `approach`는 라이다만 쓰므로 지도/SLAM이 필요 없다.

## 4. hide (벽 뒤로)

`"안 보이는 곳으로 숨어"` → `nav/hide`.

```cpp
// 현재 map/base_link TF
// global_costmap free cell 후보 (cost < 50, dist 1.0~4.5 m)
// line_of_sight_blocked(): 후보까지 ray가 cost >= 65(벽/인플레이션)를 지나면 가려짐
// 가장 먼 가려진 free cell을 nav2 NavigateToPose goal로
```

- **map + localization 필수** (`slam: idle` + 지도 없음 → 거부).
- nav2 planner/controller가 costmap+scan으로 충돌 회피.
- 목표는 벽 안이 아니라 free cell이다. 벽에 딱 붙는 게 아니라 가려지는 위치.
- `nav/status`: `navigating → done/error`.

## 5. map / localization

- 지도 로드: 웹 Map 탭 → 지도 선택 → load.
- localization 모드: `slam: mode=localization localized=yes`.
- SLAM 새 mapping: `--slam` 또는 웹 SLAM start.
- nav2의 global costmap은 `global_costmap/costmap` (OccupancyGrid, latched).

지도가 없으면 hide/nav2 pick 접근은 실패한다.
벽 approach와 가까운 pick은 지도 없이도 된다.

## 6. 예시

```text
"앞으로 1m"            → drive, meters 1.0
"뒤로 0.5m"            → drive backward
"왼쪽으로 90도"         → turn left
"앞에 보이는 벽까지"    → approach (scan 0.22 m)
"안 보이는 곳으로 숨어"  → hide (costmap line-of-sight + nav2)
```

## 7. 디버깅

- `slam: mode=... localized=... cells=...` 로그.
- `nav/status` `navigating`에 `remaining`이 줄지 않으면 planner/controller/TF 확인.
- `hide`에서 `가려진 free cell을 찾지 못했다` → costmap이 비었거나 후보 거리 밖.
- `approach`에서 `전방 0.35rad 안에 벽/물체가 없다` → 로봇을 벽 쪽으로 돌린다.
