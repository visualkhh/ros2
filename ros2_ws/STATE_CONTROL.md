# 상태 바꾸기 — 제어 상태 · 비상정지

로봇 상태를 바꾸는 모든 방법을 한곳에 정리. 노드 구조는 [ARCH.md §2](ARCH.md#2-운행-상태--바퀴-명령-경로--cmd_manager--state_manager).

## 규칙 하나

```
요청  /cmd_manual_<이름>  ──▶ cmd_manager (검사·판단) ──/state_set_<이름>──▶ state_manager ──▶ /state_<이름> (latched, 현재 값)
```

- **바꾸고 싶으면 `/cmd_manual_*`에 보낸다.** `/state_set_*`에 직접 보내지 않는다(cmd_manager의 검사를 건너뜀).
- **지금 값은 `/state_*`로 본다.** latched라 나중에 붙어도 현재 값이 바로 온다.
- 두 노드 모두 베이스(`tb-only.launch.py`)에 있다. 베이스가 꺼져 있으면 요청이 무시된다.

| 이름 | 요청 토픽 (타입) | 현재 값 토픽 | 값 |
|---|---|---|---|
| 제어 상태 | `/cmd_manual_control` (String) | `/state_control` | `MANUAL` · `NAV_PLAN` · `SLAM_AUTO_MAPPING` |
| 비상정지 | `/cmd_manual_estop` (Bool) | `/state_estop` | `true` 정지 · `false` 해제 |

읽기 전용 (노드 그래프를 보고 state_manager가 스스로 정함 — 요청 토픽 없음):

| 이름 | 토픽 | 값 |
|---|---|---|
| 운행 상태 | `/state_operation` | `SLAM` (cartographer_node) · `LOCALIZATION` (amcl·map_server) · `IDLE` |
| 적용 지도 | `/state_map` | 지도 이름 · `(SLAM 작성 중)` · `''` |

## 1. 제어 상태 — 바퀴를 누가 움직이나

| 값 | 바퀴로 가는 명령 | 이렇게 바뀜 |
|---|---|---|
| `MANUAL` | `/cmd_manual_vel` (텔레오퍼·drive)만 | 기본값 · 자동 맵핑이 끝나면 explore_bridge가 요청 |
| `NAV_PLAN` | Nav2 `/cmd_vel`만 | RViz **Nav2 Goal**(`/goal_pose`)을 찍으면 자동 · tool goal |
| `SLAM_AUTO_MAPPING` | Nav2 `/cmd_vel`만 + explore_lite가 frontier 목표를 계속 보냄 | `run-cli automap start` |

```bash
ros2 topic pub --once /cmd_manual_control std_msgs/msg/String "{data: MANUAL}"
ros2 topic echo --once /state_control
```

- 별칭도 받음: `NAV`·`AUTO` → `NAV_PLAN`, `MAPPING`·`AUTO_MAPPING` → `SLAM_AUTO_MAPPING`. 모르는 값은 cmd_manager가 경고하고 무시.
- 바꾸는 순간 이전 모드에서 온 수동 명령은 버린다(새 모드로 새지 않게).
- tool 텔레오퍼·drive는 MANUAL이 아니면 바꿀지 **물어본다**(터미널이 아니면 거부).

## 3. 비상정지

```bash
ros2 topic pub --once /cmd_manual_estop std_msgs/msg/Bool "{data: true}"   # 정지 (즉시 0)
ros2 topic pub --once /cmd_manual_estop std_msgs/msg/Bool "{data: false}"  # 해제
```

켜져 있는 동안은 어떤 모드든 바퀴 명령이 0이다.

## 4. tool에서

| 바꾸기 | 명령 | 메뉴 |
|---|---|---|
| 제어 상태 | `./run-cli mode manual` · `mode nav-plan` · `automap start` / `automap stop` | `1) 상태 → 3)` · SLAM 메뉴 `a)` |
| 비상정지 | `./run-cli estop on` · `estop off` | `1) 상태 → 5)` |

현재 값은 CLI 위쪽 상태바에 나온다: `제어 MANUAL · 운행 SLAM (지도 …) · 비상정지 off`.

## 5. 안전 (요약)

| 상황 | 결과 |
|---|---|
| 비상정지 on | 항상 0 |
| 고른 입력이 0.5초 넘게 끊김 | 0 |
| cmd_manager 자체가 죽음 | 드라이버(0.5초)·펌웨어(500 ms)가 각자 멈춤 |
