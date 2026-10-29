# 상태 바꾸기 — 제어 상태 · 경로(직결/RL) · 비상정지

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
| 경로 | `/cmd_manual_rl` (Bool) | `/state_rl` | `false` 직결 · `true` PC RL 경유 |
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

## 2. 경로 — 직결 / PC RL 경유

**배선을 다시 잇지 않는다.** 토픽 연결은 그대로이고, cmd_manager가 다음 틱(20 Hz)부터 고른 명령을 **어느 토픽으로 내보낼지만** 바뀐다.

직결 (`/state_rl = false`)
```
텔레오퍼 /cmd_manual_vel ─┐
                          ├─▶ cmd_manager ─ /cmd_vel_base ─▶ opencr_driver ─▶ 바퀴
Nav2     /cmd_vel ────────┘   (제어 상태로 하나 고름)
```

RL 경유 (`/state_rl = true`)
```
텔레오퍼 /cmd_manual_vel ─┐                 (Pi)                 (PC)
                          ├─▶ cmd_manager ─ /cmd_vel_rl_in ─────▶ rl_corrector
Nav2     /cmd_vel ────────┘        ▲                                   │ 보정
                                   └──────── /cmd_vel_rl_out ◀─────────┘
                          cmd_manager ─ /cmd_vel_base ─▶ opencr_driver ─▶ 바퀴
```

1. cmd_manager가 고른 명령을 `/cmd_vel_base` 대신 **`/cmd_vel_rl_in`**으로 보낸다.
2. PC의 rl_corrector가 보정해 **`/cmd_vel_rl_out`**으로 돌려준다.
3. cmd_manager가 그 값을 `/cmd_vel_base`로 → 바퀴.
   같이 뜨는 eval_recorder가 보정 전(`rl_in`)·후(`rl_out`)를 CSV로 남긴다(비교용).

그대로인 것: 누구 명령을 고를지는 여전히 **제어 상태**(MANUAL·NAV_PLAN·SLAM_AUTO_MAPPING), 비상정지는 경로와 무관하게 0.

**순서 — PC에서 RL 노드를 먼저**, 그다음 경로를 바꾼다:
```bash
# PC
ros2 launch tb3_bringup rl-only.launch.py                                          # 보정 없이 통과 (연결 확인)
ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=/경로/policy.onnx      # 실제 보정
ros2 launch tb3_bringup rl-only.launch.py label:=after output:=/tmp/after.csv       # 기록 파일 지정

# 어디서든 (같은 ROS_DOMAIN_ID 25)
ros2 topic pub --once /cmd_manual_rl std_msgs/msg/Bool "{data: true}"    # RL 경유
ros2 topic pub --once /cmd_manual_rl std_msgs/msg/Bool "{data: false}"   # 직결
ros2 topic echo --once /state_rl
```

- `/cmd_vel_rl_out`이 **0.3초 넘게 안 오면 0** → 로봇이 멈춘다(PC 꺼짐·WiFi 끊김·rl-only 안 띄움). 의도된 안전 동작.
- tool(`route rl` · 메뉴 `1) 상태 → 4)`)은 `rl_corrector` 노드가 안 보이면 **거부하고 띄우는 법을 알려준다**. RL 경유 중 PC 응답이 끊기면 상태바에 `PC RL 응답 없음 → 정지 · PC에서 rl-only.launch.py 켜기`.
- WiFi 한 번 왕복만큼 반응이 늦어진다.
- 처음엔 `enable:=false`(통과)로 띄워 텔레오퍼로 움직이는지부터 확인.
- 처음부터 RL로 켜기: `tb-only.launch.py rl:=true`.

## 3. 비상정지

```bash
ros2 topic pub --once /cmd_manual_estop std_msgs/msg/Bool "{data: true}"   # 정지 (즉시 0)
ros2 topic pub --once /cmd_manual_estop std_msgs/msg/Bool "{data: false}"  # 해제
```

켜져 있는 동안은 어떤 모드·경로든 바퀴 명령이 0이다.

## 4. tool에서

| 바꾸기 | 명령 | 메뉴 |
|---|---|---|
| 제어 상태 | `./run-cli mode manual` · `mode nav-plan` · `automap start` / `automap stop` | `1) 상태 → 3)` · SLAM 메뉴 `a)` |
| 경로 | `./run-cli route direct` · `route rl` | `1) 상태 → 4)` |
| 비상정지 | `./run-cli estop on` · `estop off` | `1) 상태 → 5)` |

현재 값은 CLI 위쪽 상태바에 나온다: `제어 MANUAL · 운행 SLAM (지도 …) · 경로 직결 · 비상정지 off`.

## 5. 안전 (요약)

| 상황 | 결과 |
|---|---|
| 비상정지 on | 항상 0 |
| 고른 입력이 0.5초 넘게 끊김 | 0 |
| RL 경유인데 `/cmd_vel_rl_out`이 0.3초 넘게 끊김 (PC 꺼짐·WiFi) | 0 |
| cmd_manager 자체가 죽음 | 드라이버(0.5초)·펌웨어(500 ms)가 각자 멈춤 |
