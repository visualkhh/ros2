# tool

로봇 운영 도구. **지도 만들기(SLAM, slam_toolbox)** · **지도 주행(Nav2)** 켜고 끄기, 텔레오퍼, 지도 저장, 목표 보내기,
제어 상태(MANUAL · NAV_PLAN · SLAM_AUTO_MAPPING 자동 맵핑) · 경로(직결/RL) · 비상정지 전환. **ROS 토픽이 보이는 곳 어디서나** 같은 `./run-cli`로 씁니다 (로봇 Pi든 PC든).

```text
tool/
├── README.md · MAP_NAV.md  # MAP_NAV = 지도 만들기 → 저장 → 주행 → 목표 순서
├── run-cli          # 메뉴 / 한 번 명령 (어디서나)
├── run-rviz         # RViz: 맵핑·주행 보기 (화면 있는 곳)
├── peers.sh         # 로봇이 아닌 곳에서 Pi DDS 노드를 유니캐스트로 찾게 (WiFi 멀티캐스트 불안정)
├── config/
│   ├── slam_toolbox.yaml   # slam_toolbox online_async + base_footprint · 지도 갱신 2초 · 최소 사거리 0.12 m
│   └── nav2_params.yaml    # nav2_bringup + Waffle Pi footprint · Twist cmd_vel · amcl base_footprint
├── rviz/
│   ├── start.sh · stop.sh · restart.sh
│   ├── slam.rviz           # 지도·스캔·odom·TF·포즈 그래프 (Orbit 3D, 저장된 TopDown 2D)
│   └── nav.rviz            # nav2 화면 (코스트맵·경로 · 2D Pose Estimate · Nav2 Goal)
├── cli/
│   ├── start.sh · stop.sh · restart.sh
│   ├── robot_tool.py       # 배선 + argparse + main
│   └── op/                 # terminal · dialogs · ui · state · procs(로컬/원격) · bus · pages/
└── .run/                   # (git 무시) 띄운 launch의 pid · 로그 · DDS 피어 프로필
```

## 어디서나 — 어떻게 동작하나

| 기능 | 방식 | 로봇이 아닌 곳(PC 등)에서 |
|---|---|---|
| 상태바 · 텔레오퍼 · drive · goal · initpose · 모드/RL/비상정지 · 모터 재부팅 | ROS 토픽·액션 | 그대로 됨 |
| 베이스·SLAM·Nav 켜기/끄기/재시작 · 로그 | 로봇의 프로세스 | tool이 **SSH로 로봇의 run-cli에 넘김** (`[pak.local] …`로 표시) |
| 지도 저장 · 목록 · 내용 | 로봇의 파일 (Nav가 거기서 읽음) | 위와 같이 SSH — 지도는 항상 **로봇** `~/ros2/ros2_ws/map/`에 |
| 켜짐 표시 (● / ○) | 로봇: pid 파일 · 그 밖: **ROS 노드 그래프** (`opencr_driver`·`slam_toolbox`·`amcl`·`bt_navigator`) | `pak.local (원격)`으로 표시 |

필요한 것 (로봇이 아닌 곳): 같은 WiFi · `ssh pak@pak.local`이 비번 없이 됨
([AI·SSH 키 문서](../documents/ai/ai-passwordless-robot-access.md)). `ROS_DOMAIN_ID=25`·DDS 피어는 스크립트가 맞춥니다.
"맵핑 모드로 전송" 같은 건 없고, **SLAM이 켜져 있으면 맵핑 중**입니다. RViz는 `/map`·`/scan`·TF를 받아 그릴 뿐.

`ROS_DOMAIN_ID`는 **25로 통일** — `cli/start.sh`·`rviz/start.sh`가 `export ROS_DOMAIN_ID=25` (`~/.bashrc` 무관).

> 지도 만들기 → 저장 → 지도로 주행 → 특정 위치로 보내기 전체 순서: **[MAP_NAV.md](MAP_NAV.md)**

## 지도 만들기 (SLAM)

```bash
./run-cli base start            # 바퀴·odom·라이다 (로봇에 이미 떠 있으면 생략)
./run-cli slam start            # 맵핑 시작 (로봇에서 돔)
./run-rviz slam                 # 지도 커지는 거 보기 (다른 터미널)
./run-cli teleop                # 천천히 돌기 (0.10 m/s · 벽을 따라 한 바퀴 → 출발점 → 가운데)
./run-cli map save office       # → 로봇 ~/ros2/ros2_ws/map/office/map.yaml + map.pgm
./run-cli slam stop
```

메뉴: `./run-cli` → `2) 베이스` · `3) SLAM 맵핑` (`v` RViz로 보기 · `3` 지도 저장) · `5) 수동 주행`. 메뉴별 키는 [MAP_NAV.md](MAP_NAV.md#두-가지-방법-명령-한-줄--메뉴).

## 지도로 주행 (Nav)

```bash
./run-cli nav start office      # 이름 없으면 가장 최근 지도
./run-cli initpose 0 0 0        # 지도 위 로봇 위치 (x y yaw°) — 또는 RViz 2D Pose Estimate
./run-cli goal 1.0 0.5 90       # 도착까지 기다림 · --no-wait · goal --cancel
./run-rviz nav                  # Nav2 Goal로 찍어도 됨
```

`SLAM 중 Nav`(`./run-cli explore start`)는 SLAM을 켠 채 Nav2 주행만 띄웁니다. 맵핑 중에 목표를 찍어
움직이며 지도를 넓힐 때 (초기 위치 불필요).

지도 저장은 `/map`이 들어올 때만 하고, 없으면 "라이다 /scan이 있어야 지도가 생김"으로 알려 줍니다.

동시에 못 켜는 조합: `slam` ↔ `nav`, `nav` ↔ `explore` (지도·위치를 내는 쪽이 둘이 됨). 켜려 하면 거부합니다.

## 명령

```bash
./run-cli                       # 대화형 메뉴 (TTY 아니면 status)
./run-cli status                # 프로세스 + odom·scan·map·amcl 수신 상태
./run-cli {base|slam|nav|explore} {start|stop|restart|status|log}
./run-cli nav start [지도이름|폴더|yaml]
./run-cli map list
./run-cli map save [이름]       # 기본 이름: 날짜_시각
./run-cli teleop [--linear 0.1 --angular 0.5]
./run-cli drive forward|back|left|right|stop [--speed --angular --duration]
./run-cli initpose x y [yaw°]
./run-cli goal x y [yaw°] [--no-wait] | goal --cancel
./run-cli nodes
./run-cli reboot-motors        # 모터 에러(E) 해제 — 두 바퀴 재부팅 (/opencr/reboot)
./run-cli control manual|nav-plan|slam-auto-mapping   # 제어 상태 (텔레오퍼·drive는 MANUAL 아니면 물어봄 · goal·RViz Goal은 NAV로)
./run-cli automap start|stop    # 자동 맵핑: SLAM·SLAM 중 Nav 켜고 SLAM_AUTO_MAPPING → 다 찾으면 MANUAL
./run-cli route direct|rl       # 바퀴 명령 경로: 직결 / PC RL 경유 (= rl off|on)
./run-cli estop on|off          # 비상정지 (on은 바로, 메뉴에서 해제는 확인)
./run-cli stop-all              # 바퀴 정지 → 띄운 것 전부 종료
./run-cli stop                  # CLI 프로세스만 종료 (SLAM·Nav는 계속 돔)
```

상단 상태바: ① 프로세스 (로봇: pid · 그 밖: 노드 그래프로 `pak.local (원격)`) ② `odom`·`scan`·`map` 수신 Hz ③ **로봇** — opencr_driver의 `/diagnostics`
(● ok / ✗ motor error·no S line / ? 수신 없음 · 바퀴 온도 좌/우 · 전류 좌/우). 모터 에러면 `1) 상태 → 2) 모터 재부팅`.
그 아래 줄 = state_manager의 `/state_*`: `제어 MANUAL · 운행 LOCALIZATION (지도 office) · 경로 직결 · 비상정지 off`
(제어 = 누가 바퀴를 움직이나 · 운행 = SLAM/LOCALIZATION/IDLE · 비상정지면 빨강). 전환은 `1) 상태 → 3·4·5`.

CLI를 꺼도 띄운 launch는 계속 돕니다 (로봇의 `.run/<이름>.pid`). 다시 켜면 그대로 이어서 보입니다.
로그: `./run-cli slam log` (어디서나 — 원격이면 SSH로 로봇의 `.run/slam.log`).

메뉴 키 규약 (참고: `cli/op/dialogs.py` 맨 위):

| 화면 | 키 |
|---|---|
| 메뉴 | `↑↓` 이동 · `Enter` 실행 · 번호로 포커스 · `Esc`/`◀`/`b` 뒤로 · `m` 메인 · `r` 새로고침 · `q` 종료 |
| 확인창 | `y` 예 · `n`/`Esc` 아니오 · `Enter` 기본값 |
| 입력창 | `Enter` 확인 · `Esc` 취소 |
| 텔레오퍼 | `w`/`x`/`a`/`d`·방향키 · `s`/`space` 정지 · `e`/`c` 속도 · `q` 종료 (종료 시 0 속도) |

## 설정 (환경변수)

| 변수 | 기본 | 뜻 |
|---|---|---|
| `TOOL_BASE_CMD` | `ros2 launch tb3_bringup tb-only.launch.py` | 베이스 bringup. RL on으로 시작은 `… tb-only.launch.py rl:=true` (보통은 실행 중 `./run-cli rl on`) |
| `TOOL_CMD_VEL` | `/cmd_manual_vel` | 텔레오퍼·drive가 내는 토픽 (cmd_manager 수동 입력) |
| `TOOL_CMD_VEL_STAMPED` | `0` | `1`이면 TwistStamped로 발행 |
| `TOOL_MAP_DIR` | `ros2_ws/map` | 지도 폴더 (`<이름>/map.yaml`) — 로봇·`TOOL_LOCAL`에서 |
| `TOOL_SLAM_PARAMS` / `TOOL_NAV_PARAMS` | `config/*.yaml` | 파라미터 파일 |
| `TOOL_USE_SIM_TIME` | `false` | Gazebo면 `true` |
| `ROBOT_HOST` | `pak.local` | 로봇 이름 (DDS 피어 · 원격 실행 대상) |
| `TOOL_ROBOT_SSH` | `pak@pak.local` | 원격 실행 SSH 대상 (키 인증) |
| `TOOL_ROBOT_DIR` | `~/ros2/tool` | 로봇의 tool 위치 |
| `TOOL_LOCAL` | — | `1`이면 로봇이 아니어도 **이 컴퓨터에** 띄움 (PC 시뮬레이션) |

프레임 규약 (REP-105, TurtleBot3와 같음): `map → odom → base_footprint(바닥) → base_link(바퀴축 +0.010) → base_scan`.
`odom → base_footprint`는 opencr_driver, 나머지는 URDF(robot_state_publisher). 스캔은 `/scan`.
SLAM·amcl은 `base_footprint`, costmap·footprint 다각형은 `base_link` 기준 (turtlebot3_navigation2와 같음).

`config/nav2_params.yaml`은 `enable_stamped_cmd_vel: false`입니다. Lyrical Nav2 기본은 `TwistStamped`인데
cmd_manager·opencr_driver는 `Twist`를 받기 때문입니다 (안 맞으면 Nav가 경로는 내도 바퀴가 안 움직임).

## 문제 확인

```bash
./run-cli status                # odom/scan/map 중 뭐가 안 오는지
./run-cli slam log              # slam_toolbox 에러
ros2 run tf2_tools view_frames  # map→odom→base_link→라이다 끊긴 곳
```

- RViz·상태바에 아무것도 안 보임 → 같은 WiFi · 둘 다 `ROS_DOMAIN_ID=25` · `ROS_LOCALHOST_ONLY`/`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 아님
- 로봇 노드가 보였다 안 보였다 함 (`odom ✗`, `ros2 node list` 빔, ping은 됨) → WiFi가 멀티캐스트를 버림.
  `run-cli`·`run-rviz`는 로봇이 아닌 곳에서 자동으로 Fast DDS 프로필(`.run/fastdds_peers.xml`)을 만들어 Pi IP의 참가자 슬롯 0~31을
  유니캐스트로 찾습니다 (`tool/peers.sh`). `ROS_STATIC_PEERS`는 슬롯 0~3만 찾아서 Pi 노드가 많아지면 일부가 안 보임.
  터미널에서 직접 `ros2` 명령을 칠 때: `export ROS_DOMAIN_ID=25 && source ~/workspaces/visualkhh/source/ros2/tool/peers.sh`
  (그다음 `ros2 daemon stop` 한 번).
- 지도가 안 커짐 → `scan` 수신 확인, 라이다 TF 확인, 너무 빨리 돌면 스캔 매칭 실패 (천천히)
- Nav 목표 거부 → `initpose` 먼저, `./run-cli nav log`
- `[pak.local] … SSH 실패` → `ssh pak@pak.local`이 비번 없이 되는지 (켜기/끄기·지도만 SSH, 나머지는 토픽이라 됨)
- 바퀴가 안 움직임 → 상태바 `운행`: 텔레오퍼는 MANUAL, Nav·자동 맵핑은 NAV_PLAN·SLAM_AUTO_MAPPING이어야 · `비상정지 off` · RL on이면 PC `rl-only` 떠 있는지
