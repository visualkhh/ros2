# tool

로봇 운영 도구. **지도 만들기(SLAM, slam_toolbox)** 와 **지도 주행(Nav2)** 을 켜고 끄고,
텔레오퍼로 몰고, 지도를 저장합니다. 상주 노드(state_manager 등) 없이 tool이 `ros2 launch`를 직접 띄웁니다.

```text
tool/
├── README.md
├── run-cli          # 로봇(Pi)에서: 메뉴 / 한 번 명령
├── run-rviz         # PC에서: 맵핑·주행 보기
├── config/
│   ├── slam_toolbox.yaml   # slam_toolbox online_async 기본값 + base_frame=base_link · 지도 갱신 2초
│   └── nav2_params.yaml    # nav2_bringup 기본값 + base_link · Twist cmd_vel
├── rviz/
│   ├── start.sh · stop.sh · restart.sh
│   ├── slam.rviz           # 지도(/map)·스캔·odom·TF·포즈 그래프 — 위에서 내려다봄
│   └── nav.rviz            # nav2 기본 화면 (코스트맵·경로 · 2D Pose Estimate · Goal)
├── cli/
│   ├── start.sh · stop.sh · restart.sh
│   ├── robot_tool.py       # 배선 + argparse + main
│   └── op/                 # terminal · dialogs · ui · state · procs · bus · pages/
└── .run/                   # (git 무시) 띄운 launch의 pid · 로그
```

## 어디서 무엇을

| 어디 | 무엇 | 왜 |
|---|---|---|
| 로봇 Pi (`ssh pak`) | `./run-cli` — 베이스·SLAM·Nav 켜기/끄기, 텔레오퍼, 지도 저장 | SLAM·Nav는 라이다·odom 옆에서 돌아야 함 |
| PC | `./run-rviz slam` / `./run-rviz nav` | 지도가 커지는 모습·주행 경로를 화면으로 봄 |

"맵핑 모드로 전송" 같은 명령은 따로 없습니다. **Pi에서 SLAM을 켜면 그게 맵핑 중**이고,
PC RViz는 같은 `ROS_DOMAIN_ID`(=25)로 `/map`·`/scan`·TF를 받아 그릴 뿐입니다.

`ROS_DOMAIN_ID`는 **25로 통일**합니다. `cli/start.sh`·`rviz/start.sh`가 `export ROS_DOMAIN_ID=25`를 하므로 `~/.bashrc` 값과 무관합니다.

## 지도 만들기 (SLAM)

```bash
# Pi
ssh pak
cd ~/ros2/tool                  # (저장소 위치)
./run-cli base start            # 바퀴·odom·라이다 (TOOL_BASE_CMD)
./run-cli slam start            # 맵핑 시작

# PC — 지도 커지는 거 보기
./run-rviz slam

# Pi — 천천히 돌기 (0.10 m/s · 0.5 rad/s 정도, 벽을 따라 한 바퀴 → 가운데)
./run-cli teleop
./run-cli map save office       # → ros2_ws/map/office/map.yaml + map.pgm
./run-cli slam stop
```

메뉴로 하면: `./run-cli` → `2) 베이스` → `3) SLAM 맵핑` → `5) 수동 주행` → `3) SLAM → 3) 지도 저장`.

## 지도로 주행 (Nav)

```bash
./run-cli nav start office      # 이름 없으면 가장 최근 지도
./run-cli initpose 0 0 0        # 지도 위 로봇 위치 (x y yaw°) — 또는 RViz 2D Pose Estimate
./run-cli goal 1.0 0.5 90       # 도착까지 기다림 · --no-wait · goal --cancel
# PC: ./run-rviz nav            # 2D Goal Pose로 찍어도 됨
```

`SLAM 중 Nav`(`./run-cli explore start`)는 SLAM을 켠 채 Nav2 주행만 띄웁니다. 맵핑 중에 목표를 찍어
움직이며 지도를 넓힐 때 씁니다 (초기 위치 불필요).

**베이스·SLAM·Nav 시작/정지는 로봇(Pi)에서만** 됩니다. PC에서 누르면 거부하고 `ssh -t pak '...'` 명령을 알려 줍니다
(PC에 뜨면 `/dev/opencr`를 못 찾고 지도도 PC에 저장돼 헷갈림). PC 시뮬레이션만 `TOOL_LOCAL=1`.
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
./run-cli stop-all              # 바퀴 정지 → 띄운 것 전부 종료
./run-cli stop                  # CLI 프로세스만 종료 (SLAM·Nav는 계속 돔)
```

상단 상태바 3줄: ① 프로세스(이 컴퓨터 기준) ② `odom`·`scan`·`map` 수신 Hz ③ **로봇** — opencr_driver의 `/diagnostics`
(● ok / ✗ motor error·no S line / ? 수신 없음 · 바퀴 온도 좌/우 · 전류 좌/우). 모터 에러면 `1) 상태 → 2) 모터 재부팅`.

CLI를 꺼도 띄운 launch는 계속 돕니다 (pid는 `.run/<이름>.pid`). 다시 켜면 그대로 이어서 보입니다.
로그: `./run-cli slam log` 또는 `.run/slam.log`.

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
| `TOOL_BASE_CMD` | `ros2 launch tb3_bringup tb-only.launch.py` | 베이스 bringup. RL 연동은 `tb-for-rl.launch.py`, PC 시험(OpenCR 없음)은 `basic_bringup.launch.py` |
| `TOOL_CMD_VEL` | `/cmd_vel` | 텔레오퍼·drive가 내는 토픽 |
| `TOOL_CMD_VEL_STAMPED` | `0` | `1`이면 TwistStamped로 발행 |
| `TOOL_MAP_DIR` | `ros2_ws/map` | 지도 폴더 (`<이름>/map.yaml`) |
| `TOOL_SLAM_PARAMS` / `TOOL_NAV_PARAMS` | `config/*.yaml` | 파라미터 파일 |
| `TOOL_USE_SIM_TIME` | `false` | Gazebo면 `true` |

프레임 규약 (REP-105, TurtleBot3와 같음): `map → odom → base_footprint(바닥) → base_link(바퀴축 +0.010) → base_scan`.
`odom → base_footprint`는 드라이버(opencr_driver·mock_base), 나머지는 URDF(robot_state_publisher). 스캔은 `/scan`.
SLAM·amcl은 `base_footprint`, costmap·footprint 다각형은 `base_link` 기준 (turtlebot3_navigation2와 같음).

`config/nav2_params.yaml`은 `enable_stamped_cmd_vel: false`입니다. Lyrical Nav2 기본은 `TwistStamped`인데
`mock_base`·자체 드라이버는 `Twist`를 받기 때문입니다 (안 맞으면 Nav가 경로는 내도 바퀴가 안 움직임).

## 문제 확인

```bash
./run-cli status                # odom/scan/map 중 뭐가 안 오는지
./run-cli slam log              # slam_toolbox 에러
ros2 run tf2_tools view_frames  # map→odom→base_link→라이다 끊긴 곳
```

- PC RViz에 아무것도 안 보임 → PC와 Pi가 같은 WiFi · 둘 다 `ROS_DOMAIN_ID=25` · `ROS_LOCALHOST_ONLY`/`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 아님
- PC에서 Pi 노드가 보였다 안 보였다 함 (`odom ✗`, `ros2 node list` 빔, ping은 됨) → WiFi가 멀티캐스트를 버림.
  `run-cli`·`run-rviz`는 PC에서 자동으로 Fast DDS 프로필(`.run/fastdds_peers.xml`)을 만들어 Pi IP의 참가자 슬롯 0~31을
  유니캐스트로 찾습니다 (`tool/peers.sh`). `ROS_STATIC_PEERS`는 슬롯 0~3만 찾아서 Pi 노드가 많아지면 일부가 안 보임.
  터미널에서 직접 `ros2` 명령을 칠 때: `export ROS_DOMAIN_ID=25 && source ~/workspaces/visualkhh/source/ros2/tool/peers.sh`
  (그다음 `ros2 daemon stop` 한 번).
- 지도가 안 커짐 → `scan` 수신 확인, 라이다 TF 확인, 너무 빨리 돌면 스캔 매칭 실패 (천천히)
- Nav 목표 거부 → `initpose` 먼저, `./run-cli nav log`

## 시험 (실기 없이 PC에서)

mock_base(odom)에 가짜 라이다를 붙여 전체 흐름(SLAM → 저장 → Nav 목표 → SLAM 중 Nav)을 확인했습니다 (2026-10-27).

```bash
export TOOL_LOCAL=1 TOOL_MAP_DIR=/tmp/maps   # PC에서 시작 허용 · 시험 지도는 따로
TOOL_BASE_CMD='ros2 launch tb3_bringup basic_bringup.launch.py' ./run-cli base start   # mock_base
# + /scan을 내는 무언가 (Gazebo turtlebot3 또는 가짜 스캔 노드)
./run-cli slam start && ./run-cli drive left --duration 4 && ./run-cli map save test
./run-cli slam stop && ./run-cli nav start test && ./run-cli initpose 0 0 0 && ./run-cli goal 1 0.5
./run-cli stop-all
```
