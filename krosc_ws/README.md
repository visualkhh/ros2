# krosc_ws — 실기 TurtleBot3 (C++ krosc)

라즈베리파이에서 **OpenCR + LDS-03를 직접 돌리는 C++ 워크스페이스**다.
Pi는 하드웨어와 토픽만 돌리고, PC의 C++ `viz`/웹 Viz가 PeerNetwork(9000)로 붙어서 본다.
PC에서는 같은 로봇 스택을 `krosc/simulator` 월드 위에서 돌리는 `run-simulator.sh`를 쓴다.

```
[라즈베리파이] krosc_real
  OpenCRSerial (115200, C/S/I/E/A) ─┐
  LDS-03     (230400, 10Hz) ────────┤
                                    ├─ SensorManager ─ scan/imu 발행
                                    ├─ ControlManager ─ wheel/arm read/write
                                    ├─ RobotStatus ─ arm_status/diagnostics/팔 서비스
                                    ├─ controller_manager (control)
                                    │    ├─ OpenCrControl (control layer)
                                    │    ├─ joint_state_broadcaster ─ joint_states
                                    │    ├─ diff_drive_controller ─ odom
                                    │    └─ joint_trajectory_controller ─ 팔
                                    ├─ MoveGroup (MoveIt 플러그인) ─ moveit_*
                                    ├─ robot_state_publisher (URDF/TF)
                                    ├─ ekf_filter_node ─ odometry/filtered + odom->base_footprint TF
                                    ├─ map_server / map_saver / map_catalog
                                    ├─ nav2: planner/controller/behavior/bt_navigator
                                    └─ PeerNetwork (listen 9000) ──> [맥] Viz / 웹 Viz
                                             ^ 구독 토픽 / cmd_vel / goal / RPC
```

## 준비 (라즈베리파이)

```bash
# udev 고정 이름 (한 번)
~/ros2/documents/raspberry/udev/install-udev.sh

# ROS 2 스택이 시리얼 포트를 잡고 있으면 먼저 내린다 (krosc와 동시에 못 씀)
~/ros2/tools/run-cli stop-all

ls -l /dev/opencr /dev/tb3_lidar
```

필요: `g++`(C++20), `cmake`, `make`/`ninja`. Qt는 Pi에 필요 없다.

## 스크립트

```bash
cd ~/ros2/krosc_ws

./build.sh          # Pi: krosc_real만 / PC: krosc_real + sim + viz + viz-web
./run.sh            # 실기: 빌드 후 idle/텔레옵 실행 (TCP 9000)
./run-slam.sh       # 실기: 빌드 후 SLAM(mapping) 실행
./run-tests.sh      # krosc + krosc_ws 테스트 빌드/실행

# PC에서 (Qt6 필요)
./run-viz.sh        # 빌드 후 pak.local:9000 C++ Viz (fixed frame map)
KROSC_VIZ_HOST=10.2.150.135 ./run-viz.sh
./run-simulator.sh        # krosc_ws 로봇 스택 시뮬레이터 (Qt, PC 전용)
./run-simulator-vizqt.sh   # 시뮬레이터 + Qt Viz 한 번에
./run-simulator-vizweb.sh # 시뮬레이터 + 웹 Viz 한 번에
./run-viz-web.sh    # 웹 Viz 게이트웨이 (로봇 9000에 접속)
```

**Pi는 `krosc_real`만 빌드한다.** 웹 Viz 게이트웨이(`krosc_ws_viz_web`)는 PC에서 실행해
로봇의 PeerNetwork(9000)에 붙는다. `viz`, `sim`, `viz-web`은 PC 전용이며 ARM에서 호출하면
`build.sh`가 에러를 낸다.

`run-viz.sh`는 프로젝트 전용 `krosc_ws_viz_app`을 빌드/실행한다.
`KROSC_VIZ_HOST`, `KROSC_VIZ_PORT`, `KROSC_VIZ_FIXED_FRAME`, `KROSC_VIZ_TAB`,
`KROSC_VIZ_PLUGINS` 환경 변수를 따른다.
기본은 **Map 탭**(RViz 스타일 occupancy 이미지)이고, `KROSC_VIZ_TAB=3d ./run-viz.sh`,
`KROSC_VIZ_TAB=moveit ./run-viz.sh`, `KROSC_VIZ_TAB=project ./run-viz.sh`로 시작 탭을 바꿀 수 있다.
SLAM/Map, Navigation, MoveIt, RL 도킹이 기본으로 붙는다. 텔레옵/매니퓰레이터는
`--teleop`, `--manipulator`, `--manual`로 추가한다. 추가 인자는 그대로 viz에 넘긴다:
`./run-viz.sh --screenshot /tmp/viz.png --duration 3`, `./run-viz.sh --no-moveit`.

기본 플러그인은 `project_panel,slam,navigation,moveit,rl_nav,opencr`이며
`KROSC_VIZ_PLUGINS`로 바꿀 수 있다. 코어 플러그인은 `teleop`, `slam`, `navigation`,
`manipulator`이고 프로젝트 플러그인은 `project_panel`, `moveit`, `rl_nav`, `opencr`다.

Viz 단축키(한글 두벌식 레이아웃에서도 동작):

- `W/A/S/D` 또는 방향키: 텔레옵, `Q/Z`: 속도, `F`: follow, `C`: overview
- `Backspace`: Multi Goal 모드에서 마지막 웨이포인트 삭제
- MoveIt `Plan`/`Execute`/`Cancel`은 `move_action` 액션으로 요청한다
- 웹/Qt MoveIt의 `HOME`/`STOW`는 `move_action` goal의 `preset`(joint-space)으로 기본 자세를 잡는다
- 중앙 **MoveIt 탭**에서 TCP 마커를 드래그(X/Y), `Shift+드래그`로 Z, `Ctrl+드래그`로 Pitch 조정

### 지도 폴더

기본 지도 폴더는 **`~/ros2/krosc_ws/artifacts/maps`**다.
지도 하나 = 폴더 하나(`<이름>/map.yaml` + `map.pgm`)이고, ROS 2처럼
`map_server`(load_map/unload_map)와 `map_saver`(save_map)가 실제 파일 I/O를 맡는다.
`map_catalog`는 상태 토픽(`map_list`, `map_name`, `map_status`)과
`load_map`/`save_map`/`delete_map`/`unload_map`/`set_slam_mode` 서비스를 담당한다.
`krosc_ws/CMakeLists.txt`의 `KROSC_WS_MAPS_DIR`로 정해지고, 실행할 때 바꿀 수 있다:

```bash
./run.sh --maps /home/pak/ros2/krosc_ws/artifacts/maps
KROSC_MAPS_DIR=/home/pak/ros2/krosc_ws/artifacts/maps ./run-slam.sh
```

## 실행 (라즈베리파이)

```bash
cd ~/ros2
./krosc_ws/run.sh                                      # 빌드 + 실행, PeerNetwork 9000
./krosc_ws/run-slam.sh                                 # 빌드 + SLAM(mapping)
KROSC_SERVER_PORT=0 ./krosc_ws/run.sh                  # 네트워크 없이
OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar ./krosc_ws/run.sh
```

- 시작 로그에 접속 주소가 나온다: `[krosc_ws] peer: 192.168.x.x:9000`.
- **OpenCR IMU는 켠 뒤 2초 동안 가만히** 둬야 자이로 편향을 잡는다.
- Ctrl+C/SIGTERM → 마지막 `C,0,0`을 보내고 시리얼을 닫는다.
- 펌웨어 watchdog: 500ms 동안 `C`가 없으면 바퀴 정지. 앱도 0.5초 동안 `cmd_vel`이 없으면 0을 보낸다.

## 원격 viz (맥)

```bash
cd /Users/hyunhakim/workspace/visualkhh/source/ros2/krosc

cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j

./build/viz/krosc_viz_app --server pak.local:9000 --fixed-frame odom
```

- Pi 로그의 IP를 그대로 써도 된다: `--server 192.168.x.x:9000`.
- 아직 SLAM/지도가 없으면 기준 프레임은 **`odom`**을 쓴다.
- 창 왼쪽 패널 위에 `peer: <ip>:9000`이 뜨면 연결된 것이다.
- 원격 viz의 `W/A/S/D` → `cmd_vel` 토픽 → Pi의 `cmd_vel_teleop` → OpenCR. 텔레옵 속도는 `Q/Z`.
- 여러 Viz가 동시에 붙을 수 있다 (PeerNetwork mesh).
- 네트워크는 신뢰할 수 있는 LAN 전용이다 (인증 없음). 포트 9000을 외부에 열지 마라.

원격 토픽은 고정 목록이 아니라 **Viz/플러그인이 실제로 구독한 토픽**만 PeerNetwork로 흐른다.
`tf`, `tf_static`, `robot_description`, `scan`, `imu`, `odom`, `joint_states`, `arm_status`,
`diagnostics`, `rosout`, `moveit_*`, `rl_nav_*` 등은 필요한 쪽이 구독하면 자동으로 전달된다.

## 매니퓰레이터 (OpenMANIPULATOR-X 4축 + 그리퍼)

- 실기 URDF: `engine/description/urdf/turtlebot3_real.urdf`
- 시뮬레이터 URDF: `simulator/robot/urdf/turtlebot3_real.urdf` (의도된 복제본, 공식 visual STL + collision 근사)
- 장착: 3층 판 맨 앞 중앙, `base_link` 기준 `x=+0.047985, y=-0.000230, z=+0.130175`
- OpenCR: joint1~4 = ID 11~14, gripper = ID 15 (`firmware/opencr/turtlebot_opencr/arm_config.h`)
- `joint_states`는 control `joint_state_broadcaster`가 바퀴+팔 관절을 모두 발행한다
- 팔 명령은 MoveIt → `joint_trajectory_controller/follow_joint_trajectory` 액션으로 내려간다.
  수동 `arm_joint_command`(`sensor_msgs/JointState`)는 `arm_command_controller`(forward controller)가
  활성일 때만 하드웨어로 간다. Viz가 슬라이더를 움직이면
  `controller_manager/switch_controller` 서비스로 `joint_trajectory_controller` ↔
  `arm_command_controller`를 전환한다. MoveIt 실행 전에는 `move_group`이 같은 서비스로 JTC를
  다시 활성화한다.
- Viz의 STOW/HOME은 수동 경로가 아니라 `move_action` goal의 `preset`을 쓰고 토크를 자동으로 켠다.
- 부팅 시 `arm_torque_on_start`(기본 true)로 토크를 켜 팔이 중력으로 떨어지지 않게 한다
- `arm/set_torque`(`std_srvs/SetBool`): 전체 토크 ON/OFF
- `arm/stop`(`std_srvs/Trigger`): 현재 자세 유지
- 수동/자동 전환은 `controller_manager/switch_controller` 서비스로 한다 (기존 `arm_manual` 토픽 제거)
- `arm/reset`(`krosc_ws/srv/ArmReset`): `all` / `joint1`..`joint4` / `gripper` — OpenCR `AR`로 해당 DYNAMIXEL 재부팅 (Hardware Error 해제)
- `buzzer`(`krosc_ws/srv/Buzzer`): `frequency`(Hz), `duration`(ms)
- `arm_status`(String): `present=...;err=...;temp=...` — Viz Manipulator 패널에 에러가 빨간색으로 표시된다
- 손목 roll 관절은 없다. ID 16은 사용하지 않는다.

## 주행 / Multi Goal

- nav2와 같은 노드 구조: `global_costmap`/`local_costmap`(`Costmap2DROS`) → `planner_server`
  (`NavfnPlanner` 등) → `controller_server`(`RegulatedPurePursuitController` 등) → `bt_navigator`.
- 단일 목표: Viz/웹이 nav2 표준 `navigate_to_pose` 액션을 보내고, `bt_navigator`가
  `compute_path_to_pose` 액션으로 계획한 뒤 `follow_path` 액션으로 따라간다.
  `replanning_period`마다 다시 계획하고, 실패하면 `behavior_server`(spin → backup → wait)로 회복한다.
- 경로 미리보기: `compute_path_to_pose` 액션이 `plan` 토픽을 latch로 낸다.
- Multi Goal: `navigate_through_poses` / `compute_path_through_poses` 액션을 쓴다. 각 웨이포인트에
  **위치 + yaw**를 저장하고, 구간별 경로를 하나로 이어 `follow_path` 한 번으로 보낸다. 마지막
  웨이포인트의 yaw까지 controller의 goal checker가 맞춘다.
- 취소: 현재 goal의 `cancel_goal` 서비스(액션 취소)로 처리한다.
- `global_costmap/costmap`: static_layer + inflation_layer (2D/3D Viz에 표시)
- `local_costmap/costmap`: 로봇 주변 4m rolling window, obstacle_layer + inflation_layer (2D/3D Viz에 표시)
- 색은 RViz `costmap` 스킴: 1~98 blue→red, 99 cyan(inscribed), 100 purple(lethal), -1 blue-green-gray
- 플러그인은 nav2와 같은 이름으로 고른다: `nav2_navfn_planner::NavfnPlanner`,
  `nav2_theta_star_planner::ThetaStarPlanner`, `nav2_smac_planner::SmacPlanner2D`,
  `nav2_smac_planner::SmacPlannerHybrid`, `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`,
  `dwb_core::DWBLocalPlanner`, `nav2_mppi_controller::MPPIController`,
  `nav2_rotation_shim_controller::RotationShimController`.

### Multi-client

네트워크 서버는 최대 **8 클라이언트**를 동시에 받는다. 클라이언트마다 송신 큐/writer 스레드가
따로 있어 느린 Viz 하나가 다른 클라이언트의 센서 스트림을 막지 않는다. 명령 토픽은
**마지막 명령이 이기는** 구조이며, 클라이언트별 제어권 중재(lease)는 아직 없다.

## Viz 플러그인 / 프로젝트 Viz main

로봇 쪽 진입점이 `engine/main.cpp`라면, Viz 쪽 진입점은 `viz/main.cpp`다.
`viz/main.cpp`는 **인자 파싱과 `Viz` 실행만** 하고, 앱 조립은 코어 `krosc::viz::Viz`가 맡는다.

```text
krosc_ws/
├── engine/main.cpp           # 로봇(Pi) 엔드포인트
├── viz/
│   ├── main.cpp              # 프로젝트 Viz 엔드포인트 (파라미터 → Viz)
│   ├── CMakeLists.txt
│   └── plugins/
│       ├── project_panel.cpp # 프로젝트 플러그인 예제 ("Project" 탭)
│       └── moveit_plugin.cpp # MoveIt 도크 + MoveIt 탭

krosc/viz/
├── main.cpp                  # 공통/최소 Viz 앱 (파라미터 → Viz)
├── core/
│   ├── viz.cpp               # 조립/수명주기: Context, Net Client, VizWindow, 플러그인, 타이머
│   ├── viz_cli.cpp           # 공용 인자 파서 (VizOptions)
│   └── VizWindow, SceneView, MapView2D, VizModel
├── plugins/                  # 코어 플러그인 (SLAM/Nav/Teleop/...) + registry
└── include/krosc/viz/        # Viz, VizOptions, IVizPlugin, VizHost, VizPlugin, ...
```

`Viz`가 소유/처리하는 것:

- `VizWindow`, 플러그인 로드/수명주기
- 시작 탭/고정 프레임/카메라 거리
- 접속 상태 타이머, 스크린샷, 종료 시 정리

`Context`/`Krosc`는 app main이 소유하고 `Viz`는 빌려 쓴다. 원격 연결은 `Krosc`의 `PeerNetwork`가
담당하며, 플러그인은 로컬 노드처럼 `context.topics()`/`Node`를 쓴다.

코어 API는 `krosc/viz/include/krosc/viz/viz_plugin.hpp`에 있다.

```cpp
class MyPlugin final : public krosc::viz::VizPlugin {
 public:
  MyPlugin() : VizPlugin({"my_plugin", "My Plugin", "1.0", {}}) {}
  void initialize(krosc::viz::VizHost& host) override {
    auto* page = new QWidget(&host.window());
    // ...
    host.add_tab(page, "My Plugin");
    // host.add_dock(dock, Qt::RightDockWidgetArea);
  }
};
```

레지스트리는 기본으로 비어 있고, main이 필요한 코어 플러그인만 등록한다:

```cpp
krosc::viz::VizPluginRegistry registry;
krosc::viz::register_core_plugins(registry, {"slam", "navigation"});
registry.add("my_plugin", [] { return std::make_unique<MyPlugin>(); });

krosc::viz::VizOptions options;
// parse_viz_options(argc, argv, defaults, options, help, error)로 채운다
krosc::viz::Viz viz(std::move(options), std::move(registry));
return viz.run();
```

CMake에 소스 추가 후 실행:

```bash
KROSC_VIZ_PLUGINS=project_panel,my_plugin ./run-viz.sh
```

- 플러그인은 프로젝트 안에서 자유롭게 작성/빌드한다.
- 코어 `krosc_viz_app`은 공통/최소 앱(디스플레이 토글/토픽/TF)으로 남아 있고,
  `run-viz.sh`는 프로젝트 앱을 사용한다.

### 웹 Viz (krosc_ws/viz-web)

`krosc/viz-web` 코어를 링크하고 프로젝트 플러그인(`opencr`, `moveit`, `rl_nav`, `manipulator`)을 추가한다.

```bash
./run-viz-web.sh                 # http://localhost:8080/, robot pak.local:9000
KROSC_VIZ_HOST=10.2.150.138 ./run-viz-web.sh
```

- 프레임 포맷은 코어 `krosc::net`(`net_protocol.hpp`)을 그대로 쓴다.
  브라우저 ↔ 로컬 토픽 브리지는 viz-web 전용 `ContextLink`가, 명령은 JSON RPC
  (`WebRpc`)가 맡는다. 둘 다 engine/core에는 없다.
- `krosc_ws_viz_web` = `krosc::viz_web` 코어 + 프로젝트 레지스트리
- 내보낼 토픽은 `VizWebOptions::forward_topics`로 정한다. 코어 기본 목록 +
  프로젝트 main이 추가한 `moveit_*`/`rl_nav_*` 토픽이 브라우저로 간다.
- 프로젝트 JS: `krosc_ws/viz-web/www/plugins/{opencr,moveit,rl_nav,manipulator}.js`
  (OpenCR 패널은 이 로봇 전용이라 코어 www가 아니라 프로젝트 www에 둔다)
- 정적 파일은 프로젝트 www → 코어 www 순으로 찾는다 (`VizWebOptions::overlay_root`)
- `/plugins.json`은 main이 등록한 레지스트리에서 동적으로 생성된다
- 코어 Viz 플러그인: `krosc/viz/plugins/teleop_plugin.cpp`(주행 텔레옵),
  `slam_plugin.cpp`(SLAM/지도), `navigation_plugin.cpp`(2D Pose/Goal/Multi Goal),
  `manipulator_plugin.cpp`(수동 관절).
- 프로젝트 플러그인: `krosc_ws/viz/plugins/project_panel.cpp`, `moveit_plugin.cpp`, `rl_nav_plugin.cpp`.
- 남은 코어 후보: Displays/Topics 패널의 플러그인화, YAML 프로필 로딩.

## RL navigation (rl_nav_controller)

Isaac `rl_maze` 정책(52관측 → 2행동, nav 50Hz)을 Pi에서 ONNX Runtime으로 실행한다.

- 모델 폴더: `artifacts/models/rl_nav_controller/<name>/`에 `*.onnx` + `*.onnx.data`
  (기본 `rl_maze`: `policy.onnx` + `policy.onnx.data`)
- `RlNavController`는 독립 노드가 아니라 **`NavControllerPlugin` 구현**이다.
  `controller_server`의 `FollowPath.plugin="rl_nav_controller::RlNavController"`로 선택된다.
- ONNX Runtime 실행 장치는 **CUDA → CoreML → CPU** 순으로 고른다. GPU가 없거나
  CUDA 세션 생성이 실패하면 CPUExecutionProvider로 내려간다.
  ONNX Runtime 없이 빌드해도 플러그인은 있고, 정책 로드가 실패하면 fallback 컨트롤러가 주행한다.
- 폴더 스캔: `engine/ml/model_catalog.hpp`
- Viz: `viz/plugins/rl_nav_plugin.cpp` — 모델 선택, RL ON/OFF, fallback, 실행 장치/상태
- 지도와 같은 방식으로 **모델 목록**을 고른다:
  - `rl_nav_model_list`(String, latch): 모델 이름들 (한 줄에 하나)
  - `rl_nav_model`(String): 선택한 모델 이름 → 그 자리에서 다시 로드
  - `rl_nav_status`에 `model=<이름>`이 포함된다
- 기본은 **RL OFF**. Viz의 `RL Nav` 도크에서 모델을 고르고 켠다.

```text
teleop   priority 100   cmd_vel_teleop
nav      priority  10   cmd_vel_nav (bt_navigator → controller_server → RL 또는 fallback)
```

RL 플러그인은 `ControllerServer` 안에서 돈다. `cmd_vel_rl`/twist_mux rl 레인은 없어졌다.

fallback (`FollowPath.fallback_controller` 또는 Viz의 `rl_nav_fallback`):

- `rpp` — 기본 (nav2 기본과 같게). RL이 꺼지거나 정책/스캔이 없거나 추론이 실패하면 RPP가 주행
- `mppi` — MPPI로 fallback
- `dwb` — DWB로 fallback

RL이 켜져도 scan/odom/plan/TF가 없거나 추론이 실패하면 자동으로 fallback한다.
`infer_ms`, `scan_age`, `reason`, `provider`, `onnxruntime`은 `rl_nav_status`로 Viz에 표시된다.
`rl_nav_status`는 latched이고 1초마다, 모델 목록은 2초마다 다시 내므로 Viz가 늦게 접속해도
현재 상태/모델이 보인다 (주행 중에는 compute가 0.2초마다 상태를 갱신한다).

진단은 ROS 2처럼 `diagnostics`(`diagnostic_msgs/DiagnosticArray`)에 `controller_server: RL Navigation`
상태를 낸다. 값: `onnxruntime`, `onnxruntime_version`, `provider`, `providers`,
`model`, `policy`, `fallback`, `reason`, `infer_ms`, `scan_age`.
Viz는 여러 노드의 diagnostics를 이름 기준으로 합쳐서 본다 (`viz/core/viz_model.cpp`).

ONNX Runtime은 `/opt/ros/lyrical/opt/onnxruntime_vendor`를 우선 찾고,
없으면 Homebrew prefix(`/opt/homebrew`)도 본다. 둘 다 없으면 RL은 fallback 전용으로 빌드된다.
`KROSC_WS_BUILD_RL_NAV=OFF`면 플러그인/노드를 아예 빌드하지 않고 main이 RPP를 쓴다.

## 시뮬레이터 (PC 전용)

`run-simulator.sh`는 `krosc/simulator`의 월드/로봇 위에서 **krosc_ws의 ControlManager,
SensorManager, ControllerManager, nav2**를 그대로 돌리고 **Qt 진실 뷰 창**을 띄운다.
Pi에서는 타깃을 만들지 않는다.

```bash
./run-simulator.sh               # Qt 창 + PeerNetwork 9000
./run-simulator.sh --pucks 10 --no-noise
./run-simulator.sh --slam         # Cartographer mapping 모드
./run-simulator.sh --arm          # OpenMANIPULATOR/MoveIt 포함
./run-simulator.sh --port 0 --duration 5  # 네트워크 없이 5초
```

Qt 창에서 `W/A/S/D` 텔레옵, `L` 라이다 표시 토글, 드래그 orbit, 휠 zoom을 쓸 수 있다.
오른쪽에는 `MANIPULATOR` 패널이 붙어 joint1~4/gripper를 수동 조작하고 HOME/STOW를 실행한다.

기본은 노이즈 모드다. **물리 차체 슬립**은 `WafflePiArm`이 만들고,
**엔코더 양자화/IMU 바이어스/라이다 거리 잡음·드롭아웃**은 하드웨어 경계인
`SimulatorOpenCr`/`SimulatorLds`가 센서값에 입혀 engine으로 전달한다.
`--no-noise`로 끌 수 있으며, Qt truth view는 잡음 없는 진실 값을 그린다.

```text
WafflePiArm(truth) → SimulatorOpenCr(encoder/IMU noise) ┐
                   → SimulatorLds(range/dropout noise) ┴→ Robot engine
```

실기/시뮬레이터는 `engine/robot.cpp`의 공통 `Robot` 클래스에
`OpenCrDevice`/`LdsDevice`만 다르게 주입한다. 시뮬레이터는
`simulator/hardwares/opencr/simulator_opencr.cpp`와 `simulator/hardwares/lds03/simulator_lds.cpp`가 월드 로봇(`WafflePiArm`)을 OpenCR/LDS 장치처럼 감싼다.
정적 map을 미리 발행하지 않는다. 지도는 `--slam` 매핑으로 만들거나 `artifacts/maps`의
저장 맵을 로드해서 `map_server`/Cartographer가 만들고, URDF는 시뮬레이터용
`simulator/robot/urdf` 복제본을 쓴다.

웹 Viz로 보려면:

```bash
KROSC_VIZ_HOST=127.0.0.1 KROSC_VIZ_PORT=9000 ./run-viz-web.sh
# http://localhost:8080/
```

## MoveIt / control

Viz의 **MoveIt** 도킹 패널/탭은 `krosc_ws/viz/plugins/moveit_plugin.cpp` 플러그인이다.
코어 `krosc/viz`에는 MoveIt이 없고, 프로젝트 Viz 앱에서만 로드한다.
계산은 Pi의 `krosc::moveit::MoveGroup` 노드가 하고, Viz는 `move_action` 액션/서비스
클라이언트로 명령하고 `moveit_*` 상태 토픽을 구독한다.
내부는 ROS 2처럼 플러그인으로 나뉜다:

- `kinematics.plugin` — `krosc_moveit/ArmChainKinematics` (URDF 체인 해석 IK)
- `planner.plugin` — `krosc_moveit/CartesianPlanner` 또는 `krosc_moveit/JointSpacePlanner`
- `controller_manager.plugin` — `krosc_moveit/SimpleControllerManager`
- `move_action` 액션(`moveit_msgs/action/MoveGroup` 축소판) + `move_group/scrub` 서비스

하드웨어는 `control` 구조를 따른다:

- `controller_manager` 노드: `hardware_plugin`(OpenCR) → state/command interfaces → controllers
- `joint_state_broadcaster`: `joint_states` 발행
- `diff_drive_controller`: `cmd_vel` → 바퀴 velocity command, 바퀴 position → `odom`
- `joint_trajectory_controller`: `FollowJointTrajectory` 액션으로 팔 궤적 실행
- `forward_command_controller`: `arm_joint_command` 수동 명령 (기본 비활성)
- `controller_manager/list_controllers`, `load_controller`, `configure_controller`,
  `switch_controller` 서비스
- command interface는 한 컨트롤러만 잡을 수 있다 (ROS 2와 같은 규칙)

물리/논리 레이어는 이렇게 나뉜다:

- `hardwares/`: 물리 장치만 (`opencr`, `lds03`)
- `controls/opencr/`: state/command interface와 명령 매핑 (`Control`)
- `sensors/opencr/`: IMU 읽기 → `imu` 발행
- `sensors/lds03/`: 스캔 읽기 → `scan` 발행
- `hardwares/`는 `Hardware`(물리 lifecycle), `controls/`는 `Control`(state/command),
  `sensors/`는 `Sensor`(read/publish)를 상속한다
- `SensorManager`: `add(sensor, name, period)`로 등록하고 각자 주기로 read/publish
- `ControlManager`: `add(control, name, read_period, write_period)`로 등록하고
  `control read → controller hook → control write` 순서로 실행한다
- Hardware는 main에서 생성/start/stop하고 각 Control/Sensor 생성자에 주입한다
- control hook은 `controller_manager.update()`다. `main`이 장치/유닛/매니저를 조립하고 소유한다
- LDS-03은 `scan`/`base_scan`(기본 10Hz), OpenCR IMU는 `imu`/`imu_link`(기본 50Hz)
- 실행 시 변경: `--scan-topic`, `--scan-rate`, `--imu-topic`, `--imu-rate`,
  `--executor-threads` (환경 변수 `KROSC_SCAN_TOPIC`, `KROSC_IMU_TOPIC`, `KROSC_EXECUTOR_THREADS`)
- 로봇 main은 `Context::executor_threads`를 기본 10으로 켠다 (라이브러리 기본은 0=inline)

- 그룹: `base_link` → `end_effector_link`, 관절 `joint1`~`joint4`
- 목표: TCP `x,y,z` + 접근 `pitch` (4축이라 full 6D는 불가)
- `Plan`: IK → joint-space 또는 Cartesian 궤적 → 관절 한계/속도 검사
- `Execute`: `move_action`(plan_only=false) → `FollowJointTrajectory` 액션, `Cancel`: `move_action/cancel_goal`
- 3D: 목표 TCP 프레임, ghost arm, TCP 경로 표시
- 팔 Hardware Error가 있으면 Plan/Execute를 거부한다 (먼저 Reset)
- 명령: `move_action` 액션(`preset`, `target_pose`, `joint_goal`, `grip`, `plan_only`, `cartesian`),
  `move_group/scrub` 서비스
- 상태: `moveit_status`, `moveit_state`, `moveit_target_pose`, `moveit_arm`, `moveit_path`

핵심 계산은 `krosc/engine/core/manipulation/arm_chain.cpp`, MoveIt 플러그인/노드는
`krosc/engine/include/krosc/nodes/moveit/`와 `krosc/engine/core/nodes/moveit/move_group.cpp`에 있다.
control 레이어는 `krosc/engine/include/krosc/control/`, 실기 하드웨어 플러그인은
`krosc_ws/engine/controls/opencr/opencr_control.cpp`이고 `krosc_ws/engine/main.cpp`에서 조립한다.
LDS-03은 물리 장치 `hardwares/lds03/lds03_device.cpp`와
scan을 발행하는 sensor 논리 레이어 `sensors/lds03/lds03_sensor.cpp`로 나뉜다.
OpenCR은 물리 `hardwares/opencr/opencr_serial.cpp`, 제어 `controls/opencr/opencr_control.cpp`,
IMU `sensors/opencr/opencr_imu.cpp`로 나뉘고 SensorManager/ControlManager가 주기 실행한다.
Viz 쪽은 `krosc_ws/viz/plugins/moveit_plugin.cpp`가 도크 + MoveIt 탭을 만든다.

## 구성

```
krosc_ws/
├── engine/                      # 로봇(Pi) 앱 엔진
│   ├── main.cpp                 # 노드 조립, TCP 서버, 진단 루프
│   ├── robot.cpp/.hpp            # 공통 로봇 스택 (하드웨어 주입)
│   ├── nodes/robot_status.cpp/.hpp # arm_status/diagnostics, 팔 서비스
│   ├── nodes/rl_nav_controller.cpp/.hpp # NavControllerPlugin (RL + fallback)
│   ├── ml/onnx_policy.cpp/.hpp  # ONNX Runtime (CUDA/CoreML → CPU)
│   ├── ml/model_catalog.hpp     # artifacts/models/rl_nav_controller 스캔
│   ├── hardwares/               # 물리 장치만
│   │   ├── serial.cpp/.hpp          # POSIX termios (공용)
│   │   ├── opencr/
│   │   │   ├── opencr_serial.cpp/.hpp   # hardware::Hardware + 시리얼 읽기/캐시
│   │   │   └── opencr_protocol.cpp/.hpp # C/S/I/E/M/F 파서
│   │   └── lds03/
│   │       └── lds03_device.cpp/.hpp    # hardware::Hardware + 패킷 파서
│   ├── controls/                # 논리 제어
│   │   └── opencr/
│   │       └── opencr_control.cpp/.hpp  # Control (바퀴+팔)
│   ├── sensors/                 # 논리 센서
│   │   ├── opencr/
│   │   │   └── opencr_imu.cpp/.hpp      # IMU -> imu 토픽
│   │   └── lds03/
│   │       └── lds03_sensor.cpp/.hpp    # scan -> scan 토픽
│   └── description/urdf/turtlebot3_real.urdf
├── simulator/                   # PC 전용 krosc_ws 시뮬레이터 (Pi 미빌드)
│   ├── hardwares/opencr/        # SimulatorOpenCr 스텁
│   ├── hardwares/lds03/         # SimulatorLds 스텁
│   ├── robot/                   # WafflePiArm + URDF 파서/시뮬레이터용 URDF 복제본
│   └── panels/arm_panel.*       # Qt 수동 조작 패널
├── viz/                         # PC Qt Viz 앱 + 프로젝트 플러그인
│   ├── main.cpp
│   ├── CMakeLists.txt
│   └── plugins/
├── viz-web/                     # PC 웹 Viz 앱 + 프로젝트 JS 플러그인
│   ├── main.cpp
│   ├── CMakeLists.txt
│   └── www/plugins/             # moveit.js, rl_nav.js
├── artifacts/models/rl_nav_controller/<name>/  # policy.onnx + policy.onnx.data
├── firmware/opencr/             # 이 프로젝트 전용 OpenCR 펌웨어 사본
│   ├── turtlebot_opencr/turtlebot_opencr.ino
│   ├── turtlebot_opencr/arm_config.h
│   ├── compile.sh · upload.sh · test-firmware
│   └── README.md
├── artifacts/maps/
└── tests/                       # 프로토콜/파서 + PTY 시리얼 왕복
```

실기 값:

- 바퀴 반지름 0.033 m, 바퀴 간격 0.287 m, 최대 7.8 rad/s
- 두 모터 모두 `+속도 = 뒤` → 앱 안에서 `left_dir = right_dir = -1`로 뒤집는다
- LDS-03: 230400, 360칸, 0.05~12 m, `reverse=True`, base_link 기준 x=0.060, z=0.022
- 라이다 몸통 필터: base_link 기준 x -0.21~0.075, y ±0.17 안의 점 제거
- EKF: odom pose(yaw)/vx/vyaw, 50Hz, `odom -> base_footprint` TF (IMU yaw는 bias 때문에 제외)

## 펌웨어 (이 프로젝트 전용 사본)

원본 공용 `firmware/`와 별개로, `krosc_ws`는 자기 펌웨어를 `firmware/opencr`에 둔다.

```bash
cd ~/ros2/krosc_ws/firmware/opencr
./compile.sh
./upload.sh
./test-firmware
```

스케치 이름은 폴더와 같아야 한다: `turtlebot_opencr/turtlebot_opencr.ino`.

## 테스트

```bash
cd ~/ros2/krosc_ws
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
```

하드웨어 없이 검증하는 것:

- OpenCR S/I/E 파싱, C 명령 포맷
- LDS-03 패킷 체크섬/링 시작/역방향 각도
- PTY를 이용한 OpenCrSerial 왕복 (S/I 읽기, C 쓰기)
- `map_server/load_map`·`unload_map`, `map_saver/save_map` 왕복
- control diff drive odom, MoveIt 실제 URDF IK→계획→JTC 실행
- RL 모델 폴더 스캔/선택 (`rl_nav_model_list`)

## 아직 안 된 것

- full MoveIt 2 (OMPL planning scene, collision world) — 지금은 URDF 체인 + 자체 플러그인
- 파라미터/서비스/액션의 네트워크 전달, rosbag
- Cartographer 전체 기능(loop closure, submap) — 현재는 scan matching + occupancy map 수준

지금은 **실기 SLAM + nav2 주행 + Multi Goal + control MoveIt 팔 계획/실행 + 원격 Viz**까지
`krosc_ws` 안에서 동작한다.
