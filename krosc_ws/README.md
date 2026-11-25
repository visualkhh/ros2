# krosc_ws — 실기 TurtleBot3 (C++ krosc)

라즈베리파이에서 **OpenCR + LDS-03를 직접 돌리는 C++ 워크스페이스**다.
시뮬레이터/월드/Qt 창은 없다. Pi는 하드웨어와 토픽만 돌리고, 맥의 C++ `viz`가 TCP(9000)로 붙어서 본다.

```
[라즈베리파이] krosc_real
  OpenCRSerial (115200, C/S/I/E/A) ─┐
  LDS-03     (230400, 10Hz) ────────┤
                                    ├─ TurtleBot3Driver ─ scan/imu/arm_status/diagnostics
                                    ├─ controller_manager (ros2_control)
                                    │    ├─ OpenCrSystem (hardware plugin)
                                    │    ├─ joint_state_broadcaster ─ joint_states
                                    │    ├─ diff_drive_controller ─ odom
                                    │    └─ joint_trajectory_controller ─ 팔
                                    ├─ MoveGroup (MoveIt 플러그인) ─ moveit_*
                                    ├─ robot_state_publisher (URDF/TF)
                                    ├─ ekf_filter_node ─ odometry/filtered + odom->base_footprint TF
                                    ├─ map_server / map_saver / map_catalog
                                    ├─ nav2: planner/controller/behavior/bt_navigator
                                    └─ net::Server (TCP 9000) ──> [맥] krosc_ws_viz_app
                                             ^ cmd_vel / goal / moveit_*
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

./build.sh          # krosc(core/sim/viz) + krosc_ws를 한 번에 빌드
./run.sh            # 실기: 빌드 후 idle/텔레옵 실행 (TCP 9000)
./run-slam.sh       # 실기: 빌드 후 SLAM(mapping) 실행
./run-tests.sh      # krosc + krosc_ws 테스트 빌드/실행

# PC에서 (Qt6 필요)
./run-viz.sh        # 빌드 후 pak.local:9000 C++ Viz (fixed frame map)
KROSC_VIZ_HOST=10.2.150.135 ./run-viz.sh
./run-sim.sh        # 시뮬레이터(진실) + Viz 데모
```

`run-viz.sh`는 프로젝트 전용 `krosc_ws_viz_app`을 빌드/실행한다.
`KROSC_VIZ_HOST`, `KROSC_VIZ_PORT`, `KROSC_VIZ_FIXED_FRAME`, `KROSC_VIZ_TAB`,
`KROSC_VIZ_PLUGINS` 환경 변수를 따른다.
기본은 **Map 탭**(RViz 스타일 occupancy 이미지)이고, `KROSC_VIZ_TAB=3d ./run-viz.sh`,
`KROSC_VIZ_TAB=moveit ./run-viz.sh`, `KROSC_VIZ_TAB=project ./run-viz.sh`로 시작 탭을 바꿀 수 있다.
SLAM/Map, Navigation, MoveIt, RL 도킹이 기본으로 붙는다. 텔레옵/매니퓰레이터는
`--teleop`, `--manipulator`, `--manual`로 추가한다. 추가 인자는 그대로 viz에 넘긴다:
`./run-viz.sh --screenshot /tmp/viz.png --duration 3`, `./run-viz.sh --no-moveit`.

기본 플러그인은 `project_panel,slam,navigation,moveit,rl_nav`이며
`KROSC_VIZ_PLUGINS`로 바꿀 수 있다. 코어 플러그인은 `teleop`, `slam`, `navigation`,
`manipulator`이고 프로젝트 플러그인은 `project_panel`, `moveit`, `rl_nav`다.

Viz 단축키(한글 두벌식 레이아웃에서도 동작):

- `W/A/S/D` 또는 방향키: 텔레옵, `Q/Z`: 속도, `F`: follow, `C`: overview
- `Backspace`: Multi Goal 모드에서 마지막 웨이포인트 삭제
- MoveIt `Plan`/`Execute`/`Cancel`은 `move_group` 노드에 토픽으로 요청한다
- 중앙 **MoveIt 탭**에서 TCP 마커를 드래그(X/Y), `Shift+드래그`로 Z, `Ctrl+드래그`로 Pitch 조정

### 지도 폴더

기본 지도 폴더는 **`~/ros2/kos_ws/artifacts/maps`** (Python `kos_ws`와 공유)다.
지도 하나 = 폴더 하나(`<이름>/map.yaml` + `map.pgm`)이고, ROS 2처럼
`map_server`(load_map/unload_map)와 `map_saver`(save_map)가 실제 파일 I/O를 맡는다.
`map_catalog`는 목록/선택 토픽(`map_list`, `map_change`, `map_save`, `map_status`)만 담당한다.
`krosc_ws/CMakeLists.txt`의 `KROSC_WS_MAPS_DIR`로 정해지고, 실행할 때 바꿀 수 있다:

```bash
./run.sh --maps /home/pak/ros2/krosc_ws/artifacts/maps
KROSC_MAPS_DIR=/home/pak/ros2/krosc_ws/artifacts/maps ./run-slam.sh
```

맵을 `krosc_ws` 소유로 두려면 위 경로에 폴더를 만들고 기존 지도를 복사하거나 심볼릭 링크하면 된다.

## 실행 (라즈베리파이)

```bash
cd ~/ros2
./krosc_ws/run.sh                                      # 빌드 + 실행, TCP 9000
./krosc_ws/run-slam.sh                                 # 빌드 + SLAM(mapping)
KROSC_SERVER_PORT=0 ./krosc_ws/run.sh                  # 네트워크 없이
OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar ./krosc_ws/run.sh
```

- 시작 로그에 접속 주소가 나온다: `[krosc_ws] connect: 192.168.x.x:9000`.
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
- 아직 SLAM/지도가 없으므로 기준 프레임은 **`odom`**을 쓴다.
- 창 왼쪽 패널 위에 `remote: <ip>:9000 · connected`가 뜨면 연결된 것이다.
- 원격 viz의 `W/A/S/D` → TCP로 `cmd_vel` → OpenCR. 텔레옵 속도는 `Q/Z`.
- 한 번에 viz 하나만 붙는다. 새 클라이언트가 붙으면 기존 연결을 교체한다.
- 네트워크는 신뢰할 수 있는 LAN 전용이다 (인증 없음). 포트 9000을 외부에 열지 마라.

원격 토픽: `tf`, `tf_static`, `robot_description`, `scan`, `imu`, `odom`, `odometry/filtered`,
`joint_states`, `arm_status`, `diagnostics`, `rosout`, `rl_nav_model_list`
(+ `cmd_vel`, `goal_*`, `moveit_*`, `rl_nav_model`, `arm_joint_command`, `arm_torque`, `arm_stop`,
`arm_reset` 역방향).

## 매니퓰레이터 (OpenMANIPULATOR-X 4축 + 그리퍼)

- URDF: `robot/description/urdf/turtlebot3_real.urdf` (공식 visual STL + collision 근사, C++ Viz는 visual STL을 12mm 격자로 간소화해 렌더)
- 장착: 3층 판 맨 앞 중앙, `base_link` 기준 `x=+0.047985, y=-0.000230, z=+0.130175`
- OpenCR: joint1~4 = ID 11~14, gripper = ID 15 (`firmware/opencr/turtlebot_opencr/arm_config.h`)
- `joint_states`는 ros2_control `joint_state_broadcaster`가 바퀴+팔 관절을 모두 발행한다
- 팔 명령은 MoveIt → `joint_trajectory_controller/follow_joint_trajectory` 액션으로 내려간다.
  수동 `arm_joint_command`(`sensor_msgs/JointState`)는 `arm_command_controller`(forward controller)가
  활성일 때만 하드웨어로 간다 (기본은 비활성, `controller_manager/switch_controller`로 전환)
- `arm_torque`(Bool): 전체 토크 ON/OFF, `arm_stop`(Empty): 현재 자세 유지
- `arm_reset`(String): `all` / `joint1`..`joint4` / `gripper` — OpenCR `AR`로 해당 DYNAMIXEL 재부팅 (Hardware Error 해제)
- `arm_status`(String): `present=...;err=...;temp=...` — Viz Manipulator 패널에 에러가 빨간색으로 표시된다
- 손목 roll 관절은 없다. ID 16은 사용하지 않는다.

## 주행 / Multi Goal

- nav2와 같은 노드 구조: `global_costmap`/`local_costmap`(`Costmap2DROS`) → `planner_server`
  (`NavfnPlanner` 등) → `controller_server`(`RegulatedPurePursuitController` 등) → `bt_navigator`.
- `goal_pose`(단일), `goal_poses`(multi): `bt_navigator`가 `compute_path_to_pose` 액션으로 계획하고
  `follow_path` 액션으로 따라간다. `replanning_period`마다 다시 계획하고, 실패하면
  `behavior_server`(spin → backup → wait)로 회복한다.
- Multi Goal은 각 웨이포인트에 **위치 + yaw**를 저장하고, 구간별 경로를 하나로 이어
  `follow_path` 한 번으로 보낸다 (nav2 `navigate_through_poses`와 같은 방식). 마지막 웨이포인트의
  yaw까지 controller의 goal checker가 맞춘다.
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

로봇 쪽 진입점이 `robot/main.cpp`라면, Viz 쪽 진입점은 `viz/main.cpp`다.
`viz/main.cpp`는 **인자 파싱과 `Viz` 실행만** 하고, 앱 조립은 코어 `krosc::viz::Viz`가 맡는다.

```text
krosc_ws/
├── robot/main.cpp            # 로봇(Pi) 엔드포인트
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

- `Context`, `net::Client`, `VizWindow`
- 플러그인 로드와 시작 탭/고정 프레임/카메라 거리
- 접속 상태 타이머, 스크린샷, 종료 시 정리

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

코어 플러그인은 `register_core_plugins()`로 한 번에 등록하고, 프로젝트 플러그인만 추가한다:

```cpp
krosc::viz::VizPluginRegistry registry;
krosc::viz::register_core_plugins(registry);
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
- 코어 Viz 플러그인: `krosc/viz/plugins/teleop_plugin.cpp`(주행 텔레옵),
  `slam_plugin.cpp`(SLAM/지도), `navigation_plugin.cpp`(2D Pose/Goal/Multi Goal),
  `manipulator_plugin.cpp`(수동 관절).
- 프로젝트 플러그인: `krosc_ws/viz/plugins/project_panel.cpp`, `moveit_plugin.cpp`, `rl_nav_plugin.cpp`.
- 남은 코어 후보: Displays/Topics 패널의 플러그인화, YAML 프로필 로딩.

## RL navigation (rl_nav_controller)

Isaac `rl_maze` 정책(52관측 → 2행동, 20Hz)을 Pi에서 ONNX Runtime으로 실행한다.

- 모델 폴더: `artifacts/models/rl_nav_controller/<name>/`에 `*.onnx` + `*.onnx.data`
  (기본 `rl_maze`: `policy.onnx` + `policy.onnx.data`)
- `RlNavController`는 독립 노드가 아니라 **`NavControllerPlugin` 구현**이다.
  `controller_server`의 `FollowPath.plugin="rl_nav_controller::RlNavController"`로 선택된다.
- ONNX Runtime 실행 장치는 **CUDA → CoreML → CPU** 순으로 고른다. GPU가 없거나
  CUDA 세션 생성이 실패하면 CPUExecutionProvider로 내려간다.
  ONNX Runtime 없이 빌드해도 플러그인은 있고, 정책 로드가 실패하면 fallback 컨트롤러가 주행한다.
- 폴더 스캔: `robot/ml/model_catalog.hpp`
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

## MoveIt / ros2_control

Viz의 **MoveIt** 도킹 패널/탭은 `krosc_ws/viz/plugins/moveit_plugin.cpp` 플러그인이다.
코어 `krosc/viz`에는 MoveIt이 없고, 프로젝트 Viz 앱에서만 로드한다.
계산은 Pi의 `krosc::moveit::MoveGroup` 노드가 하고, Viz는 토픽 클라이언트로만 동작한다.
내부는 ROS 2처럼 플러그인으로 나뉜다:

- `kinematics.plugin` — `krosc_moveit/ArmChainKinematics` (URDF 체인 해석 IK)
- `planner.plugin` — `krosc_moveit/CartesianPlanner` 또는 `krosc_moveit/JointSpacePlanner`
- `controller_manager.plugin` — `krosc_moveit/SimpleControllerManager`
- `move_action` 액션(`moveit_msgs/action/MoveGroup` 축소판) + 기존 `moveit_*` 토픽

하드웨어는 `ros2_control` 구조를 따른다:

- `controller_manager` 노드: `hardware_plugin`(OpenCR) → state/command interfaces → controllers
- `joint_state_broadcaster`: `joint_states` 발행
- `diff_drive_controller`: `cmd_vel` → 바퀴 velocity command, 바퀴 position → `odom`
- `joint_trajectory_controller`: `FollowJointTrajectory` 액션으로 팔 궤적 실행
- `forward_command_controller`: `arm_joint_command` 수동 명령 (기본 비활성)
- `controller_manager/list_controllers`, `load_controller`, `configure_controller`,
  `switch_controller` 서비스
- command interface는 한 컨트롤러만 잡을 수 있다 (ROS 2와 같은 규칙)

- 그룹: `base_link` → `end_effector_link`, 관절 `joint1`~`joint4`
- 목표: TCP `x,y,z` + 접근 `pitch` (4축이라 full 6D는 불가)
- `Plan`: IK → joint-space 또는 Cartesian 궤적 → 관절 한계/속도 검사
- `Execute`: `joint_trajectory_controller`의 `FollowJointTrajectory` 액션, `Cancel`: `arm_stop`
- 3D: 목표 TCP 프레임, ghost arm, TCP 경로 표시
- 팔 Hardware Error가 있으면 Plan/Execute를 거부한다 (먼저 Reset)
- 토픽: `moveit_target`(String CSV), `moveit_plan`, `moveit_execute`, `moveit_cancel`, `moveit_scrub`
- 상태: `moveit_status`, `moveit_state`, `moveit_target_pose`, `moveit_arm`, `moveit_path`

핵심 계산은 `krosc/engine/core/manipulation/arm_chain.cpp`, MoveIt 플러그인/노드는
`krosc/engine/include/krosc/nodes/moveit/`와 `krosc/engine/core/nodes/moveit/move_group.cpp`에 있다.
ros2_control 레이어는 `krosc/engine/include/krosc/ros2_control/`, 실기 하드웨어 플러그인은
`krosc_ws/robot/hardwares/opencr/opencr_system.cpp`이고 `krosc_ws/robot/main.cpp`에서 조립한다.
LDS-03은 `SensorInterface`(on_init → on_activate → read → on_deactivate)를 따르며
`krosc_ws/robot/hardwares/lds03/lds03.cpp`에 있다.
Viz 쪽은 `krosc_ws/viz/plugins/moveit_plugin.cpp`가 도크 + MoveIt 탭을 만든다.

## 구성

```
krosc_ws/
├── robot/                       # 로봇(Pi) 앱
│   ├── main.cpp                 # 50Hz 실시간 루프, 노드 조립, TCP 서버
│   ├── turtlebot3_driver.cpp/.hpp # scan/imu/arm_status/diagnostics, 팔 서비스
│   ├── real_turtlebot3.cpp/.hpp # 이 로봇의 치수/장착 위치/시리얼 조립
│   ├── nodes/rl_nav_controller.cpp/.hpp # NavControllerPlugin (RL + fallback)
│   ├── ml/onnx_policy.cpp/.hpp  # ONNX Runtime (CUDA/CoreML → CPU)
│   ├── ml/model_catalog.hpp     # artifacts/models/rl_nav_controller 스캔
│   ├── hardwares/
│   │   ├── serial.cpp/.hpp          # POSIX termios
│   │   ├── opencr/
│   │   │   ├── opencr_system.cpp/.hpp   # ros2_control SystemInterface (바퀴+팔)
│   │   │   ├── opencr_serial.cpp/.hpp   # OpenCR 읽기 스레드 + 20Hz C/A 명령
│   │   │   └── opencr_protocol.cpp/.hpp # C/S/I/E/M/F 파서 (순수 함수)
│   │   └── lds03/
│   │       └── lds03.cpp/.hpp       # SensorInterface(read-only) + 패킷 파서 + 몸통 필터
│   └── description/urdf/turtlebot3_real.urdf
├── viz/                         # PC Viz 앱 + 프로젝트 플러그인
│   ├── main.cpp
│   ├── CMakeLists.txt
│   └── plugins/
├── artifacts/models/rl_nav_controller/<name>/  # policy.onnx + policy.onnx.data
├── firmware/opencr/             # 이 프로젝트 전용 OpenCR 펌웨어 사본
│   ├── turtlebot_opencr/turtlebot_opencr.ino
│   ├── turtlebot_opencr/arm_config.h
│   ├── compile.sh · upload.sh · test-firmware
│   └── README.md
├── artifacts/maps/
└── tests/                       # 프로토콜/파서 + PTY 시리얼 왕복
```

실기 값은 Python `kos_ws`와 같다:

- 바퀴 반지름 0.033 m, 바퀴 간격 0.287 m, 최대 7.8 rad/s
- 두 모터 모두 `+속도 = 뒤` → 앱 안에서 `left_dir = right_dir = -1`로 뒤집는다
- LDS-03: 230400, 360칸, 0.05~12 m, `reverse=True`, base_link 기준 x=0.060, z=0.022
- 라이다 몸통 필터: base_link 기준 x -0.21~0.075, y ±0.17 안의 점 제거
- EKF: odom pose(yaw)/vx/vyaw, 20Hz, `odom -> base_footprint` TF (IMU yaw는 bias 때문에 제외)

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
- ros2_control diff drive odom, MoveIt 실제 URDF IK→계획→JTC 실행
- RL 모델 폴더 스캔/선택 (`rl_nav_model_list`)

## 아직 안 된 것

- full MoveIt 2 (OMPL planning scene, collision world) — 지금은 URDF 체인 + 자체 플러그인
- 파라미터/서비스/액션의 네트워크 전달, rosbag
- Cartographer 전체 기능(loop closure, submap) — 현재는 scan matching + occupancy map 수준

지금은 **실기 SLAM + nav2 주행 + Multi Goal + ros2_control MoveIt 팔 계획/실행 + 원격 Viz**까지
`krosc_ws` 안에서 동작한다.
