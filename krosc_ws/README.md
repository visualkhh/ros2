# krosc_ws — 실기 TurtleBot3 (C++ krosc)

라즈베리파이에서 **OpenCR + LDS-03를 직접 돌리는 C++ 워크스페이스**다.
시뮬레이터/월드/Qt 창은 없다. Pi는 하드웨어와 토픽만 돌리고, 맥의 C++ `viz`가 TCP(9000)로 붙어서 본다.

```
[라즈베리파이] krosc_real
  OpenCRSerial (115200, C/S/I/E) ──┐
  LDS-03     (230400, 10Hz) ───────┤
                                   ├─ RobotBridge ─ scan/imu/joint_states/diagnostics
                                   ├─ RobotStatePublisher (URDF/TF)
                                   ├─ DiffDriveOdometry ─ odom
                                   ├─ Ekf ─ odometry/filtered + odom->base_footprint TF
                                   └─ net::Server (TCP 9000) ──> [맥] krosc_viz_app --server
                                            ^ cmd_vel
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
Teleop/Manipulator/MoveIt 도킹 패널을 함께 띄운다. 추가 인자는 그대로 viz에 넘긴다:
`./run-viz.sh --screenshot /tmp/viz.png --duration 3`, `./run-viz.sh --no-moveit`.

기본 플러그인은 `project_panel,moveit`이며 `KROSC_VIZ_PLUGINS`로 바꿀 수 있다.

Viz 단축키(한글 두벌식 레이아웃에서도 동작):

- `W/A/S/D` 또는 방향키: 텔레옵, `Q/Z`: 속도, `F`: follow, `C`: overview
- `Backspace`: Multi Goal 모드에서 마지막 웨이포인트 삭제
- MoveIt `Plan`/`Execute`/`Cancel`은 `moveit_lite` 노드에 토픽으로 요청한다
- 중앙 **MoveIt 탭**에서 TCP 마커를 드래그(X/Y), `Shift+드래그`로 Z, `Ctrl+드래그`로 Pitch 조정

### 지도 폴더

기본 지도 폴더는 **`~/ros2/kos_ws/artifacts/maps`** (Python `kos_ws`와 공유)다.
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
`joint_states`, `arm_status`, `diagnostics`, `rosout` (+ `cmd_vel`, `arm_joint_command`, `arm_torque`,
`arm_stop`, `arm_reset` 역방향).

## 매니퓰레이터 (OpenMANIPULATOR-X 4축 + 그리퍼)

- URDF: `robot/description/urdf/turtlebot3_real.urdf` (공식 visual STL + collision 근사, C++ Viz는 visual STL을 12mm 격자로 간소화해 렌더)
- 장착: 3층 판 맨 앞 중앙, `base_link` 기준 `x=+0.047985, y=-0.000230, z=+0.130175`
- OpenCR: joint1~4 = ID 11~14, gripper = ID 15 (`firmware/opencr/turtlebot_opencr/arm_config.h`)
- 토픽: `joint_states`에 팔 관절 포함, 명령은 `arm_joint_command` (`sensor_msgs/JointState`)
- `arm_torque`(Bool): 전체 토크 ON/OFF, `arm_stop`(Empty): 현재 자세 유지
- `arm_reset`(String): `all` / `joint1`..`joint4` / `gripper` — OpenCR `AR`로 해당 DYNAMIXEL 재부팅 (Hardware Error 해제)
- `arm_status`(String): `present=...;err=...;temp=...` — Viz Manipulator 패널에 에러가 빨간색으로 표시된다
- 손목 roll 관절은 없다. ID 16은 사용하지 않는다.

## 주행 / Multi Goal

- `goal_pose`(단일), `goal_poses`(multi): Navigtor가 A*로 경로를 만들고 `plan`으로 발행
- Multi Goal은 각 웨이포인트에 **위치 + yaw**를 저장한다. 중간 웨이포인트에 도착하면
  그 yaw로 제자리 회전해 방향을 맞춘 뒤 다음 구간으로 진행한다.
- `global_costmap/costmap`: A*용 inflated grid (2D/3D Viz에 표시)
- `local_costmap/costmap`: 로봇 주변 `local_costmap_size`(기본 4m) 창 (2D/3D Viz에 표시)
- 색은 RViz `costmap` 스킴: 1~98 blue→red, 99 cyan(inscribed), 100 purple(lethal), -1 blue-green-gray
- A*는 inflation cost를 약한 패널티로 사용한다.

### Multi-client

네트워크 서버는 최대 **8 클라이언트**를 동시에 받는다. 클라이언트마다 송신 큐/writer 스레드가
따로 있어 느린 Viz 하나가 다른 클라이언트의 센서 스트림을 막지 않는다. 명령 토픽은
**마지막 명령이 이기는** 구조이며, 클라이언트별 제어권 중재(lease)는 아직 없다.

## Viz 플러그인 / 프로젝트 Viz main

로봇 쪽 진입점이 `robot/main.cpp`라면, Viz 쪽 진입점은 `viz/main.cpp`다.

```text
krosc_ws/
├── robot/main.cpp            # 로봇(Pi) 엔드포인트
├── viz/
│   ├── main.cpp              # 프로젝트 Viz 엔드포인트
│   ├── CMakeLists.txt
│   └── plugins/
│       ├── project_panel.cpp # 프로젝트 플러그인 예제 ("Project" 탭)
│       └── moveit_plugin.cpp # MoveIt 도크 + MoveIt 탭

krosc/viz/
├── main.cpp                  # 공통/최소 Viz 앱
├── core/                     # VizWindow, SceneView, MapView2D, VizModel
├── plugins/                  # 코어 플러그인 (SLAM/Nav/Teleop/...)
└── include/krosc/viz/        # IVizPlugin, VizHost, VizPlugin, ...
```

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

프로젝트 Viz main에 등록:

```cpp
registry.add("my_plugin", [] { return std::make_unique<MyPlugin>(); });
```

CMake에 소스 추가 후 실행:

```bash
KROSC_VIZ_PLUGINS=project_panel,my_plugin ./run-viz.sh
```

- 플러그인은 프로젝트 안에서 자유롭게 작성/빌드한다.
- 코어 `krosc_viz_app`은 공통/최소 앱으로 남아 있고, `run-viz.sh`는 프로젝트 앱을 사용한다.
- MoveIt은 `krosc_ws/viz/plugins/moveit_plugin.cpp` 프로젝트 플러그인으로 분리되어 있다.
- 다음 단계는 Teleop/Manipulator, SLAM/Navigation/Displays의 코어 플러그인 이동과 프로필(YAML) 로딩이다.

## MoveIt-lite

Viz의 **MoveIt** 도킹 패널/탭은 `krosc_ws/viz/plugins/moveit_plugin.cpp` 플러그인이다.
코어 `krosc/viz`에는 MoveIt이 없고, 프로젝트 Viz 앱에서만 로드한다.
계산은 Pi의 `krosc::nodes::MoveItLite` 노드가 하고, Viz는 토픽 클라이언트로만 동작한다.

- 그룹: `base_link` → `end_effector_link`, 관절 `joint1`~`joint4`
- 목표: TCP `x,y,z` + 접근 `pitch` (4축이라 full 6D는 불가)
- `Plan`: IK → joint-space 또는 Cartesian 궤적 → 관절 한계/속도 검사
- `Execute`: 20Hz `arm_joint_command`, `Cancel`: `arm_stop`
- 3D: 목표 TCP 프레임, ghost arm, TCP 경로 표시
- 팔 Hardware Error가 있으면 Plan/Execute를 거부한다 (먼저 Reset)
- 토픽: `moveit_target`(String CSV), `moveit_plan`, `moveit_execute`, `moveit_cancel`, `moveit_scrub`
- 상태: `moveit_status`, `moveit_state`, `moveit_target_pose`, `moveit_arm`, `moveit_path`

핵심 계산은 `krosc/engine/core/manipulation/arm_chain.cpp`, 노드는
`krosc/engine/core/nodes/moveit_lite.cpp`에 있고 `krosc_ws/robot/main.cpp`에서 조립한다.
Viz 쪽은 `krosc_ws/viz/plugins/moveit_plugin.cpp`가 도크 + MoveIt 탭을 만든다.

## 구성

```
krosc_ws/
├── robot/                       # 로봇(Pi) 앱
│   ├── main.cpp                 # 50Hz 실시간 루프, 노드 조립, TCP 서버
│   ├── robot_bridge.cpp/.hpp    # cmd_vel, scan/imu/joint_states/diagnostics
│   ├── real_turtlebot3.cpp/.hpp # 이 로봇의 치수/장착 위치/시리얼 조립
│   ├── hardwares/
│   │   ├── serial.cpp/.hpp          # POSIX termios
│   │   ├── opencr_protocol.cpp/.hpp # C/S/I/E/M/F 파서 (순수 함수)
│   │   ├── opencr_serial.cpp/.hpp   # OpenCR 읽기 스레드 + 20Hz C/A 명령
│   │   └── lds03.cpp/.hpp           # LDS-03 패킷 파서 + 시리얼 + 몸통 필터
│   └── description/urdf/turtlebot3_real.urdf
├── viz/                         # PC Viz 앱 + 프로젝트 플러그인
│   ├── main.cpp
│   ├── CMakeLists.txt
│   └── plugins/
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

## 아직 안 된 것

- full MoveIt 2 / ros2_control 연동 (현재는 custom MoveIt-lite)
- 파라미터/서비스/액션의 네트워크 전달, rosbag
- Cartographer 전체 기능(loop closure, submap) — 현재는 scan matching + occupancy map 수준
- Smac/MPPI 등 nav2 고급 플래너·컨트롤러 (현재 A* + pure pursuit)

지금은 **실기 SLAM + A* 주행 + Multi Goal + MoveIt-lite 팔 계획/실행 + 원격 Viz**까지
`krosc_ws` 안에서 동작한다.
