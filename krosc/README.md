# krosc

`kros`(순수 파이썬 ROS 클론)를 C++20으로 다시 만든 것. 목표는 같다: **ROS와 같은 이름/사용법**을 유지하면서,
파이썬 구현의 스레드/복사 비용을 없애 더 빠르고 예측 가능하게 만드는 것.

| 패키지 | 내용 |
|---|---|
| `core/` | ROS 프레임워크: Node, 토픽+QoS, TF, 타이머, 파라미터, 서비스, 메시지, URDF, 기본 노드, **PeerNetwork 토픽 mesh(9000)** |
| `sim/` | **진실 시뮬레이션**: MazeWorld, TurtleBot3(슬립/잡음), SimulationRunner, Snapshot, **Qt SimWindow** |
| `viz/` | **토픽 뷰어**: rviz 스타일 Qt 화면 (TF, map, costmap, scan, odom/filtered, 경로, 로그, 패널, teleop, **URDF visual STL 메시**) + **원격 `--server` 모드** |
| `viz-web/` | **브라우저 Viz**: 로봇과 `PeerNetwork`로 토픽을 주고받고 HTTP+WebSocket으로 브라우저에 넘기는 경량 웹 게이트웨이. `www/plugins/*.js`로 패널을 붙인다 |
| `simulator/` | **시뮬레이터 앱/코어**(스캔 10Hz): MazeWorld + TurtleBot3 + ROS 브리지 + EKF를 Context에 조립하고, `Simulator`가 Runner/스냅샷을 소유한다 (`--server`로 원격 Viz) |
| `../krosc_ws/` | **실기 Pi 워크스페이스**: OpenCR/LDS-03 C++ 드라이버, EKF, 원격 viz 서버 (`krosc`를 링크) |

![krosc sim(truth) + viz, noVNC로 본 리눅스 데스크톱](docs/vnc.png)

*왼쪽 `SimWindow`(진실), 오른쪽 `VizWindow`(토픽). 이 화면은 Docker의 Ubuntu 26.04 안에서 두 Qt 창을 띄우고,
맥 브라우저(noVNC)로 그대로 본 것이다.*

원격 모드 검증 (같은 TCP 경로, 로컬 서버 + 원격 viz): `docs/remote-viz.png`  
실기 C++ vs Python 성능 비교: [`docs/performance-vs-python.md`](docs/performance-vs-python.md)

## 맥에서 UI 보기 (Docker + noVNC)

Docker Desktop(macOS)에는 X 서버가 없다. 그래서 컨테이너 안에서 **Xvfb(가상 디스플레이) + openbox + x11vnc + noVNC**를
띄우고, 맥 브라우저가 그 리눅스 데스크톱을 WebSocket으로 본다.

```bash
cd /Users/hyunhakim/workspace/visualkhh/source/ros2

./docker/build.sh          # Ubuntu 26.04 + GCC + CMake + Qt6 + noVNC (처음 한 번)
./docker/vnc.sh            # 컨테이너 안에서 sim + viz 두 창을 띄운다
```

브라우저에서:

```
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

- sim/viz/데이터는 **전부 컨테이너의 리눅스 안**에서 돈다. 맥은 화면(WebSocket 6080)만 본다.
- 왼쪽 `krosc sim (truth)` 창에서 `W/A/S/D`로 직접 로봇을 몰 수 있고, 오른쪽 `krosc viz` 창에서도 조종할 수 있다.
- `Ctrl+C`로 `docker/vnc.sh`를 끝내면 컨테이너도 정리된다.
- noVNC는 로컬 개발용으로 **비밀번호 없이**(`-nopw`) 띄운다. `6080`을 외부에 공개하지 마라.

`docker/README.md`에 같은 내용과 XQuartz 대안이 있다.

## 웹 Viz

```bash
./docker/web.sh
# 브라우저: http://localhost:8080/
```

![krosc web: 지도/스캔/플러그인 패널](docs/web.png)

실기(Pi)에서는 두 프로세스로 띄운다:

```bash
./krosc_real --port 9000                    # 로봇 앱 (PeerNetwork listen)
./krosc_viz_web --robot 127.0.0.1:9000 --port 8080
# krosc_ws 워크스페이스에서는 프로젝트 플러그인이 붙은 krosc_ws_viz_web을 쓴다 (run-viz-web.sh)
```

- Viz/VizWeb은 자기 `Context`/`net::Client`를 만들지 않는다. app main이 `Krosc`를 만들고
  `Krosc::context()`를 넘긴다. 원격 연결은 `PeerNetwork`가 토픽/서비스/액션/파라미터
  RPC 단위로 처리한다.
- 웹 게이트웨이의 브라우저 프레임 ↔ 토픽 브리지는 `krosc/viz-web/core/context_link.cpp`에 있다.
  코어(`engine/`)는 브라우저 프레임이나 웹 구독 정책을 모른다.
- `main.cpp`는 인자만 `VizWebOptions`로 넘기고, `krosc::viz_web::VizWeb`이 ContextLink/HttpServer/
  시그널/종료를 소유한다. 브라우저에는 `4바이트 길이 + 본문` 프레임을 WebSocket 바이너리로 넘긴다.
- 웹으로 내보낼 토픽은 `VizWebOptions::forward_topics`로 정한다. 기본은 코어 웹 앱 토픽이고,
  프로젝트 main/플러그인이 자기 토픽(`moveit_*`, `rl_nav_*`)을 추가한다.
- 브라우저는 `www/app.js`에서 프레임을 해석해 지도/스캔/TF/경로를 Canvas에 그린다.
- 명령(지도 load/save/delete, 파라미터, MoveIt goal)은 텍스트 JSON RPC(`host.rpc`)로 보낸다.
  `krosc/viz-web/core/web_rpc.cpp`가 로컬/원격 서비스로 바꾸고, 프로젝트가 `web.rpc().add(...)`로
  자기 RPC를 붙인다.
- 웹 플러그인은 main이 `VizWebPluginRegistry`에 등록하고, 게이트웨이가 `/plugins.json`을 **동적으로** 만든다.
  코어 앱은 SLAM/Navigation/Teleop/Diagnostics를 기본으로 하고, 프로젝트 앱(`krosc_ws/viz-web`)이
  MoveIt/RL Nav를 추가한다. JS 모듈은 `www/plugins/*.js`다.
- 늦게 접속한 브라우저를 위해 지도/상태/모델 목록 등 최신 프레임을 캐시했다가 새 연결에 보낸다.

## 빌드/테스트 (Docker, 화면 없이)

```bash
cd /Users/hyunhakim/workspace/visualkhh/source/ros2
./docker/build.sh
./docker/compile.sh RelWithDebInfo --clean
```

`docker/compile.sh`는 `ros2/` 전체를 컨테이너 `/workspace`에 마운트하고 `/workspace/krosc`에서
GCC + CMake + Ninja로 빌드한 뒤 `ctest`를 돌린다.

```bash
# 오프스크린 스크린샷 (Docker, GUI 없이)
docker compose -f docker/docker-compose.yml run --rm krosc bash -lc \
  "QT_QPA_PLATFORM=offscreen ./build/simulator/krosc_simulator \
   --screenshot /workspace/krosc/docs/screenshot.png \
   --screenshot-sim /workspace/krosc/docs/simulation.png \
   --duration 4 --headless"
```

## 로컬 GUI (macOS/Linux, Qt6 설치된 경우)

```bash
cd krosc
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
./build/simulator/krosc_simulator
```

`SimWindow`(진실)와 `VizWindow`(토픽) 두 창이 같이 뜬다. Docker 없이 바로 조종할 때 이 방법을 쓴다.

## 데모

```
MazeWorld(진실) ─ TurtleBot3(바퀴 슬립/IMU 잡음/라이다 잡음, seed 고정)
        │ tick 100Hz
        ├─ RobotBridge: scan/imu/joint_states 발행, cmd_vel 수신
        ├─ DiffDriveOdometry: joint_states -> odom (TF는 끔)
        ├─ Ekf: odom(vx, vyaw) + imu(vyaw) -> odometry/filtered + odom -> base_footprint TF
        ├─ RobotStatePublisher: URDF -> robot_description/tf_static/관절 tf
        ├─ MapPublisher: MazeWorld 벽 -> map(latch), map -> odom 정적 TF
        ├─ SnapshotRecorder: 진실 상태(로봇/퍽/라이다)를 불변 스냅샷으로 -> SimWindow
        ├─ SimWindow: 월드/로봇/퍽/진실 라이다를 3D 투영으로 그린다 (QPainter)
        └─ VizWindow: tf/tf_static, robot_description, map, scan, odom, filtered, 로그를 구독해 그린다
```

두 화면의 차이가 핵심이다: `SimWindow`는 **진실**(시뮬레이션 내부), `VizWindow`는 **로봇이 토픽으로 내보낸 추정**을 보여 준다.

| 입력 | 동작 |
|---|---|
| `W/A/S/D` 또는 방향키 | 조종 (sim 창은 로봇 직접, viz 창은 `cmd_vel` 발행) |
| `Q` / `Z` | 조종 속도 ×1.25 / ÷1.25 |
| `F` | 로봇 추적 on/off |
| `C` / `R` | 카메라 초기화 |
| 왼쪽 드래그 | 궤도 회전 |
| 오른쪽 드래그 | 평행 이동 |
| 휠 | 줌 |
| `Esc` | 창 닫기 |

## 구조

```
krosc/
├── CMakeLists.txt                 # 최상위 (engine, viz, simulator, viz-web)
├── engine/                        # krosc_core (표준 라이브러리만)
│   ├── CMakeLists.txt
│   ├── include/krosc/             # node/context/topic/tf/time/parameter/service/action/msgs/net(rpc)
│   │   ├── nodes/                 # DiffDriveOdometry, Ekf, robot_state_publisher, TfEcho, TwistMux, MapServer/MapSaver, nav2
│   │   ├── control/               # controller_manager, JSB/JTC/forward/diff_drive
│   │   ├── sensor/                # Sensor/SensorManager (read-only 센서 수명주기)
│   │   └── moveit/                # kinematics/planner/controller_manager 플러그인 + move_group
│   ├── core/                      # 라이브러리 소스
│   ├── tests/                     # core 62 tests (nav2/control/sensor/MoveIt/net/RPC 포함)
│   └── benchmarks/                # bench.cpp, nav2_bench.cpp, net_bench.cpp
├── simulator/                     # krosc_simulator (+ krosc_simulator_qt)
│   ├── CMakeLists.txt
│   ├── main.cpp                   # krosc_simulator 실행기 (Krosc + Simulator + 창)
│   ├── include/krosc/simulator/    # Simulator, world, maze_world, turtlebot3, runner, snapshot, view_window
│   ├── core/                      # Simulator/월드/ 브리지/러너 소스
│   └── tests/                     # simulator 4 tests
├── viz/                           # krosc_viz (Qt6 Widgets)
│   ├── CMakeLists.txt
│   ├── main.cpp                   # krosc_viz_app
│   ├── include/krosc/viz/         # VizModel, SceneView, VizWindow, plugin API
│   ├── core/                      # Viz shell/렌더 소스
│   └── plugins/                   # 코어 Viz 플러그인
├── viz-web/                       # krosc_viz_web (HTTP + WebSocket, Qt 없음)
│   ├── main.cpp                   # krosc_viz_web 실행기 (파라미터 → VizWeb)
│   ├── include/krosc/viz_web/     # VizWeb, HttpServer, WebSocket
│   ├── core/                      # VizWeb/게이트웨이 소스
│   └── www/                       # index.html/app.js/style + JS 플러그인
└── docs/                          # 검증 스크린샷 (vnc.png, screenshot.png, simulation.png)
```

Docker 설정은 저장소 루트의 `../docker/`에 있다.

### 의존성

- `core`: C++20 표준 라이브러리 + pthread (Eigen/numpy 없음)
- `sim`: `core`의 메시지 타입 + pthread. **Qt가 없어도 물리/테스트는 빌드된다.**
- `sim_qt`: `sim` + Qt6 Widgets (`SimWindow`)
- `viz`: `core` + Qt6 Widgets (`VizWindow`)
- `viz_web`: 표준 라이브러리 + pthread만 (Qt/Node 없음). `VizWeb`이 로봇 TCP 프레임을 WebSocket으로 중계
- `examples`: `core` + `sim_qt` + `viz`

## Peer 네트워크 (다른 Krosc와 토픽 공유)

`host`(로컬 listen 주소)와 `servers`(원격 Krosc 목록)로 peer mesh를 켠다.
`host`와 `servers`가 모두 비면 네트워크를 만들지 않고 **local-only**로 동작한다.

```cpp
krosc::net::PeerNetworkOptions network;
network.name = "robot_a";
network.host = "0.0.0.0";   // 이 주소로 listen. 비우면 incoming 없음
network.port = 9100;        // 0이면 임시 포트
network.add_server("192.168.0.12", 9100);  // 여러 개 추가 가능
// network.servers = {"192.168.0.12:9100", "192.168.0.13:9100"};

krosc::Krosc app([](krosc::Context& context) {
  // ...
  return std::vector<std::unique_ptr<krosc::Node>>{};
}, network);
```

`Context::Options::network`로도 같은 설정을 할 수 있다. `Krosc(create, {"host:port"}, port)`는
간단한 문자열 목록 버전이다.

- 로컬 토픽 publish가 자동으로 peer에게 전송된다.
- peer에서 온 토픽은 로컬 토픽으로 다시 발행된다 (재브로드캐스트 방지).
- 상대가 구독한 토픽만 보낸다 (`subscribe` 필터).
- latched 토픽은 나중에 구독한 peer에게도 재전송한다.
- 현재는 **토픽 mesh 1차**다. 서비스/액션은 같은 peer 연결 위에 다음 단계로 올린다.
  C++ 액션은 서비스+토픽으로 구현되어 있어 서비스 포워딩이 들어가면 함께 동작할 수 있다.

## API 맛보기

### 노드, 토픽, 타이머, 파라미터

```cpp
#include "krosc/krosc.hpp"

class Talker : public krosc::Node {
 public:
  Talker(krosc::Context& context) : Node(context, "Talker") {
    rate_ = declare_parameter<double>("rate", 10.0, [](double hz) { return hz > 0.0; });
    timer_ = create_timer(1.0 / rate_.value(), [this] {
      krosc::msgs::String message;
      message.data = "hello " + std::to_string(count_++);
      publish("chatter", std::move(message));
    });
  }

 private:
  krosc::Parameter<double> rate_;
  krosc::Timer timer_;
  int count_ = 0;
};
```

### TF

```cpp
krosc::Buffer buffer;
krosc::TransformListener listener(buffer, node);

krosc::msgs::TransformStamped tf;
tf.header.stamp = node.now();
tf.header.frame_id = "odom";
tf.child_frame_id = "base_footprint";
tf.transform.translation = {x, y, 0.0};
tf.transform.rotation = krosc::tfmath::from_yaw(yaw);
broadcaster.send_transform(tf);

const auto result = buffer.lookup_transform("map", "base_scan");
```

### 서비스

```cpp
auto server = node.create_service<krosc::msgs::SetBoolRequest, krosc::msgs::SetBoolResponse>(
    "enable", [](const auto& request) {
      return krosc::msgs::SetBoolResponse{request.data, "ok"};
    });
const auto response = node.call_service<krosc::msgs::SetBoolRequest, krosc::msgs::SetBoolResponse>(
    "enable", {true});
```

### 시뮬레이션 + 진실 화면

```cpp
krosc::Krosc app(nullptr, network);  // network를 주면 Viz/VizWeb이 PeerNetwork로 붙는다
krosc::simulator::Simulator simulator(
    app.context(), {.noisy = true, .pucks = 10, .seed = 42,
                    .urdf_path = "simulator/description/turtlebot3_waffle_pi.urdf"});

krosc::simulator::SimWindow sim_window(
    simulator.world().walls(), simulator.robot(), simulator.snapshots());
sim_window.show();
simulator.start();
```

Simulator는 Viz/VizWeb과 같이 **Context를 밖에서 받는다**. 같은 Context의 토픽/서비스/RPC를
그대로 쓰므로 시뮬레이터·Viz·웹이 같은 통신 방식을 공유한다.

## 최적화 포인트

1. **메시지 복사 제거** — 발행은 `std::shared_ptr<const T>` 하나를 구독자 큐에 넣는다.
2. **구독 목록 스냅샷** — 발행마다 구독자 vector를 복사하지 않는다 (구독/해제 때만 copy-on-write).
3. **스레드 하나/구독 하나 제거** — 기본 inline 실행기(결정적), 실시간은 공용 워커 풀. 구독 하나는 drain 작업 하나로 순서 유지.
4. **타이머 스레드 하나** — 실제 시간은 스케줄 스레드 1개, 시뮬레이션 시간은 `set()`을 부른 스레드가 즉시 판정.
5. **컴파일 타임 타입** — 토픽 타입은 C++ 템플릿으로 고정. 런타임 타입 검사/딕셔너리 룩업 없음.
6. **TF 이력** — 좌표계별 정렬 vector + `lower_bound` 보간, 조건 변수 timeout.
7. **라이다** — AABB slab 교차를 C++로 직접 계산 (360광선 x 20벽 ~60–70µs).
8. **스냅샷 경계** — 시뮬레이션 스레드는 `SnapshotStore`에 불변 스냅샷만 넣고, Qt 스레드는 그것만 읽는다 (락 경합 최소).

측정값 (`./build/engine/krosc_bench`, Ubuntu 26.04 컨테이너, 대략값):

```
pub/sub inline      :   ~200 ns/op   (콜백 포함, 100만 회)
pub/sub pool(4)     :   ~400 ns/op   (워커 4개, 20만 회 모두 전달)
tf lookup           :   ~0.6 us/op   (300개 이력, 보간)
lidar 360 rays      :  60-70 us/scan (20개 벽)
```

nav2 알고리즘 비교는 `krosc_nav2_bench`(C++)로 돌린다.
결과와 해석은 [`docs/performance-vs-python.md`](docs/performance-vs-python.md) 9장에 있다(당시 Python 비교 기록).

## 테스트

```bash
ctest --test-dir build --output-on-failure
# core 62 + sim 4 = 66 tests, 2 executables
```

- core: 토픽/타입 잠금/latch/depth, 실행기, 타이머(시뮬/실시간), 파라미터, TF(보간/순환/timeout), URDF, 서비스, 액션, RSP, odometry, twist_mux, map_server/map_saver
- nav2: costmap 레이어/inflation, Navfn·Theta*·Smac2D·SmacHybrid, RPP·DWB·MPPI·RotationShim, planner/controller/behavior server, bt_navigator (계획·주행·취소)
- control: controller_manager(수명주기/서비스/command interface 점유), joint_state_broadcaster, joint_trajectory_controller, forward_command_controller, diff_drive_controller
- sensor: Sensor lifecycle(init/activate/update/deactivate/finish/error), SensorManager(주기 스케줄러 + Executor 워커 풀 기본 10, 실패 롤백/집계)
- MoveIt: kinematics/planner/controller_manager 플러그인, move_group(move_action + move_group/scrub + 상태 토픽), 실제 OpenMANIPULATOR URDF IK/궤적 end-to-end
- RPC: 서비스/액션/파라미터 포워딩(discovery, timeout, latched), 웹 JSON RPC
- sim: 레이/직진, scan/imu/odom/filtered 발행 + EKF, SimulationRunner
- ASan/UBSan으로 core/sim 테스트와 데모 실행 검증 완료 (오류/누수 없음)

네트워크 benchmark:

```bash
./build/engine/krosc_net_bench   # loopback 토픽 RTT/대역폭/fanout/RPC
```

현재 Mac loopback 기준: 토픽 RTT p50 ~0.11ms, 1KiB payload ~80k msg/s,
3 peer fanout ~114k deliveries/s, RPC p50 ~0.06ms. Pi에서는 별도 측정이 필요하다.

## RPC (PeerNetwork)

토픽은 기존 바이너리 프레임으로 흐르고, 서비스/액션/파라미터는 같은 TCP 연결 위에 RPC 프레임으로
올라간다.

- 서비스: `kFrameRpcRequest`/`kFrameRpcResponse` + `RpcEnvelope(request_id, type, payload, error)`.
  로컬 서버가 없으면 `PeerNetwork`가 원격 피어로 보내고, 응답을 request_id로 맞춘다 (기본 5초 timeout).
- discovery: 서비스 생성/삭제 시 `__services` control로 이름 목록을 광고한다. 아직 광고 전이면
  연결된 피어에 broadcast하고, 응답이 없으면 timeout.
- 타입 코덱: `engine/core/rpc.cpp`. `std_msgs`, `std_srvs`, nav2 map 서비스, `rcl_interfaces`
  파라미터, `control` 컨트롤러 서비스, 액션 goal/feedback/result를 등록한다.
- 액션: 기존 `ActionServer`/`ActionClient` API 그대로. `send_goal`/`cancel_goal` 서비스와
  `/_action/event` 이벤트를 PeerNetwork가 인코딩한다. `register_action_codec()`로 프로젝트 액션을 추가한다.
- 파라미터: 모든 `Node`가 `<node>/get_parameters`, `set_parameters`, `set_parameters_atomically`,
  `list_parameters`, `describe_parameters` 서비스를 만든다.

## 플러그인 구조

nav2/control/MoveIt은 **가상 베이스 클래스 + 알고리즘별 파생 클래스 + 이름→팩토리 레지스트리**로 되어 있다.
`builtin_*()` 함수는 알고리즘이 아니라 "등록"만 한다.

| 영역 | 베이스 | 구현 (파일 한 세트씩) |
|---|---|---|
| nav2 controller | `NavControllerPlugin` (`nodes/nav2/controllers/controller.hpp`) | `RegulatedPurePursuitController`, `DWBLocalPlanner`, `MPPIController`, `RotationShimController` |
| nav2 planner | `NavPlannerPlugin` (`nodes/nav2/planners/global_planner.hpp`) | `NavfnPlanner`, `ThetaStarPlanner`, `SmacPlanner2D`, `SmacPlannerHybrid` |
| control hardware | `SystemInterface` (`control/hardware_interface.hpp`) | `OpenCrSystem` (krosc_ws), `MockArmSystem` (test) |
| control controller | `ControllerInterface` (`control/controller_interface.hpp`) | `JointStateBroadcaster`, `JointTrajectoryController`, `ForwardCommandController`, `DiffDriveController` |
| MoveIt kinematics | `MoveItKinematicsPlugin` (`moveit/kinematics/kinematics_base.hpp`) | `ArmChainKinematics` |
| MoveIt planner | `MoveItPlannerPlugin` (`moveit/planners/planner_manager.hpp`) | `CartesianPlanner`, `JointSpacePlanner` |
| MoveIt controller manager | `MoveItControllerManagerPlugin` (`moveit/controllers/controller_manager.hpp`) | `SimpleControllerManager` |

새 알고리즘은 베이스 클래스를 상속해 `compute_velocity_commands`/`create_plan`/`plan`/`read`/`write`를
오버라이드하고, `builtin_*()`에 팩토리를 한 줄 추가하면 된다.

플러그인의 `configure(Node&, name)`에서 첫 인자는 서버 타입이 아니라 **parent Node**다
(nav2의 `configure(LifecycleNode::WeakPtr parent, ...)`와 같은 역할). `ControllerServer`는
자기 자신을 `Node&`로 넘기므로 플러그인은 파라미터/토픽/TF만 쓰고 서버 내부에는 의존하지 않는다.
primary controller를 만드는 플러그인만 `set_registry()`로 정적 registry를 받는다 (pluginlib 대체).

```text
engine/include/krosc/nodes/nav2/
├── controllers/   controller.hpp, rpp/dwb/mppi/rotation_shim 플러그인, controller_plugins.hpp
├── planners/      global_planner.hpp, navfn/theta*/smac2d/smac_hybrid 플러그인
├── costmap/       costmap.hpp, costmap_2d_ros.hpp
├── controller_server.hpp, planner_server.hpp, behavior_server.hpp, bt_navigator.hpp
engine/include/krosc/nodes/moveit/
├── kinematics/    kinematics_base.hpp, arm_chain_kinematics.hpp
├── planners/      planner_manager.hpp, cartesian/joint_space 플러그인
└── controllers/   controller_manager.hpp, simple_controller_manager.hpp
engine/include/krosc/nodes/moveit/
└── move_group.hpp  # 노드는 nodes 아래
```

소스도 같은 구조다 (`engine/core/nodes/nav2/controllers`, `planners`, `costmap`,
`engine/core/nodes/moveit/move_group.cpp`, `engine/core/nodes/moveit/kinematics`, `planners`, `controllers`).

ROS 2와 맞춘 노드 이름:

`robot_state_publisher`, `ekf_filter_node`, `cartographer_node`, `map_server`, `map_saver`,
`map_catalog`(krosc 전용), `twist_mux`, `velocity_smoother`, `collision_monitor`,
`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, `controller_manager`, `move_group`.

## 지금 된 것 / 다음

**된 것**

- [x] Node/Context/Krosc, 토픽 + QoS + latch, 실행기 (inline/thread pool)
- [x] 타이머 (실제/시뮬레이션 시간), 파라미터(+이벤트), 서비스
- [x] TF2 스타일 Buffer/Listener/Broadcaster, 변환 수학, 정적 TF
- [x] 메시지: TF, Odometry, LaserScan, Imu, JointState, OccupancyGrid, Path, DiagnosticArray, Log 등
- [x] URDF 파서 (링크/관절/visual/collision, xacro 인자 치환)
- [x] DiffDriveOdometry, Ekf, RobotStatePublisher, TfEcho, TwistMux
- [x] `sim` 패키지: MazeWorld, TurtleBot3(슬립/잡음), SimulationRunner, SnapshotRecorder
- [x] `sim` Qt `SimWindow`: 진실 월드/퍽/로봇/잡음 없는 라이다 (QPainter 3D)
- [x] `viz` Qt `VizWindow`: 격자, TF 축, URDF, 라이다, odom/filtered, map/costmap, 경로, 궤적, 로그
- [x] `viz` 코어 플러그인: `teleop`, `slam`, `navigation`, `manipulator` (VizWindow는 기반 UI만)
- [x] TCP 원격 토픽(9000): 텔레메트리 + `cmd_vel`, latched 다중 tf_static, Mac `krosc_viz_app --server`
- [x] PeerNetwork RPC: 서비스/액션/파라미터 포워딩 + discovery/timeout + 웹 JSON RPC
- [x] Peer mesh(`Context::Options::network`): 다른 Krosc와 임의 토픽 discover/subscribe + latched 공유 (서비스/액션은 다음 단계)
- [x] noVNC: Docker Ubuntu 26.04 안의 sim+viz 두 창을 맥 브라우저로 본다
- [x] `viz-web`: HTTP + WebSocket 게이트웨이, 동적 `/plugins.json`, 지도/스캔/TF/경로/상태 뷰
- [x] 오프스크린 스크린샷 검증 (`docs/`)
- [x] `../krosc_ws/`: OpenCR/LDS-03 C++ 드라이버 + EKF + 원격 viz를 붙인 실기 Pi용 워크스페이스 (하드웨어 실측은 Pi에서 해야 함)

**다음 후보 / 진행 상황** (파이썬 kros에 이미 있는 것 + 이번에 추가)

- [x] 액션 (`ActionServer`/`ActionClient`)
- [ ] `DiagnosticUpdater`/`DiagnosticAggregator`
- [ ] MapServer/MapSaver (yaml + pgm), Cartographer 스타일 SLAM
- [x] nav2: costmap(`Costmap2DROS`), planner/controller 플러그인, `planner_server`/`controller_server`, `bt_navigator`, `behavior_server`
- [x] control: `controller_manager`, JSB/JTC/forward controller, command interface 점유와 `switch_controller`
- [x] MoveIt: kinematics/planner/controller_manager 플러그인 + `move_group` (`move_action`, 기존 `moveit_*` 호환)
- [ ] 진짜 하드웨어: OpenCR 시리얼, LDS-03 시리얼
- [ ] 네트워크 심화: 파라미터/서비스/액션, 다중 클라이언트, 보안, rosbag 기록/재생
- [ ] viz: 2D Goal Pose/초기 자세 추정, 마우스로 지도 편집, STL 메시 렌더링
- [x] `krosc_ws`: Cartographer SLAM, nav2 스택(Navfn/RPP 기본, Smac/MPPI는 파라미터 교체), collision_monitor, 지도 저장/불러오기

## 메모

- 기존 파이썬 `../kros/`와는 별개다. 이름만 `krosc`로 구분한다.
- noVNC는 로컬 개발용이다. 포트 6080을 밖에 열지 마라.
- `viz`는 **토픽만** 본다. 진실은 `sim`의 `SimWindow`가 그린다. 같은 화면에서 바퀴 오도메트리/EKF가 진실에서
  얼마나 벗어나는지 비교할 수 있다.
- `.cpp` 스타일은 ROS 2 C++와 비슷하게 맞췄다: `Node::create_timer`, `publish`, `subscribe`, `declare_parameter`,
  `lookup_transform`, `send_transform`.
