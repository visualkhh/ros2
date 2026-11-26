# krosc simulator

시뮬레이터 코어는 **월드와 로봇을 직접 만들지 않는다**. 호출자가 만든 `World`와
`SimulatorRobot`을 받아서 고정 주기로 돌리고, 화면이 읽을 스냅샷만 만든다.

```
krosc/simulator/
  core/       World/SimulatorRobot 런타임, SimulationRunner, Snapshot, Qt truth view
  worlds/     바로 쓰는 월드 예제 (MazeWorld, RoomWorld)
  objects/    바로 쓰는 오브젝트 예제 (Puck, BoxObstacle, CylinderObstacle)
  robots/     바로 쓰는 로봇 예제 (TurtleBot3 + URDF)
  apps/demo/  krosc_simulator 데모 앱. RobotBridge/MapPublisher 같은 ROS 접착제는 여기에만 있다.
```

## 코어에 넣는 것

- `World`: 벽/오브젝트/레이 충돌/물리 스텝
- `WorldObject`: 월드 위의 물체
- `SimulatorRobot`: `WorldObject` + `state()` (명령, 조인트, 진실 라이다)
- `Simulator`: `World&` + `SimulatorRobot&`를 받아 `SimulationRunner`와 `SnapshotStore`를 연결

ROS 토픽, URDF, map publisher, `cmd_vel` 브리지는 코어에 없다. 각 앱이 자기
하드웨어 스텁/브리지를 만들어 로봇 스택과 연결한다.

## 사용 예

```cpp
MyWorld world;                                  // World 상속
auto robot_holder = std::make_unique<MyRobot>();// SimulatorRobot 상속
auto* robot = static_cast<MyRobot*>(world.add(std::move(robot_holder)));

krosc::simulator::Simulator simulator(context, world, *robot);
simulator.start();

auto snapshot = simulator.snapshots().latest(); // 화면/텔레메트리용
```

`krosc_ws` 시뮬레이터는 `MazeWorld` + `krosc_ws::sim::WafflePiArm`을 쓰고,
실제 로봇과 같은 `SimulatorOpenCr`/`SimulatorLds` 하드웨어 스텁을 통해 `Robot`에 연결한다.
시뮬레이터용 URDF는 `krosc_ws/simulator/robot/urdf/turtlebot3_real.urdf`에
의도적으로 복제해 두고, 실기 쪽 `krosc_ws/engine/description`과 분리한다.

노이즈는 하드웨어 스텁이 아니라 `TurtleBot3`/`WafflePiArm` 모델이 생성한다
(바퀴 슬립, 엔코더 양자화, IMU 바이어스, 라이다 거리/드롭아웃). 스텁은 센서값을
실기 장치처럼 전달하고, truth view는 잡음 없는 스냅샷을 사용한다.

## Qt truth view

- `W/A/S/D`: 주행, `Q/Z`: 속도, `F`: follow, `L`: 라이다 표시 토글
- `C`: 카메라 초기화, 드래그: 회전, 휠: 줌

패널은 `SimulatorWindow`에 주입한다. 시뮬레이터 코어는 Qt를 모른다.

```cpp
auto* arm_panel = new ArmPanel(robot);  // 앱에서 만든 QWidget
SimulatorWindow window(world.walls(), simulator.snapshots(), node,
                 "cmd_vel", {arm_panel});
```
