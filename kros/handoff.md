# kros 인수인계 (handoff)

새 세션은 이 문서와 `README.md`, `TODO.md`만 읽으면 이어서 할 수 있게 적었다. (작성 시점 마지막 커밋: `12cde75`)

## 1. 프로젝트

**kros** = 순수 파이썬으로 만드는 ROS 클론. 목표는 "만들면서 학습하기". ROS(rclpy, tf2_ros, robot_state_publisher, robot_localization 등)와 **이름과 사용법을 최대한 똑같이** 만든다.

- 위치: `/home/pank/workspaces/visualkhh/source/ros2/kros` (git 루트는 `.../source/ros2`).
- **푸시한 적 없음.** 모든 작업은 로컬 커밋뿐이다. 커밋은 kros 경로만 `git add` 한다 (`isaac/` 등 다른 폴더의 변경은 건드리지 않는다).
- 커밋 메시지는 한글, 끝에 `Co-Authored-By: Claude ...` 줄을 붙인다.
- 사용자는 한국어로 대화한다. 설명은 짧게, 결론부터. 사용자는 "ROS랑 똑같이", "왜 여기 있어?" 같은 구조/이름 지적을 자주 하니 위치와 이름을 먼저 ROS 기준으로 생각한다.

## 2. 명령어

모든 명령은 `kros/` 안에서 실행한다 (python은 `python`이 아니라 venv의 것).

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/kros

./test.sh                          # 전체 테스트 + mypy --strict (현재 470개 OK, mypy 184 파일 clean)
./test.sh -v -k odometry           # 특정 테스트만 (이때 mypy는 건너뜀)
./examples/turtlebot3/run.sh       # 시뮬레이터 실행: Viewer(진실) + Viz(rviz 같은 창) 두 창

# 한 줄 스크립트는 ros2/ (kros의 부모)에서, PYTHONPATH를 줘서:
cd .. && PYTHONPATH=. kros/.venv/bin/python -m unittest kros.tests.test_viz
```

- 3D 화면은 `window-type offscreen`으로 스크린샷을 찍어 눈으로 확인할 수 있다: `win.saveScreenshot(Filename(...))` 후 Read로 이미지를 본다 (`tests/test_viz.py` 참고).
- 불안정한(flaky) 테스트 확인은 같은 테스트를 6~15번 반복해서 본다. 새 로직은 일부러 망가뜨려서 테스트가 실패하는지(뮤테이션)도 본다.
- 알려진 드문 flake: `test_diagnostics_period_and_timeout_are_live_parameters` (200번 중 2번). 손대지 않았다.

## 3. 구조 (kros/ 아래)

의존 방향: **현실 쪽은 kros(Node)를 모른다.** 이 경계는 `tests/test_maze_world.py`가 검사한다.

| 폴더 | 내용 |
|---|---|
| `nodes/` | kros가 제공하는 노드. `Node`(기반 클래스), `RobotStatePublisher`, `TfEcho`, `DiagnosticAggregator`, `DiffDriveOdometry` |
| `tf/` | `Buffer`, `TransformListener`, `TransformBroadcaster`, `StaticTransformBroadcaster`, `view_frames`, `transform_math` (tf2_ros와 같은 API, 시각은 float 초) |
| `topic/ service/ action/ parameter/ timer/ log/ diagnostics/ urdf/` | 코어 도구들 (순수 파이썬, 표준 라이브러리만) |
| `msgs/` | geometry_msgs, sensor_msgs(Imu, LaserScan, JointState), nav_msgs(Odometry), tf2_msgs, std_msgs ... |
| `hardwares/` | **현실 쪽, 월드를 모른다 (Clock만 받는다).** `Hardware/Sensor/Actuator`, `controllers/`(OpenCr, DiffDrive, DiffDriveBoard), `imus/`, `lidars/`, `motors/`, `noise.py`(Noise) |
| `worlds/` | 시뮬레이션. `World`, `objects/`(WorldObject, Puck), `maze/`(MazeWorld, Track), `robots/turtlebot3/`(TurtleBot3, 진짜 `opencr_serial/`) |
| `interactions/keyboards/` | 키보드 입력 장치 |
| `viewer/` | Panda3D **진실** 화면 (Viewer, SnapshotRecorder, UrdfModel, `physical_keys.py`) |
| `viz/` | rviz 같은 화면. `Viz(kros, package_root, base=None)`가 토픽만 구독해 그린다 |
| `examples/turtlebot3/` | `main.py`(조립), `robot_bridge.py`(cmd_vel, scan, imu, joint_states, 진단), `run.sh` |

panda3d를 import해도 되는 곳은 `viewer/`와 `viz/`뿐이다.

## 4. 지금까지 한 것 (이번 대화 구간)

- **TF를 tf2_ros와 똑같이**: `Buffer`, `lookup_transform`, `can_transform`, `transform`, `TransformListener(buffer, node)` 등.
- **구조 정리**: `Track` 자체가 WorldObject, `Node`는 `kros/nodes/`, `WorldObject`는 `worlds/objects/`, `hardwares`는 월드 밖, 키보드는 `interactions/keyboards/`.
- **하드웨어**: IMU(MPU-9250), OpenCR 보드(`OpenCr`: 구동부+IMU), Hardware는 월드를 모르고 Clock만 받는다.
- **진짜 OpenCR 시리얼**: `OpenCrSerial` (펌웨어 `firmware/opencr/tb3_wheels`의 ASCII 프로토콜, 115200). TurtleBot3가 `serial_port`가 있으면 진짜 보드, 없으면 시뮬레이션으로 **스스로 조립**한다. 매니퓰레이터 값(`nan`)은 "연결 안 됨"으로 처리.
- **`DiffDriveOdometry` 노드**: `joint_states` -> `odom` 토픽 + `odom->base_footprint` TF. 점프(1 rad 초과)는 버린다.
- **`viz`**: 두 번째 창(`openWindow`). 격자, TF 축/부모 선, URDF 모델(TF로 배치), 라이다 점, odom/imu 글자. 왼쪽 패널에 구독 토픽 목록(타입, 개수, Hz)과 `DirectCheckButton` 체크박스(grid/tf/모델/scan/imu/odom 켜기/끄기). 기준 프레임 `odom`.
- **하드웨어 오차/지연** (시뮬레이션 하드웨어 안에서만): `Noise(sigma, bias, scale)`, 바퀴 슬립+엔코더 해상도, IMU 잡음/편향/방향 드리프트, 라이다 잡음/측정 실패, 센서 `latency`/`jitter`. `TurtleBot3(noisy=True, seed=...)`가 현실적인 값을 준다 (기본은 오차 0). 예제 `main.py`는 `noisy=True`.
- **키 입력**: 한글 배열에서도 되도록 X11 물리 키를 읽는다 (`viewer/physical_keys.py`). `Viz`가 ShowBase의 `window-event`를 덮어쓰던 버그를 고쳤다.

## 5. 앞으로 할 것

### 5-1. 다음 작업: `Ekf` 노드 (robot_localization의 ekf_node)

사용자가 "ekf도 해야 odom이 의미 있다"고 확정했다. 순서:

1. **메시지에 공분산 추가**: `Odometry`(pose/twist 6x6), `Imu`(orientation/angular_velocity/linear_acceleration 3x3). 지금은 공분산이 없다 (`nav_msgs/odometry.py`, `sensor_msgs/imu.py`).
2. **시뮬레이션 하드웨어가 공분산을 채운다**: `noise.py`의 오차 크기에서 값을 정한다 (바퀴는 슬립이 커서 믿음을 낮게, 자이로는 편향이 있다). 바퀴 오도메트리 쪽은 `DiffDriveOdometry`가 채운다.
3. **`kros/nodes/ekf.py`에 `Ekf(Node)`**:
   - 상태 `[x, y, yaw, v, w]` (2D, `two_d_mode`), 일정 속도 모델 + 프로세스 노이즈, `frequency`마다 예측.
   - 입력: `odom0`(바퀴 v, w), `imu0`(자이로 z, 옵션 yaw). `odom0_config`/`imu0_config`로 쓸 값 선택, 마할라노비스 거리로 이상값 버림.
   - 출력: `odometry/filtered` 토픽, `publish_tf`가 켜지면 `odom->base_footprint` TF. 이때 `DiffDriveOdometry`는 `publish_tf=False`로 끈다 (이 파라미터는 이미 있다).
   - 코어는 표준 라이브러리만 쓰므로 5x5 행렬 연산을 직접 짠다 (numpy 사용 안 함).
4. **테스트**: 노이즈 낀 바퀴+IMU를 합치면 바퀴만 쓸 때보다 진짜 위치(`robot.pose`)에 더 가까운지 비교. 오차 수치가 `seed`로 고정되므로 재현 가능하다.
5. 예제 `main.py` 연결, viz에서 바퀴 오도메트리와 EKF 결과를 같이 보게 하기 (사용자가 Viewer(진실)와 Viz를 나란히 보며 odom이 튀는 걸 직접 비교한다).

### 5-2. 그 밖의 TODO (`TODO.md`에 더 자세히, 끝낸 항목은 지워 두었다)

- 진짜 하드웨어를 도는 **실시간 루프 러너** (월드 없이 `TurtleBot3(serial_port=...)`에 Clock을 내려준다). 진짜 OpenCR의 R/T/B 명령, 바퀴 방향 부호, 진짜 라이다(LDS-03) 구현.
- 모터(엔코더) 값의 지연, 송신 주기 흔들림. 오차 수치를 진짜 값에 맞춰 보정.
- viz 확장: IMU 화살표, odom 궤적, 목표 지정, `scan_frame`을 토픽 header에서 읽기(지금은 `LaserScan`에 header가 없어 파라미터), 패널 스크롤, `TransformListener`에 수신 카운트 훅(지금 viz가 tf를 두 번 구독).
- viz 조종 키: viz 창에 포커스가 있을 때도 먹게 할지.
- URDF 파서와 `viewer/urdf_model.py` 중복 정리, `Clock`을 `kros/clock.py`로 옮길지, 시뮬레이션 `OpenCr`을 `examples/turtlebot3`로 옮길지.
- 네트워크(허브 방식), bag(기록/재생), QoS 확장 등은 보류 중 (`TODO.md`).

## 6. 새 세션 시작 방법

1. 컨텍스트 읽기:
   ```bash
   cd /home/pank/workspaces/visualkhh/source/ros2/kros
   cat handoff.md                 # 이 문서
   git log --oneline | head -20   # 최근 흐름
   ./test.sh                      # 현재 상태 확인 (470 OK + mypy clean이어야 한다)
   ```
2. 새 세션에 줄 첫 지시 예:
   > `kros/handoff.md`를 읽고 5-1의 `Ekf` 노드를 1번(메시지 공분산)부터 이어서 진행해 줘. 한 단계가 끝날 때마다 테스트와 mypy를 돌리고 kros 경로만 커밋해 줘.
3. 작업 방식(이 프로젝트에서 지켜 온 것):
   - 새 로직마다 테스트를 쓰고, `./test.sh`(mypy strict 포함)가 통과해야 커밋한다.
   - 구조/이름은 ROS와 같게. 사용자가 위치를 지적하면 그 자리가 맞는지부터 따진다.
   - 현실 쪽(`hardwares/ worlds/ viewer/`)은 kros의 `Node/Context/topic`을 import하지 않는다. 순환 import는 `TYPE_CHECKING`으로 푼다.
   - README와 TODO를 같이 갱신한다 (구현한 건 `[x]`).
   - 눈으로 봐야 하는 화면 변경은 오프스크린 스크린샷으로 직접 확인하고, 확인하지 못한 것은 그렇다고 말한다.
   - 푸시는 사용자가 시킬 때만 한다.

## 7. 알아 두면 좋은 주의점

- `ShowBase`는 프로세스에 하나라서 Viz는 Viewer의 `base`에 두 번째 창을 연다 (`Viz(kros, robot.description_dir, base=viewer)`). 전역 `taskMgr`를 공유하므로 destroy된 Viewer의 작업이 남지 않게 `_update`가 스스로 끝낸다.
- 시뮬레이션 시간: 러너가 매 스텝 `SimulatedTimeSource`를 밀어 준다. 테스트에서 타이머를 쓰면 스텝마다 `timer_manager.join()`이 필요하다.
- TF 콜백 안에서는 timeout을 주고 기다리지 않는다 (데드락).
- 시뮬레이션에서 로봇이 벽에 막혀도 바퀴 모터는 계속 돈다고 센다 (오도메트리가 벽을 뚫은 것으로 추정). 진짜 슬립과 같은 종류의 오차이고, EKF/슬립 모델링으로 다룰 부분이다.
- python이 없다는 오류가 나면 `kros/.venv/bin/python`을 쓴다. venv가 없으면 `./setting-venv.sh`(프로필 `sim`)로 만든다.
