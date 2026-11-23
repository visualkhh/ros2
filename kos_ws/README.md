# kos_ws — 실기 TurtleBot3 (kros, 시뮬레이터·Viz 없음)

라즈베리파이에서 OpenCR + LDS-03를 직접 돌리는 kros 앱입니다. 화면(Panda3D)도 월드도 없고,
PC의 **Viz**가 TCP(9000)로 붙어서 로봇을 봅니다. 컨트롤러는 **MPPI**입니다 (RL 정책 없음).

## 준비 (라즈베리파이)

```bash
# udev 고정 이름 (한 번)
~/ros2/documents/raspberry/udev/install-udev.sh      # /dev/opencr, /dev/tb3_lidar

# ROS 2 스택이 시리얼 포트를 잡고 있으면 먼저 내린다 (kros와 동시에 못 씀)
~/ros2/tools/run-cli stop-all                        # 없으면 해당 launch/노드 종료

# 확인
ls -l /dev/opencr /dev/tb3_lidar
```

venv는 없어도 됩니다 (`run.sh`가 kos_ws/.venv → kros/.venv → python3 순서로 고릅니다).
`kros` core venv를 만들려면 `./kos_ws/setting-venv.sh` (python3-venv 패키지 필요).

## 실행 (라즈베리파이)

```bash
cd ~/ros2
./kos_ws/run.sh                                    # 0.0.0.0:9000
KROS_SERVER_PORT=0 ./kos_ws/run.sh                 # 네트워크 없이
OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar ./kos_ws/run.sh
```

- 시작 로그에 `host mode: port 9000`과 접속 주소가 나옵니다.
- OpenCR IMU는 켜고 **2초 동안 가만히** (자이로 편향 보정).
- Ctrl+C / SIGTERM → `C,0,0` 정지 후 시리얼을 닫고 종료.
- 펌웨어 watchdog: 500ms 동안 `C`가 없으면 바퀴 자동 정지. kros 쪽도 0.5초 동안 새 `cmd_vel`이 없으면 마지막 속도를 버리고 0을 보냅니다.

## PC에서 보기

```bash
cd /home/pank/workspaces/visualkhh/source/ros2
./kros/.venv/bin/python kros/examples/turtlebot3/viz_remote.py <라즈베리파이-IP> 9000
# URDF 메시가 없는 PC라면 package_root를 직접:
# ... viz_remote.py <ip> 9000 /path/to/turtlebot3/description
```

## Viz 조작

| 입력 | 동작 |
|---|---|
| 좌클릭 드래그 | 화면 회전 |
| 우클릭 드래그 / 휠 | 팬 / 줌 |
| **W/A/S/D 또는 화살표** | 텔레옵 (`cmd_vel_teleop` → twist_mux가 nav보다 우선) |
| **start SLAM** | Cartographer mode=mapping (파라미터가 네트워크로 전달) |
| **save map** | map_saver/save_map (서비스가 네트워크로 전달, Pi의 `artifacts/maps/<이름>/`) |
| 지도 메뉴 + **load** | `map_change` 토픽으로 그 지도 로드 + localization |
| **delete** | 빈 `map_change` → 지금 띄운 지도 폴더를 삭제 |
| **unload** | 지도만 내림 (파일 유지) |
| 2D Pose Estimate / 2D Goal Pose / Multi Goal / Cancel | rviz처럼 (목표는 localization일 때만) |

## kros에만 있는 지도 토픽 (ROS에는 없음)

- `map_list` (`std_msgs/String`, latch): 지도 이름들을 줄바꿈으로. 폴더가 바뀌면 1초 안에 갱신.
- `map_change` (`std_msgs/String`): 이름이 있으면 그 지도로 변경, **빈 문자열이면 지금 띄운 지도 삭제**.
- `map_status` (`std_msgs/String`, latch): 마지막 결과 (`loaded ...`, `deleted ...`, `no map loaded`).

## 구성

- `main.py`: 실시간 50Hz 루프 + kros 노드 (RobotBridge, TF, EKF, Cartographer, map_server/saver/catalog,
  Smac, MPPI, behavior_server, bt_navigator, twist_mux, collision_monitor)
- `robot.py`: `RealTurtleBot3` — `OpenCrSerial`(방향 -1/-1, 7.8 rad/s 제한) + `SerialLds03`(230400, 몸통 필터)
- `urdf/turtlebot3_real.urdf`: LDS-03 1층 앞(x=0.060, z=0.022), OpenCR IMU 위치
- `robot_bridge.py`, `hardware_status.py`, `topics.py`: cmd_vel/scan/imu/joint_states + 진단
