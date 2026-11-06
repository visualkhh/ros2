# ros2_ws — 로봇 bringup + Nav2 RL 워크스페이스

TurtleBot3 Waffle Pi 개조 로봇 (Raspberry Pi 4 · ROS 2 Lyrical). 노드 계통도 · 명령 경로 · TF · 토픽 · 장치: **[ARCH.md](ARCH.md)**

## 패키지

| 패키지 | 내용 |
|---|---|
| `tb3_bringup` | 로봇 bringup. `state_manager`(상태) · `cmd_manager`(바퀴 명령 mux) · `opencr_driver`(OpenCR 시리얼) · `scan_resample`(LDS-03 고정 칸) · `explore_bridge`(자동 맵핑 ↔ 제어 상태) · launch 5개 · URDF |
| `rl_nav_controller` | Nav2 `FollowPath` 플러그인 — RL 정책(`isaac/training/rl_maze`) / inner controller(DWB) 바이패스 |
| `rl_policy` | ONNX Runtime C++ 래퍼 (`rl_nav_controller`가 사용) |
| `coin_d4_driver` (외부) | LDS-03 라이다 드라이버 (ROBOTIS) — 아래 `deps.repos`로 받음 |
| `explore_lite` · `explore_lite_msgs` (외부) | 자동 맵핑 frontier 탐색 (m-explore-ros2) — 아래 `deps.repos`로 받음 |
| `cartographer` · `cartographer_ros` (외부) | SLAM — 아래 `deps.repos` + `patches/`로 소스 빌드 |

SLAM(cartographer, 소스 빌드)·Nav2(apt)는 켜고 끄는 건 [`tool/run-cli`](../tool/README.md)가 합니다 (토픽이 보이는 곳 어디서나).

## deps.repos — apt에 없는 외부 소스

[`deps.repos`](deps.repos)는 **apt로 설치할 수 없는 ROS 패키지의 소스 목록**입니다 ([vcstool](https://github.com/dirk-thomas/vcstool) 형식).
`vcs import`가 적힌 git 저장소를 `src/` 아래로 받고, 우리 워크스페이스에서 **Lyrical로 같이 빌드**합니다.

| 저장소 | 왜 | 버전 |
|---|---|---|
| `coin_d4_driver` (ROBOTIS) | LDS-03 라이다 드라이버. Lyrical apt에 없음 (TurtleBot3 `LDS_MODEL=LDS-03`과 같은 드라이버) | 커밋 `79d1833` 고정 |
| `m-explore-ros2` (robo-friends, BSD) | 자동 맵핑 `explore_lite`. Lyrical apt에 없음. 딸림 `multirobot_map_merge`는 `image_geometry` apt + `patches/m-explore-ros2-ament-target-deps.patch` 필요 → `setup/pi/build.sh`가 둘 다 처리 | 커밋 `326cf8a` 고정 |
| `cartographer` | SLAM 코어. Lyrical apt에 없음. ceres 2 대응 패치 `patches/cartographer-ceres2.patch`. CMake 4.x 빌드는 `--cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5` 필수 | 커밋 `877157a` 고정 |
| `cartographer_ros` | SLAM ROS 2 래퍼. 패치 `patches/cartographer_ros-ceres2.patch` + `patches/cartographer_ros-gflags-unique.patch` + `patches/cartographer_ros-absl-deadlock.patch` (같은 `--cmake-args` 공유) | 커밋 `7433452` 고정 |

- `version`은 ROS 배포판이 아니라 **그 저장소의 git 커밋/브랜치**입니다. ROBOTIS는 `humble`·`jazzy`·`main` 브랜치만 있고
  lyrical 브랜치가 없어서, 코드가 같은 최신(jazzy 끝)을 커밋으로 고정했습니다. 순수 rclcpp 코드라 Lyrical에서 빌드됩니다.
- 받은 폴더(`src/coin_d4_driver/` · `src/m-explore-ros2/` · `src/cartographer/` · `src/cartographer_ros/`)는 `.gitignore` — 우리 저장소엔 `deps.repos` 한 파일만 올라갑니다.
- coin_d4_driver가 없어도 빌드·실행은 됩니다 (`tb-only`가 라이다만 빼고 뜸, 경고 출력). m-explore-ros2가 없으면 tb3_bringup이 `explore_lite_msgs`를 못 찾아 빌드는 되지만 자동 맵핑(`explore-nav`)이 안 뜸.

## 빌드

**Pi**: 한 줄 — 외부 소스 받기 + cartographer 패치 + 빌드 (처음이든 `git pull` 뒤든 같은 명령, 몇 번 돌려도 같은 결과):

```bash
~/ros2/setup/pi/build.sh                                  # 전부
~/ros2/setup/pi/build.sh --packages-select tb3_bringup    # 뒤 인자는 colcon build로
```

**PC**: 외부 소스(라이다·explore·cartographer)는 Pi 전용이라 필요 없음 — 우리 패키지만:

```bash
cd ~/ros2/ros2_ws && source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select tb3_bringup rl_policy rl_nav_controller
```

설정·launch·URDF 파일을 **새로 추가**했는데 설치 폴더에 안 보이면: `rm -rf build/tb3_bringup install/tb3_bringup` 후 다시 빌드.

## bringup (tb3_bringup)

외부 요청은 **cmd_manager**가 받아 운행 상태(MANUAL · NAV_PLAN · SLAM_AUTO_MAPPING · 비상정지)를 바꾸고 바퀴 명령을 고르며,
**state_manager**는 현재 상태를 `/state_*`로 알립니다 — [ARCH.md 2장](ARCH.md#2-운행-상태--바퀴-명령-경로--cmd_manager--state_manager). 바꾸는 법(토픽·명령·메뉴): [STATE_CONTROL.md](STATE_CONTROL.md).

| launch | 어디서 | 내용 |
|---|---|---|
| `tb-only.launch.py` | Pi | state_manager · cmd_manager · OpenCR 드라이버 · robot_state_publisher · 라이다(+몸통 필터·고정 칸) |
| `explore-nav.launch.py` | Pi | SLAM 중 Nav — Nav2 `navigation_launch.py` + lifecycle_manager (Lyrical은 manager가 `bringup_launch.py`에만 있음) + explore_lite(frontier 탐색) + explore_bridge(제어 상태 ↔ 탐색). `run-cli explore start`·자동 맵핑이 씀 |
| `cartographer.launch.py` | Pi | SLAM — cartographer_node + occupancy_grid_node (`config/cartographer_2d.lua`). `run-cli slam start`가 씀 |
| `view-model.launch.py` | PC | URDF 확인 (robot_state_publisher + joint_state_publisher + RViz) |

보통은 `tool/run-cli base start`가 `tb-only`를 띄웁니다. 직접(포그라운드) 띄우려면 — 스크립트가 ROS 환경·domain 25를 맞춘다:

```bash
./base-only.sh                     # Pi — = ros2 launch tb3_bringup tb-only.launch.py
./base-only.sh lidar:=false        # 라이다 없이
```

외부에서 보내는 명령 (`/cmd_manual_*` → state_manager / cmd_manager):

| 토픽 | 타입 | tool | 효과 |
|---|---|---|---|
| `/cmd_manual_vel` | Twist | `teleop` · `drive` | MANUAL일 때 바퀴로 |
| `/cmd_manual_control` | String `MANUAL`·`NAV_PLAN`·`SLAM_AUTO_MAPPING` | `control …` · `automap start/stop` | 제어 상태 (RViz Nav2 Goal → NAV_PLAN, 자동 맵핑 끝 → MANUAL) |
| `/cmd_manual_estop` | Bool | `estop on/off` | 비상정지 |
| `/opencr/reboot` | Empty | `reboot-motors` | 모터 에러(E) 해제 |

상태는 `/state_control` · `/state_estop` (latched) 로 나옵니다.

`opencr_driver`: `/cmd_vel_base` → `C,l,r` (20 Hz 재전송), `S` → `/odom` · TF `odom→base_footprint` · `/joint_states`(effort=전류 A) ·
`/diagnostics`(전류·온도·모터 에러) · `/battery_state`(S의 배터리 V), `I` → `/imu`(frame `imu_link`). 파라미터 `config/robot.yaml` (바퀴 반지름 0.033 · 간격 0.287 · 방향 부호 -1/-1).
라이다: `config/lidar.yaml` (`/scan_raw` → 몸통 상자 필터 → `/scan_filtered` → 400칸 고정 → `/scan`).
로봇 모델: `urdf/robot.urdf.xacro` — 실측 값은 파일 위쪽 property에 (`view-model.launch.py`로 확인).

## Nav2 RL controller

`FollowPath` 플러그인(`rl_nav_controller::RlNavController`)이 학습 정책(ONNX)으로 직접 Twist를 낸다.
`FollowPath.enable=false`(기본)면 inner controller(DWB)로 바이패스한다.

```bash
# 켜기/끄기 (controller_server가 도는 중)
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}"
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: false}"

# 상태 — RL/DWB, 모델 로드, 스캔, fallback 이름
./run-cli node-status /controller_server
```

- 모델: `isaac/training/rl_maze` play → `policy.onnx` + `policy.onnx.data` → `ros2_ws/src/tb3_bringup/config/nav2_params.yaml`의 `FollowPath.model_path`
- fallback: `FollowPath.fallback_controller` (`dwb` 코드 기본 · 이 로봇 설정도 `dwb`). `inner.*` 파라미터 블록을 선택한 플러그인에 맞출 것.
- 상태 토픽: `/controller_server/FollowPath/status` — `[rl_active, reason, policy_loaded, enable, scan_ok, scan_age, scan_min, v, w, inner_kind]`
- ONNX Runtime C++: `sudo apt install ros-lyrical-onnxruntime-vendor` → `colcon build --packages-select rl_policy rl_nav_controller`
- 구 `rl_correction`/`rl_eval`/`rl-only`/`route rl` 경로는 제거됨.
