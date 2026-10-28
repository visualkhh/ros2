# ros2_ws — 로봇 bringup + RL 보정 워크스페이스

TurtleBot3 Waffle Pi 개조 로봇 (Raspberry Pi 4 · ROS 2 Lyrical). 노드 계통도 · 명령 경로 · TF · 토픽 · 장치: **[ARCH.md](ARCH.md)**

## 패키지

| 패키지 | 내용 |
|---|---|
| `tb3_bringup` | 로봇 bringup. `state_manager`(상태) · `cmd_manager`(바퀴 명령 mux) · `opencr_driver`(OpenCR 시리얼) · `scan_resample`(LDS-03 고정 칸) · `frontier_explorer`(자동 맵핑) · launch 3개 · URDF |
| `rl_correction` | `corrector` — `cmd_vel_in` → (ONNX 정책) → clip → `cmd_vel_out`. rl-only가 `/cmd_vel_rl_in` → `/cmd_vel_rl_out`으로 띄움 |
| `rl_eval` | `eval_recorder` — 보정 전/후 cmd·odom을 CSV로 · `benchmark_onnx.py` (Pi 추론 속도) |
| `nav2_rl_controller` | `controller_corrector` — Nav2 컨트롤러 출력(`cmd_vel_nav`) 단계 보정. **아직 launch에 연결 안 함** |
| `coin_d4_driver` (외부) | LDS-03 라이다 드라이버 (ROBOTIS) — 아래 `deps.repos`로 받음 |

SLAM(slam_toolbox)·Nav2는 apt 패키지이고, 켜고 끄는 건 [`tool/run-cli`](../tool/README.md)가 합니다 (토픽이 보이는 곳 어디서나).

## deps.repos — apt에 없는 외부 소스

[`deps.repos`](deps.repos)는 **apt로 설치할 수 없는 ROS 패키지의 소스 목록**입니다 ([vcstool](https://github.com/dirk-thomas/vcstool) 형식).
`vcs import`가 적힌 git 저장소를 `src/` 아래로 받고, 우리 워크스페이스에서 **Lyrical로 같이 빌드**합니다.

| 저장소 | 왜 | 버전 |
|---|---|---|
| `coin_d4_driver` (ROBOTIS) | LDS-03 라이다 드라이버. Lyrical apt에 없음 (TurtleBot3 `LDS_MODEL=LDS-03`과 같은 드라이버) | 커밋 `79d1833` 고정 |

- `version`은 ROS 배포판이 아니라 **그 저장소의 git 커밋/브랜치**입니다. ROBOTIS는 `humble`·`jazzy`·`main` 브랜치만 있고
  lyrical 브랜치가 없어서, 코드가 같은 최신(jazzy 끝)을 커밋으로 고정했습니다. 순수 rclcpp 코드라 Lyrical에서 빌드됩니다.
- 받은 폴더(`src/coin_d4_driver/`)는 `.gitignore` — 우리 저장소엔 `deps.repos` 한 파일만 올라갑니다.
- 없어도 빌드·실행은 됩니다 (`tb-only`가 라이다만 빼고 뜸, 경고 출력).

## 빌드

```bash
cd ~/ros2/ros2_ws                       # PC: ~/workspaces/visualkhh/source/ros2/ros2_ws
source /opt/ros/lyrical/setup.bash
vcs import src < deps.repos             # 최초 1회 (외부 소스 받기)
colcon build --symlink-install          # Pi 4는 --parallel-workers 2 (메모리)
```

설정·launch·URDF 파일을 **새로 추가**했는데 설치 폴더에 안 보이면: `rm -rf build/tb3_bringup install/tb3_bringup` 후 다시 빌드.

## bringup (tb3_bringup)

외부 요청은 **cmd_manager**가 받아 운행 상태(MANUAL · NAV_PLAN · SLAM_AUTO_MAPPING · 경로 직결/RL · 비상정지)를 바꾸고 바퀴 명령을 고르며,
**state_manager**는 현재 상태를 `/state_*`로 알립니다 — [ARCH.md 2장](ARCH.md#2-바퀴-명령-경로--state_manager--cmd_managermux).

| launch | 어디서 | 내용 |
|---|---|---|
| `tb-only.launch.py` | Pi | state_manager · cmd_manager · OpenCR 드라이버 · robot_state_publisher · 라이다(+몸통 필터·고정 칸). RL 노드는 없음 — RL off로 시작 (`rl:=true`면 on으로, 실행 중엔 `./run-cli rl on`) |
| `rl-only.launch.py` | PC | RL 노드만: corrector(`/cmd_vel_rl_in` → `/cmd_vel_rl_out`) + eval_recorder |
| `view-model.launch.py` | PC | URDF 확인 (robot_state_publisher + joint_state_publisher + RViz) |

보통은 `tool/run-cli base start`가 `tb-only`를 띄웁니다. 직접 띄우려면:

```bash
ros2 launch tb3_bringup tb-only.launch.py                     # Pi
ros2 launch tb3_bringup tb-only.launch.py lidar:=false        # 라이다 없이
```

외부에서 보내는 명령 (`/cmd_manual_*` → state_manager / cmd_manager):

| 토픽 | 타입 | tool | 효과 |
|---|---|---|---|
| `/cmd_manual_vel` | Twist | `teleop` · `drive` | MANUAL일 때 바퀴로 |
| `/cmd_manual_control` | String `MANUAL`·`NAV_PLAN`·`SLAM_AUTO_MAPPING` | `control …` · `automap start/stop` | 제어 상태 (RViz Nav2 Goal → NAV_PLAN, 자동 맵핑 끝 → MANUAL) |
| `/cmd_manual_rl` | Bool | `route direct/rl` | 경로: 직결 / PC RL 경유 (실행 중 전환) |
| `/cmd_manual_estop` | Bool | `estop on/off` | 비상정지 |
| `/opencr/reboot` | Empty | `reboot-motors` | 모터 에러(E) 해제 |

상태는 `/state_control` · `/state_rl` · `/state_estop` (latched) 로 나옵니다.

`opencr_driver`: `/cmd_vel_base` → `C,l,r` (20 Hz 재전송), `S` → `/odom` · TF `odom→base_footprint` · `/joint_states`(effort=전류 A) ·
`/diagnostics`(전류·온도·모터 에러). 파라미터 `config/robot.yaml` (바퀴 반지름 0.033 · 간격 0.287 · 방향 부호 -1/-1).
라이다: `config/lidar.yaml` (`/scan_raw` → 몸통 상자 필터 → `/scan_filtered` → 400칸 고정 → `/scan`).
로봇 모델: `urdf/robot.urdf.xacro` — 실측 값은 파일 위쪽 property에 (`view-model.launch.py`로 확인).

## RL 보정 Before / After

RL은 bringup을 바꾸지 않고 **실행 중에** 켜고 끕니다. 같은 경로·같은 계측으로 두 번 수집해서 비교합니다.

```bash
# PC — RL 노드 (켤 때만 필요)
ros2 launch tb3_bringup rl-only.launch.py enable:=false label:=before output:=/tmp/before.csv   # 보정 없이 통과
./run-cli rl on                 # 바퀴 명령이 PC를 거치게
#   ... 같은 경로 주행 (teleop 또는 goal) ...
# Ctrl+C 후 정책을 켜서 다시
ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=/path/policy.onnx label:=after output:=/tmp/after.csv
#   ... 같은 경로 주행 ...
./run-cli rl off                # 끝나면 직결로
```

- `enable:=false`는 보정 없이 통과 — WiFi 왕복 지연까지 같은 조건으로 "Before"를 잽니다.
- RL on인데 rl-only가 없거나 WiFi가 끊기면 0.3초 뒤 정지 (cmd_manager 안전).

Pi에서 직접 추론할 경우 (탑재 전 체크):

```bash
python3 src/rl_eval/benchmark_onnx.py model.onnx --iters 1000   # 평균 50 ms 이하 권장 (10-20 Hz 루프)
pip install --break-system-packages onnxruntime                  # Pi에는 torch 설치 금지 (무거움)
```
