# ros2_ws — Lyrical RL 실험 워크스페이스

시스템 노드 계통도 · TF · 토픽 · 장치: **[ARCH.md](ARCH.md)**

## 패키지

- `rl_correction/` — 이동 보정 RL 노드 (`corrector`: `cmd_vel_in` → `cmd_vel_out`)
- `nav2_rl_controller/` — Nav2 Controller단 보정 RL 노드 (`controller_corrector`: `cmd_vel_nav_in` → `cmd_vel_nav_out`)
- `rl_eval/` — Before/After 계측 (`eval_recorder` + `benchmark_onnx.py`)

## Before/After 방법 (remap 토글, 코드 수정 없음)

```bash
# Before (RL 없음): teleop -> cmd_vel (직결)
# After (RL 있음):  teleop -> cmd_vel_raw -> corrector -> cmd_vel
ros2 run rl_correction corrector --ros-args \
  -r cmd_vel_in:=cmd_vel_raw -r cmd_vel_out:=cmd_vel \
  -p enable:=true -p model_path:=/path/to/policy.onnx

# 계측 (label만 바꿔 2회 수집)
ros2 run rl_eval eval_recorder --ros-args -p output:=/tmp/before.csv -p label:=before
ros2 run rl_eval eval_recorder --ros-args -p output:=/tmp/after.csv -p label:=after
```

## Pi 탑재 전 체크 (필수)

```bash
# Pi 위에서 실행, 평균 50ms 이하 권장 (10-20Hz 루프)
python3 src/rl_eval/benchmark_onnx.py model.onnx --iters 1000
pip install onnxruntime numpy  # Pi에는 torch 풀설치 금지 (무거움)
```

## 빌드

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

## bringup (tb3_bringup)

명령 경로와 상태는 [ARCH.md 2장](ARCH.md#2-바퀴-명령-경로--state_manager--cmd_managermux): 모든 바퀴 명령은 **cmd_manager(mux)** 를 거치고,
무엇을 보낼지(MANUAL/AUTO · RL on/off · 비상정지)는 **state_manager** 의 `/state_*` 가 정합니다.

| launch | 어디서 | 내용 |
|---|---|---|
| `tb-only.launch.py` | Pi | state_manager · cmd_manager · OpenCR 드라이버 · robot_state_publisher · 라이다. RL off로 시작 |
| `tb-for-rl.launch.py` | Pi | tb-only를 RL on으로 시작 (PC의 rl-only가 있어야 움직임, 끊기면 정지) |
| `rl-only.launch.py` | PC | RL 노드만: corrector(`/cmd_vel_rl_in` → `/cmd_vel_rl_out`) + eval_recorder |
| `view-model.launch.py` | PC | URDF 확인 (robot_state_publisher + joint_state_publisher + RViz) |
| `basic_bringup.launch.py` | PC | OpenCR 없이 시험: state_manager · cmd_manager · mock_base (같은 명령 경로) |

```bash
# Pi
ros2 launch tb3_bringup tb-only.launch.py
# RL 켜기/끄기 (실행 중) — PC에서 rl-only를 먼저
ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=/path/policy.onnx   # PC
ros2 topic pub --once /cmd_manual_rl std_msgs/msg/Bool "{data: true}"             # 또는 tool: ./run-cli rl on
# 모드 · 비상정지
ros2 topic pub --once /cmd_manual_mode std_msgs/msg/String "{data: AUTO}"         # ./run-cli mode auto
ros2 topic pub --once /cmd_manual_estop std_msgs/msg/Bool "{data: true}"          # ./run-cli estop on
```

`opencr_driver` (`tb3_bringup/opencr_driver_node.py`): `/cmd_vel_base` → `C,l,r` (20Hz 재전송), `S` → `/odom` · TF `odom→base_footprint` ·
`/joint_states`(effort=전류 A) · `/diagnostics`(전류·온도·모터 에러). `E` 에러 시 `ros2 topic pub --once /opencr/reboot std_msgs/msg/Empty`.
파라미터: `config/robot.yaml` (바퀴 반지름·간격·좌우 방향 부호).

로봇 모델: `urdf/robot.urdf.xacro` — TB3 Waffle Pi 개조(맨 위 판 제거, 라이다 1층 앞). 실측 값은 파일 위쪽 property에.

## 기본 주행 (실기 없이 PC에서)

`tb3_bringup` 패키지: corrector(passthrough) + mock base + eval. LiDAR 없이 주행 체인 검증용.

```bash
# 터미널 1: bringup
source install/setup.bash
ros2 launch tb3_bringup basic_bringup.launch.py

# 터미널 2: 조종 (별도 실행, 키보드 입력 필요)
source /opt/ros/lyrical/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw

# 터미널 3: 확인
ros2 topic echo /odom --once
head /tmp/basic.csv
```

체인: teleop(`cmd_vel_raw`) → corrector → `cmd_vel` → mock_base → `odom`.
실기 전환: `mock_base`만 TurtleBot3 드라이버(OpenCR/Dynamixel)로 교체, 토픽 동일.
LiDAR 장착 후: `hls_lfcd_lds_driver` + `slam_toolbox` 추가 예정.
