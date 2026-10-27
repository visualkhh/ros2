# ros2_ws — Lyrical RL 실험 워크스페이스

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
