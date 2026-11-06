# rl_nav_controller — nav2 controller 플러그인 (학습한 주행 정책 + 바이패스)

`rl_nav_controller::RlNavController : nav2_core::Controller`. nav2 planner가 준 경로를 isaac/training/rl_maze에서 학습한 정책으로 따라간다.
안에 기본 controller(DWB)를 품고 있어서 **toggle 끔(기본) = DWB 그대로 · 켬 = 정책**.

```bash
# 정책 켬/끔 (nav 도는 중에)
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}"    # 정책
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: false}"   # DWB (바이패스)
```

상태는 매 제어 주기 `/controller_server/FollowPath/status`로도 나온다.

```text
[rl_active, reason, policy_loaded, enable, scan_ok, scan_age, scan_min, v, w, inner_kind]
reason: 0=RL · 1=모델 없음/로드 실패 · 2=enable=false · 3=/scan 없음 · 4=추론 실패
inner_kind: 1=DWB · 2=MPPI · 3=RPP · 4=RotationShim · 0=기타
```

`./run-cli` → `8) 노드` → `controller_server`에서 같은 내용을 볼 수 있다.

## 설정 (tb3_bringup/config/nav2_params.yaml · controller_server.FollowPath)

| 파라미터 | 기본 | |
|---|---|---|
| `plugin` | `rl_nav_controller::RlNavController` | |
| `enable` | `false` | 시작 때 정책 적용 |
| `model_path` | `""` | `policy.onnx` — 비우면 늘 DWB (켜려 해도 거부) |
| `fallback_controller` | `dwb` (코드 기본 · 이 설정도 `dwb`) | 바이패스 controller: `dwb`·`mppi`·`rpp`·`rotation_shim` 또는 전체 pluginlib 클래스명 · 설정은 `FollowPath.inner.*` |
| `inner_plugin` | (예전 이름) | `fallback_controller`가 비었을 때만 사용하는 호환 파라미터 |
| `max_lin` · `max_ang` · `v_min_ratio` | 0.26 · 1.82 · -0.2 | 정책 [v, w] 스케일 = 학습 env |
| `lookahead` · `wp_step` · `wp_scale` | 5 · 0.30 · 2.0 | 웨이포인트 관측 = 학습 env |
| `lidar_rays` · `lidar_max` | 36 · 3.5 | 라이다 관측 = 학습 env |
| `lidar_inflation` · `inflation_scale` | 0.22 · 0.50 | LiDAR 안전거리 [m] (사용자 입력, 학습 범위 0.20~0.24) · `inflation_scale`은 정규화 상수 (사용자 값 아님) |
| `scan_topic` · `imu_topic` | `scan` · `imu` | |

관측은 **52차원** (`[웨이포인트 10, 라이다 36, v/w/gyro 3, 이전 행동 2, inflation 1]`).
`scan_rays()`가 `r - lidar_inflation`을 하고, obs 마지막에 `lidar_inflation / inflation_scale`을 넣는다.
`lidar_inflation`은 `ros2 param set /controller_server FollowPath.lidar_inflation 0.24`로 런타임 변경 가능.
51차원 모델이면 `lidar_inflation: 0`으로 두면 inflation 입력 없이 기존처럼 동작한다.

관측 · 행동 약속은 `src/rl_nav_controller.cpp` 맨 위 주석 = isaac/training/rl_maze/env.py (바꾸면 같이).

## 정책 파일

```bash
cd isaac
./run-training.sh rl_maze --play --checkpoint artifacts/rl_maze/training/<시각>/model_N.pt --num_envs 1
#   → exported/policy.onnx + policy.onnx.data — 둘 다 nav2가 도는 곳에 복사하고 model_path에 policy.onnx
```

ONNX Runtime C++(`rl_policy` 패키지 · `ros-lyrical-onnxruntime-vendor`)로 그대로 읽는다.

## 빌드 (nav2가 도는 곳 = Pi)

```bash
sudo apt install ros-lyrical-onnxruntime-vendor            # 한 번 (ONNX Runtime C++)
cd ~/ros2/ros2_ws && colcon build --packages-select rl_policy rl_nav_controller && source install/setup.bash
```
nav2_params.yaml이 이 플러그인을 쓰므로, **빌드 안 하면 nav가 시작하지 않는다** (FollowPath 플러그인 없음).
