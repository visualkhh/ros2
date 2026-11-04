# rl_nav_controller — nav2 controller 플러그인 (학습한 주행 정책 + 바이패스)

`rl_nav_controller::RlNavController : nav2_core::Controller`. nav2 planner가 준 경로를 isaac/training/rl_maze에서 학습한 정책으로 따라간다.
안에 기본 controller(MPPI)를 품고 있어서 **toggle 끔(기본) = MPPI 그대로 · 켬 = 정책**.

```bash
# 정책 켬/끔 (nav 도는 중에)
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}"    # 정책
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: false}"   # MPPI (바이패스)
```

## 설정 (tool/config/nav2_params.yaml · controller_server.FollowPath)

| 파라미터 | 기본 | |
|---|---|---|
| `plugin` | `rl_nav_controller::RlNavController` | |
| `enable` | `false` | 시작 때 정책 적용 |
| `model_path` | `""` | `policy.onnx` — 비우면 늘 MPPI (켜려 해도 거부) |
| `inner_plugin` | `nav2_mppi_controller::MPPIController` | 바이패스 controller · 설정은 `FollowPath.inner.*` |
| `max_lin` · `max_ang` · `v_min_ratio` | 0.26 · 1.82 · -0.2 | 정책 [v, w] 스케일 = 학습 env |
| `lookahead` · `wp_step` · `wp_scale` | 5 · 0.30 · 2.0 | 웨이포인트 관측 = 학습 env |
| `lidar_rays` · `lidar_max` | 36 · 3.5 | 라이다 관측 = 학습 env |
| `scan_topic` · `imu_topic` | `scan` · `imu` | |

관측 · 행동 약속은 `src/rl_nav_controller.cpp` 맨 위 주석 = isaac/training/rl_maze/env.py (바꾸면 같이).

## 정책 파일

```bash
cd isaac
./run-training.sh rl_maze --play --checkpoint artifacts/rl_maze/training/<시각>/model_N.pt --num_envs 1
#   → exported/policy.onnx + policy.onnx.data — 둘 다 nav2가 도는 곳에 복사하고 model_path에 policy.onnx
```

ONNX Runtime C++(`rl_policy` 패키지 · `ros-lyrical-onnxruntime-vendor`)로 그대로 읽는다 — corrector(rl_correction)와 같은 코드.

## 빌드 (nav2가 도는 곳 = Pi)

```bash
sudo apt install ros-lyrical-onnxruntime-vendor            # 한 번 (ONNX Runtime C++)
cd ~/ros2/ros2_ws && colcon build --packages-select rl_policy rl_nav_controller && source install/setup.bash
```
nav2_params.yaml이 이 플러그인을 쓰므로, **빌드 안 하면 nav가 시작하지 않는다** (FollowPath 플러그인 없음).
