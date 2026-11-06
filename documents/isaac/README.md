# Isaac Sim / Isaac Lab — RL 정책 만들기

TurtleBot3 Waffle Pi 개조 로봇의 **Nav2 주행 RL** 학습 쪽 문서.
학습은 lab PC(x86_64 + RTX), 추론은 Nav2 `rl_nav_controller` 플러그인(ONNX Runtime C++)이 한다.

```text
Isaac Lab (rl_maze) → policy.onnx + policy.onnx.data
  → ros2_ws/src/tb3_bringup/config/nav2_params.yaml FollowPath.model_path
  → controller_server → /cmd_vel_nav → twist_mux → 바퀴
```

## 정책

| 노드 | 입력 (obs 52) | 출력 | 학습 환경 |
|---|---|---|---|
| `rl_nav_controller::RlNavController` (Nav2 `FollowPath`) | 웨이포인트 5×2 + 라이다 36(`r - lidar_inflation`) + v/w/gyro + 이전 행동 2 + inflation/0.5 | `[v, w]` | `isaac/training/rl_maze` |

- 한계 0.26 m/s · 1.82 rad/s (Waffle Pi).
- `enable=false`(기본) 또는 모델 로드 실패 시 fallback controller(DWB)로 바이패스.
- fallback 선택: `FollowPath.fallback_controller` (현재 `dwb`; `mppi` · `rpp` · `rotation_shim` 가능).
- `lidar_inflation` [m] — 사용자 입력. 학습 `inflation_range 0.20~0.24` 안. `ros2 param set`으로 런타임 변경.

## 로봇 쪽 연결

```bash
# ros2_ws/src/tb3_bringup/config/nav2_params.yaml
FollowPath:
  enable: true
  model_path: "/path/policy.onnx"   # 옆에 policy.onnx.data
  fallback_controller: "dwb"        # 또는 mppi/RPP/RotationShim
  lidar_inflation: 0.22             # [m] 학습 env.inflation_range(0.20~0.24) 안
  inflation_scale: 0.50             # 정규화 상수 — 사용자 값 아님, 학습/추론 동일
```

```bash
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}"
./run-cli node-status /controller_server   # RL/fallback · 모델 · 스캔 확인
```

상태: `/controller_server/FollowPath/status`
`[rl_active, reason, policy_loaded, enable, scan_ok, scan_age, scan_min, v, w, inner_kind]`

## 관련

- 학습 task: [../../isaac/training/rl_maze/README.md](../../isaac/training/rl_maze/README.md)
- 실행: [../../isaac/README.md](../../isaac/README.md)
- 설계 기록: [task-spec.md](task-spec.md) — 2026-11-06 이전 `rl_correction`/`rl_eval` 구조는 제거됨.
