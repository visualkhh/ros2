# RL 주행 — 목적 · 설계 (2026-11-06)

## 0. 목적

같은 경로·같은 goal을 **RL 끔(inner controller)** 과 **RL 켬(`rl_nav_controller`)** 으로 주행해
추종 오차·도착 시간·충돌 없이 통과한 비율을 비교한다.

```text
A: FollowPath.enable=false → fallback controller (DWB)
B: FollowPath.enable=true  → rl_nav_controller (ONNX 정책)
```

## 1. 범위

- Nav2 `FollowPath` 컨트롤러 단계만.
- 2026-11-06 이전의 `rl_correction`/`rl_eval`/`rl-only`/`route rl`(PC 보정 경로)은 **제거됨**.
- 학습 task: `isaac/training/rl_maze` (관측 52 / 행동 2).

## 2. 관측 · 행동

```text
관측 52 = 웨이포인트 5×2 + 라이다 36 + v/max_lin + w/max_ang + gyro/max_ang + 이전 행동 2 + inflation/0.50
행동 2  = [v, w] ∈ [-1, 1] → × (max_lin 0.26 m/s, max_ang 1.82 rad/s)
```

- 로봇 기준 웨이포인트는 Nav2 `transformed_global_plan`에서 `wp_step` 간격으로 샘플.
- 라이다는 `/scan`(실제 LiDAR) 또는 점구름 가상 스캔(bee 변형). 스캔은 `r - lidar_inflation` (m).
- `lidar_inflation`은 사용자 입력(ROS 파라미터, 기본 0.22 m) — 학습 `inflation_range 0.20~0.24` 안.
- fallback 선택: `FollowPath.fallback_controller` (DWB · MPPI · RPP · RotationShim).
- 모델 로드 실패·`enable=false`·센서 없음·추론 실패 시 fallback controller로 자동 바이패스.

## 3. 평가

| 지표 | 보는 곳 |
|---|---|
| RL/fallback 여부 | `/controller_server/FollowPath/status` · `./run-cli node-status /controller_server` |
| 경로 추종 오차 | `/odometry/filtered` + `/plan` |
| 도착 시간 · 성공률 | `/navigate_to_pose` 액션 결과 |
| 충돌 | `collision_monitor` 상태 · `/cmd_vel` |

상태 토픽:

```text
[rl_active, reason, policy_loaded, enable, scan_ok, scan_age, scan_min, v, w, inner_kind]
reason: 0=RL · 1=모델 없음/로드 실패 · 2=enable=false · 3=센서 없음 · 4=추론 실패
inner_kind: 1=DWB · 2=MPPI · 3=RPP · 4=RotationShim
```
