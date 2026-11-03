# rl_correction_waypoint — 랜덤 웨이포인트를 이탈 없이 따라가기 (task 이름 RlCorrectionWaypoint)

목적: 웨이포인트까지 가는 **동안에도** 경로(앞 점 → 다음 점 직선)를 벗어나지 않게. 기본 제어기가 웨이포인트로 몰고,
정책이 그 명령에 보정 `[Δv, Δw]`를 더해 바퀴 gain 차이 · 지연 · 미끄러짐으로 생기는 드리프트를 잡는다.

## 구조

```
training/rl_correction_waypoint/
├── __init__.py   gym.register("RlCorrectionWaypoint") — 공식 train.py · play.py가 이 이름으로 찾음
├── env_cfg.py    RlCorrectionWaypointEnvCfg — 랜덤 폭 (중심 = 로봇 config 실측 응답) · 기본 제어기 · 보상 가중치 · 지형
├── env.py        RlCorrectionWaypointEnv(BaseEnv) — 추정 자세 · 기본 제어기 · 모터 모델 · 관측 · 보상 · 도착/실패
├── command.py    RlCorrectionWaypointCommand — env마다 랜덤 웨이포인트 4개 (각 0.5~1.5 m, 방향 랜덤)
├── calibration/  학습 전 1단계 측정 — command.py CalibrationCommand(네모) · env.py · env_cfg.py (./run-calibration.sh sim|real)
├── robot_cfg.py · sim_robot.py · real_robot.py · usda/robot.usda   로봇 (설정 · sim · 실기기 ROS · USD)
└── agent.py      RlCorrectionWaypointAgentCfg(BaseAgentCfg) — PPO 값 (500회 기본)
```

## 한 스텝 (20 Hz) — 로봇 corrector goal 모드와 같은 약속

```
추정 자세 (시작 자세 기준) = 거리는 엔코더 v · 방향은 자이로 w 적분
  ↓ 다음 웨이포인트
기본 제어기: v = min(max_lin, k_dist·거리) · w = clip(k_heading·방향오차)   (방향오차 > 0.5 rad면 제자리 회전 먼저)
  ↓ + 정책 [-1, 1] × action_scale (0.10 m/s · 0.60 rad/s) — 기본 제어기가 멈춤이면 0
  ↓ [명령 지연] → 바퀴 (드라이버 식) → 모터 (좌/우 gain · 데드밴드 · 1차 지연 τ) → 물리
추정 위치가 5 cm 안이면 다음 웨이포인트 · 4개 다 가면 끝
```

| 항목 | 값 |
|---|---|
| 관측 (10) | `[gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_a0, prev_a1]` (센서 지연 포함) |
| 행동 (2) | `[Δv, Δw]` 정규화 [-1, 1] |
| 보상 (진짜 위치로 채점) | − 2·경로 이탈 [m] − 0.5·방향 오차 + 2·진행 [m] + 도착 1 − 5·도착 순간 진짜 오차 [m] − 보정 크기 · 떨림 |
| 종료 | 4개 다 도착 · 경로에서 1 m 이상 이탈(실패 −5) · 40 s |
| 랜덤 (에피소드마다) | 바퀴별 τ · gain(실측 중심 0.9927/0.9938) · 데드밴드 · 명령/센서 지연 · 반지름/간격 기준 오차 · 자이로 영점 ±0.005 |
| 지형 · 마찰 | 경사 0~3° (`env.slope_deg`) · `env.uneven=true` · env마다 바퀴 마찰 랜덤 |

**/odom pose를 안 쓰는 이유**: 드라이버의 /odom 위치·방향은 엔코더만 적분한 값이라 바퀴 gain 차이로 실제로 휘어도 "똑바로"로 나온다.
방향을 자이로로 봐야 휨이 보인다. 미끄러져서 생긴 거리 오차는 여전히 못 본다 (나중에 SLAM 위치를 쓰면 됨).

## 실행

```bash
cd isaac
./run-training.sh --max_iterations 500                                   # 기본 task = RlCorrectionWaypoint (창 없음, 512대)
./run-training.sh --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 창 (노란 점 = 웨이포인트 경로)
./run-training.sh play --checkpoint logs/rsl_rl/rl_waypoint/<시각>/model_499.pt --num_envs 16 --viz kit env.debug_vis=true
tensorboard --logdir logs/rsl_rl          # Metrics/cross_track (경로 이탈 평균) · Metrics/waypoints_done (평균 도착 수)
```

## 실기기

- corrector goal 모드(`ros2_ws/src/rl_correction`)가 같은 관측 · 기본 제어기 · 스케일을 쓴다. `goal_mode=absolute`(기본)로
  첫 웨이포인트 때 시작 자세를 잡고, `/rl_goal`(Point, 시작 자세 기준 x · y)을 하나씩 보낸다. `/rl_goal_stop`으로 끝.
- 웨이포인트를 차례로 보내는 쪽(도착하면 다음 점)은 아직 없다 — 다음 단계.
- 기본 제어기 파라미터(k_dist · k_heading · turn_thresh · goal_tol)와 action_scale은 `env_cfg.py`와 corrector가 같아야 한다.
