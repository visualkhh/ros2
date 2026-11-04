# rl_maze — 미로 안 웨이포인트를 가장 빨리 따라가기 (task 이름 RlMaze)

목적: nav2의 **controller(로컬 플래너) 자리**를 RL로. planner가 준 경로(웨이포인트)를 받아 정책이 `[v, w]`를 직접 내고,
벽에 닿지 않으면서 가장 빨리 · 짧게 끝 점까지 간다. (rl_correction_waypoint는 기본 제어기 + 작은 보정 — 성격이 다르다.)

## 구조

```
training/rl_maze/
├── __init__.py     gym.register("RlMaze")
├── env.py          RlMazeEnv(BaseEnv) — 라이다(격자 광선) · 웨이포인트 진행 · 보상 · 충돌 · 화면
├── env_cfg.py      RlMazeEnvCfg — 관측 51 / 행동 2 · 보상 가중치 · 라이다 · 256대 한 미로
├── command.py      RlMazeCommand — paths.npz에서 env마다 랜덤 경로
├── agent.py        RlMazeAgentCfg — PPO (128×128)
├── maze_grid.py    미로 USD → 점유 격자(2 cm) · 벽까지 거리 · A* 경로 → usda/maze/grid.npz · paths.npz
├── calibration/ · robot_cfg.py · sim_robot.py · real_robot.py · dummy_robot.py · markers.py · usda/robot.usda
│                   rl_correction_waypoint에서 의도적으로 복사 (같은 로봇 — 바뀌면 같이)
└── usda/maze/      maze.usda · geo.usd (Lv2_bee_Assignment lv2_module6) · grid.npz · paths.npz · grid.png(확인용)
```

## 한 스텝 (20 Hz)

| 항목 | 값 |
|---|---|
| 관측 (51) | 다음 웨이포인트 5개 (로봇 기준 x, y ÷ 2 m) · 라이다 36방향 (÷ 3.5 m) · v_enc · w_enc · w_gyro · 이전 행동 2 |
| 행동 (2) | `[v, w]` ∈ [-1, 1] → × (0.26 m/s, 1.82 rad/s) · 후진은 20%까지 |
| 보상 | +10 × 웨이포인트 쪽으로 줄인 거리 · +1 웨이포인트 통과 · +10 도착 − 0.05/스텝 − 10 충돌 − 떨림 − 후진 |
| 끝 | 마지막 웨이포인트 도착 · 벽 충돌(로봇 중심이 벽에서 0.17 m 안) · 60 s |
| 웨이포인트 | 0.15 m 안이면 다음 · 다음 점이 더 가까우면 건너뜀 (모서리 질러가기) |
| 경로 | 랜덤 출발 · 도착 → A* (벽에서 0.20 m 이상 + 통로 가운데로) → 0.30 m 간격 · 2000개를 미리 |
| 세상 | 미로 하나에 256대 (env끼리 충돌 없음) · 위치 관측 잡음 2 cm · 2° · 라이다 잡음 1 cm |

## 실행

```bash
cd isaac
$ISAACLAB_DIR/isaaclab.sh -p training/rl_maze/maze_grid.py        # 미로 USD가 바뀌었을 때만 (grid.npz · paths.npz 다시)
./run-training.sh rl_maze --max_iterations 1000
./run-training.sh rl_maze --num_envs 16 --viz env.debug_vis=true   # 보기 (노란 점선 = 로봇마다 → 앞 웨이포인트 5개)
./run-calibration.sh rl_maze --robot sim                            # 같은 로봇 calibration (복사본)
```

## 실기기 (nav2 controller)

`ros2_ws/src/rl_nav_controller` — nav2 `FollowPath` 플러그인. 안에 MPPI를 품고 toggle 끔(기본) = MPPI · 켬 = 이 정책.
```bash
./run-training.sh rl_maze --play --checkpoint artifacts/rl_maze/training/<시각>/model_N.pt --num_envs 1
#   → exported/policy.onnx (+ .onnx.data) → nav2_params FollowPath.model_path
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}"
```

## 아직 없는 것

- 바퀴 응답 랜덤(gain · 지연 · τ — rl_correction_waypoint의 모터 모델)은 아직 안 넣음 — BaseEnv 드라이버 식 그대로.
- 라이다는 로봇 중심 기준으로 학습 — 실기기 LDS는 앞쪽에 달려 있어 조금 어긋난다 (방향만 tf로 맞춤).
