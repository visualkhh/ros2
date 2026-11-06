> **레거시 task.** 2026-11-06 `rl_correction`/`rl_eval`/`rl-only`/`route rl` 제거.
> 현재 실기 RL은 `isaac/training/rl_maze` + Nav2 `rl_nav_controller`다.

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
├── calibration/  학습 전 1단계 측정 — command.py CalibrationCommand(네모) · env.py · env_cfg.py ·
│                 report.py (분석: python3 calibration/report.py <sim 폴더> <real 폴더> → fit · 그림 · RESPONSE_* 값)  (./run-calibration.sh rl_correction_waypoint --robot sim|real)
├── robot_cfg.py · sim_robot.py · real_robot.py · dummy_robot.py · usda/robot.usda   로봇 (설정 · sim · 실기기 ROS · 가짜 · USD)
└── agent.py      RlCorrectionWaypointAgentCfg(BaseAgentCfg) — PPO 값 (500회 기본)
```

## 한 스텝 (20 Hz) — 레거시 task의 관측·행동

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
| 지형 · 마찰 | 평평 + 요철(턱 1~3 cm) 칸 · 언덕 없음 (`env.ground.uneven=false` = 평면만) · 마찰 패치 env마다 8개 (노랑 끈적 1.1 · 파랑 미끄럼 0.2, 출발점 0.3~3 m, `env.ground.friction_patches=0` = 끔) · env마다 바퀴 마찰 랜덤 |

**/odom pose를 안 쓰는 이유**: 드라이버의 /odom 위치·방향은 엔코더만 적분한 값이라 바퀴 gain 차이로 실제로 휘어도 "똑바로"로 나온다.
방향을 자이로로 봐야 휨이 보인다. 미끄러져서 생긴 거리 오차는 여전히 못 본다 (나중에 SLAM 위치를 쓰면 됨).

## 실행

```bash
cd isaac
./run-training.sh rl_correction_waypoint --max_iterations 500                                   # task 폴더 → gym RlCorrectionWaypoint (창 없음, 512대)
./run-training.sh rl_correction_waypoint --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 창 (노란 점 = 웨이포인트 경로)
./run-training.sh rl_correction_waypoint --play --checkpoint artifacts/rl_correction_waypoint/training/<시각>/model_499.pt --num_envs 16 --viz kit env.debug_vis=true
./run-training.sh rl_correction_waypoint --max_iterations 500 --label v1 --snap 60   # 60초마다 화면 → training/<시각>_v1/snap/ + latest.png
tensorboard --logdir artifacts/rl_correction_waypoint/training          # Metrics/cross_track (경로 이탈 평균) · Metrics/waypoints_done (평균 도착 수)
```

## calibration (학습 전에 먼저) — sim · real에 같은 네모를 넣고 차이를 `robot_cfg.py` `RESPONSE_*`에

```bash
cd isaac
./run-calibration.sh rl_correction_waypoint --robot sim                  # sim (창 없음 · 보려면 --viz)
./run-calibration.sh rl_correction_waypoint --robot real --check-only    # real: 안 움직이고 상태만 → "check-only — 준비됨"
./run-calibration.sh rl_correction_waypoint --robot real --label real_floor   # real 측정 (약 54 s)
#   → artifacts/rl_correction_waypoint/calibration/<시각>_<label>/raw.csv
python3 training/rl_correction_waypoint/calibration/report.py artifacts/rl_correction_waypoint/calibration/<sim 폴더> artifacts/rl_correction_waypoint/calibration/<real 폴더>
#   → fit.json · report.png (폴더마다) · compare.png (real 폴더) · robot_cfg.py에 붙일 RESPONSE_* 값
```

### real 준비 순서 (`거부: MANUAL 아님 (state=None)`이면 1~2가 안 된 것)

`state=None` = PC가 로봇의 `/state_control`을 못 받음 → Pi에서 베이스(tb-only: 드라이버 · cmd_manager · state_manager)가 안 떠 있거나
네트워크(DDS)가 안 닿음. 로봇이 꺼져 있으면 아무것도 안 된다.

1. **로봇 켜기**: Raspberry Pi 전원 + 부팅 기다리기 (1~2분). OpenCR(모터 보드)은 로봇을 가만히 둔 채 켠다 (IMU 영점).
   PC에서 닿는지: `ping pak.local` (안 되면 같은 Wi-Fi인지)
2. **베이스 켜기** — Pi에서 직접 또는 PC에서 (PC의 run-cli는 켜기/끄기를 SSH로 Pi에 넘긴다). 2 · 4의 run-cli는 어느 쪽이든 같다:
   ```bash
   # Pi에서:  ssh pak@pak.local  →  cd /home/pak/ros2/tool
   # PC에서:  cd ~/workspaces/visualkhh/source/ros2/tool
   ./run-cli base start          # 바퀴 · odom · 라이다 (tb-only)
   ./run-cli status              # 로봇 수신 · odom 확인
   ```
3. **제어 MANUAL** (run-cli, Pi든 PC든):
   ```bash
   ./run-cli control manual
   ```
4. **check-only** → `준비됨`이 나오면 로봇 주변 1 m 비우고 측정. 사각형(한 변 0.5 m) → 180° 턴 → 역주행, 중간에 잡지 말 것.
5. 끝나면 베이스는 그대로 둬도 된다 (끄기: `./run-cli base stop`).

## 실기기

- 이 task는 레거시다. 실기 배포는 `rl_maze` + Nav2 `rl_nav_controller`를 쓴다.

## 실기 배포

이 task는 **레거시**다. 2026-11-06 `rl_correction`/`rl_eval`/`rl-only`/`route rl` 제거와 함께
실기 A/B 스크립트(`run-real-ab.sh`, `run-real-goal-ab.sh`)도 삭제됐다.

현재 실기 RL은 `isaac/training/rl_maze` 정책을 Nav2 `rl_nav_controller`(`FollowPath`)에 올려 쓴다.
