# rl_nav_correction — 명령 (v, w)를 몸체가 그대로 따라가게 하는 보정 (task 이름 RlNavCorrection)

바퀴 드리프트 · 지연을 정책이 `[Δv, Δw]`로 보정한다. 로봇 = **beebot** (Lv2_bee_Assignment `isaac/sim/usda/beebot` 복사 → `usda/robot.usda`).
배포 자리 = Lv2_bee_Assignment `ros2_ws/src/ml` 의 `rl_nav_correction` 노드:

```text
planner → controller → /cmd_vel_nav → rl_nav_correction (ONNX) → /cmd_vel_corrected → twist_mux → velocity_smoother → collision_monitor → /cmd_vel
```

## 정책 계약 (노드와 같아야 한다 — 바꾸면 양쪽 같이)

| | 학습 (env_cfg.py) | 노드 (ml.yaml) |
|---|---|---|
| 관측 6 | `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` 원래 단위 | `[v_cmd, w_cmd, v_odom, w_odom, w_gyro, t_change]` |
| 출력 2 | `[Δv, Δw]` ∈ [-1, 1] | 같음 |
| 크기 | `action_scale = (0.05, 0.10)` | `action_scale_lin/ang` · `max_delta_lin/ang` |
| 한계 | `robot_cfg.py` MAX_LIN 0.26 · MAX_ANG 1.82 | `max_vel_x` · `max_vel_theta` |
| t_change 상한 | `t_change_max = 2.0` s | `t_change_max` |

명령이 0이면 보정도 0 (노드 `apply_delta`와 같음).

## 구조

```text
training/rl_nav_correction/
├── __init__.py   gym.register("RlNavCorrection")
├── env_cfg.py    관측 6 · 행동 2 · 랜덤 폭 (중심 = robot_cfg 실측 응답) · 보상 가중치 · 지형
├── env.py        RlNavCorrectionEnv(BaseEnv) — 명령 → 보정 → 지연 → 바퀴 → 모터 → 센서(지연) · 팔은 기본 자세로 붙잡음
├── command.py    명령 (v, w)가 0.5~2 s마다 바뀜 (정지 · 직진 · 제자리 회전 · 호)
├── agent.py      PPO 값 (64×64 · 500회)
├── robot_cfg.py  beebot — 바퀴 숫자 · 응답(RESPONSE_* = 실측 4회) · ROBOT_CFG
├── usda/         beebot USD (robot.usda + payloads/ — 그대로, Physics 변형 physx)
├── ground.py · markers.py   요철 · 마찰 패치 지형 · 화면 막대 (초록 = 명령, 파랑 = 실제)
├── sim_robot.py · real_robot.py · dummy_robot.py   calibration 입출력 (real = 로봇 bee에서 run-cli calibrate로 재생 · 기록)
└── calibration/  sim 측정 (command · env · env_cfg · report)
```

## 실행

```bash
./run-training.sh rl_nav_correction --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 짧게 (창)
./run-training.sh rl_nav_correction --max_iterations 500 --label v1 --snap 60                        # 본 학습
./run-make-onnx.sh artifacts/rl_nav_correction/training/<시각>_v1/model_499.pt                        # → exported/policy.onnx (+ .onnx.data)
tensorboard --logdir artifacts/rl_nav_correction/training      # Metrics/track_err_v (m/s) · track_err_w (rad/s) — 작을수록 좋음
./run-tb-summary.sh rl_nav_correction --watch 10               # 터미널 요약 (first/last/min/max)
./run-training-report.sh rl_nav_correction                    # → artifacts/rl_nav_correction/training/<최신>/report.html
```

배포: `policy.onnx` **와 `policy.onnx.data` 둘 다** → Lv2_bee_Assignment `ros2_ws/model/nav_correction/` (빌드 때 `share/ml/model/nav_correction/`로 설치) → `ros2 param set /rl_nav_correction model_path ...` 또는 재시작.

## 칼리브레이션 (sim → real)

```bash
./run-calibration.sh rl_nav_correction --robot sim --label sim                 # 1) sim (로봇 안 움직임)
./run-calibration.sh rl_nav_correction --robot real --check-only               # 2) 실기기 준비 확인 (안 움직임)
./run-calibration.sh rl_nav_correction --robot real --label real_floor         # 3) 실기기 측정 (로봇이 움직임 · 약 54 s)
python3 training/rl_nav_correction/calibration/report.py artifacts/rl_nav_correction/calibration/<sim 폴더> artifacts/rl_nav_correction/calibration/<real 폴더>
#   → fit · 그림 · RESPONSE_* 값 → robot_cfg.py에 반영
```

- **계획 JSON 만들기** (로봇 쪽 재생기가 읽는 파일 — sim과 같은 명령): `./run-calibration.sh rl_nav_correction --robot plan --out wheel_square.json`
  (`calibration/command.py`의 CalibrationCommand를 20 Hz로 펼친 `{dt, label, steps[[v, w]...]}` — 로봇의 `tool/calibration/wheel_square.json`이 이걸로 만든 것. `--speed --turn --side`로 다른 경로)
- real = PC가 sim과 같은 네모 명령을 계획(JSON)으로 만들어 `bee@bee.local`로 보내고, 로봇의 `./run-cli calibrate wheel`(Lv2_bee_Assignment `tool/` · 메뉴 13 → 바퀴)가 수동 경로 `/cmd/manual/cmd_vel`로 재생하며 `/odom` · `/imu` · `/joint_states` · `/cmd_vel`을 기록한 `raw.csv`를 PC가 가져옵니다. 수동 경로는 `rl_nav_correction`을 건너뛰므로 보정 없는 순수 응답입니다.
- 준비: 로봇에서 bringup · `./run-cli mode manual` · 사방 약 1.5 m 비움 (팔이 시작 자세가 아니면 로봇이 재생 전에 자동으로 보냄). 멈추려면 Ctrl+C 또는 `./run-cli estop on`.
- 환경변수: `BEE_HOST`(기본 bee@bee.local) · `BEE_TOOL`(로봇의 tool 폴더) · `BEE_ENV`(로봇 ROS 환경 — 기본 도메인 88 · 로컬 DDS).

## 아직 안 한 것
- τ는 리포트 fit이 격자값만 내서 신뢰가 낮다 (gain · 지연은 4회 일정). 회전은 자이로가 엔코더의 0.955 — env의 sep_nominal_err 범위에 둔다.
- **실기기 A/B(`--onnx`)와 `report.py`는 없다** — 정책이 생긴 뒤 `rl_nav_correction`을 `/cmd_vel_nav` 경로로 켜고 비교한다.
- 체인 뒤쪽(`velocity_smoother`의 가속 한계)은 sim에 없다 (실기기 측정에는 들어간다).
