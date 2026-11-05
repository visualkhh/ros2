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
├── robot_cfg.py  beebot — 바퀴 숫자 · 응답(RESPONSE_*, 아직 안 잼 = 중립값) · ROBOT_CFG
├── usda/         beebot USD (robot.usda + payloads/ — 그대로, Physics 변형 physx)
├── ground.py · markers.py   요철 · 마찰 패치 지형 · 화면 막대 (초록 = 명령, 파랑 = 실제)
├── sim_robot.py · dummy_robot.py   calibration 입출력 (real_robot.py 없음 — 아래)
└── calibration/  sim 측정 (command · env · env_cfg · report)
```

## 실행

```bash
./run-training.sh rl_nav_correction --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 짧게 (창)
./run-training.sh rl_nav_correction --max_iterations 500 --label v1 --snap 60                        # 본 학습
./run-make-onnx.sh artifacts/rl_nav_correction/training/<시각>_v1/model_499.pt                        # → exported/policy.onnx (+ .onnx.data)
tensorboard --logdir artifacts/rl_nav_correction/training      # Metrics/track_err_v (m/s) · track_err_w (rad/s) — 작을수록 좋음
```

배포: `policy.onnx` **와 `policy.onnx.data` 둘 다** → Lv2_bee_Assignment `ros2_ws/model/nav_correction/` (빌드 때 `share/ml/model/nav_correction/`로 설치) → `ros2 param set /rl_nav_correction model_path ...` 또는 재시작.

## 아직 안 한 것
- **beebot calibration**: `robot_cfg.py`의 RESPONSE_*는 중립값 (gain 1 · 지연 1 step). `./run-calibration.sh rl_nav_correction --robot sim`은 되지만, 실기기 측정 · A/B는 `real_robot.py`가 없다 (beebot ROS는 `/cmd_vel_nav` · `/cmd_vel_corrected` 체인이라 pak의 corrector와 토픽이 다르다).
- 체인 뒤쪽(`velocity_smoother`의 가속 한계)은 sim에 없다.
