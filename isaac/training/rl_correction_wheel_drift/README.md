# wheel_drift — 바퀴 드리프트 보정 RL (학습만)

목적: `(v, w)` 명령을 넣었을 때 모터 지연·좌우 gain 차이·데드밴드로 생기는 드리프트를
정책이 `[Δv, Δw]`로 보정하도록 **sim에서 학습**한다. 여기는 학습만 한다 (검증·실기기는 밖에서).
설계·평가 기준은 [../../../documents/isaac/task-spec.md](../../../documents/isaac/task-spec.md).

## 구조

```
training/rl_correction_wheel_drift/
├── env_cfg.py                  RlCorrectionWheelDriftEnvCfg (obs 6 · action 2 · 랜덤 범위 · 지형 · 바퀴 마찰 · robot=beebot + 실측 중심)
├── env.py                      환경 RlCorrectionWheelDriftEnv(BaseEnv) — 모터 모델 · 관측 · 보상 · 리셋
├── command.py                  TeleopCommand(BaseCommand) — 0.5~2 s마다 텔레오퍼 키 · 곡선 · 정지 (학습: sample(k) · 한 대: __call__)
├── agent.py                    학습자 설정 RlCorrectionWheelDriftAgentCfg(../../agents/base_agent.py BaseAgentCfg) — 이 task 값
├── __init__.py                 gym.register("Beebot-WheelDrift") — 공식 train.py · play.py가 이 이름으로 찾음
└── README.md
```

공용: `../../robots/beebot/config.py` · `../../envs/base_env.py` · `../../calibration/` (측정 — 실측 값은 `env_cfg.py`에 숫자)

## 환경

| 항목 | 값 |
|---|---|
| obs (6) | `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` |
| action (2) | `[Δv, Δw]` — 명령에 더하는 보정 (`action_scale` = 0.10 m/s, 0.60 rad/s) |
| 주기 | 20 Hz (= cmd_manager) |
| 명령 | 0.5~2 s마다 바뀜: 텔레오퍼 키 · 부드러운 곡선 · 정지 |
| 보상 | `−(|v_cmd−v| + 0.5·|w_cmd−w|) − action/smooth penalty` (sim true 속도 기준) |
| 랜덤화 (에피소드마다) | 바퀴별 τ·gain, 데드밴드, 명령/관측 지연, 바퀴 반지름·간격 기준 오차, 초기 yaw |
| 마찰 (기본 켜짐) | env마다 바퀴 마찰 랜덤 0.7~1.1 (Isaac Lab events `EventCfg`, 시작 시 1회). 바닥은 고정 (0.8, 0.7). 끄려면 `env.events=null` |
| 지형 | 경사 칸 0~`env.slope_deg`°(기본 3) · `env.uneven=true`면 요철 칸 (턱 1~3 cm) |
| 실기기 맞춤 | `RlCorrectionWheelDriftEnvCfg`: `gain_center` 0.9927/0.9938 · `cmd_delay_steps` (0, 4) · `tau_range` (0.01, 0.10) — 2026-11-02 real − sim |

- obs에는 실기기에서 보이는 값만 넣는다 (true 속도는 보상에만).

## 실행

```bash
cd isaac
./run-training.sh --max_iterations 300                                   # 기본: 창 없음, 512대, 바퀴 마찰 랜덤 켜짐
./run-training.sh --num_envs 16 --viz kit env.debug_vis=true             # 창으로 보기 (16~32대)
./run-training.sh play --checkpoint logs/rsl_rl/rl_correct/<시각>/model_299.pt --num_envs 1   # → exported/policy.onnx
tensorboard --logdir logs/rsl_rl
```

- 화면(`env.debug_vis=true`): 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 랜덤 명령의 목표 경로
  (명령이 바뀐 순간부터 그 명령대로 갈 길). 구현: `env.py` `vis_path` + `../../envs/base_env.py`
- 학습 명령을 한 대로 보기: `../../run-world.sh --cmd training.rl_correction_wheel_drift.command:TeleopCommand --speed 0.26 --turn 1.82`
- 결과: `logs/rsl_rl/rl_correct/<시각>/` — `model_<N>.pt` · `params/env.yaml`(실제로 쓴 범위) · TensorBoard · (play 후) `exported/`

## 주의

- 학습 전에 `calibration/`으로 sim/real 신호를 먼저 맞춘다. 안 맞으면 학습이 잘 돼도 real에서 효과가 없다.
- 실측 값을 바꾸면(하드웨어 변경 등) `RlCorrectionWheelDriftEnvCfg` 숫자를 고친다 — real − sim 차이 (`../../README.md` 3.3).
