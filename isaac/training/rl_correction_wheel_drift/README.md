# wheel_drift — 바퀴 드리프트 보정 RL (학습만)

목적: `(v, w)` 명령을 넣었을 때 모터 지연·좌우 gain 차이·데드밴드로 생기는 드리프트를
정책이 `[Δv, Δw]`로 보정하도록 **sim에서 학습**한다. 여기는 학습만 한다 (검증·실기기는 밖에서).
설계·평가 기준은 [../../../documents/isaac/task-spec.md](../../../documents/isaac/task-spec.md).

## 구조

```
training/rl_correction_wheel_drift/
├── task/
│   ├── TaskScene.py            ground+마찰 패치+바퀴/캐스터 바인딩
│   ├── rl_correct_env_cfg.py   환경 설정 (obs 6 · action 2 · 랜덤 범위)
│   └── rl_correct_env.py       환경 (장면은 TaskScene에 위임)
├── agents/rsl_rl_ppo_cfg.py    PPO 설정
├── FitPanel.py                 --watch 때 보정값/랜덤 범위/마찰 보기 창 (읽기만)
├── train.py                    WheelDriftTrain(../Train.py 상속) — 학습 → 끝나면 ONNX export
└── README.md
```

공용: `../../urdf/robot_spec.py` · `../onnx/onnx_export.py` · `../../calibration/` (`--fit`)

## 환경

| 항목 | 값 |
|---|---|
| obs (6) | `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` |
| action (2) | `[Δv, Δw]` — 명령에 더하는 보정 (`action_scale` = 0.10 m/s, 0.60 rad/s) |
| 주기 | 20 Hz (= cmd_manager) |
| 명령 | 0.5~2 s마다 바뀜: 텔레오퍼 키 · 부드러운 곡선 · 정지 |
| 보상 | `−(|v_cmd−v| + 0.5·|w_cmd−w|) − action/smooth penalty` (sim true 속도 기준) |
| 랜덤화 (에피소드마다) | 바퀴별 τ·gain, 데드밴드, 명령/관측 지연, 바퀴 반지름·간격 기준 오차, 초기 yaw |
| 마찰 (기본 켜짐) | env별 바닥 마찰 + 파랑=미끄럼 / 주황=끈적 네모 패치 (기본 4개, `--patch-count`). 배치는 시작 시 1회. 끄려면 `--no-friction` |
| `--uneven` | env별 낮은 턱 (1~3 cm) |
| `--latency` | rl-only Before CSV → RL 경로 왕복을 명령 지연에 더하고 관측 지연 중심으로 |
| `--fit` | `real/fit.json,sim/fit.json` → 차이를 τ·gain·지연(·데드밴드) 랜덤 범위의 **중심**으로 (폭 유지) |

- obs에는 실기기에서 보이는 값만 넣는다 (true 속도는 보상에만).

## 실행

```bash
cd isaac   # Lab env

# 기본: headless, 마찰 패치 켜짐
./isaaclab.sh -p -m training rl_correction_wheel_drift --iterations 300 \
  --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json
# 창으로 보기 (512 env는 무거우니 16~32개만) — TRAINING FIT 창에 fit·범위·마찰이 같이 뜸
./isaaclab.sh -p -m training rl_correction_wheel_drift --watch --envs 16 \
  --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json

tensorboard --logdir artifacts/training
```

- `--watch` 창에서 **P** 키 → 결과 폴더에 `screen_shot_<시각>.png`
- 결과: `artifacts/training/<시각>-rl_correct/` — checkpoint, `policy.onnx`, `eval.json`(sim 정책 끔/켬 비교), (`--fit`/`--latency`면) `base-fit.json`

## 주의

- 학습 전에 `calibration/`으로 sim/real 신호를 먼저 맞춘다. 안 맞으면 학습이 잘 돼도 real에서 효과가 없다.
- `--fit`에는 `real/fit.json,sim/fit.json`(차이)을 넣는다. sim fit은 raw · 지금 `ActuatorParam`과 같은 damping으로 잰 것 (아니면 거부). 1개면 real 값 그대로.
