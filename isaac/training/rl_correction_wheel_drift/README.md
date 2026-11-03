# wheel_drift — 바퀴 드리프트 보정 RL (학습만)

목적: `(v, w)` 명령을 넣었을 때 모터 지연·좌우 gain 차이·데드밴드로 생기는 드리프트를
정책이 `[Δv, Δw]`로 보정하도록 **sim에서 학습**한다. 여기는 학습만 한다 (검증·실기기는 밖에서).
설계·평가 기준은 [../../../documents/isaac/task-spec.md](../../../documents/isaac/task-spec.md).

## 구조

```
training/rl_correction_wheel_drift/
├── env_cfg.py                  환경 설정 (obs 6 · action 2 · 랜덤 범위 · 실측 중심 · 지형 make_terrain · 바퀴 마찰 EventCfg)
├── env.py                      환경 RlCorrectEnv(BaseEnv) — 모터 모델 · 관측 · 보상 · 리셋
├── command.py                  학습 명령 — 0.5~2 s마다 텔레오퍼 키 · 곡선 · 정지 (env마다)
├── agent.py                    학습자 설정 RlCorrectAgentCfg(../../agents/base_agent.py BaseAgentCfg) — 다른 값만
├── fit_panel.py                --watch 때 랜덤 범위/마찰 보기 창 (읽기만)
├── train.py                    WheelDriftTrain(../trainer.py Train 상속) — 학습 → 끝나면 ONNX export
└── README.md
```

공용: `../../robots/beebot/config.py` · `../../utils/onnx_export.py` · `../../calibration/` (측정 — 실측 값은 `env_cfg.py`에 숫자)

## 환경

| 항목 | 값 |
|---|---|
| obs (6) | `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` |
| action (2) | `[Δv, Δw]` — 명령에 더하는 보정 (`action_scale` = 0.10 m/s, 0.60 rad/s) |
| 주기 | 20 Hz (= cmd_manager) |
| 명령 | 0.5~2 s마다 바뀜: 텔레오퍼 키 · 부드러운 곡선 · 정지 |
| 보상 | `−(|v_cmd−v| + 0.5·|w_cmd−w|) − action/smooth penalty` (sim true 속도 기준) |
| 랜덤화 (에피소드마다) | 바퀴별 τ·gain, 데드밴드, 명령/관측 지연, 바퀴 반지름·간격 기준 오차, 초기 yaw |
| 마찰 (기본 켜짐) | env마다 바퀴 마찰 랜덤 0.7~1.1 (Isaac Lab events `EventCfg`, 시작 시 1회). 바닥은 고정 (0.8, 0.7). 끄려면 `--no-friction` |
| `--uneven` | env별 낮은 턱 (1~3 cm) |
| 실기기 맞춤 | `env_cfg.py`: `gain_center` 0.9927/0.9938 · `cmd_delay_steps` (0, 4) · `tau_range` (0.01, 0.10) — 2026-11-02 real − sim |

- obs에는 실기기에서 보이는 값만 넣는다 (true 속도는 보상에만).

## 실행

```bash
cd isaac   # Lab env

# 기본: headless, 바퀴 마찰 랜덤 켜짐
./isaaclab.sh -p -m training rl_correction_wheel_drift --iterations 300
# 창으로 보기 (512 env는 무거우니 16~32개만) — 랜덤 범위 창이 같이 뜸
./isaaclab.sh -p -m training rl_correction_wheel_drift --watch --envs 16

tensorboard --logdir artifacts/training
```

- `--watch` 창에서 **P** 키 → 결과 폴더에 `screen_shot_<시각>.png`
- 화면(`--watch`·`--snap`): 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 랜덤 명령의 목표 경로
  (명령이 바뀐 순간부터 그 명령대로 갈 길). 구현: `env.py` `_set_debug_vis_impl` · `_debug_vis_callback`
- 학습한 정책 몰아보기: `../../run-training.sh --drive --watch --policy <결과>/policy.onnx` (C = 정책 켬/끔)
- 결과: `artifacts/training/<시각>-rl_correct/` — checkpoint, `policy.onnx`, `eval.json`(sim 정책 끔/켬 비교), `ranges.json`

## 주의

- 학습 전에 `calibration/`으로 sim/real 신호를 먼저 맞춘다. 안 맞으면 학습이 잘 돼도 real에서 효과가 없다.
- 실측 값을 바꾸면(하드웨어 변경 등) `env_cfg.py` 숫자를 고친다 — real − sim 차이 (`../../README.md` 3.3).
