# rl — 바퀴 드리프트 보정 RL

목적: **같은 경로를 RL 끄고(Before) / 켜고(After) 달려서 로봇이 명령대로 움직이는 정도를 수치로** 보인다.
설계·평가 기준은 [../documents/isaac/task-spec.md](../documents/isaac/task-spec.md).

- `steps/` = 환경·센서·실기기 싱크 (S0~S8)
- `rl/` = 그 위에서 도는 **바퀴 보정 정책** (환경 · 학습 · export · play)

## 구조

```
rl/
├── task/                     rl_correct v2 환경 (obs 6 · action 2)
│   ├── rl_correct_env_cfg.py
│   └── rl_correct_env.py
├── agents/rsl_rl_ppo_cfg.py  PPO 설정
├── smoke.py                  물리 스모크 (오차 0일 때 명령대로 가나)
├── train.py · export.py · play.py
├── train.sh · export.sh · play.sh
└── README.md
```

공용: `../tasks/common/robot_spec.py` (로봇 값 한 곳) · `../tasks/common/onnx_export.py` (actor → ONNX)

## 정책

| 항목 | 값 |
|---|---|
| obs (6) | `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` |
| action (2) | `[Δv, Δw]` → clip 0.26 m/s · 1.82 rad/s |
| 주기 | 20 Hz (= cmd_manager) |
| 보상 | `-(|v_cmd−v_true| + 0.5|w_cmd−w_true|) − 0.01|Δ|² − 0.05|Δ−Δ_prev|²` |
| 랜덤화 | 모터 τ·좌우 gain·데드밴드·WiFi 지연·계단 명령 |

## 실행

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

./rl/train.sh 300                     # 창 없이 (기본)
./rl/train.sh 300 --visualizer kit    # 창 보면서 (느림)
tensorboard --logdir logs/rl_correct

./rl/export.sh logs/rl_correct/<날짜_시간>/model_299.pt rl_correct_v2.onnx
./rl/play.sh export/rl_correct/rl_correct_v2.onnx 4
```

- 로그: `logs/rl_correct/<날짜_시간>/`
- ONNX: `export/rl_correct/<이름>.onnx`

## 실기기 검증 (Before / After)

1. **먼저 센서/액추에이터 싱크** — `steps/README.md` S5 (공중 지지 박스 → 바닥)
2. PC: `ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=<onnx>`
3. Pi: `./run-cli route rl`
4. 같은 스크립트 주행으로 `rl_eval/eval_recorder` before/after CSV
5. `python3 ros2_ws/src/rl_eval/compare_csv.py /tmp/before.csv /tmp/after.csv`

합격(초안): 회전 RMS 20% 이상 감소, 직진 RMS 악화 없음.

## 주의

- `rl/task` 환경은 **sim 전용**. 실기기에서 관측 가능한 값만 obs로 쓴다.
- 학습 전에 `steps/`의 S5로 sim/real 신호를 먼저 맞춘다. 안 맞으면 학습이 아무리 잘 돼도 real에서 효과가 없다.
