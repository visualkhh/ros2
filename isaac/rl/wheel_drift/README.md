# wheel_drift — 바퀴 드리프트 보정 RL

목적: **같은 명령을 RL 끄고(Before) / 켜고(After) 줬을 때 목표에 더 정확히 가는지** 수치로 보인다.
설계·평가 기준은 [../../../documents/isaac/task-spec.md](../../../documents/isaac/task-spec.md).

- `steps/` = RL 전 셋팅 01~06 (환경·센서·실기기 싱크)
- `rl/wheel_drift/` = 그 위에서 도는 **바퀴 보정 정책** (환경 · 학습 · export · play)

## 두 가지 task

| task | 명령 | 보상 | 파일 |
|---|---|---|---|
| `track` (기본) | `(v, w)` 속도 | 명령 속도와 실제 속도 차이 | `rl_correct_env*` · `train.sh` |
| `goal` | 목표 변위 `(gx, gy)` | 목표까지 진행 − 옆으로 샌 거리 | `goal_env*` · `train_goal.sh` |

## 구조

```
rl/wheel_drift/
├── task/
│   ├── rl_correct_env_cfg.py  track 환경 (obs 6 · action 2)
│   ├── rl_correct_env.py
│   ├── goal_env_cfg.py        goal 환경 (obs 10 · action 2)
│   └── goal_env.py
├── agents/rsl_rl_ppo_cfg.py   PPO 설정
├── smoke.py                   track 물리 스모크
├── smoke_goal.py              goal 스모크 (목표 (1,0) m까지)
├── train.py · export.py · play.py
├── train.sh · train_goal.sh · export.sh · play.sh
└── README.md
```

공용: `../../tasks/common/robot_spec.py` · `../../tasks/common/onnx_export.py`

## goal 정책

| 항목 | 값 |
|---|---|
| obs (10) | `[gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_dv, prev_dw]` |
| action (2) | `[Δv, Δw]` — nominal controller 출력에 더하는 보정 |
| 주기 | 20 Hz (= cmd_manager) |
| 목표 | 시작 자세 기준 `(gx, gy)`, 반경 `0.4~1.2 m` 랜덤 |
| 보상 | `(d_prev−d)/‖g‖ − 0.5·max(0, lateral−0.05)/‖g‖ − action/smooth penalty` |
| 종료 | `d < 0.05 m` 성공, 또는 12초 timeout |
| 랜덤화 | 목표 방향·거리, 초기 yaw, 모터 τ·좌우 gain·데드밴드·WiFi 지연 |

- `px, py, yaw`는 `/odom`을 흉내낸 적분값 (시작점 0).
- `v_enc, w_enc`는 `/odom` twist, `w_gyro`는 `/imu` z.
- **보상은 sim의 true 변위**로 계산하지만, **obs에는 true pose를 넣지 않는다** (sim-to-real).
- nominal controller: 목표 방향으로 `(v, w)`를 만들고, 정책은 `[Δv, Δw]`만 보정.

## 실행

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

# goal 학습 (창 없이)
./rl/wheel_drift/train_goal.sh 300
tensorboard --logdir logs/rl_goal

# 학습 끝나면 자동 export: logs/rl_goal/<날짜_시간>/rl_goal.onnx
./rl/wheel_drift/play.sh logs/rl_goal/<날짜_시간>/rl_goal.onnx 4

# checkpoint에서 직접 export
./rl/wheel_drift/export.sh logs/rl_goal/<날짜_시간>/model_299.pt goal_v1.onnx goal
```

- `play.py`는 ONNX 입력 차원으로 task를 자동 판별함 (6=track, 10=goal).
- 기존 track 학습은 그대로:

```bash
./rl/wheel_drift/train.sh 300
./rl/wheel_drift/play.sh export/rl_correct/rl_correct_v2.onnx 4
```

- 로그: `logs/rl_goal/<날짜_시간>/` · `logs/rl_correct/<날짜_시간>/`
- ONNX: `export/rl_goal/<이름>.onnx` · `export/rl_correct/<이름>.onnx`

## 실기기 검증 (Before / After)

1. **먼저 센서/액추에이터 싱크** — [`steps/PRE_RL.md`](../../steps/PRE_RL.md) · 02~04
2. PC: `ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=<onnx>`
3. Pi: `./run-cli route rl`
4. 같은 목표 주행으로 `rl_eval/eval_recorder` before/after CSV
5. `python3 ros2_ws/src/rl_eval/compare_csv.py /tmp/before.csv /tmp/after.csv`

합격(초안): 목표 도달 오차 감소, 회전 RMS 20% 이상 감소, 직진 RMS 악화 없음.

## 주의

- `rl/wheel_drift/task` 환경은 **sim 전용**. 실기기에서 관측 가능한 값만 obs로 쓴다.
- 학습 전에 `steps/`의 02~04로 sim/real 신호를 먼저 맞춘다. 안 맞으면 학습이 아무리 잘 돼도 real에서 효과가 없다.
