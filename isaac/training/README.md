# training — RL 학습 task (Isaac Lab 관례)

task마다 폴더 하나. 학습 · 재생은 **Isaac Lab 공식 스크립트**(`scripts/reinforcement_learning/train.py` · `play.py`)가 한다 —
task는 `__init__.py`에서 `gym.register`로 이름을 등록하고, 공식 스크립트가 `--external_callback training.register`로 그 등록을 부른다.

```text
calibration/ 측정 (real · 순수 sim)  →  task env cfg에 차이를 숫자로
  → training/<task>/ 학습 (공식 train.py, rsl_rl PPO) → logs/rsl_rl/<exp>/<시각>/model_<N>.pt
  → 공식 play.py → exported/policy.onnx (신경망 출력 [-1, 1])
  → PC rl_correction 노드 (clamp × action_scale) → /cmd_vel_rl_out → Pi cmd_manager → 모터
```

## 구조

```text
training/
├── __init__.py                 register() — training/ 아래 task 폴더를 import (각 __init__.py의 gym.register)
└── rl_correction_wheel_drift/  바퀴 드리프트 보정 — task 이름 Beebot-WheelDrift
    ├── __init__.py             gym.register — 로봇마다 이름 하나 (Beebot-WheelDrift → BeebotWheelDriftEnvCfg)
    ├── env.py · env_cfg.py     RlCorrectEnv(BaseEnv) · RlCorrectEnvCfg(BaseEnvCfg, 로봇과 상관없는 task 규칙)
    │                           + BeebotWheelDriftEnvCfg(robot="beebot" + beebot 실측 중심)
    ├── command.py              TeleopCommand(BaseCommand) — 학습 명령
    └── agent.py                RlCorrectAgentCfg(BaseAgentCfg) — PPO 값
```

새 task: 폴더를 만들고 위 다섯 파일을 둔다 (`__init__.py`의 등록 이름이 `--task` 값).
새 로봇 (같은 task): `env_cfg.py`에 `XWheelDriftEnvCfg(RlCorrectEnvCfg)`(robot="x" + 그 로봇 실측) + `gym.register(id="X-WheelDrift", …)`.
실험으로 로봇만 바꾸기: `./run-training.sh env.robot=<이름|폴더>` (실측 중심은 그대로).
`__init__.py`는 Isaac 앱보다 먼저 import되므로 등록은 문자열 위치만 적는다.

## 실행 (`../run-training.sh` — 공식 스크립트를 우리 task로)

```bash
cd isaac
./run-training.sh --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 짧게 확인 (창)
./run-training.sh --max_iterations 300 env.uneven=true                             # 본 학습 (창 없음, 512대)
./run-training.sh play --checkpoint logs/rsl_rl/rl_correct/<시각>/model_299.pt --num_envs 1   # 재생 + ONNX 내보내기 (Ctrl+C)
tensorboard --logdir logs/rsl_rl
./run-training.sh -h                                                               # 자주 쓰는 옵션 · cfg 덮어쓰기
```

- cfg 덮어쓰기 (Hydra): `env.slope_deg=5` · `env.uneven=true` · `env.events=null`(마찰 랜덤 끔) · `env.gyro_bias=0.1` ·
  `agent.algorithm.entropy_coef=0.0`. 실제로 쓴 값은 `logs/…/<시각>/params/env.yaml` · `agent.yaml`.
- 결과: `logs/rsl_rl/<experiment_name>/<시각>/` — `model_<N>.pt`(50회마다) · `params/` · TensorBoard.
  `play`를 하면 같은 폴더에 `exported/policy.onnx` + `policy.onnx.data`(가중치) + `policy.pt`.
- 로봇에 올리기: `exported/policy.onnx`와 `policy.onnx.data`를 **둘 다** 복사. 출력은 [-1, 1] 그대로 —
  corrector가 `clamp × action_scale`(0.10 m/s · 0.60 rad/s, 학습 env `action_scale`과 같아야 함)을 한다.

### 화면 (`--viz kit` + `env.debug_vis=true`)

| 표시 | 뜻 |
|---|---|
| 초록 막대 | 지금 명령 (v, w)대로라면 갈 방향 (길이 ∝ 속도) |
| 파랑 막대 | 실제 몸체 속도로 가는 방향 |
| 노란 점 | 목표 경로 — 랜덤 명령이 바뀐 순간의 자세에서 그 명령대로 유지 시간(0.5~2 s) 동안 갈 길 |

로봇이 노란 점에서 벗어나는 만큼이 드리프트다. 학습이 될수록 줄어야 한다.

## 보정값과 RL의 관계

- 바퀴 **τ·gain·지연·데드밴드**는 env가 에피소드마다 랜덤으로 뽑는다. 범위의 **중심 = 실측 값**
  (calibration의 real − sim 차이를 task env cfg에 숫자로, 폭은 그대로).
- 바닥 마찰은 `envs/base_env_cfg.py`, 바퀴 · 캐스터 마찰은 `robots/beebot/config.py` (+ env마다 바퀴 마찰 랜덤 events).
- RL은 그 위의 **잔차**(좌우 비대칭, 슬립, 드리프트)를 학습한다.
