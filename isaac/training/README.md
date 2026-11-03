# training — RL 학습 task (Isaac Lab 관례)

task마다 폴더 하나. 학습 · 재생은 **Isaac Lab 공식 스크립트**(`scripts/reinforcement_learning/train.py` · `play.py`)가 한다 —
task는 `__init__.py`에서 `gym.register`로 이름을 등록하고, 공식 스크립트가 `--external_callback training.register`로 그 등록을 부른다.

```text
1. calibration (학습 전에 먼저): ./run-calibration.sh sim|real --task <task> → training/<task>/calibration/의 명령 · env로 측정
   → 차이를 training/<task>/robot_cfg.py RESPONSE_*에 숫자로
2. 학습
  → training/<task>/ 학습 (공식 train.py, rsl_rl PPO) → logs/rsl_rl/<exp>/<시각>/model_<N>.pt
  → 공식 play.py → exported/policy.onnx (신경망 출력 [-1, 1])
  → PC rl_correction 노드 (clamp × action_scale) → /cmd_vel_rl_out → Pi cmd_manager → 모터
```

## 구조

```text
training/
├── __init__.py                 register() — training/ 아래 task 폴더를 import (각 __init__.py의 gym.register)
└── rl_correction_waypoint/     랜덤 웨이포인트를 이탈 없이 — task 이름 RlCorrectionWaypoint
    ├── __init__.py             gym.register
    ├── env.py · env_cfg.py     RlCorrectionWaypointEnv(BaseEnv) · RlCorrectionWaypointEnvCfg(BaseEnvCfg — 로봇과 상관없음, 실측 중심은 robot_cfg.py)
    ├── command.py              RlCorrectionWaypointCommand — env마다 랜덤 웨이포인트
    ├── calibration/            1단계 측정: command.py CalibrationCommand(SquareCommand) · env.py CalibrationEnv(DefaultEnv) · env_cfg.py CalibrationEnvCfg
    ├── agent.py                RlCorrectionWaypointAgentCfg(BaseAgentCfg) — PPO 값
    ├── robot_cfg.py            로봇: ROBOT_CFG · 바퀴 숫자 · 모터 · 마찰 · 실측 응답 RESPONSE_*
    ├── sim_robot.py            SimRobot(robots.sim_robot.SimRobot) — 이 로봇의 sim 입출력 (지금은 공용 그대로)
    ├── real_robot.py           RealRobot(RobotInterface) — 이 로봇의 실기기 ROS 입출력 (/cmd_manual_vel · /odom · /imu · /joint_states)
    └── usda/robot.usda         로봇 USD (외부 제공, 변환 없이 그대로)
```

새 task: 폴더를 만들고 위 파일들을 둔다 (이름 고정 — `robot_cfg.py` · `usda/robot.usda`) (`__init__.py`의 등록 이름이 `--task` 값).
로봇 = task 폴더의 `robot_cfg.py` + `usda/robot.usda` (`robots.DEFAULT`). 다른 로봇: `./run-training.sh env.robot=<이름|폴더>`. 그 로봇의 `robot_cfg.py`가 바퀴 숫자와 **실측 응답**(`RESPONSE_*`)을 채우고, task는 그 주변 랜덤 폭만 갖는다.
`__init__.py`는 Isaac 앱보다 먼저 import되므로 등록은 문자열 위치만 적는다.

## 실행 (`../run-training.sh` — 공식 스크립트를 우리 task로)

```bash
cd isaac
./run-training.sh --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 짧게 확인 (창)
./run-training.sh --max_iterations 500 env.uneven=true                             # 본 학습 (창 없음, 512대)
./run-training.sh play --checkpoint logs/rsl_rl/rl_waypoint/<시각>/model_499.pt --num_envs 1   # 재생 + ONNX 내보내기 (Ctrl+C)
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
| 노란 점 | 웨이포인트 경로 — 시작 → 웨이포인트 4개를 잇는 직선 |

로봇이 노란 선에서 벗어나는 만큼이 경로 이탈이다. 학습이 될수록 줄어야 한다 (TensorBoard `Metrics/cross_track`).

## 보정값과 RL의 관계

- 바퀴 **τ·gain·지연·데드밴드**는 env가 에피소드마다 랜덤으로 뽑는다. 범위의 **중심 = 실측 값**
  (calibration의 real − sim 차이를 `robot_cfg.py` `RESPONSE_*`에 숫자로, task는 폭만).
- 바닥 마찰은 `envs/base_env_cfg.py`, 바퀴 · 캐스터 마찰은 `robot_cfg.py` (+ env마다 바퀴 마찰 랜덤 events).
- RL은 그 위의 **잔차**(좌우 비대칭, 슬립, 드리프트)를 학습한다.
