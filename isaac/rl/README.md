# rl — RL task 모음

Isaac Lab에서 돌리는 RL task를 **폴더별**로 둔다. 공통 로봇 값·산출물은 `isaac/` 루트에 있다.

```text
steps/ 보정 (sim plant를 real에 맞춤)
  → rl/ 학습 (Isaac Lab, PPO)
  → ONNX
  → PC의 rl_correction 노드
  → /cmd_vel_rl_out
  → Pi cmd_manager → 모터
```

- Pi는 **모델을 돌리지 않는다**. Pi는 base·cmd_manager·드라이버만.
- RL 추론은 **PC**에서 한다.
- 학습은 sim에서 하고, real에서는 ONNX만 실행한다.

---

## 1. task 종류

| task | 명령 | 관측 | 보상 | 실행 |
|---|---|---|---|---|
| `track` | `(v, w)` 속도 | 6 | 명령 속도와 실제 속도 차이 | `train.sh` |
| `goal` | 목표 변위 `(gx, gy)` | 10 | 목표까지 진행 − 옆으로 샌 거리 | `train_goal.sh` |

둘 다 action은 `[Δv, Δw]` **보정량** 2개다.

---

## 2. goal task가 도는 방식

### 2.1 에피소드 시작 (`_reset_idx`)

- 로봇 위치를 env origin에 놓고 **초기 yaw를 랜덤**으로 준다.
- 목표 변위를 뽑는다:
  - 반경 `0.4 ~ 1.2 m`
  - 방향 `-π ~ +π` 균등
  - 시작 자세 기준 `(gx, gy)`
- wheel 모델을 랜덤화한다 (τ, 좌우 gain, deadband, 지연).
- odom 적분값 `(px, py, yaw)`를 0으로 초기화.

### 2.2 20 Hz 정책 스텝

```text
1) nominal controller
   odom(지연) 기준 목표 방향/거리 계산
   → 기본 (v, w)

2) policy
   obs 10개 → ONNX → [Δv, Δw] ([-1,1])

3) 보정 적용
   v_out = clip(v + Δv·0.10, ±0.26)
   w_out = clip(w + Δw·0.60, ±1.82)

4) 명령 지연
   0~2 step (0~0.1 s) 버퍼

5) 바퀴 모델
   (v,w) → 좌/우 wheel rad/s
   → wheel별 gain
   → deadband
   → 1차 지연 τ
   → PhysX

6) 관측
   odom 적분 + /imu w_gyro
   + 목표 (gx,gy) + 직전 action
   → 잡음 + 0~2 step 지연

7) 보상
   sim의 true 변위로 계산 (정책 obs에는 안 넣음)

8) 종료
   목표 5 cm 이내 → success
   12초 → timeout
```

핵심은 **nominal controller가 목표점까지 대략 가고, RL은 wheel drift만 보정**하는 구조라는 점.

### 2.3 관측 (10)

```text
[gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_dv, prev_dw]
```

| 값 | 출처 |
|---|---|
| `gx, gy` | 목표 변위 (시작 자세 기준) |
| `px, py, yaw` | `/odom` pose를 시작점 기준으로 적분 |
| `v_enc, w_enc` | `/odom` twist |
| `w_gyro` | `/imu` angular z |
| `prev_dv, prev_dw` | 직전 정책 action |

- **true pose / true velocity는 obs에 넣지 않는다.**
- real에서 볼 수 있는 값만 넣어야 sim-to-real이 된다.

### 2.4 보상

```text
d       = ||goal - true_pos||
progress = (d_prev - d) / ||goal||
lateral  = max(0, 목표 직선에서 벗어난 거리 - 0.05 m) / ||goal||

reward = 1.0 · progress
         - 0.5 · lateral
         - 0.01 · ||action||²
         - 0.05 · ||action - prev_action||²
```

- 목표에 가까워지면 `+`
- 멀어지면 `-`
- 옆으로 새면 `-`
- 목표까지 곧게 가면 **누적 보상 ≈ +1**
- `d < 0.05 m`면 성공 종료

### 2.5 랜덤화

| 항목 | 범위 |
|---|---|
| wheel τ | `0.05 ~ 0.25 s` (좌/우 따로) |
| wheel gain | `0.90 ~ 1.10` (좌/우 따로) |
| deadband | `0 ~ 0.3 rad/s` |
| 명령 지연 | `0 ~ 2 step` (0~0.1 s) |
| 관측 지연 | `0 ~ 2 step` |
| odom wheel radius 오차 | ±3% |
| odom wheel separation 오차 | ±8% |
| encoder / gyro 잡음 | 0.005 / 0.01 |
| 초기 yaw | `-π ~ +π` |
| 목표 | 반경 0.4~1.2 m, 랜덤 방향 |
| ground friction | 기본: `sim_params` floor 값 · `--friction`: env별 static 0.50~0.90 / dynamic 0.40~0.80 |
| wheel / caster friction | `sim_params` floor 값 |

### 2.6 PPO

`--friction`을 켜면 env마다 ground slab을 따로 만들고 static/dynamic friction을 다르게 준다.
(학습 시작 시 env별로 고정. wheel/caster는 `sim_params` floor 값을 씀.)
기본 학습은 ground plane 하나에 floor 보정 마찰값을 쓴다.

`agents/rsl_rl_ppo_cfg.py`:

- 512 envs (기본), 20 Hz 정책 주기
- `num_steps_per_env = 64`
- `max_iterations = 300`
- actor/critic 64×64 ELU
- lr `3e-4`, gamma `0.99`, lambda `0.95`
- entropy `0.005`, clip `0.2`
- 학습 끝나면 actor를 ONNX로 자동 export

---

## 3. track task (기존)

- obs 6: `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]`
- action 2: `[Δv, Δw]`
- 보상: `-(|v_cmd−v_true| + 0.5|w_cmd−w_true|) - action/smooth penalty`
- 명령은 텔레오퍼 키·부드러운 곡선·정지가 랜덤으로 바뀜.
- `steps` 센서 싱크와 같은 wheel 모델을 쓴다.

---

## 4. 보정값과 RL의 관계

현재 코드 기준:

- `steps`의 `sim_calib_floor.json`은 **`01_random_world_sim` · `02_sensor_sync_sim`** 이 읽는다.
- RL task(`goal_env`, `rl_correct_env`)는 `sim_params`에서 **바닥/바퀴/캐스터 마찰**을 읽어 장면에 적용한다.
- 다만 wheel `tau/gain/deadband`와 actuator damping/effort는 아직 RL cfg의 자체 randomization 범위를 쓴다.
- 즉 “steps 보정값을 RL 학습의 **중심값**으로 넣는 작업”은 tau/gain 쪽에 아직 남아 있다.

권장 방향:

1. `steps`에서 floor plant를 real에 맞춘다.
2. RL env의 randomization **중심**을 그 값으로 잡는다.
3. 그 주변을 넓게 랜덤화해서 real의 불확실성을 덮는다.
4. RL은 그 위의 **잔차**(좌우 비대칭, 슬립, 드리프트)를 학습한다.

RL이 보정값 자체를 대체하지는 못한다. 특히 wheel radius/sep 스케일, 바닥 슬립, 캐스터 마찰은
odom만으로 관측이 안 되므로 sim plant를 맞추는 게 먼저다.

---

## 5. 실행

### 5.1 준비

- `steps/EXPERIMENT.md`로 floor 보정값을 먼저 만든다.
- Isaac 창은 한 번에 하나.
- 학습은 GPU/CPU에서 headless로 돌리는 것을 권장.

### 5.2 goal 학습

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

# 기본: 창 없이, 512 envs
./rl/wheel_drift/train_goal.sh 300

# 창 보면서 (512 envs는 무거움 → 16~32 권장)
./rl/wheel_drift/train_goal.sh 300 --watch --envs 16

# 바닥 마찰 randomization 켜기 (env별 ground friction)
./rl/wheel_drift/train_goal_friction.sh 300
./rl/wheel_drift/train_goal_friction.sh 300 --watch --envs 16

# 로그
tensorboard --logdir logs/rl_goal
tensorboard --logdir logs/rl_goal_friction
```

- 로그: `logs/rl_goal/<날짜_시간>/` · friction은 `logs/rl_goal_friction/<날짜_시간>/`
- 학습 끝나면 자동 export: `logs/rl_goal/<날짜_시간>/rl_goal.onnx`
- 중간 체크포인트: `model_<iter>.pt` (`save_interval=50`)

### 5.3 track 학습

```bash
./rl/wheel_drift/train.sh 300
./rl/wheel_drift/train.sh 300 --watch --envs 16
tensorboard --logdir logs/rl_correct
```

로그: `logs/rl_correct/<날짜_시간>/`

### 5.4 export

```bash
# goal
./rl/wheel_drift/export.sh logs/rl_goal/<날짜_시간>/model_299.pt goal_v1.onnx goal

# track
./rl/wheel_drift/export.sh logs/rl_correct/<날짜_시간>/model_299.pt rl_correct_v2.onnx
```

- `goal` ONNX 입력: `[1, 10]`
- `track` ONNX 입력: `[1, 6]`
- 출력: `[1, 2]` (물리 보정량 m/s, rad/s)
- 저장: `export/rl_goal/` · `export/rl_correct/`

### 5.5 play (sim에서 구경)

```bash
./rl/wheel_drift/play.sh export/rl_goal/goal_v1.onnx 4
./rl/wheel_drift/play.sh export/rl_correct/rl_correct_v2.onnx 4
```

- `play.py`가 ONNX 입력 차원으로 task를 자동 판별한다 (`6=track`, `10=goal`).
- 창이 뜨고 로봇이 정책을 따른다.

---

## 6. real 배포 (PC / Pi)

```text
Pi
  base + cmd_manager
  /cmd_vel_rl_out → 드라이버 → 바퀴

PC
  rl_correction/corrector
  /cmd_vel_rl_in + /odom + /imu
    → ONNX
    → /cmd_vel_rl_out
```

- Pi에는 ONNX가 없다. PC에서만 추론.
- 실행:

```bash
ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=<onnx>
```

- **현재 `corrector_node.py`는 obs 6 (track) 기준**이다.
- goal ONNX(obs 10)를 real에 올리려면 goal용 obs를 계산하는 모드/노드가 필요하다:
  - 목표 `(gx, gy)` 입력
  - `/odom` pose 적분 (`px, py, yaw`)
  - `/imu` `w_gyro`
  - 직전 action
- goal real 배포는 아직 미구현.

---

## 7. 파일 구조

```text
rl/
├── wheel_drift/
│   ├── task/
│   │   ├── rl_correct_env_cfg.py   track (obs 6)
│   │   ├── rl_correct_env.py
│   │   ├── goal_env_cfg.py         goal (obs 10)
│   │   └── goal_env.py
│   ├── agents/rsl_rl_ppo_cfg.py
│   ├── train.py · export.py · play.py
│   ├── smoke.py · smoke_goal.py
│   ├── train.sh · train_goal.sh · train_goal_friction.sh · export.sh · play.sh
│   └── README.md
└── README.md
```

공통:

- `tasks/common/robot_spec.py` — 바퀴/기구학/최대 속도
- `tasks/common/onnx_export.py` — actor → ONNX

산출물 (git 제외):

- `logs/` — TensorBoard, checkpoint
- `export/` — ONNX

---

## 8. 문제 생기면

| 증상 | 확인 |
|---|---|
| 학습이 느림 | `--envs` 줄이기, `--watch` 끄기 |
| GUI가 너무 무거움 | `--watch --envs 16` |
| 보상이 안 오름 | `Metrics/goal_dist`, `Metrics/success`, action std |
| goal이 안 감 | nominal controller, obs delay, wheel clipping |
| real에서 안 됨 | obs 차원(6/10), `corrector_node` 모드, ONNX 경로 |
| sim이 real과 다름 | `steps/EXPERIMENT.md`로 floor 보정 먼저 |

---

## 9. 평가

### goal

- 목표 도달 성공률
- 최종 `||goal - pos||`
- 시작/끝 궤적 오차
- 같은 목표를 RL Before/After로 비교

### track

- `v_enc`, `w_enc` RMS
- `w_gyro` RMS
- 직진/회전 bias

real Before/After는 `rl_eval/eval_recorder` + `compare_csv.py`로 비교한다.
