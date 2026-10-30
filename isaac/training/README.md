# training — RL 학습/보정 task 모음

Isaac Lab에서 돌리는 RL task를 **폴더별**로 둔다. 공통 로봇 값·산출물은 `isaac/` 루트에 있다.

```text
training/calibration/ 보정 (sim plant를 real에 맞춤)
  → training/rl_correction_wheel_drift/ 학습 (Isaac Lab, PPO)
  → ONNX
  → PC의 rl_correction 노드
  → /cmd_vel_rl_out
  → Pi cmd_manager → 모터
```

- Pi는 **모델을 돌리지 않는다**. Pi는 base·cmd_manager·드라이버만.
- RL 추론은 **PC**에서 한다.
- 학습은 sim에서 하고, real에서는 ONNX만 실행한다.

## 0. 전체 흐름

```text
[1] training/calibrate.sh
    sim + real에 같은 사각형+삼각형 명령
    → training/artifacts/{sim_sensor_floor,real_floor}.csv
    → training/artifacts/sim_calib_floor.json

[2-A] goal 정책                         [2-B] track 정책
    train --task goal                       train --task track
    → rl_goal.onnx                          → rl_correct.onnx
         │                                       │
    real_rl ... goal                        real_rl ... track
         │                                       │
    real_square.sh                          verify_track.sh
    (waypoint 재생)                          (같은 (v,w) 재생 + 비교)
         │                                       │
    plot_goal_square.sh                     compare_track_rl.png
```

- **goal** = 목표점 `(gx, gy)`를 따라감. 보정 때와 같은 **경로**를 waypoint로 재생.
- **track** = `(v,w)` 명령을 직접 보정. 보정 때와 **완전히 같은 명령**을 재생.
- 모든 결과는 `training/artifacts/`에 쌓인다.
- `training/artifacts/`는 git 제외.

## 0.1 training/ 실행 스크립트

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

# 1) calibration sim + real 동시 실행 + 비교 + 보고서
./training/calibrate.sh \
  --profile training/calibration/calib_default_profile.py \
  --side 0.5 --show

# 2) 학습 (task 디렉토리를 첫 인자로)
./training/train.sh rl_correction_wheel_drift 300 --task goal --friction

# 3) PC에서 RL corrector 노드 실행 (ONNX 파일)
./training/real_rl.sh \
  logs/rl_goal_friction/<날짜_시간>/rl_goal_friction.onnx goal

# 4) track 검증 (같은 v,w 재생 + 비교)
./training/verify_track.sh --show
```

- `calibrate.sh --profile PATH`: `.py` 파일 또는 그 디렉토리.
  - 기본 `training/calibration/calib_default_profile.py`
  - `rl_correction_wheel_drift/calib_profile.py`를 주면 task 전용 profile로 실행.
- `calibrate.sh`는 sim/real을 동시에 돌리고 `training/artifacts/calibrate_*/`에
  로그·CSV·PNG·JSON 보고서를 복사한다.
- `train.sh`는 task 디렉토리의 `train.py`를 실행하고
  `training/artifacts/train_*/`에 실행 로그와 최종 ONNX 사본을 남긴다.
- `real_rl.sh`는 Pi가 `route rl`인 상태에서 PC corrector를 띄우고,
  `training/artifacts/real_rl_<시각>/launch.log`에 실행 로그를 남긴다.

## 0.2 실행 순서 (보정 → 학습 → real_rl → 같은 경로 재생)

| 목적 | 학습 | PC 노드 | 재생/검증 | 그래프 |
|---|---|---|---|---|
| 같은 **경로** (waypoint) | `--task goal` | `real_rl ... goal` | `real_square.sh` | `plot_goal_square.sh` |
| 같은 **(v,w) 명령** | `--task track` | `real_rl ... track` | `verify_track.sh` | `compare_track_rl.png` |

### 1) Pi 준비 (보정 중에는 RL 끔)

```bash
ssh pak@pak.local
cd ~/ros2/tool
./run-cli base start
./run-cli control manual
./run-cli route direct      # 보정/02는 RL을 거치지 않게
```

### 2) 보정 — sim + real 동시에

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

./training/calibrate.sh \
  --profile training/rl_correction_wheel_drift/calib_profile.py \
  --side 0.5 --show
```

- `--profile`은 **파일 경로** 또는 그 디렉토리. 기본은
  `training/calibration/calib_default_profile.py`.
- 이 profile은 **사각형 → 180° 턴 → 같은 사각형 역주행 → 사각형 안쪽 삼각형**을 한다.
- sim과 real을 동시에 돌리고 약 73초 걸린다.
- 결과:
  - `training/artifacts/sim_sensor_floor.csv`
  - `training/artifacts/real_floor.csv`
  - `training/artifacts/sim_calib_floor.json`
  - `training/artifacts/calibrate_<시각>_<label>/` (로그·CSV·PNG·JSON 보고서)

### 3) 학습

```bash
# goal 정책 (waypoint 재생용)
./training/train.sh rl_correction_wheel_drift 300 --task goal --friction

# track 정책 (보정 때와 같은 (v,w) 명령 재생용)
./training/train.sh rl_correction_wheel_drift 300 --task track --friction

# 창 보면서 (goal/track 공통)
./training/train.sh rl_correction_wheel_drift 300 \
  --task goal --friction --watch --envs 16
```

- 학습 로그: `logs/rl_goal_friction/<시각>/`
- 실행 보고서: `training/artifacts/train_rl_correction_wheel_drift_<시각>/`

### 4) PC RL 노드 + Pi RL 경로

```bash
# PC — goal
./training/real_rl.sh \
  logs/rl_goal_friction/<시각>/rl_goal_friction.onnx goal

# PC — track (같은 (v,w) 재생용)
./training/real_rl.sh \
  training/artifacts/train_rl_correction_wheel_drift_<시각>/rl_correct.onnx track
```

노드가 뜨면 Pi에서:

```bash
ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli route rl'
```

로그에 `ONNX loaded ... mode=goal, goal_mode=absolute`가 나오고,
1초마다 `[goal] ... nominal=... model=... out=... applied=...`가 찍힌다.

### 5) 보정 때와 같은 경로를 RL로 재생

```bash
./training/rl_correction_wheel_drift/real_square.sh --side 0.5
```

- `calib_default_profile.make_waypoints()`의 **사각형 8 + 안쪽 삼각형 4 waypoint**를 보낸다.
- 결과: `training/artifacts/real_goal_square.csv`
- 비교 그래프:

```bash
./training/rl_correction_wheel_drift/plot_goal_square.sh --show
# → training/artifacts/compare_goal_square.png
```

그래프에서:

- Before = `real_floor.csv` (보정 때 open-loop)
- After = `real_goal_square.csv` (RL goal)
- x-y 경로, 목표 거리, **적용된 보정 Δv/Δw**, nominal vs out을 같이 본다.

### 5-T) track — 같은 `(v,w)` 명령으로 검증

track 노드가 `real_rl ... track`으로 떠 있고 Pi가 `route rl`이면:

```bash
./training/verify_track.sh   --profile training/rl_correction_wheel_drift/calib_profile.py   --show
```

이 한 번으로:

- `02_sensor_sync_real.sh`로 보정 때와 **같은 `(v,w)` 명령** 재생
- `training/artifacts/real_track_rl.csv`
- `training/artifacts/compare_track_rl.png`
- `training/artifacts/verify_track_<시각>/` 보고서

> goal은 **waypoint**, track은 **`(v,w)` 명령** 재생임.
> “보정 때와 완전히 같은 명령”이 목적이면 **track + verify_track.sh**를 쓴다.

### 6) 끝나면 안전하게 복귀

```bash
ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli route direct'
# PC corrector: Ctrl+C
```

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
| local friction patch | `--friction`이면 env마다 4개, **파랑=미끄럼 / 주황=끈적**, `--patch-count`로 개수 변경 |
| wheel / caster friction | `sim_params` floor 값 |

### 2.6 PPO

`--friction`을 켜면 env마다 ground slab을 따로 만들고 static/dynamic friction을 다르게 준다.
그 위에 **국소 마찰 패치**를 뿌린다:

- **파랑** = 미끄러운 구간 (static 0.20 / dynamic 0.15)
- **주황** = 끈적한 구간 (static 1.10 / dynamic 0.90)
- 기본 env당 4개, `--patch-count 0`이면 패치 없이 base friction만.
- 패치는 얇은 판이라 로봇이 지나가면서 국소적으로 마찰이 바뀐다.
- 학습 시작 시 env별로 고정. wheel/caster는 `sim_params` floor 값을 씀.

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

- `training/calibration/EXPERIMENT.md`로 floor 보정값을 먼저 만든다.
- Isaac 창은 한 번에 하나.
- 학습은 GPU/CPU에서 headless로 돌리는 것을 권장.

### 5.2 goal 학습

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

# 기본: 창 없이, 512 envs
./training/rl_correction_wheel_drift/train_goal.sh 300

# 창 보면서 (512 envs는 무거움 → 16~32 권장)
./training/rl_correction_wheel_drift/train_goal.sh 300 --watch --envs 16

# 바닥 마찰 randomization 켜기 (env별 ground friction)
./training/rl_correction_wheel_drift/train_goal_friction.sh 300
./training/rl_correction_wheel_drift/train_goal_friction.sh 300 --watch --envs 16

# 로그
tensorboard --logdir logs/rl_goal
tensorboard --logdir logs/rl_goal_friction
```

- 로그: `logs/rl_goal/<날짜_시간>/` · friction은 `logs/rl_goal_friction/<날짜_시간>/`
- 학습 끝나면 자동 export: `logs/rl_goal/<날짜_시간>/rl_goal.onnx`
- 중간 체크포인트: `model_<iter>.pt` (`save_interval=50`)

### 5.3 track 학습

```bash
./training/rl_correction_wheel_drift/train.sh 300
./training/rl_correction_wheel_drift/train.sh 300 --watch --envs 16
tensorboard --logdir logs/rl_correct
```

로그: `logs/rl_correct/<날짜_시간>/`

### 5.4 export

```bash
# goal
./training/rl_correction_wheel_drift/export.sh logs/rl_goal/<날짜_시간>/model_299.pt goal_v1.onnx goal

# track
./training/rl_correction_wheel_drift/export.sh logs/rl_correct/<날짜_시간>/model_299.pt rl_correct_v2.onnx
```

- `goal` ONNX 입력: `[1, 10]`
- `track` ONNX 입력: `[1, 6]`
- 출력: `[1, 2]` (물리 보정량 m/s, rad/s)
- 저장: `export/rl_goal/` · `export/rl_correct/`

### 5.5 play (sim에서 구경)

```bash
./training/rl_correction_wheel_drift/play.sh export/rl_goal/goal_v1.onnx 4
./training/rl_correction_wheel_drift/play.sh export/rl_correct/rl_correct_v2.onnx 4
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
- `corrector_node.py`는 ONNX 입력 차원으로 모드를 자동 판별한다:
  - `6` → `track`: `/cmd_vel_rl_in`을 보정
  - `10` → `goal`: `/rl_goal` 목표를 받아 nominal controller + 보정
- Pi는 `cmd_manager`가 `/cmd_vel_rl_out`을 받아 드라이버로 보낸다.

### 6.1 Pi — RL 경로로 전환

```bash
ssh pak@pak.local
cd ~/ros2/tool
./run-cli base start          # 이미 떠 있으면 생략
./run-cli control manual
./run-cli route rl            # 또는 ./run-cli rl on
./run-cli status              # RL on 확인
```

### 6.2 PC — corrector 빌드/실행

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/ros2_ws
colcon build --packages-select rl_correction
source install/setup.bash

source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=25
source /home/pank/workspaces/visualkhh/source/ros2/tool/peers.sh

ros2 launch tb3_bringup rl-only.launch.py \
  enable:=true \
  mode:=goal \
  model:=/home/pank/workspaces/visualkhh/source/ros2/isaac/logs/rl_goal_friction/<날짜_시간>/rl_goal_friction.onnx
```

`mode:=goal`은 생략해도 ONNX가 10차원이면 자동으로 goal 모드가 된다.

또는 래퍼:

```bash
./training/rl_correction_wheel_drift/real_launch.sh \
  logs/rl_goal_friction/<날짜_시간>/rl_goal_friction.onnx goal
```

이 래퍼는 `goal_mode=absolute`로 띄워서 **사각형+삼각형 waypoint**를 그대로 재생할 수 있게 한다.

### 6.3 goal 보내기 / 정지

목표는 **현재 자세 기준 변위** `(gx, gy)` m 이다.

```bash
# 앞으로 1 m
ros2 topic pub --once /rl_goal geometry_msgs/msg/Point "{x: 1.0, y: 0.0}"

# 왼쪽으로 0.5 m, 앞으로 0.5 m
ros2 topic pub --once /rl_goal geometry_msgs/msg/Point "{x: 0.5, y: 0.5}"

# 즉시 정지
ros2 topic pub --once /rl_goal_stop std_msgs/msg/Empty "{}"
```

- `/rl_goal`을 받은 순간의 `/odom` pose가 시작점이 된다.
- `dist < 0.05 m`면 자동 정지.
- `enable:=false`면 goal 모드에서도 정지.

### 6.4 보정 때 쓴 사각형+삼각형을 RL로 재생

`calib_profile.py`의 `make_waypoints()`가 02 보정과 같은 사각형+삼각형을 만든다.

```bash
# real_launch.sh는 goal_mode=absolute로 띄운다
./training/rl_correction_wheel_drift/real_launch.sh <model.onnx> goal

# 같은 사각형+삼각형을 RL goal로 주행 + CSV
./training/rl_correction_wheel_drift/real_square.sh --side 0.5
# → training/artifacts/real_goal_square.csv

# RL 전(real_floor.csv) / 후(real_goal_square.csv) 궤적 비교
./training/rl_correction_wheel_drift/plot_goal_square.sh --show
# → training/artifacts/compare_goal_square.png
```

- Before는 `training/calibration/02_sensor_sync_real.sh`가 만든 `real_floor.csv`.
- After는 `real_square.sh`가 만든 `real_goal_square.csv`.
- 그래프에 `final err`, `RMS dev`, `path len`이 같이 나온다.

### 6.4 goal 노드가 하는 일

1. `/rl_goal` 수신 → `(x0, y0, yaw0)` 저장
2. `/odom` pose → 시작점 기준 `(px, py, yaw)`
3. nominal controller → `(v, w)`
4. obs 10 → ONNX → `[Δv, Δw]`
5. `(v+Δv, w+Δw)` clip → `/cmd_vel_rl_out`
6. 20 Hz로 publish, 도착하면 0 publish

real 배포 전에 **300 iteration 이상** 충분히 학습하는 것을 권장.

---

### 6.5 track — 보정 때와 **완전히 같은 `(v,w)` 명령** 재생

goal은 waypoint 방식이라서 `02_sensor_sync_real.sh`의 `(v,w)` 명령을 직접 쓰지 않는다.
정확히 같은 명령을 RL로 재생하려면 **track 정책**을 쓴다.

```bash
# 1) track 학습
./training/train.sh rl_correction_wheel_drift 300 --task track --friction

# 2) PC corrector (track)
./training/real_rl.sh \
  training/artifacts/train_rl_correction_wheel_drift_<시각>/rl_correct.onnx track

# 3) Pi
ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli base start && ./run-cli control manual && ./run-cli route rl'

# 4+5) 검증 한 번에: 같은 (v,w) 재생 + before/after 비교
./training/verify_track.sh \
  --profile training/rl_correction_wheel_drift/calib_profile.py \
  --show
# → training/artifacts/real_track_rl.csv
# → training/artifacts/compare_track_rl.png
# → training/artifacts/verify_track_<시각>/ (로그·CSV·PNG 보고서)
```

- `verify_track.sh`가 내부에서 `02_sensor_sync_real.sh`와 `plot_compare.sh`를 순서대로 실행함.
- `02`는 `calib_default_profile.make_profile()`의 `(v,w)` 명령을 그대로 보낸다.
- `Pi cmd_manager → /cmd_vel_rl_in → PC track corrector → /cmd_vel_rl_out → Pi`.
- 비교용 `--calib-out`은 report 폴더로 보내서 실제 보정 JSON을 덮어쓰지 않음.

정리:

```text
goal  → real_square.sh (waypoint)
track → 02_sensor_sync_real.sh (똑같은 v,w 명령)
```

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
| sim이 real과 다름 | `training/calibration/EXPERIMENT.md`로 floor 보정 먼저 |

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
