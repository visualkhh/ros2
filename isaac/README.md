# isaac — 바퀴 드리프트 보정 RL (sim 보정 → 학습 → 실기기)

같은 `(v, w)` 명령을 넣어도 실제 로봇은 모터 지연 · 좌우 바퀴 차이 · 데드밴드 · 슬립 때문에 다르게 움직인다.
이걸 PC의 정책이 `[Δv, Δw]`로 보정한다. **학습은 Isaac sim에서, 추론은 PC(ROS)에서** — Pi에는 모델이 없다.

목적·합격 기준: [../documents/isaac/task-spec.md](../documents/isaac/task-spec.md)
(같은 경로를 RL 끄고/켜고 달려 회전 추종 RMS 20%↓ · 직진 RMS 악화 없음).

```text
calibration/  real·sim 응답 측정 → fit.json (sim을 real에 맞춤)
  → training/   Isaac Lab PPO 학습 (실기기 맞춤 값 중심 랜덤화 — cfg에 숫자) → policy.onnx
  → ros2_ws/rl-only.sh   PC corrector가 ONNX로 보정 + 기록(raw.csv)
  → ros2_ws/report.py    Before/After 그림
```

## 1. 처음이면 — 강화학습 5단어

| 단어 | 뜻 | 여기서 |
|---|---|---|
| 환경(env) | 연습장 | Isaac Lab 지형(평평·경사·요철) + 모터가 늦고 좌우가 다르고 통신이 늦는 가짜 로봇 512대 |
| 관측(obs) | 정책이 보는 것 | `[명령 v, 명령 w, 엔코더 v, 엔코더 w, 자이로 w, 명령 바뀐 뒤 시간]` — **실기기에서도 보이는 것만** |
| 행동(action) | 정책의 답 | `[Δv, Δw]` 보정량 (최대 0.10 m/s, 0.60 rad/s) |
| 보상(reward) | 점수 | 실제 몸체 속도가 명령에 가까우면 +, 보정이 크거나 출렁이면 − |
| 에피소드 | 한 판 | 10초. 명령은 0.5~2초마다 바뀜 (텔레오퍼 키 · 부드러운 곡선 · 정지) |

Isaac Sim = 물리 시뮬레이터, Isaac Lab = 그 위의 강화학습 도구(로봇 수백 대 복제). PPO는 그냥 쓴다.
개념 실습(하드웨어 없이): [learn/README.md](learn/README.md) — `$ISAACLAB_DIR/isaaclab.sh -p learn/l01_robot_anatomy.py --viz kit`.

## 2. 구조

```text
isaac/
├── run-calibration.sh        보정 실행기 (sim=IsaacLab · real=ROS · dummy/plot=.venv)
├── run-real-ab.sh            실기기 A/B — ./run-real-ab.sh <task> --onnx <policy.onnx> (A 바이패스 → B 정책 → training/<task>/report.py)
├── run-make-onnx.sh          체크포인트 → ONNX: ./run-make-onnx.sh <model_N.pt> [--quantize] (tools/onnx_tool.py · play 창 없이, 스스로 끝남)
├── run-training.sh           학습 · 재생 — ./run-training.sh <task> [--play] (Isaac Lab 공식 train.py · play.py → artifacts/<task>/training/)
├── run-usd.sh                로봇 URDF → USD 변환만 (./run-usd.sh <이름|디렉토리>)
├── run-world.sh              world 보기 — ./run-world.sh <task> [--calibration] [--command keyboard]
├── envs/                     환경 바탕 — 모든 env가 상속
│   ├── base_env.py · base_env_cfg.py   BaseEnv (로봇 1대 · 평평 바닥 · 480/20 Hz · 드라이버 식 · 리셋 · 화면 막대)
│   ├── default/              DefaultEnv — 아무것도 없는 기본 환경 (새 환경은 이걸 복사해 시작)
│   ├── rooms/                RoomsEnv — 벽 + 장애물 방 여러 개
│   ├── drive.py              운전 창 (./run-world.sh) — 로봇 · env · 명령 선택
├── agents/base_agent.py      학습자 설정 바탕 BaseAgentCfg (PPO · 신경망 64×64 · 학습률 · 저장 간격) — task agent.py가 상속
├── commands/                 명령 경로 (Isaac Lab command): base_command.BaseCommand · square(네모 54 s — calibration 측정) · keyboard(직접 운전)
├── tools/onnx_tool.py        ONNX 내보내기 · INT8 양자화 + fp32와 오차 비교 (isaac venv)
├── calibration/__main__.py   calibration 실행기 하나 (공식 train.py 역할) — ./run-calibration.sh <task> --robot sim|real|dummy → raw.csv
├── reports/base_report.py    BaseReport(ABC) — 측정 폴더 1개 single() · 2개 compare() (task 보고서가 상속: calibration/report.py CalibrationReport)
├── training/                 학습                                           → training/README.md
│   ├── __init__.py           register() — 공식 train.py · play.py가 --external_callback으로 부름
│   └── rl_correction_waypoint/      task RlCorrectionWaypoint (랜덤 웨이포인트 이탈 없이): __init__(gym.register) · env · env_cfg · command · agent ·
│                                    calibration/(1단계 측정: command · env · env_cfg) ·
│                                    robot_cfg.py(로봇 ROBOT_CFG · 바퀴 숫자 · 모터 · 마찰 · 실측 응답 — 기본 로봇) · usda/robot.usda(외부 제공 USD) ·
│                                    sim_robot.py · real_robot.py · dummy_robot.py(이 로봇의 sim · 실기기 · 가짜 입출력)
├── robots/                   로봇 공통 · urdf_to_usd.py 변환 · check_same.sh(ROS URDF와 같은지)
│   ├── robot_interface.py    RobotInterface(ABC) — 추상 write(v, w) · read() 상태 (ros2_control hardware_interface 역할), 구현은 task 폴더
│   └── beebot/               urdf/robot.urdf.xacro · meshes/ · usd/(변환물, git 제외) — URDF만 (로봇으로 쓰려면 robot_cfg.py 추가)
├── learn/                    Isaac 개념 실습 L1~L5
├── artifacts/                산출물 (git 제외) — calibration/ · training/ · real/
├── requirements.txt · setup_venv.sh   실기기·리포트용 .venv (sim/학습은 IsaacLab venv)
```

로봇 숫자(`training/rl_correction_waypoint/robot_cfg.py`): 바퀴 반지름 0.033 m · 간격 0.287 m · 최대 0.26 m/s · 1.82 rad/s · 바퀴 8 rad/s. 주기(`envs/base_env_cfg.py`): 정책 20 Hz · 물리 480 Hz.
로봇 폴더 모양은 두 가지: `training/<task>/{robot_cfg.py, usda/robot.usda}`(외부 USD 그대로 — 기본) · `robots/<이름>/{robot_cfg.py, urdf/, meshes/}`.
URDF 쪽 USD는 URDF · 메시가 바뀌면 자동으로 다시 변환된다 (`robots/urdf_to_usd.py` `robot_usd()` → `robots/<robot>/usd/`, git 제외).
따로 변환만: `./run-usd.sh [robot]` (Isaac Sim을 창 없이 켜서 변환 — 변환기가 Isaac Sim 안에 있다).
다른 로봇: `robots/<이름>/`에 `urdf/robot.urdf.xacro` + `meshes/` + `robot_cfg.py`(숫자 · `ROBOT_CFG`)를 두고 task 폴더에서 쓴다 (코드에선 `robots.use(env_cfg, "<이름>")`).
측정 경로를 바꾸려면 `commands/square.py`(또는 BaseCommand를 상속한 새 명령), 측정 장면은 `envs/default`(또는 `envs/rooms`)다.

## 3. 절차

real 측정 → 순수 sim 측정 (calibration은 여기까지) → **RL 쪽 env cfg에 차이를 숫자로** → 그 env로 맞았는지 확인 → 학습 → 실기기에서 A/B.
경로는 `isaac/` 기준. 예시 폴더 이름은 2026-11-02 실제 결과. `--fit` 같은 런타임 보정 옵션은 없다 — 숫자는 코드에 있다.

### 3.0 준비 (처음 한 번)

```bash
cd isaac && ./setup_venv.sh                                               # real 측정·리포트용 .venv
cd ../ros2_ws && colcon build --packages-select rl_correction rl_eval tb3_bringup   # PC
```

### 3.1 real 측정 — 실기기 사각형

```bash
# 로봇: 베이스 (Pi에서 또는 PC tool로)
./tool/run-cli base start
# PC: RL 노드 바이패스 (보정 없이 통과 — 정책을 쓸 때와 같은 경로 · 같은 PC 왕복 지연)
cd ros2_ws && ./rl-only.sh enable:=false label:=calib   # 바이패스 필수 (기본은 정책 켬)
# MANUAL · 경로 rl
./tool/run-cli control manual  &&  ./tool/run-cli route rl
cd isaac
./run-calibration.sh rl_correction_waypoint --robot real --check-only     # 움직이지 않고 상태만 확인 (state=MANUAL)
./run-calibration.sh rl_correction_waypoint --robot real --label real_floor
#   → artifacts/rl_correction_waypoint/calibration/<시각>_real_floor/raw.csv  (fit.json · report.png는 report.py가)   (--label 없으면 <시각>_02-real)
```

- 로봇 주변 1 m 비우기. 사각형(한 변 0.5 m) → 180° 턴 → 역주행, 약 54초. 중간에 잡지 말 것. 자세한 순서: `training/rl_correction_waypoint/README.md` calibration 절.
- 2026-11-02 측정은 직결(`route direct`)이었다 — rl 경로로 다시 재면 PC 왕복만큼 지연이 늘 수 있다 (`RESPONSE_DELAY_STEPS`).
- `fit.json` = 측정 결과 (gain 좌/우 · 지연 · τ · 데드밴드). 2026-11-02(`20261102_140931-02-real`): gain 0.9943/0.9951 · 지연 2스텝(0.1 s) · τ 0.046 s.
  τ는 20 Hz 기록의 한계값 · 데드밴드 0은 "못 잼"(저속 구간 없음).

### 3.2 sim 측정 — 같은 명령, 순수 sim (envs/default, 보정 없음)

```bash
./run-calibration.sh rl_correction_waypoint --robot sim --label sim          # 창 없음 (보려면 --viz)
python3 training/rl_correction_waypoint/calibration/report.py artifacts/rl_correction_waypoint/calibration/<시각>_sim artifacts/rl_correction_waypoint/calibration/<시각>_real_floor
#   → 폴더마다 fit.json · report.png, real 폴더에 compare.png, robot_cfg.py에 붙일 RESPONSE_* 값 출력
```

- 사각형 기본값(`--side 0.5 --speed 0.15 --turn 1.0`)이 real과 같아 **같은 명령**이 재생된다.
- 2026-11-02(`20261102_144928-sim_480`): gain 1.0016/1.0013 · 지연 0 · τ 0.05 s — sim 로봇 자체 응답.
- sim이 real과 크게 다르면 먼저 sim 물리를 의심한다 (2026-11-02에 고친 것 — 아래 4번).

### 3.3 RL 쪽 env에 숫자 넣기 — real − sim 차이

calibration은 측정만 한다. 잰 차이는 **로봇 설정**(`robot_cfg.py`의 `RESPONSE_*`)에 숫자로 넣는다 — 로봇의 성질이라서.
학습 task는 그 값을 랜덤 범위의 **중심**으로 쓰고 폭만 자기가 정한다 (`env.robot=`으로 로봇을 바꾸면 그 로봇 값이 따라온다).
기본 로봇(`training/rl_correction_waypoint/robot_cfg.py`)에 넣은 값 (2026-11-03, real = route rl + rl-only 바이패스):

| 값 | 계산 | 2026-11-03 (`artifacts/rl_correction_waypoint/calibration/2026-11-03_20-34-42_real_floor` vs `…_20-22-20_sim_aa`) | `robot_cfg.py` |
|---|---|---|---|
| 좌/우 gain | real ÷ sim | 0.9947/1.0010 · 0.9943/1.0012 = 0.9937 / 0.9931 | `RESPONSE_GAIN_LR` |
| 명령 지연 | real − sim | 3 − 0 = +3 스텝 (real 0.173 s, PC 왕복 포함 — direct였던 11-02는 +2) | `RESPONSE_DELAY_STEPS` |
| τ | real − sim (음수면 0) | 0.0227 − 0.05 < 0 → 0 | `RESPONSE_TAU_S` |
| 데드밴드 | real ÷ sim gain (sim이 더 크면 0) | 0 | `RESPONSE_DEADBAND` |

real 값을 그대로 넣으면 안 된다 — sim 로봇의 자체 응답(3.2)에 **더해지는** 값이라 차이만 넣는다.
맞았는지는 그 env로 학습 장면을 보며(`./run-training.sh rl_correction_waypoint --num_envs 16 --viz kit env.debug_vis=true`) 확인한다.

### 3.4 학습 — Isaac Lab 공식 train.py · play.py

```bash
./run-training.sh rl_correction_waypoint --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 짧게 확인 (창)
./run-training.sh rl_correction_waypoint --max_iterations 500                                            # 본 학습 (창 없음, 512대)
./run-training.sh rl_correction_waypoint --max_iterations 500 env.ground.uneven=false                          # 평면만 (기본은 평평 + 요철)
tensorboard --logdir artifacts/rl_correction_waypoint/training                                                  # 진행 그래프 (다른 터미널)
#   → artifacts/rl_correction_waypoint/training/<시각>/ (model_<N>.pt · params/env.yaml · TensorBoard)
./run-training.sh rl_correction_waypoint --play --checkpoint artifacts/rl_correction_waypoint/training/<시각>/model_499.pt --num_envs 1
#   → <시각>/exported/policy.onnx + policy.onnx.data (+ policy.pt) — 내보낸 뒤 재생이 계속 돈다, Ctrl+C로 끝
```

- task 이름 `RlCorrectionWaypoint` — 랜덤 웨이포인트 4개를 이탈 없이 (기본 제어기 + 정책 보정). 자세히: [training/rl_correction_waypoint/README.md](training/rl_correction_waypoint/README.md). `./run-training.sh -h`에 옵션 · 덮어쓰기.
- 보는 지표: TensorBoard `Metrics/cross_track`(경로 이탈 평균 — 줄어야 함) · `Metrics/waypoints_done`(평균 도착 수 — 늘어야 함).
- 실측 값(3.3)이 학습 env 랜덤 범위의 **중심**이다 (폭은 유지). 실제로 쓴 값은 `params/env.yaml`.
- 바닥: Isaac Lab 지형(8 m 칸: 평평 · 요철 턱 1~3 cm, 언덕 없음 · `env.ground.uneven=false`면 평면만) + env마다 바퀴 마찰 랜덤 0.7~1.1 (`env.events=null`로 끔).
- 자이로 영점 오차: 에피소드마다 ±0.005 rad/s (방향을 자이로 적분으로 보므로 작게 · `env.gyro_bias=…`). 실기기 −0.257 같은 큰 값은 OpenCR 재부팅(가만히)으로 고친다.
- 체크포인트는 50회마다 — 끝에서 망가져도 이전 `model_<N>.pt`를 `play`로 내보내면 된다.
- `policy.onnx` 출력은 신경망 그대로 [-1, 1]. 로봇 corrector가 `clamp × action_scale`(0.10 · 0.60)을 한다 — **`.onnx.data`도 같이 복사**.

### 3.4b 정책 보기 — 실기기 전에

```bash
./run-training.sh rl_correction_waypoint --play --checkpoint artifacts/rl_correction_waypoint/training/<시각>/model_499.pt --num_envs 16 --viz kit env.debug_vis=true   # 학습 env에서 정책 재생
```

- 학습 env는 실측 값이 들어간 sim이라 정책 효과가 보인다 (노란 선 = 웨이포인트 경로).
- 창: **초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 목표 경로**. 노란 점에서 벗어나는 만큼이 드리프트.

### 3.4c world 보기 — `run-world.sh <task>` (학습 · 기록 없이 창으로)

```bash
./run-world.sh rl_correction_waypoint                       # task env + command.py — 노란 점(랜덤 웨이포인트)을 기본 제어기가 따라감 (보정 0)
./run-world.sh rl_correction_waypoint --calibration         # task calibration/ — CalibrationEnv + 네모
./run-world.sh rl_correction_waypoint --command keyboard    # 키보드로 직접 운전 (CalibrationEnv 빈 바닥)
./run-world.sh -h
```

- 키: W/A/S/D(화살표) · Shift 느리게 · X 정지 · R 원점 · **F 카메라를 로봇 쪽으로** · 창 닫으면 끝
- 카메라는 고정하지 않는다 — 마우스 휠 줌 · 오른쪽 드래그 회전 · 가운데 드래그 이동. 로봇이 화면 밖으로 가면 F.
- `--env`는 행동이 [v, w] 명령인 env(`BaseEnv` 계열). 터미널에 0.5 s마다 명령 / 엔코더 / 자이로.

### 3.5 실기기 A/B — 같은 명령, 보정 끔(A) · 켬(B)

```bash
# 준비 = 3.1과 같음: Pi 베이스 · MANUAL · route rl · PC ../ros2_ws/rl-only.sh (corrector)
./run-make-onnx.sh artifacts/rl_correction_waypoint/training/<시각>/model_499.pt        # → exported/policy.onnx (+ .onnx.data)
./run-real-ab.sh rl_correction_waypoint --onnx artifacts/rl_correction_waypoint/training/<시각>/exported/policy.onnx
#   A (corrector 바이패스) → 로봇을 출발 자리로 돌려놓고 Enter → B (그 정책) → 보고서
#   → artifacts/rl_correction_waypoint/ab/<시각>_ab/{A,B}/raw.csv · compare.png
python3 training/rl_correction_waypoint/report.py <A 폴더> <B 폴더>                       # 보고서만 다시
```

- 측정 실행기(`calibration/__main__.py`)와 task의 calibration 명령(네모)을 그대로 쓴다. A/B 차이는 corrector 보정 하나 —
  RealRobot이 측정 직전에 `/rl_corrector/set_parameters`(model_path) · `/rl_corrector/toggle`을 직접 바꾼다 (A = 모델 내림 + 끔 · B = 그 ONNX + 켬).
- 보고서 지표 (작을수록 좋음): 경로 오차 · 끝점 오차 · 직진 중 휨 — 궤적은 거리 = 엔코더 · 방향 = 자이로 (odom 궤적은 휘어도 직선이라 안 씀).
  B 보정량이 0이면 경고 (정책이 일을 안 함).
- **웨이포인트 정책(goal 모드)은 속도 입력(네모)을 보정하지 않는다** — 이 A/B로 차이를 보려면 track 모드 정책(관측 6)이 필요하다.
- OpenCR는 로봇을 가만히 둔 채 켠다 (부팅 때 자이로 영점). 정지 중에도 자이로가 0이 아니면 정책 입력이 오염된다.
- maze 정책(nav2 controller)의 A/B는 nav2 `/controller_server/FollowPath/toggle`로 같은 목표를 두 번 — 이 스크립트 범위 밖.

## 4. sim 물리 — 고친 것 · 남은 것

1. **볼 캐스터 마찰** (2026-11-02 고침): calibration sim이 캐스터 마찰 0을 바닥과 `average`로 합쳐 0.4로 끌림 →
   바퀴가 명령의 87~95%만 냄(사각형 37 cm 어긋남). 학습 env처럼 `min` → gain ≈ 1.
2. **물리 480 Hz** (2026-11-02, `envs/base_env_cfg.py` `PHYSICS_HZ`): 120 Hz는 제자리 회전 때 바퀴가 붙었다 미끄러져 떨림.
   정상 회전 구간 바퀴 속도 표준편차 120 Hz 0.62 · 240 Hz 0.15 · 360 Hz 0.11 · **480 Hz 0.027** · real 0.018 rad/s.
   solver 반복(16/4)·바퀴 마찰 0.6은 거의 효과 없음. calibration sim · 랜덤 월드 · 학습 env가 같은 값을 쓴다.
3. **damping/effort는 고정** (`training/rl_correction_waypoint/robot_cfg.py` WHEEL_DAMPING 50) — 바꿔도 sim 응답이 안 변해서 calibration 변수에서 뺐다.
4. **남은 차이**: 회전 때 몸체가 바퀴보다 덜 돎 — real 자이로/명령 0.950 vs 엔코더 0.995 (5%), sim 0.956 vs 1.017.
   학습 env의 바퀴 간격 오차 랜덤 범위(±8%)가 덮는다.
5. **데드밴드** — 측정 경로에 저속 구간이 없어 fit의 0은 "못 잼". 학습은 기본 범위(0~0.3) 사용.
6. 개념: [../documents/isaac/concepts.md](../documents/isaac/concepts.md).

## 5. sim ROS 브리지 (계획 · 미구현)

Nav2·SLAM·`eval_recorder`를 sim에 직접 붙일 때 쓸 이름만 정해 둔다. 코드에는 아직 없다.

- `ROS_DOMAIN_ID=26` (real은 25 — 같은 WiFi에서 섞이지 않게), 네임스페이스 `/isaacros`, 모든 노드 `use_sim_time:=true`
- 토픽: `/isaacros/{clock, tf, tf_static, odom, imu, joint_states, scan, battery_state, cmd_vel(sub)}`
  · 평가 전용 `/isaacros/ground_truth/pose` (정책 obs 금지). TF 이름은 real과 같게(`odom`, `base_footprint`, `base_link`, `imu_link`, `base_scan`).
- `eval_recorder`를 붙일 때 입력과 출력을 같은 토픽으로 remap하지 말 것 (보정량이 0으로 기록됨).

## 6. 환경

- 학습 PC: RTX 5060 Laptop 8GB · Isaac Sim 6.1 · Isaac Lab 6.1 (`ISAACLAB_DIR`, 기본 `/home/pank/pa-course/IsaacLab-6.1`)
- Lab 6.1 쿼터니언은 **(x, y, z, w)** — 손으로 풀지 말고 `root_*_b` · `quat_from_euler_xyz` 사용
- 실기기·리포트용 `.venv`: `./setup_venv.sh` (ROS lyrical 상속 + matplotlib/numpy)
