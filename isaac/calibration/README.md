# calibration — RL 전 셋팅 (한 단계씩)

학습(`training/rl_correction_wheel_drift/`) 전에 **sim이 real의 대리인**이 되도록 맞추는 단계.
한 번에 다 하지 않고 한 단계씩.

sim+real을 동시에 돌리고 비교 보고서까지 만들려면:

```bash
./run-calibration.sh calibrate --side 0.5 --show
```

| 단계 | 내용 | 상태 |
|---|---|---|
| `world/DefaultWorld.py` | 랜덤 방(3.5~6 m) + 장애물 + 우리 로봇, `N` 다음 방 | ✅ |
| `sensor-sync-sim` + `sensor-sync-real` | IMU/odom/raw wheel 신호를 같은 명령에서 비교 | 둘 다 직접 실행 |
| `step_floor.py` | 바닥 계단응답 sim/real | ☐ |
| 차이 반영 | URDF / 마찰 / env cfg에 실측값 넣기 | ☐ |
| `run-training.sh --fit <real>/fit.json` | 학습 → ONNX → real Before/After | ☐ |

## 지금 있는 파일

이름 규칙: **Isaac에서 도는 건 `_sim`**, **실제 로봇은 `_real`**.

```
calibration/
├── Calibrate.py             기록 오케스트레이터 (Sensor 주입, sim/real 공용)
├── report/report.py         단건 리포트 + sim/real 비교·fit (단일 파일, `plot-compare`도 이걸 씀)
├── world/World.py             실행 단위 계약 (ABC, 한 파일 한 클래스)
├── world/DefaultWorld.py    랜덤 주행 (오버라이딩 지점, run-calibration.sh random-world)
├── sensor/Sensor.py         공용 베이스 (CSV+리포트, 한 파일 한 클래스)
├── sensor/DefaultSensorSim.py  sim 센서 기록 (run-calibration.sh sensor-sync-sim)
├── sensor/DefaultSensorReal.py real 센서 기록 (run-calibration.sh sensor-sync-real, ROS/peers 자동)
├── excitation/CalibExcitation.py   excitation 계약 (ABC)
├── excitation/DefaultCalibExcitation.py   기본 사각형/waypoint (sim·real 공용)
├── parameter/               휠/액추에이터/마찰 타입 + WheelLag (sim·env 공용)
├── scene/Scene.py             장면 계약 (ABC, 한 파일 한 클래스)
├── scene/DefaultScene.py      기본 방+로봇+카메라 (오버라이딩 지점)
├── scene/Room.py              방 한 칸
├── (산출물은 `../artifacts/` — calibration/RL/real 공용, git 제외)
├── PRE_RL.md              차이값을 URDF/마찰/env cfg 중 어디에 넣는지
├── hypotheses.md       가설·실측 표
└── README.md              이 파일
```

## 실행

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./run-calibration.sh random-world
```

- 방 크기·장애물이 랜덤. `--seed 7`로 재현, 없으면 매번 새 seed.
- `W/A/S/D` 주행 · `Shift` 느리게 · `X` 정지 · `R` 현재 방 리셋 · `N` 다음 방 · `C` 보정값 창.
- 왼쪽 위 **SIM CALIBRATION** 창:
  - tau/dead steps/gain/deadband/마찰을 **실행 중 수정** (즉시 적용, JSON 저장 안 함)
  - **0 set** → 표시된 숫자 전부 0, **reset** → 창을 열 때 읽은 값(JSON/DEFAULTS)으로 복구
  - 맨 아래 `source`가 `real_floor.csv`면 JSON 사용, `defaults`면 기본값 사용

## 02 — 센서 싱크 (sim 기준 생성됨)

```bash
# 눈으로 보기 (창 닫을 때까지 같은 주행 반복)
./run-calibration.sh sensor-sync-sim --watch

# 바닥 — 기록만 (기본: 장애물 없는 빈 바닥)
./run-calibration.sh sensor-sync-sim --viz none

# 보정값 넣어서 (없으면 raw plant — 숨은 보정 파일은 안 읽음)
./run-calibration.sh sensor-sync-sim --viz none --fit <real>/fit.json,<sim>/fit.json
```

- 기본 환경은 **장애물 없는 빈 바닥** (`--room 0`). 랜덤 사물이 있는 방에서 돌리고 싶을 때만 `--room 4.0`.
- 프로파일: **한 변 0.5 m 정사각형 왼쪽(반시계) → 180° 턴 → 같은 사각형 역주행**. `--side 0.4`로 변 길이 변경.
- 각 변은 `전진 → 0.25 s 정지 → 90° 제자리 회전 → 0.25 s 정지`.
- 변마다 속도를 다르게 함: 기본 `0.15 / 0.10 / 0.20 / 0.12 m/s` (`--speed`는 기준값). 속도별 gain/지연/τ를 보기 위함.
- `plot_compare` 맨 위에 **odom x-y 궤적**이 나와서 사각형이 얼마나 틀어지는지 바로 보임.
- real odom은 시작 위치와 초기 yaw를 0으로 맞춰서 sim과 같은 방향으로 겹쳐 그림.

- 실행 결과: `artifacts/calibration/<시각>-<label>/` (`--label` 없으면 `<TAG>` — sim `02-sim`, real `02-real`)에
  `raw.csv` + `report.png` + `fit.json`(gain/delay/tau 피팅), `--fit`이면 `base-fit.json`.
  경로를 정하려면 sim `--out`, real `--output`.
- sim은 `--fit`으로 준 보정만 쓴다 (없으면 raw). 숨은 보정 파일은 안 읽는다.
- real 기록 (PC):
  ```bash
  source /opt/ros/lyrical/setup.bash
  export ROS_DOMAIN_ID=25
  source tool/peers.sh

  # 로봇 안 움직이고 상태만 확인
  ./run-calibration.sh sensor-sync-real --check-only

  ./run-calibration.sh sensor-sync-real --label real_floor
  ```
  `/state_control`은 TRANSIENT_LOCAL(latched)라 스크립트가 그에 맞춰 구독한다.
- 비교 (기본은 PNG 저장만):
  ```bash
  ./run-calibration.sh plot-compare <sim>/raw.csv <real>/raw.csv
  ```
- **창으로 직접 보고 싶을 때만 `--show`**:
  ```bash
  ./run-calibration.sh plot-compare <sim>/raw.csv <real>/raw.csv --show
  ```
  창을 닫으면 터미널로 돌아온다. `--show` 없으면 `artifacts/compare_<tag>.png`로 저장 (`--out`으로 지정).
- CSV에는 세 층이 같이 들어감:
  - `/joint_states` → `wheel_l_vel`, `wheel_r_vel` (rad/s), `wheel_l_pos`, `wheel_r_pos` (rad) — **firmware 원값**
- `t_ros/t_odom/t_imu/t_joint` → 수신 시각과 각 토픽 stamp. `plot_compare` 맨 밑에 **msg age [ms]** 패널로 그림 (sim은 0, real은 WiFi 지터가 보임). `/cmd_vel_base`는 `Twist`라 stamp 없음
  - `/odom` → `v_enc`, `w_enc` (m/s, rad/s) — 드라이버가 `r`/`sep`로 계산한 값
  - `/imu` → `w_gyro` (rad/s)
- 정상: `w_gyro` 부호·크기, `v_enc` 추종, raw 바퀴의 좌우 차이가 핵심.
- `plot_compare.py`의 맨 아래 subplot이 **raw 바퀴 속도**(실선 sim, 점선 real).

## 다음 단계

1. `step_floor.py` — 바닥 계단응답
2. 차이를 `PRE_RL.md` 표에 따라 URDF / 마찰 / `training/rl_correction_wheel_drift/task/rl_correct_env_cfg.py`에 반영
4. `run-training.sh --fit <real>/fit.json`로 학습
