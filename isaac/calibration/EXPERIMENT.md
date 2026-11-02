# 실험 방법 — sim/real 센서 싱크와 floor 보정값 찾기 (02)

목적:

1. 같은 명령을 **sim과 real**에 넣었을 때 `v_enc`, `w_enc`, `w_gyro`, raw 바퀴가 같은 신호를 내는지 확인.
2. sim(raw)과 real 각각의 square 주행에서 **wheel gain / dead time / tau**를 뽑아 `fit.json`으로 저장.
3. `--fit real/fit.json,sim/fit.json`으로 그 차이를 sim 입력에 넣어 sim plant를 real에 맞춤.

원칙:

- **real이 기준**. sim을 real에 맞춘다.
- 바닥(floor) 조건으로 맞춘다.
- 한 번에 한 조건만 바꾼다.
- CSV/PNG/JSON은 `artifacts/` (git 제외). 코드·문서만 커밋.
- **USD 직접 수정 금지** — 형상/질량은 URDF, 동역학/마찰/지연은 Python.

---

## 0. 준비

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
```

- Isaac 창은 **한 번에 하나만**.
- real: Pi base + cmd_manager, 제어 **MANUAL**

```bash
# Pi
cd ~/ros2/tool
./run-cli base start
./run-cli mode manual
```

- PC: `run-calibration.sh sensor-sync-real`가 ROS/peers/domain 25를 자동으로 source.
- 로봇이 안 움직이는 상태에서 확인:

```bash
./run-calibration.sh sensor-sync-real --check-only
# [02-real] ROS_DOMAIN_ID=25 · state=MANUAL 이어야 함
```

---

## 1. 공통 프로파일 — 정사각형 2개 (약 54초, 20 Hz)

```text
0~2 s    정지
2 s~     왼쪽 사각형 (반시계, w=+1.0)
         각 변: 0.5 m 전진 → 0.25 s 정지 → 90° 회전 → 0.25 s 정지 ×4
         변 속도: 0.15 → 0.10 → 0.20 → 0.12 m/s (기본)
         → 시작점에서 +x 방향
180° 턴  제자리에서 오른쪽으로 180° (w=-1.0)
         → 같은 사각형의 마지막 변을 거꾸로 타기 시작
         오른쪽 사각형 (시계): 같은 네 변을 반대 방향으로, 속도는 역순
2 s      정지
```

- 한 변 `0.5 m` (`--side`로 변경), 기준 전진 `0.15 m/s`, 회전 `1.0 rad/s`.
- **변마다 속도를 다르게** 해서 속도별 gain / dead time / tau를 볼 수 있게 함.
- `--speed`는 기준값. 실제 변 속도는 `1.0 / 0.67 / 1.33 / 0.8` 배.
- 두 번째 사각형은 첫 사각형의 **같은 선을 반대로** 지나므로, 궤적이 겹쳐 보이면 정상.
- `calibration/excitation/DefaultCalibExcitation.py`를 sim/real이 똑같이 import한다.
- `plot_compare` 맨 위에 **odom x-y 궤적**이 나와서 사각형이 얼마나 틀어지는지 바로 보임.

---

## 2. sim 보정값 — `--fit`으로 준 값만

`sensor-sync-sim`은 **숨은 보정 파일을 읽지 않는다.** 기본은 raw plant
(`gain=1 · delay=0 · tau=0 · deadband=0`)이고, 보정은 `--fit`으로 준 값만 들어간다.

```text
[02-sim] ... · RAW (no --fit) · ...                           ← 기본
[02-sim] ... · FIT .../real/fit.json, .../sim/fit.json · ...  ← --fit
```

| `--fit` | sim 입력 보정 (`WheelLag`) |
|---|---|
| 없음 | raw — real,sim 차이 계산의 **sim 기준**으로 그대로 쓴다 |
| `real/fit.json` | real fit 절대값 (sim plant가 이상적이라고 볼 때) |
| `real/fit.json,sim/fit.json` | real − sim 차이 (gain 비율, delay/tau 차, deadband 문턱) |

실행마다 `artifacts/calibration/<시각>-<TAG>/`에 `raw.csv` · `fit.json` · `report.png`가,
`--fit`이면 `base-fit.json`(실제로 넣은 보정값)이 같이 떨어진다.

> `random-world`도 같다: 기본 raw, `--fit`으로 준 값만 (C 키 보정값 창에 그 값이 뜬다).

## 2.5 damping/effort — 쓰지 않음

`damping`/`effort`는 `calibration/parameter/ActuatorParam.py`의 **고정 모델값**이다.  
`--damping`/`--effort`로 여러 값을 재봐도 sim 응답이 변하지 않아서(50/100/200/400, effort 3/10 모두 같은 fit)
calibration 변수와 CLI 옵션에서 제거했다. 지금 calibration은 **real/sim raw fit + `--fit real,sim` 차이**만 쓴다.

## 3. 실험 — 바닥 (최종)

로봇 주변 **앞뒤좌우 1 m**를 비운다. sim도 기본이 **장애물 없는 빈 바닥**이다
(`--room 0`). 랜덤 사물/벽은 센서 비교에 변수만 추가하므로 넣지 않는다.

```bash
# 한 번에: sim(raw) + real 기록 → 비교
./run-calibration.sh calibrate --side 0.5
# → artifacts/calibration/<시각>-calibrate/{sim,real}/fit.json · compare.png

# 따로 돌릴 때
./run-calibration.sh sensor-sync-real --label real_floor   # → artifacts/calibration/<시각>-real_floor/
./run-calibration.sh sensor-sync-sim --viz none            # → artifacts/calibration/<시각>-02-sim/
./run-calibration.sh plot-compare <sim>/raw.csv <real>/raw.csv --show
```

정상:

- `w_gyro`가 **왼쪽 회전에서 +**, 오른쪽에서 −
- 각 변 `v_enc ≈ cmd` (0.10 / 0.12 / 0.15 / 0.20 m/s)
- 직진에서 `w_enc ≈ 0` (곧게 감)
- `02` 로그의 `gain = v_enc / cmd`가 속도별로 크게 안 변하면 좋음

---

## 4. 보정값 찾는 절차 (핵심)

### 4.1 전체 흐름

```text
1) sim raw (--fit 없음)     → sim/fit.json   (보정 없는 sim plant)
2) real square 측정          → real/fit.json
3) sim --fit real,sim        → 차이를 입력 보정으로 → base-fit.json
4) 3의 fit.json ≈ real/fit.json 이면 성공 (plot-compare로 궤적도 확인)
5) 학습: ./run-training.sh --fit real/fit.json
```

중요:

- 차이 계산의 **sim 기준은 반드시 raw**(`--fit` 없이)로 잰 것이어야 한다.
  보정이 들어간 sim의 fit을 기준으로 쓰면 보정이 두 번 섞인다.

### 4.2 1단계 — raw sim 기준

```bash
./run-calibration.sh sensor-sync-sim --viz none
```

- 로그 `RAW (no --fit)`, `gain=1.000`, `delay=0.00 s`, `tau=0.000 s`.
- 이 궤적은 “보정 전 sim”이다. real과 얼마나 다른지 먼저 본다.

### 4.3 2단계 — real 측정

```bash
./run-calibration.sh sensor-sync-real --check-only
./run-calibration.sh sensor-sync-real --label real_floor
```

- 로봇이 사각형을 두 번 그린 뒤 약 54초 뒤 정지.
- `artifacts/calibration/<시각>-real_floor/`에 `raw.csv` · `fit.json` · `report.png` 생성 (`--output`으로 경로 지정).

### 4.4 3단계 — 비교 + 피팅

```bash
./run-calibration.sh plot-compare <sim>/raw.csv <real>/raw.csv --show
```

출력 (경로를 안 주면 `artifacts/compare_<tag>.png` · `artifacts/sim_calib_<tag>.json`, `--out`/`--calib-out`으로 지정):

```text
[plot] artifacts/compare_<tag>.png
[calib] artifacts/sim_calib_<tag>.json · runs=... gain L/R=... delay=...s tau=...s
```

> fit 자체는 각 기록(`sensor-sync-*`)이 이미 자기 폴더에 `fit.json`으로 남긴다.

`fit_wheel`이 하는 일:

1. real CSV에서 `lin_raw > 0.01`인 **전진 구간 8개**를 찾는다.
2. 각 구간의 뒤 절반(정상상태)에서:
   - `target = cmd_v / WHEEL_RADIUS`
   - `gain = 평균(wheel_l, wheel_r) / target`
   - `t10`, `t90` = 계단 시작 후 10% / 90% 도달 시간
   - `tau = (t90 - t10) / 2.2`
3. 8개 구간의 **중앙값**을 대표값으로 저장.
4. 속도별 값은 `speed_fit`에 전부 저장 (진단용).

> 저속에서는 `t10`이 0.15 s, 고속에서는 0.10 s처럼 다를 수 있다.
> 이때 `speed_fit`을 보고 어느 속도 구간이 문제인지 판단한다.

### 4.5 `fit.json` 구조

```json
{
  "wheel_gain": 0.9938,
  "wheel_delay_s": 0.15,
  "wheel_delay_steps": 3,
  "wheel_tau_s": 0.0455,
  "wheel_deadband": 0.0,
  "condition": "floor",
  "source": "real_floor.csv",
  "fitted_at": "2026-10-30 18:44:26",
  "speed_fit": [
    {
      "cmd_v": 0.10,
      "wheel_ss": 3.011,
      "wheel_l": 3.0115,
      "wheel_r": 3.0105,
      "gain": 0.9936,
      "gain_l": 0.9938,
      "gain_r": 0.9935,
      "t10": 0.15,
      "tau": 0.01
    }
  ]
}
```

| 키 | 의미 |
|---|---|
| `wheel_gain` | 대표 gain (8구간 중앙값) |
| `wheel_delay_s` | dead time [s] (t10 중앙값) |
| `wheel_delay_steps` | 20 Hz 기준 step 수 (`round(delay/0.05)`) |
| `wheel_tau_s` | 1차 지연 τ [s] |
| `wheel_deadband` | 아직 0 (저속 시험 후 조정) |
| `speed_fit` | 속도별 gain_l/gain_r/t10/tau — **진단용** |

### 4.6 4단계 — 보정값 넣고 다시 기록

```bash
./run-calibration.sh sensor-sync-sim --viz none \
  --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json
```

- 로그 `FIT ...`와 `calib: tau=... delay=... gain L/R=...`로 들어간 값 확인.
- 같은 폴더의 `base-fit.json` = 실제로 넣은 보정값 (그대로 다시 `--fit`에 넣을 수 있다).

### 4.7 5단계 — 검증

```bash
./run-calibration.sh plot-compare <4.6 폴더>/raw.csv <real>/raw.csv --show
```

볼 것:

- `<4.6 폴더>/fit.json`이 `real/fit.json`과 같은지 (gain L/R · delay · tau)
- x-y 궤적에서 sim/real 사각형이 겹치는지
- `w_gyro` 부호와 크기 (슬립은 바퀴 fit으로 안 잡힌다 → 마찰 쪽)

### 4.8 안 맞을 때

| 증상 | 볼 곳 |
|---|---|
| sim이 전보다 더 굼떠짐 | `speed_fit`의 저속 `t10`/`tau`가 median을 끌었는지 |
| gain이 속도별로 다름 | `speed_fit`의 `gain_l/r`; 필요하면 `WheelLag` 속도 보간 |
| 직진에서 옆으로 샘 | wheel 마찰, caster 마찰, `w_enc` drift |
| 회전만 틀어짐 | `wheel_separation`, 회전 시 좌우 wheel 차이 |
| sim이 real보다 미끄러움 | `FrictionParam`·`ActuatorParam` 값 |
| 예전 값이 적용된 것 같음 | 로그 `RAW`/`FIT ...`와 폴더의 `base-fit.json` |

---

## 5. 데이터 분석

CSV 열:

| 열 | 의미 |
|---|---|
| `lin_raw`, `ang_raw` | 보낸 명령 |
| `lin_out`, `ang_out` | `/cmd_vel_base` (드라이버가 받은 값) |
| `odom_x`, `odom_y`, `odom_yaw` | `/odom` pose |
| `v_enc`, `w_enc` | `/odom` twist (엔코더 기반) |
| `w_gyro` | `/imu` z |
| `wheel_l_vel`, `wheel_r_vel` | `/joint_states` raw 바퀴 rad/s |
| `wheel_l_pos`, `wheel_r_pos` | raw 바퀴 rad |
| `t_ros` | 기록 시점 ROS 시각 (수신 시각) |
| `t_odom`, `t_imu`, `t_joint` | 각 토픽 `header.stamp` (sim은 기록 시각과 동일 → age 0) |

분석 구간:

- 각 변: `02` 로그의 `LEFT move1~4`, `RIGHT move1~4` 줄 사용
- 회전: `LEFT/RIGHT turn` 줄
- 전체 궤적: `plot_compare` 맨 위 x-y

지표:

- 구간 평균 `v_enc`, `w_enc`, `w_gyro`, `wl`, `wr`
- `gain = v_enc / cmd`
- RMS(명령 − 실제), bias
- 계단 후 `t10/t50/t90` (raw 바퀴)
  - `t10` = dead time (WiFi/20 Hz/명령 지연)
  - `t90 − t10` ≈ 모터 상승시간(τ)
- 사각형 시작점/끝점 거리 (x-y 궤적)

---

## 6. 합격 기준 (초안)

| 항목 | 바닥 기준 |
|---|---|
| `v_enc` 차이 | ≤ 0.02 m/s |
| `w_enc` 차이 | ≤ 0.05 rad/s |
| raw 바퀴 차이 | ≤ 0.2 rad/s |
| `w_gyro` | 왼쪽 `+`, 오른쪽 `−`, 크기 차 ≤ 0.10 |
| 직진 `w_enc` | ≤ 0.02 rad/s (drift) |
| t90 | real 0.1~0.25 s, sim도 지연/τ 모델로 맞춤 |
| 사각형 시작/끝 | ≤ 0.10 m (초안) |

---

## 7. 차이가 났을 때 어디를 고치나

| 차이 | 고칠 곳 |
|---|---|
| 바퀴 스케일/부호 (`wl`, `wr`) | `robot.yaml` `left_dir/right_dir` · 펌웨어 ID/방향 · URDF 바퀴 위치 |
| `v_enc`/`w_enc` 스케일 | `wheel_radius` · `wheel_separation` (real: `robot.yaml`, sim: `robot_spec.py`/URDF) |
| sim이 바닥에서만 미끄러짐/틀어짐 | `calibration/parameter/`의 마찰 (또는 `sim_calib_floor.json`) |
| real만 느림 (t90 큼) | real은 그대로. sim `WheelLag`의 `tau`/`delay` |
| `w_gyro` 부호 반대 | IMU 축/부호 (real: driver/firmware/URDF; sim: `root_ang_vel_b`) |
| 속도별 gain 차이 | `speed_fit` 확인 → 필요하면 `WheelLag` 속도 보간 |

sim 보정값은 `isaac/calibration/parameter/` 한 곳 (SimParam: 휠·액추에이터·마찰).
`random-world`와 `sensor_sync_sim`가 같은 값을 쓴다.

**USD 직접 수정 금지** — `robot_usd()`가 URDF에서 매번 다시 만든다.
형상/질량은 URDF를 고치고 `./convert_usd.sh`.

---

## 8. 기록 양식

| 날짜 | 조건 | sim CSV | real CSV | v_enc sim/real | w_gyro sim/real | 판정 | 메모 |
|---|---|---|---|---|---|---|---|
| 2026-10-30 | 바닥 (9초) | `sim_sensor_floor.csv` | `real_floor.csv` | 0.134 / 0.149 | 0.841 / 0.960 | IMU 부호 통과, sim 슬립 큼 | sim 액추에이터/마찰 튜닝 필요 |
| 2026-10-30 | 바닥 (사각형, 속도 4종) | `sim_sensor_floor.csv` | `real_floor.csv` | — | — | 1차 측정 완료 | `speed_fit` 8개 |

---

## 9. 주의

- `--watch`는 약 70초 기록 후 창을 닫을 때까지 같은 사각형 주행을 반복. 데이터는 첫 1회만 저장.
- `--show`는 저장 후 matplotlib 창. Isaac 아님.
- real `state=None`이면 Pi base/MANUAL/peers 확인. `/state_control`은 TRANSIENT_LOCAL.
- `/imu`가 없으면 `w_gyro=0`; `/joint_states`가 없으면 raw 바퀴 0.
- sim을 돌릴 때마다 어떤 JSON을 읽었는지 **`src=` 줄을 확인**한다.
- `random-world`와 `sensor-sync-sim`은 floor JSON을 읽는다.
