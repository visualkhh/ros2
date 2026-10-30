# 실험 방법 — sim/real 센서 싱크와 floor 보정값 찾기 (02)

목적:

1. 같은 명령을 **sim과 real**에 넣었을 때 `v_enc`, `w_enc`, `w_gyro`, raw 바퀴가 같은 신호를 내는지 확인.
2. real의 square 주행에서 **wheel gain / dead time / tau**를 뽑아 `sim_calib_floor.json`으로 저장.
3. 그 보정값을 `01_random_world_sim` · `02_sensor_sync_sim`이 읽어서 sim plant를 real에 맞춤.

원칙:

- **real이 기준**. sim을 real에 맞춘다.
- 공중(무부하)은 **진단용**, 최종은 **바닥(floor)**.
- 한 번에 한 조건만 바꾼다.
- CSV/PNG/JSON은 `calibration/artifacts/` (git 제외). 코드·문서만 커밋.
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

- PC: `02_sensor_sync_real.sh`가 ROS/peers/domain 25를 자동으로 source.
- 로봇이 안 움직이는 상태에서 확인:

```bash
./calibration/02_sensor_sync_real.sh --check-only
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
- `rl/wheel_drift/calib_profile.py`를 sim/real/RL이 **똑같이 import**해서 같은 시간표를 씀.
- `plot_compare` 맨 위에 **odom x-y 궤적**이 나와서 사각형이 얼마나 틀어지는지 바로 보임.

---

## 2. sim 보정값 로딩과 `--no-calib`

`02_sensor_sync_sim`과 `01_random_world_sim`은 시작할 때
`calibration/artifacts/sim_calib_floor.json`을 **자동으로 읽는다.**

```text
[02-sim] calib: ... src=real_floor.csv   ← JSON 읽음
[02-sim] calib: ... src=defaults         ← JSON 없음 → sim_params.DEFAULTS
[02-sim] calib: ... src=raw              ← --no-calib
```

| 모드 | 명령 | wheel 모델 |
|---|---|---|
| 정상 (floor) | `02_sensor_sync_sim.sh` | `sim_calib_floor.json` |
| JSON 없음 | 파일 삭제 | `sim_params.py`의 `DEFAULTS` (**역시 보정된 값**) |
| **raw 기준** | `--no-calib` | `gain=1 · delay=0 · tau=0 · deadband=0` |

> JSON을 지워도 `DEFAULTS`에 gain 0.994 / delay 2 / tau 0.05가 남아 있어서 완전 raw가 아니다.
> 진짜 보정 없는 기준을 보려면 **`--no-calib`** 을 쓴다.

```bash
# raw plant 기준
./calibration/02_sensor_sync_sim.sh --viz none --no-calib

# 랜덤월드도 raw로
./calibration/01_random_world_sim.sh --no-calib
```

---

## 3. 실험 A — 공중 (배 밑 지지 박스, 진단용)

로봇을 **작은 박스 위**에 올려 바퀴가 바닥/박스에 안 닿게 한다.
모터·엔코더·IMU의 스케일/부호만 보는 단계다. **최종 보정은 바닥.**

```bash
# real
./calibration/02_sensor_sync_real.sh --label real_stand
# → calibration/artifacts/real_stand.csv

# sim (박스 높이는 실제 박스에 맞춤; 기본 0.06 m)
./calibration/02_sensor_sync_sim.sh --viz none --stand
# → calibration/artifacts/sim_sensor_stand.csv

# 비교 + 창
./calibration/plot_compare.sh \
  calibration/artifacts/sim_sensor_stand.csv \
  calibration/artifacts/real_stand.csv --show
# → calibration/artifacts/compare_stand.png
```

정상:

- `v_enc ≈ cmd` (각 변 속도), `w_enc ≈ 1.0` (회전)
- raw 바퀴 `wl ≈ wr` (직진), `wl ≈ −wr` (회전)
- `w_gyro ≈ 0` (몸체는 안 돌아감)

---

## 4. 실험 B — 바닥 (최종)

로봇 주변 **앞뒤좌우 1 m**를 비운다. sim도 기본이 **장애물 없는 빈 바닥**이다
(`--room 0`). 랜덤 사물/벽은 센서 비교에 변수만 추가하므로 넣지 않는다.

```bash
# real
./calibration/02_sensor_sync_real.sh --label real_floor
# → calibration/artifacts/real_floor.csv

# sim — 현재 sim_calib_floor.json 적용
./calibration/02_sensor_sync_sim.sh --viz none
# → calibration/artifacts/sim_sensor_floor.csv

# sim raw — 이전 보정 무시
./calibration/02_sensor_sync_sim.sh --viz none --no-calib

# 비교 + 새 보정값 생성
./calibration/plot_compare.sh \
  calibration/artifacts/sim_sensor_floor.csv \
  calibration/artifacts/real_floor.csv --show
# → compare_floor.png
# → sim_calib_floor.json (다음 실행부터 적용)
```

정상:

- `w_gyro`가 **왼쪽 회전에서 +**, 오른쪽에서 −
- 각 변 `v_enc ≈ cmd` (0.10 / 0.12 / 0.15 / 0.20 m/s)
- 직진에서 `w_enc ≈ 0` (곧게 감)
- `02` 로그의 `gain = v_enc / cmd`가 속도별로 크게 안 변하면 좋음

---

## 5. 보정값 찾는 절차 (핵심)

### 5.1 전체 흐름

```text
1) sim raw (--no-calib)  → 보정 없는 기준 궤적
2) real square 측정       → real_floor.csv
3) plot_compare           → real에서 gain/dead/tau 피팅
                          → sim_calib_floor.json 생성
4) sim 정상 실행          → 새 JSON 적용
5) plot_compare 다시       → 궤적/파형이 real에 가까워졌는지 확인
6) 부족하면 3~5 반복
```

중요:

- **피팅은 `real_floor.csv`만 사용**한다. sim에 어떤 JSON이 있었는지는 새 보정값 계산에 영향이 없다.
- 하지만 **sim 궤적**은 그 시점의 JSON을 그대로 쓰므로, “before”와 “after”를 비교하려면
  sim을 어느 JSON으로 돌렸는지 로그(`src=...`)를 꼭 확인한다.

### 5.2 1단계 — raw sim 기준

```bash
./calibration/02_sensor_sync_sim.sh --viz none --no-calib
```

- `src=raw`, `gain=1.000`, `delay=0.00 s`, `tau=0.000 s`.
- 이 궤적은 “보정 전 sim”이다. real과 얼마나 다른지 먼저 본다.

### 5.3 2단계 — real 측정

```bash
./calibration/02_sensor_sync_real.sh --check-only
./calibration/02_sensor_sync_real.sh --label real_floor
```

- 로봇이 사각형을 두 번 그리고 약 54초 뒤 정지.
- `calibration/artifacts/real_floor.csv` 생성.
- `--label`을 바꾸면 파일명도 바뀐다.

### 5.4 3단계 — 비교 + 피팅

```bash
./calibration/plot_compare.sh \
  calibration/artifacts/sim_sensor_floor.csv \
  calibration/artifacts/real_floor.csv --show
```

출력:

```text
[plot] calibration/artifacts/compare_floor.png
[calib] calibration/artifacts/sim_calib_floor.json · runs=8 gain=... delay=...s tau=...s
```

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

### 5.5 `sim_calib_floor.json` 구조

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

### 5.6 4단계 — 새 보정값 적용

`sim_params.py`가 `sim_calib_floor.json`을 읽는다. 다음 실행부터 자동 적용:

```bash
# 센서 싱크
./calibration/02_sensor_sync_sim.sh --viz none

# 랜덤월드
./calibration/01_random_world_sim.sh
```

로그 확인:

```text
[02-sim] calib: tau=0.045s delay=0.15s gain=0.994 deadband=0.00 damping=100 src=real_floor.csv
[01] calib: tau=0.045s delay=0.15s gain=0.994 deadband=0.00 damping=100 src=real_floor.csv
```

`01_random_world_sim`의 **SIM CALIBRATION HUD**에서도 같은 값을 볼 수 있다.
`C` 키로 HUD를 켜고 끈다.

### 5.7 5단계 — 검증

새 JSON으로 sim을 다시 돌리고 같은 real CSV와 비교:

```bash
./calibration/02_sensor_sync_sim.sh --viz none
./calibration/plot_compare.sh \
  calibration/artifacts/sim_sensor_floor.csv \
  calibration/artifacts/real_floor.csv --show
```

볼 것:

- x-y 궤적에서 sim/real 사각형이 겹치는지
- 각 변 `v_enc`, `gain`이 real과 가까운지
- `w_gyro` 부호와 크기
- `t10`, `tau`가 속도별로 너무 다르지 않은지

### 5.8 안 맞을 때

| 증상 | 볼 곳 |
|---|---|
| sim이 전보다 더 굼떠짐 | `speed_fit`의 저속 `t10`/`tau`가 median을 끌었는지 |
| gain이 속도별로 다름 | `speed_fit`의 `gain_l/r`; 필요하면 `WheelLag` 속도 보간 |
| 직진에서 옆으로 샘 | wheel 마찰, caster 마찰, `w_enc` drift |
| 회전만 틀어짐 | `wheel_separation`, 회전 시 좌우 wheel 차이 |
| sim이 real보다 미끄러움 | `sim_params.py`의 `friction_*`, `actuator_damping/effort` |
| 예전 값이 적용된 것 같음 | 로그 `src=...`; `--no-calib`로 raw 확인 |

---

## 6. 데이터 분석

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

## 7. 합격 기준 (초안)

| 항목 | 공중 | 바닥 |
|---|---|---|
| `v_enc` 차이 | ≤ 0.005 m/s | ≤ 0.02 m/s |
| `w_enc` 차이 | ≤ 0.01 rad/s | ≤ 0.05 rad/s |
| raw 바퀴 차이 | ≤ 0.05 rad/s | ≤ 0.2 rad/s |
| `w_gyro` | 둘 다 ≤ 0.02 | 왼쪽 `+`, 오른쪽 `−`, 크기 차 ≤ 0.10 |
| 직진 `w_enc` | — | ≤ 0.02 rad/s (drift) |
| t90 | real 0.1~0.25 s | sim도 지연/τ 모델로 맞춤 |
| 사각형 시작/끝 | — | ≤ 0.10 m (초안) |

---

## 8. 차이가 났을 때 어디를 고치나

| 차이 | 고칠 곳 |
|---|---|
| 바퀴 스케일/부호 (`wl`, `wr`) | `robot.yaml` `left_dir/right_dir` · 펌웨어 ID/방향 · URDF 바퀴 위치 |
| `v_enc`/`w_enc` 스케일 | `wheel_radius` · `wheel_separation` (real: `robot.yaml`, sim: `robot_spec.py`/URDF) |
| sim이 바닥에서만 미끄러짐/틀어짐 | `tasks/common/sim_params.py`의 마찰·damping/effort (또는 `sim_calib_floor.json`) |
| real만 느림 (t90 큼) | real은 그대로. sim `WheelLag`의 `tau`/`delay` |
| `w_gyro` 부호 반대 | IMU 축/부호 (real: driver/firmware/URDF; sim: `root_ang_vel_b`) |
| 속도별 gain 차이 | `speed_fit` 확인 → 필요하면 `WheelLag` 속도 보간 |

sim 보정값은 `isaac/tasks/common/sim_params.py` 한 곳 (τ·delay·gain·deadband·액추에이터·마찰).
`01_random_world_sim`와 `02_sensor_sync_sim`가 같은 값을 쓴다.

**USD 직접 수정 금지** — `robot_usd()`가 URDF에서 매번 다시 만든다.
형상/질량은 URDF를 고치고 `./convert_usd.sh`.

---

## 9. 기록 양식

| 날짜 | 조건 | sim CSV | real CSV | v_enc sim/real | w_gyro sim/real | 판정 | 메모 |
|---|---|---|---|---|---|---|---|
| 2026-10-30 | 공중 (9초) | `sim_sensor_stand.csv` | `real_stand.csv` | 0.150 / 0.149 | 0 / 0 | 스케일·부호 통과, 지연 차 | real t90 0.2 s |
| 2026-10-30 | 바닥 (9초) | `sim_sensor_floor.csv` | `real_floor.csv` | 0.134 / 0.149 | 0.841 / 0.960 | IMU 부호 통과, sim 슬립 큼 | sim 액추에이터/마찰 튜닝 필요 |
| 2026-10-30 | 바닥 (사각형, 속도 4종) | `sim_sensor_floor.csv` | `real_floor.csv` | — | — | 1차 측정 완료 | `speed_fit` 8개 |

---

## 10. 주의

- `--watch`는 약 54초 기록 후 창을 닫을 때까지 같은 사각형 주행을 반복. 데이터는 첫 1회만 저장.
- `--show`는 저장 후 matplotlib 창. Isaac 아님.
- real `state=None`이면 Pi base/MANUAL/peers 확인. `/state_control`은 TRANSIENT_LOCAL.
- `/imu`가 없으면 `w_gyro=0`; `/joint_states`가 없으면 raw 바퀴 0.
- sim을 돌릴 때마다 어떤 JSON을 읽었는지 **`src=` 줄을 확인**한다.
- `01_random_world_sim`은 floor JSON만 읽는다. stand는 `02 --stand` 진단용.
