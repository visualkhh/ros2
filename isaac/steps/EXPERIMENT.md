# 실험 방법 — sim/real 센서 싱크 (02)

목적: **같은 명령을 sim과 real에 넣었을 때 `v_enc`, `w_enc`, `w_gyro`, raw 바퀴가 같은 신호를 내는지** 확인한다.
RL 학습 전에 이걸 맞춰야 학습이 real에서 의미가 있다.

원칙:
- **real이 기준**. sim을 real에 맞춘다 (real을 sim에 맞추지 않음).
- 공중(무부하) → 바닥(부하) 순서로 본다.
- 한 번에 한 조건만 바꾼다.
- 결과 CSV/PNG는 `steps/artifacts/` (git 제외). 스크립트·문서만 커밋.

## 0. 준비

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
```

- Isaac 창은 **한 번에 하나만** (메모리).
- real: Pi base + cmd_manager, 제어 **MANUAL**
  ```bash
  # Pi
  cd ~/ros2/tool
  ./run-cli base start
  ./run-cli mode manual
  ```
- PC: `02_sensor_sync_real.sh`가 ROS/peers/domain 25를 자동으로 source함.
- 상태 확인 (로봇 안 움직임):
  ```bash
  ./steps/02_sensor_sync_real.sh --check-only
  # [02-real] ROS_DOMAIN_ID=25 · state=MANUAL 이어야 함
  ```

## 1. 공통 프로파일 (정사각형 2개, 약 48초, 20 Hz)

```
0~2 s    정지
2 s~     왼쪽 사각형 (반시계, w=+1.0)
         각 변: 0.5 m 전진 → 0.25 s 정지 → 90° 회전 → 0.25 s 정지, 4번
         → 시작점에서 +x 방향
180° 턴  제자리에서 오른쪽으로 180° (w=-1.0)
         → 같은 사각형의 마지막 변을 거꾸로 타기 시작
         오른쪽 사각형 (시계, w=-1.0): 같은 네 변을 반대 방향으로 역주행
2 s      정지
```

- 한 변 `0.5 m` (`--side`로 변경), 전진 `0.15 m/s`, 회전 `1.0 rad/s`.
- 바퀴 목표: 직진 `4.545 rad/s`, 회전 `±4.348 rad/s` (1.0·0.287/2/0.033).
- `square_profile.py`를 sim/real이 **똑같이** import해서 같은 시간표를 쓴다.
- 같은 프로파일을 sim/real에 각각 넣고 CSV를 만든 뒤 `t=0` 기준으로 겹쳐 본다.
- `plot_compare` 맨 위에 **odom x-y 궤적**이 나와서 두 사각형이 얼마나 틀어지는지 눈으로 비교.
- 두 번째 사각형은 **첫 사각형과 같은 선을 반대로** 지나므로, 선이 겹쳐 보이면 정상.

## 2. 실험 A — 공중 (배 밑 지지 박스)

로봇을 **작은 박스 위**에 올려 바퀴가 바닥/박스에 안 닿게 한다.

```bash
# real
./steps/02_sensor_sync_real.sh --label real_stand
# → steps/artifacts/real_stand.csv

# sim (박스 높이는 실제 박스에 맞춤; 기본 0.06 m)
./steps/02_sensor_sync_sim.sh --viz none --stand
# → steps/artifacts/sim_sensor_stand.csv

# 비교 + 창
./steps/plot_compare.sh \
  steps/artifacts/sim_sensor_stand.csv \
  steps/artifacts/real_stand.csv --show
# → steps/artifacts/compare_stand.png
```

눈으로 보고 싶을 때만:

```bash
./steps/02_sensor_sync_sim.sh --stand --watch
```

정상:
- `v_enc ≈ 0.15`, `w_enc ≈ 1.0`
- raw 바퀴 `wl ≈ wr ≈ 4.52` (직진), `wl ≈ −4.33 / wr ≈ +4.33` (회전)
- `w_gyro ≈ 0` (몸체는 안 돌아감)

## 3. 실험 B — 바닥

로봇을 바닥에 내려놓고 **앞뒤좌우 1 m**를 비운다 (0.5 m 사각형이 두 번 그려짐).
sim도 기본이 **장애물 없는 빈 바닥**이라 주변 사물/벽 충돌 변수를 없앤다 (`--room 0`).

```bash
# real
./steps/02_sensor_sync_real.sh --label real_floor
# → steps/artifacts/real_floor.csv

# sim
./steps/02_sensor_sync_sim.sh --viz none
# → steps/artifacts/sim_sensor_floor.csv

./steps/plot_compare.sh \
  steps/artifacts/sim_sensor_floor.csv \
  steps/artifacts/real_floor.csv --show
# → steps/artifacts/compare_floor.png
```

정상:
- `w_gyro`가 **왼쪽 회전에서 +**
- `v_enc`가 명령 0.15에 가까움
- 직진에서 `w_enc ≈ 0` (곧게 감)

## 4. 데이터 분석

CSV 열:

| 열 | 의미 |
|---|---|
| `lin_raw`, `ang_raw` | 보낸 명령 |
| `lin_out`, `ang_out` | `/cmd_vel_base` (드라이버가 받은 값) |
| `v_enc`, `w_enc` | `/odom` twist (엔코더 기반) |
| `w_gyro` | `/imu` z (몸체 실제 회전) |
| `wheel_l_vel`, `wheel_r_vel` | `/joint_states` raw 바퀴 rad/s |
| `wheel_l_pos`, `wheel_r_pos` | raw 바퀴 rad |

분석 구간 (기본 `side=0.5`):
- 첫 왼쪽 직진: `2.5 ~ 4.5 s`
- 첫 왼쪽 회전: `5.9 ~ 7.0 s`
- 오른쪽은 `02` 스크립트가 `RIGHT move/turn` 평균을 따로 출력함
- 전체 궤적: `plot_compare` 맨 위 x-y 그래프

지표:
- 구간 평균 `v_enc`, `w_enc`, `w_gyro`, `wl`, `wr`
- RMS(명령 − 실제), bias
- 계단 후 `t10/t50/t90` (raw 바퀴, 첫 전진 시작 기준)
  - `t10` = dead time (WiFi/20 Hz/명령 지연)
  - `t90 − t10` ≈ 모터 상승시간(τ)
- 사각형이 시작점으로 얼마나 돌아오는지 (x-y 궤적의 시작/끝 거리)

## 5. 합격 기준 (초안)

| 항목 | 공중 | 바닥 |
|---|---|---|
| `v_enc` 차이 | ≤ 0.005 m/s | ≤ 0.02 m/s |
| `w_enc` 차이 | ≤ 0.01 rad/s | ≤ 0.05 rad/s |
| raw 바퀴 차이 | ≤ 0.05 rad/s | ≤ 0.2 rad/s |
| `w_gyro` | 둘 다 ≤ 0.02 | 왼쪽 `+`, 오른쪽 `−`, 크기 차 ≤ 0.10 |
| 직진 `w_enc` | — | ≤ 0.02 rad/s (drift) |
| t90 | real 0.1~0.25 s | sim도 지연/τ 모델로 맞춤 |

## 6. 차이가 났을 때 어디를 고치나

| 차이 | 고칠 곳 |
|---|---|
| 바퀴 스케일/부호 (`wl`, `wr`) | `robot.yaml` `left_dir/right_dir` · 펌웨어 ID/방향 · URDF 바퀴 위치 |
| `v_enc`/`w_enc` 스케일 | `wheel_radius` · `wheel_separation` (real: `robot.yaml`, sim: `robot_spec.py`/URDF) |
| sim이 바닥에서만 미끄러짐/틀어짐 | `tasks/common/sim_params.py`의 마찰·damping/effort (또는 `sim_calib_floor.json`) |
| real만 느림 (t90 큼) | real은 그대로. sim env에 `tau`/`delay` 추가 (`rl/wheel_drift/task/rl_correct_env_cfg.py`) |
| `w_gyro` 부호 반대 | IMU 축/부호 (real: driver/firmware/URDF; sim: `root_ang_vel_b`) |

sim 보정값은 `isaac/tasks/common/sim_params.py` 한 곳 (τ·delay·gain·deadband·액추에이터·마찰).
`01_random_world_sim`와 `02_sensor_sync_sim`가 같은 값을 쓴다.

**USD 직접 수정 금지** — `robot_usd()`가 URDF에서 매번 다시 만든다. 형상/질량은 URDF를 고치고 `./convert_usd.sh`.

## 7. 기록 양식

| 날짜 | 조건 | sim CSV | real CSV | v_enc sim/real | w_gyro sim/real | 판정 | 메모 |
|---|---|---|---|---|---|---|---|
| 2026-10-30 | 공중 (9초) | `sim_sensor_stand.csv` | `real_stand.csv` | 0.150 / 0.149 | 0 / 0 | 스케일·부호 통과, 지연 차 | real t90 0.2 s |
| 2026-10-30 | 바닥 (9초) | `sim_sensor_floor.csv` | `real_floor.csv` | 0.134 / 0.149 | 0.841 / 0.960 | IMU 부호 통과, sim 슬립 큼 | sim 액추에이터/마찰 튜닝 필요 |
| — | 바닥 (사각형) | `sim_sensor_floor.csv` | `real_floor.csv` | 측정 대기 | 측정 대기 | — | 왼쪽/오른쪽 사각형 2회 |

## 8. 현재까지 결과 (2026-10-30, 9초 프로파일 기준)

- **공중**: 스케일·부호·odom 일치 (0.5% 이내). real t90 0.2 s (dead 0.1 + τ 0.1), sim 0.05 s.
- **바닥**: real이 더 정확 (v 0.149, w_enc 0.995, w_gyro 0.96). sim은 슬립·직진 drift (v 0.134, w_enc 0.942, w_gyro 0.84, w_enc −0.038).
- **IMU 부호**: 왼쪽 회전에서 real `w_gyro +` → 정상.
- **사각형 프로파일**: sim만 먼저 확인. 왼쪽/오른쪽 회전 모두 부호는 맞고, 오른쪽이 약간 더 처짐 (`w_gyro +0.843 / −0.830`). real 측정 대기.
- 다음: real 사각형 측정 → `plot_compare` x-y 궤적으로 시작/끝 오차 확인 → sim 마찰/액추에이터 튜닝.

## 9. 주의

- `--watch`는 약 48초 기록 후 창을 닫을 때까지 같은 사각형 주행을 반복. 데이터는 첫 1회만 저장.
- `--show`는 저장 후 matplotlib 창. Isaac 아님.
- real `state=None`이면 Pi base/MANUAL/peers 확인. `/state_control`은 TRANSIENT_LOCAL.
- `/imu`가 없으면 `w_gyro=0`; `/joint_states`가 없으면 raw 바퀴 0.
