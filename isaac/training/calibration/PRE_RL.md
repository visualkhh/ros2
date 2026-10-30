# PRE-RL — RL 들어가기 전 셋팅

`training/rl_correction_wheel_drift/` 학습을 돌리기 전에 **sim이 real의 대리인**이 되도록 맞추는 단계.
`training/calibration/`의 S3~S6이 여기에 해당한다.

- 랜덤 환경: `training/calibration/01_random_world_sim.py`
- sim↔real 신호 비교: `02~04`에서 다시 만든다 (현재 `plot_compare.py`만 유지)
- 가설/실측 표: `training/calibration/S3_hypotheses.md`

## 제일 중요한 원칙

**USD는 "로봇 자산"이고, 튜닝값 저장소가 아니다.**
`robot_usd()`가 매 실행마다 URDF에서 USD를 다시 만들기 때문에, USD를 손으로 고친 값은 다음 변환에서 날아간다.
→ **기하/질량은 URDF**, **동역학/마찰/지연/잡음은 Python 설정**에 넣는다.

## 차이별로 어디에 넣나

| 차이 | 어디에 | 파일 |
|---|---|---|
| 몸통·바퀴 질량, 관성, 무게중심 | **URDF** (정본) → USD 재생성 | `isaac/urdf/beebot/urdf.xacro` · `ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro` |
| 바퀴 반지름·간격, 라이다 높이, 링크 위치 | **URDF** + `robot_spec.py` 값 | 위 URDF · `isaac/tasks/common/robot_spec.py` |
| 바닥·장애물·바퀴 마찰, 반발 | Isaac Lab **physics material** (Python) | `tasks/common/sim_params.py` → `training/calibration/scene.py`가 적용 |
| 모터 τ(1차 지연), gain, 데드밴드 | **sim_params** (Python) | `tasks/common/sim_params.py` (`WheelLag`) · `training/calibration/artifacts/sim_calib_<condition>.json` |
| 명령/관측 지연 (WiFi) | **sim_params / env cfg** (Python) | `sim_params.py` (`WHEEL_DELAY_STEPS`) · `rl_correct_env_cfg.py` |
| 엔코더·자이로 잡음, bias | **env cfg** (Python) | 같은 파일 (`enc_noise`, `gyro_noise`) |
| IMU 축·부호, 엔코더 스케일 | **real 먼저** (driver/firmware/URDF) → sim에 반영 | `opencr_driver_node.py` · `firmware/opencr/...ino` · URDF |
| LiDAR 높이/FOV/거리 잡음 | URDF(높이) + sensor cfg(Python) | URDF · `learn/l03_sensors.py` 패턴 |

## 순서

1. **S3 가설 표** — 무엇을 잴지 정한다 (`S3_hypotheses.md`).
2. **03 공중(지지 박스)** — 모터·엔코더·IMU 스케일/부호/지연.
   - `v_enc ≈ cmd`, `w_enc ≈ cmd`, `w_gyro ≈ 0`이면 통과.
3. **04 바닥** — 마찰·슬립 포함 실제 응답.
   - `w_gyro` 부호·크기, `v_enc` 추종.
4. 차이를 위 표에 따라 **URDF / scene material / env cfg**에 반영.
5. 03/04를 다시 돌려 파형이 겹치는지 확인.
6. 그다음 `training/rl_correction_wheel_drift/train.sh`로 학습.

## USD를 고쳐야 하는 경우

- 정말 **형상/질량/관성**이 real과 다를 때만.
- 그래도 USD를 직접 고치지 말고 **URDF를 고친 뒤** `./convert_usd.sh`로 재생성한다.
- USD의 physics material을 직접 바꾸는 것도 가능하지만, 우리는 `scene.py`에서 Python으로 거는 쪽이 재현성이 좋다.

## 현재 상태 (2026-10-30)

- 01 랜덤 환경 완료 (`01_random_world_sim.py`)
- 02 센서 싱크 스크립트 준비 (`02_sensor_sync_sim.sh` sim / `02_sensor_sync_real.sh` real); 둘 다 직접 실행
- 이전 측정값(참고): 공중 `v_enc 0.150 / w_enc 1.000 / w_gyro≈0`, 바닥 `rms_v 0.014 / rms_w 0.093`
- 02~04 스크립트를 다시 만든 뒤 real 공중/바닥을 측정한다. 그 값으로 env cfg의 τ/gain/deadband/지연을 맞춘다.


## 보정값 위치 (2026-10-30)

`isaac/tasks/common/sim_params.py` 한 곳 + 조건별 JSON:

```
training/calibration/artifacts/sim_calib_stand.json   ← plot_compare가 real_stand.csv에서 생성
training/calibration/artifacts/sim_calib_floor.json   ← plot_compare가 real_floor.csv에서 생성
```

- JSON이 있으면 그 값이 기본값을 덮어쓴다. 없으면 `sim_params.py`의 `DEFAULTS`.
- `plot_compare`가 real의 **모든 전진 구간(변마다 속도 다름)** 에서
  **gain / dead time / τ**를 뽑아 JSON으로 저장. 속도별 값은 `speed_fit`에 같이 남는다.
  대표값은 중앙값.
- `01_random_world_sim` → **floor** JSON, `02_sensor_sync_sim --stand` → **stand** JSON, `02` 바닥 → **floor** JSON.
- `sim_params.values(condition)` · `actuator(condition)` · `friction(condition)` · `WheelLag(condition=...)`로 읽는다.

현재 값:

```json
{ "wheel_gain": 0.994, "wheel_delay_steps": 2, "wheel_tau_s": 0.0455 }
```

마찰/액추에이터 기본값은 `sim_params.py`의 `DEFAULTS`:
`damping 100`, `ground (0.8, 0.7)`, `wheel (1.0, 1.0)`, `caster (0.0, 0.0)`.

- 공중(stand)은 real과 거의 일치: `v_enc 0.149/0.149 · 바퀴 4.517/4.517`.
- 바닥(floor)은 sim 슬립·drift가 남음: `v_enc 0.133/0.149 · w_gyro 0.843/0.960` → 다음은 caster/마찰 튜닝.
