# PRE-RL — RL 들어가기 전 셋팅

`rl/wheel_drift/` 학습을 돌리기 전에 **sim이 real의 대리인**이 되도록 맞추는 단계.
`steps/`의 S3~S6이 여기에 해당한다.

- 랜덤 환경: `steps/s06_random_world.py`
- sim↔real 신호 비교: `steps/s05_step_response.py` · `real_step_profile.py` · `compare_step.py` · `plot_compare.py`
- 가설/실측 표: `steps/S3_hypotheses.md`

## 제일 중요한 원칙

**USD는 "로봇 자산"이고, 튜닝값 저장소가 아니다.**
`robot_usd()`가 매 실행마다 URDF에서 USD를 다시 만들기 때문에, USD를 손으로 고친 값은 다음 변환에서 날아간다.
→ **기하/질량은 URDF**, **동역학/마찰/지연/잡음은 Python 설정**에 넣는다.

## 차이별로 어디에 넣나

| 차이 | 어디에 | 파일 |
|---|---|---|
| 몸통·바퀴 질량, 관성, 무게중심 | **URDF** (정본) → USD 재생성 | `isaac/urdf/beebot/urdf.xacro` · `ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro` |
| 바퀴 반지름·간격, 라이다 높이, 링크 위치 | **URDF** + `robot_spec.py` 값 | 위 URDF · `isaac/tasks/common/robot_spec.py` |
| 바닥·장애물·바퀴 마찰, 반발 | Isaac Lab **physics material** (Python) | `isaac/steps/scene.py` (`FRICTION_*`) |
| 모터 τ(1차 지연), 좌우 gain, 데드밴드 | **env cfg** (Python) | `isaac/rl/wheel_drift/task/rl_correct_env_cfg.py` |
| 명령/관측 지연 (WiFi) | **env cfg** (Python) | 같은 파일 (`cmd_delay_steps`, `obs_delay_steps`) |
| 엔코더·자이로 잡음, bias | **env cfg** (Python) | 같은 파일 (`enc_noise`, `gyro_noise`) |
| IMU 축·부호, 엔코더 스케일 | **real 먼저** (driver/firmware/URDF) → sim에 반영 | `opencr_driver_node.py` · `firmware/opencr/...ino` · URDF |
| LiDAR 높이/FOV/거리 잡음 | URDF(높이) + sensor cfg(Python) | URDF · `learn/l03_sensors.py` 패턴 |

## 순서

1. **S3 가설 표** — 무엇을 잴지 정한다 (`S3_hypotheses.md`).
2. **S5a 공중(지지 박스)** — 모터·엔코더·IMU 스케일/부호/지연.
   - `v_enc ≈ cmd`, `w_enc ≈ cmd`, `w_gyro ≈ 0`이면 통과.
3. **S5b 바닥** — 마찰·슬립 포함 실제 응답.
   - `w_gyro` 부호·크기, `v_enc` 추종.
4. 차이를 위 표에 따라 **URDF / scene material / env cfg**에 반영.
5. S5를 다시 돌려 파형이 겹치는지 확인.
6. 그다음 `rl/wheel_drift/train.sh`로 학습.

## USD를 고쳐야 하는 경우

- 정말 **형상/질량/관성**이 real과 다를 때만.
- 그래도 USD를 직접 고치지 말고 **URDF를 고친 뒤** `./convert_usd.sh`로 재생성한다.
- USD의 physics material을 직접 바꾸는 것도 가능하지만, 우리는 `scene.py`에서 Python으로 거는 쪽이 재현성이 좋다.

## 현재 상태 (2026-10-30)

- S6 랜덤 환경 1차 완료 (`s06_random_world.py`)
- S5 공중 sim 기준: `v_enc 0.150 / w_enc 1.000 / w_gyro≈0`
- S5 바닥 sim 기준: `rms_v 0.014 / rms_w 0.093`
- real 공중/바닥 측정 대기 — 이 값이 들어와야 env cfg의 τ/gain/deadband/지연을 맞춘다.
