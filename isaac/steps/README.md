# steps — RL 전 셋팅 (한 단계씩)

학습(`rl/wheel_drift/`) 전에 **sim이 real의 대리인**이 되도록 맞추는 단계.
한 번에 다 하지 않고 01부터 하나씩.

| # | 파일/단계 | 내용 | 상태 |
|---|---|---|---|
| 01 | `01_random_world.py` | 랜덤 방(3.5~6 m) + 장애물 + 우리 로봇, `N` 다음 방 | ✅ |
| 02 | `02_sensor_sync.py` + `02_sensor_sync_real.py` | IMU/odom 신호를 real과 같은 명령에서 비교 | sim ✅ · real 대기 |
| 03 | `03_step_stand.py` | 공중(배 밑 지지 박스) 계단응답 sim/real | ☐ |
| 04 | `04_step_floor.py` | 바닥 계단응답 sim/real | ☐ |
| 05 | 차이 반영 | URDF / 마찰 / env cfg에 실측값 넣기 | ☐ |
| 06 | `rl/wheel_drift/train.sh` | 학습 → export → real Before/After | ☐ |

## 지금 있는 파일

```
steps/
├── 01_random_world.py     랜덤 환경 + 로봇
├── 02_sensor_sync.py      sim 센서 기록 (--stand / 바닥)
├── 02_sensor_sync_real.py real 센서 기록 (PC, rclpy)
├── scene.py               방·로봇 생성 공용
├── plot_compare.py        sim/real CSV 파형 비교 도구
├── artifacts/             실행 산출물 CSV/PNG (git 제외)
├── baselines/             sim 비교 기준 CSV (커밋)
├── PRE_RL.md              차이값을 URDF/마찰/env cfg 중 어디에 넣는지
├── S3_hypotheses.md       가설·실측 표
├── run.sh                 실행 래퍼 (--viz kit 자동)
└── README.md              이 파일
```

## 실행

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./steps/run.sh 01_random_world.py
```

- 방 크기·장애물이 랜덤. `--seed 7`로 재현, 없으면 매번 새 seed.
- `W/A/S/D` 주행 · `Shift` 느리게 · `X` 정지 · `R` 현재 방 리셋 · `N` 다음 방.

## 02 — 센서 싱크 (sim 기준 생성됨)

```bash
# 공중(지지 박스) — 눈으로 보기 (9초 기록 후 창 닫을 때까지 반복)
./steps/run.sh 02_sensor_sync.py --stand --watch

# 공중 — 기록만 (창 없이)
./steps/run.sh 02_sensor_sync.py --viz none --stand

# 바닥 — 기록만
./steps/run.sh 02_sensor_sync.py --viz none
```

`--stand --watch`는 로봇이 **배 밑 박스 위**에 올라가 바퀴만 도는 장면을 옆에서 계속 보여줌.
박스 높이가 실제와 다르면 `--stand-height 0.04`처럼 지정.

- sim 실행 결과 기본: `artifacts/sim_sensor_stand.csv` · `sim_sensor_floor.csv`
- 비교 기준(커밋): `baselines/sim_sensor_stand.csv` · `sim_sensor_floor.csv`
  (기준을 갱신할 때만 `./steps/run.sh 02_sensor_sync.py --viz none --stand --out steps/baselines/sim_sensor_stand.csv`)
- real 기록 (PC):
  ```bash
  source /opt/ros/lyrical/setup.bash
  export ROS_DOMAIN_ID=25
  source tool/peers.sh

  # 로봇 안 움직이고 상태만 확인
  python3 steps/02_sensor_sync_real.py --check-only

  python3 steps/02_sensor_sync_real.py --label real_stand
  python3 steps/02_sensor_sync_real.py --label real_floor
  ```
  `/state_control`은 TRANSIENT_LOCAL(latched)라 스크립트가 그에 맞춰 구독한다.
- 비교 (기본은 PNG 저장만):
  ```bash
  python3 steps/plot_compare.py steps/baselines/sim_sensor_stand.csv steps/artifacts/real_stand.csv
  python3 steps/plot_compare.py steps/baselines/sim_sensor_floor.csv steps/artifacts/real_floor.csv
  ```
- **창으로 직접 보고 싶을 때만 `--show`**:
  ```bash
  python3 steps/plot_compare.py     steps/baselines/sim_sensor_stand.csv steps/artifacts/real_stand.csv --show
  ```
  창을 닫으면 터미널로 돌아온다. `--show` 없으면 `steps/artifacts/compare.png`만 저장.
- 공중: `v_enc≈cmd`, `w_enc≈cmd`, `w_gyro≈0`이 정상.
- 바닥: `w_gyro` 부호·크기, `v_enc` 추종이 핵심.

## 03 이후

1. `03_step_stand.py` — 지지 박스 위 미세 계단응답
2. `04_step_floor.py` — 바닥 계단응답
3. 차이를 `PRE_RL.md` 표에 따라 URDF / `scene.py` 마찰 / `rl/wheel_drift/task/rl_correct_env_cfg.py`에 반영
4. `rl/wheel_drift/train.sh`로 학습
