# steps — RL 전 셋팅 (한 단계씩)

학습(`rl/wheel_drift/`) 전에 **sim이 real의 대리인**이 되도록 맞추는 단계.
한 번에 다 하지 않고 01부터 하나씩.

| # | 파일/단계 | 내용 | 상태 |
|---|---|---|---|
| 01 | `01_random_world.py` | 랜덤 방(3.5~6 m) + 장애물 + 우리 로봇, `N` 다음 방 | ✅ |
| 02 | `02_sensor_sync.py` | IMU/odom 신호를 real과 같은 명령에서 비교 | ☐ |
| 03 | `03_step_stand.py` | 공중(배 밑 지지 박스) 계단응답 sim/real | ☐ |
| 04 | `04_step_floor.py` | 바닥 계단응답 sim/real | ☐ |
| 05 | 차이 반영 | URDF / 마찰 / env cfg에 실측값 넣기 | ☐ |
| 06 | `rl/wheel_drift/train.sh` | 학습 → export → real Before/After | ☐ |

## 지금 있는 파일

```
steps/
├── 01_random_world.py     랜덤 환경 + 로봇 (지금 실행할 것)
├── scene.py               방·로봇 생성 공용 (01이 사용)
├── plot_compare.py        sim/real CSV 파형 비교 도구 (02 이후 사용)
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

## 02 이후

삭제하고 다시 만든다. 순서:
1. `02_sensor_sync.py` — `/odom`·`/imu`를 sim과 같은 명령에서 기록/비교
2. `03_step_stand.py` — 로봇을 **배 밑 지지 박스** 위에 올린 상태 (바퀴 공중)
3. `04_step_floor.py` — 바닥에 내려놓은 상태
4. `plot_compare.py`로 파형 겹쳐 보기
5. 차이를 `PRE_RL.md` 표에 따라 URDF / `scene.py` 마찰 / `rl/wheel_drift/task/rl_correct_env_cfg.py`에 반영
6. `rl/wheel_drift/train.sh`로 학습
