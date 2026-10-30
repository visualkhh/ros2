# Isaac 단계별 실험 노트

> Isaac **개념**부터 하나씩: [LEARN.md](LEARN.md) (L1 Stage/Prim/Joint → L7 학습·export).
> 이 문서는 그 위의 **실험 단계** S0~S8.

`train`부터 바로 돌리지 않는다. 단계마다 **명령 1개 → 눈으로 확인 → 통과 기준 → 기록**.
한 단계가 통과해야 다음으로 넘어간다. 막히면 그 단계의 로그·스크린샷을 `steps/artifacts/`에 남긴다.

- 현재: **S2 통과** (2026-10-30, demo 주행 `x=+0.27 m · yaw=+1.62 rad · 실시간 0.99×`, 벽/장애물 충돌 확인)
- 실행 래퍼: `./steps/run.sh <스크립트> [옵션]` — Isaac Lab venv로 돌리고 창 옵션(`--viz kit`)을 자동으로 붙임
- Isaac Lab 경로(이 PC): `/home/pank/pa-course/IsaacLab-6.1`

## 진행표

| # | 단계 | 질문(가설) | 눈으로 보는 것 | 통과 기준 |
|---|---|---|---|---|
| S0 | 도구 확인 | 이 PC에서 Isaac이 뜨나 | 빈 창 + GPU 이름 | 창이 뜨고 에러 없음 |
| S1 | 로봇 + 방 | 우리 URDF가 Isaac에서 우리 로봇인가 | 방·로봇·바퀴·라이다 | base 높이 ≈ 0.01 m, 링크 6개 |
| S2 | 직접 몰기 | 키보드 명령대로 움직이나 | 바퀴·회전·충돌 | 실시간 1.0×, demo 위치/각도 일치 |
| S3 | 가설 고정 | sim이 real의 대리인이 되려면 무엇이 같아야 하나 | 표(가설 H1~H6) | 각 가설에 **측정 방법**이 붙음 |
| S4 | 센서 싱크 | 같은 자극에 sim/real 센서가 같은 값을 내나 | 엔코더·IMU·라이다 수치 | S4 표의 허용오차 안 |
| S5 | 주행 싱크 | 같은 명령에 같은 시간응답이 나오나 | v/w 시간그래프 겹치기 | 상승시간·정상상태 오차 기준 |
| S6 | 랜덤 환경 | 방·장애물이 바뀌어도 깨지지 않나 | 시드별 방·스캔 | 20시드 넘어짐/끼임 0 |
| S7 | 학습 | 보상이 오르고 트래킹 오차가 주나 | TensorBoard·주행 | 오차 감소, 보상 해킹 없음 |
| S8 | 검증 | sim 개선이 real 개선인가 | Before/After CSV | 회전 RMS −20%, 직진 악화 없음 |

---

## S0 — 도구 확인

```bash
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
# NVIDIA GeForce RTX 5060 Laptop GPU, 8151 MiB, 580.178.04
```

Isaac Sim/Lab은 `isaaclab.sh`가 알아서 켠다. 별도 설치 확인은 S1 창이 뜨는 것으로 끝.

## S1 — 로봇 + 방 띄우기 ✅

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./steps/run.sh s01_robot_in_world.py
```

- 하는 일: `urdf/beebot/urdf.xacro` → URDF → USD 변환 → 4×4 m 방(벽 4개·기준 물체 3개)에 스폰 → 1초 정착 → 창 유지
- 눈으로: 로봇이 바퀴로 서 있고, 라이다(주황 상자)가 앞쪽에 있고, 몸통이 2층인지. 마우스 오른쪽 드래그로 돌려서 뒤·옆도.
- 콘솔 통과 기준:
  ```
  [s1] links : base_link, wheel_left_link, wheel_right_link, caster_back_left_link, caster_back_right_link, base_scan
  [s1] base 높이: 0.010 m
  ```
- 실패 시: `0.000` = USD/스폰 문제, 링크 수가 다름 = xacro 문제. `urdf/check_same.sh`로 ROS 쪽과 비교.
- 기록: 창 스크린샷 → `steps/artifacts/s1_robot_room.png` (git 제외)

## S2 — 직접 몰기 ✅

```bash
./steps/run.sh s02_drive_robot.py
```

| 키 | 동작 |
|---|---|
| `W` / `↑` | 전진 |
| `S` / `↓` | 후진 |
| `A` / `←` | 좌회전 (+) |
| `D` / `→` | 우회전 (−) |
| `Shift` | 느리게 (40%) |
| `X` | 정지 |
| `R` | 현재 방 처음 위치로 |
| `N` | **다음 랜덤 방** (방마다 장애물 배치·색이 다름) |

- 물리 120 Hz · 명령 20 Hz(= cmd_manager) · 화면은 가능한 만큼. 렌더가 느려도 물리는 벽시계를 따라잡음.
- 바퀴 명령은 실제 드라이버와 같은 식(`wheel_targets`: 반지름 0.033 · 간격 0.287 · 8 rad/s 포화).
- **콜라이더**: 벽·장애물에 `PhysxCollisionPropertiesCfg`를 붙임. (초기 버전은 시각 전용이라 통과했음 — 2026-10-30 수정)
- **마찰** (static, dynamic): 바닥 `(0.8, 0.7)` · 장애물 `(0.7, 0.5)` · 바퀴 `(1.0, 1.0)` · 캐스터 `(0, 0)`.
  이 값은 실측이 아니라 출발값 — S4/S5에서 실기기 계단응답으로 맞춘다.
- **랜덤 방**: `--rooms 6`(기본) · `--seed 0`(기본, 같으면 재현). 방은 x축 8 m 간격에 미리 깔고, `N`은 로봇만 순간이동시킨다 (프림 삭제/재생성 없음).
- 키보드 없이 물리만 확인:
  ```bash
  ./steps/run.sh s02_drive_robot.py --demo --seconds 5
  ```
  통과 기준(2026-10-30 실측, 마찰 반영 후):
  ```
  [s2] 끝 위치 x=+0.27 m y=-0.01 m yaw=+1.62 rad
  [s2] sim 5.0 s / 95 frames / 벽시계 5.0 s (0.99×)
  ```
  앞 2초 0.15 m/s → 0.27 m, 다음 2초 1.0 rad/s → 약 1.6 rad (가속·마찰 손실).
- **충돌 검증**(별도 headless): 0.25 m/s로 10초 직진 → 벽 앞 `x=1.89 m`에서 정지, 방 이동 후 장애물 앞 `x=8.81 m`에서 정지. 통과.
- 창이 안 움직이면 뷰포트를 한 번 클릭(포커스)하고 다시 키.

## S3 — 가설 고정 (코드 아님, 문서)

전체 표: **[S3_hypotheses.md](S3_hypotheses.md)** — H1~H6, sim 값, real 측정법, 합격 기준, TODO.

학습을 돌리기 전에 "무엇이 같아야 sim을 믿는가"를 못 박는다. 각 가설은 **실기기에서 잴 수 있는 것**이어야 한다.

| # | 가설 | sim 쪽 | real 쪽 측정 | 어디서 |
|---|---|---|---|---|
| H1 | 질량·관성·기하가 같다 | `urdf.xacro` (몸통 1.4 kg) | 저울·줄자 | S4a |
| H2 | 바퀴 명령→속도 식이 같다 | 반지름 0.033 · 간격 0.287 · 8 rad/s 포화 | `/odom` + 바퀴 실측 | S4b |
| H3 | 모터 지연·좌우 차이·데드밴드가 범위 안 | τ 0.05~0.25 s · gain ±10% · 데드밴드 0~0.3 | 계단 명령 응답 | S4b/S5 |
| H4 | 엔코더·IMU 잡음·지연이 범위 안 | 잡음 0.005/0.01 · 지연 0~2스텝 | 정지/정속 로그 | S4c |
| H5 | 라이다 높이·시야·거리 잡음이 같다 | 스캔 높이 0.022 m · 360° | `/scan` + 줄자 | S4d |
| H6 | 정책이 보는 것만 준다 (정답 누설 없음) | obs 6개 = 실제 센서로 계산 | 코드 리뷰 | S4e |

> H5는 라이다를 쓰는 단계에서만. 지금 `rl_correct`는 바퀴·자이로만 보므로 S4d는 보류.

## S4 — 실기기와 센서 싱크

한 번에 다 하지 않고 a→e 순서로. 각 항목은 **sim 값 · real 값 · 차이**를 표에 적고 통과/실패.

```bash
# PC에서 Pi 토픽 (사내 WiFi는 멀티캐스트 불가 → peers.sh)
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=25
source tool/peers.sh
ros2 topic hz /odom /imu /scan
ros2 topic echo --once /odom
ros2 topic echo --once /imu
```

- **S4a 기하·질량**: `./urdf/check_same.sh` (ROS용과 글자까지 같은지) + 실제 무게. 다르면 URDF부터 고친다.
- **S4b 바퀴·엔코더**: sim에서 `v_cmd=0.15` 정속 2초 → 바퀴 rad/s·`v_enc` 기록. real은 같은 명령을 `run-cli`로. 비교식:
  `v_enc = r·(wl+wr)/2`, `w_enc = r·(wr−wl)/sep` — 드라이버와 같은 식인지.
- **S4c IMU**: 제자리 왼쪽 회전(+w)에서 sim/real `gz` 부호가 둘 다 +인지. 축·부호가 다르면 정책이 반대로 배운다.
- **S4d 라이다**: 스캔 평면 높이(0.022 m)와 360° 유무, 최소/최대 거리. 벽까지 줄자 vs `/scan` 값.
- **S4e 누설 점검**: obs가 `root_lin_vel` 같은 sim 정답을 직접 쓰지 않는지 (`rl_correct_env.py _get_observations`).

## S5 — 주행 싱크 (시간응답)

같은 계단 명령을 sim/real에 넣고 `(t, v_cmd, v_enc, w_gyro)`를 겹쳐 본다. **둘 다 같은 CSV 포맷**을 쓴다.

```
0~1 s  정지 (계단 기준점)
1~3 s  v=0.15, w=0
3~5 s  v=0.25, w=0
5~6 s  정지
6~8 s  v=0, w=+1.0
8~9 s  정지
```

**sim 기록** (창 없이, 20 Hz, 9 s):
```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./steps/run.sh s05_step_response.py --viz none
# → steps/artifacts/sim_step.csv
```

**real 기록** (PC에서 실행):
```bash
# 1) Pi: base 띄우고 제어 MANUAL
#    ./run-cli base start
#    ./run-cli mode manual
# 2) PC
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=25
source tool/peers.sh
python3 isaac/steps/real_step_profile.py --output /tmp/real_step.csv
```

**비교**:
```bash
python3 isaac/steps/compare_step.py isaac/steps/baselines/sim_step.csv /tmp/real_step.csv
```

- 지표: `rms_v` · `rms_w`(자이로) · `bias` · `p95` · `t10/t90`(첫 전진·회전) · 구간별 평균
- 합격(초안): 90% 상승시간 차이 ≤ 0.15 s, 정상상태 `v` 오차 ≤ 0.02 m/s, `w` 오차 ≤ 0.10 rad/s
- sim baseline(2026-10-30): `rms_v 0.015 m/s · rms_w 0.092 rad/s · true yaw 1.56 vs odom 1.65`
- `t90_w = nan`이면 "명령의 90%에 못 미침" = 회전 슬립이 큼. real과 비교할 때 이 값이 핵심.
- real `w_gyro`가 0이면 `/imu`가 없는 것 (펌웨어 `I` 줄 확인).

## S6 — 랜덤 환경

- 방 크기·벽 위치·장애물·마찰·조명을 시드로 바꾼다. `--seed` 하나로 재현.
- 눈으로: 로봇이 벽/장애물 안으로 파고들지 않는지, 라이다가 방 형상을 그리는지.
- 통과: 20시드 연속 주행에서 넘어짐 0·끼임 0, 스캔 최소거리 < 실제 벽 거리 오차 5 cm.
- 기록: 시드 3장 스크린샷 + 주행 로그.

## S7 — 학습

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./train_rl_correct.sh 300 --viz kit     # 창 보면서 (느림) / 빼면 창 없이
tensorboard --logdir logs/rl_correct
```

- 통과: `Metrics/track_err` 하락, 보상이 특정 행동(예: 항상 정지)으로 붕괴하지 않음. S5 시간응답이 sim에서 개선.

## S8 — 검증 (sim → real)

1. ONNX: `./export_rl.sh logs/rl_correct/<날짜>/model_299.pt rl_correct_v2.onnx`
2. sim: S6 랜덤 시드에서 before(보정 0)/after(ONNX) 비교
3. real: `rl-only.launch.py enable:=true model:=...` → `./run-cli route rl` → `eval_recorder` before/after → `compare_csv.py`
4. 합격: 회전 RMS −20% 이상, 직진 RMS 악화 없음 (task-spec §4)

---

## 기록 규칙

- 한 번에 **한 가지**만 바꾼다. 바꾼 것·시드·명령·커밋을 그 단계 표에 적는다.
- 스크린샷·CSV는 `steps/artifacts/` (git 제외). 문서에는 경로와 수치만.
- 단계 통과/실패 판정은 **콘솔 수치**로. "눈으로 좋아 보임"은 보조.
