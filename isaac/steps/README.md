# Isaac 단계별 실험 노트

`train`부터 바로 돌리지 않는다. 단계마다 **명령 1개 → 눈으로 확인 → 통과 기준 → 기록**.
한 단계가 통과해야 다음으로 넘어간다. 막히면 그 단계의 로그·스크린샷을 `steps/artifacts/`에 남긴다.

- 현재: **S1 통과** (2026-10-30, `base 높이 0.010 m`)
- 실행 래퍼: `./steps/run.sh <스크립트> [옵션]` — Isaac Lab venv로 돌리고 창 옵션(`--viz kit`)을 자동으로 붙임
- Isaac Lab 경로(이 PC): `/home/pank/pa-course/IsaacLab-6.1`

## 진행표

| # | 단계 | 질문(가설) | 눈으로 보는 것 | 통과 기준 |
|---|---|---|---|---|
| S0 | 도구 확인 | 이 PC에서 Isaac이 뜨나 | 빈 창 + GPU 이름 | 창이 뜨고 에러 없음 |
| S1 | 로봇 + 방 | 우리 URDF가 Isaac에서 우리 로봇인가 | 방·로봇·바퀴·라이다 | base 높이 ≈ 0.01 m, 링크 6개 |
| S2 | 가설 고정 | sim이 real의 대리인이 되려면 무엇이 같아야 하나 | 표(가설 H1~H6) | 각 가설에 **측정 방법**이 붙음 |
| S3 | 센서 싱크 | 같은 자극에 sim/real 센서가 같은 값을 내나 | 엔코더·IMU·라이다 수치 | S3 표의 허용오차 안 |
| S4 | 주행 싱크 | 같은 명령에 같은 시간응답이 나오나 | v/w 시간그래프 겹치기 | 상승시간·정상상태 오차 기준 |
| S5 | 랜덤 환경 | 방·장애물이 바뀌어도 깨지지 않나 | 시드별 방·스캔 | 20시드 넘어짐/끼임 0 |
| S6 | 학습 | 보상이 오르고 트래킹 오차가 주나 | TensorBoard·주행 | 오차 감소, 보상 해킹 없음 |
| S7 | 검증 | sim 개선이 real 개선인가 | Before/After CSV | 회전 RMS −20%, 직진 악화 없음 |

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

## S2 — 가설 고정 (코드 아님, 문서)

학습을 돌리기 전에 "무엇이 같아야 sim을 믿는가"를 못 박는다. 각 가설은 **실기기에서 잴 수 있는 것**이어야 한다.

| # | 가설 | sim 쪽 | real 쪽 측정 | 어디서 |
|---|---|---|---|---|
| H1 | 질량·관성·기하가 같다 | `urdf.xacro` (몸통 1.4 kg) | 저울·줄자 | S3a |
| H2 | 바퀴 명령→속도 식이 같다 | 반지름 0.033 · 간격 0.287 · 8 rad/s 포화 | `/odom` + 바퀴 실측 | S3b |
| H3 | 모터 지연·좌우 차이·데드밴드가 범위 안 | τ 0.05~0.25 s · gain ±10% · 데드밴드 0~0.3 | 계단 명령 응답 | S3b/S4 |
| H4 | 엔코더·IMU 잡음·지연이 범위 안 | 잡음 0.005/0.01 · 지연 0~2스텝 | 정지/정속 로그 | S3c |
| H5 | 라이다 높이·시야·거리 잡음이 같다 | 스캔 높이 0.022 m · 360° | `/scan` + 줄자 | S3d |
| H6 | 정책이 보는 것만 준다 (정답 누설 없음) | obs 6개 = 실제 센서로 계산 | 코드 리뷰 | S3e |

> H5는 라이다를 쓰는 단계에서만. 지금 `rl_correct`는 바퀴·자이로만 보므로 S3d는 보류.

## S3 — 실기기와 센서 싱크

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

- **S3a 기하·질량**: `./urdf/check_same.sh` (ROS용과 글자까지 같은지) + 실제 무게. 다르면 URDF부터 고친다.
- **S3b 바퀴·엔코더**: sim에서 `v_cmd=0.15` 정속 2초 → 바퀴 rad/s·`v_enc` 기록. real은 같은 명령을 `run-cli`로. 비교식:
  `v_enc = r·(wl+wr)/2`, `w_enc = r·(wr−wl)/sep` — 드라이버와 같은 식인지.
- **S3c IMU**: 제자리 왼쪽 회전(+w)에서 sim/real `gz` 부호가 둘 다 +인지. 축·부호가 다르면 정책이 반대로 배운다.
- **S3d 라이다**: 스캔 평면 높이(0.022 m)와 360° 유무, 최소/최대 거리. 벽까지 줄자 vs `/scan` 값.
- **S3e 누설 점검**: obs가 `root_lin_vel` 같은 sim 정답을 직접 쓰지 않는지 (`rl_correct_env.py _get_observations`).

## S4 — 주행 싱크 (시간응답)

같은 계단 명령을 sim/real에 넣고 `(t, v_cmd, v_enc, w_gyro)`를 겹쳐 본다.

```
0.00~2.00 s  v=0.15, w=0
2.00~4.00 s  v=0.25, w=0
4.00~5.00 s  v=0,    w=0
5.00~7.00 s  v=0,    w=+1.0
7.00~8.00 s  v=0,    w=0
```

- 합격(초안): 90% 상승시간 차이 ≤ 0.15 s, 정상상태 `v` 오차 ≤ 0.02 m/s, `w` 오차 ≤ 0.10 rad/s
- 도구: `rl_eval/eval_recorder` CSV + `compare_csv.py` (real), sim은 `steps/s04_step_response.py` (다음에 만든다)

## S5 — 랜덤 환경

- 방 크기·벽 위치·장애물·마찰·조명을 시드로 바꾼다. `--seed` 하나로 재현.
- 눈으로: 로봇이 벽/장애물 안으로 파고들지 않는지, 라이다가 방 형상을 그리는지.
- 통과: 20시드 연속 주행에서 넘어짐 0·끼임 0, 스캔 최소거리 < 실제 벽 거리 오차 5 cm.
- 기록: 시드 3장 스크린샷 + 주행 로그.

## S6 — 학습

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./train_rl_correct.sh 300 --viz kit     # 창 보면서 (느림) / 빼면 창 없이
tensorboard --logdir logs/rl_correct
```

- 통과: `Metrics/track_err` 하락, 보상이 특정 행동(예: 항상 정지)으로 붕괴하지 않음. S4 시간응답이 sim에서 개선.

## S7 — 검증 (sim → real)

1. ONNX: `./export_rl.sh logs/rl_correct/<날짜>/model_299.pt rl_correct_v2.onnx`
2. sim: S5 랜덤 시드에서 before(보정 0)/after(ONNX) 비교
3. real: `rl-only.launch.py enable:=true model:=...` → `./run-cli route rl` → `eval_recorder` before/after → `compare_csv.py`
4. 합격: 회전 RMS −20% 이상, 직진 RMS 악화 없음 (task-spec §4)

---

## 기록 규칙

- 한 번에 **한 가지**만 바꾼다. 바꾼 것·시드·명령·커밋을 그 단계 표에 적는다.
- 스크린샷·CSV는 `steps/artifacts/` (git 제외). 문서에는 경로와 수치만.
- 단계 통과/실패 판정은 **콘솔 수치**로. "눈으로 좋아 보임"은 보조.
