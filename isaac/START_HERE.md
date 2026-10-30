# Isaac 처음부터 (개념 → 실행)

ROS는 아는데 Isaac이 처음인 사람을 위한 문서. 수식 없음.

## 1. Isaac Sim vs Isaac Lab (한 줄씩)

- **Isaac Sim**: 로봇용 가상 세계 + 물리엔진. "가짜 로봇과 가짜 방"을 띄우는 프로그램.
- **Isaac Lab**: 그 위에서 **강화학습**을 돌리는 도구 모음. "로봇 연습장 + 코치" 역할.

관계: Lab이 Sim을 켜고, 로봇 수백 대를 복제해서 동시에 연습시킵니다.

## 2. 우리 폴더 지도

```
isaac/
├── steps/                    실험 S0~S8 + 개념 L1~L7 (LEARN.md) ← 먼저
├── tasks/rl_correct/         연습장 (빈 바닥, 명령대로 움직이게 보정)
├── urdf/beebot/urdf.xacro     학습용 로봇 URDF (ROS용과 같은 내용 — urdf/check_same.sh)
├── usd/                       URDF→USD 결과 (매 실행 생성, git 제외)
├── tasks/common/robot_spec.py  우리 로봇 숫자 한 곳 + 우리 URDF를 Isaac용(USD)으로 변환
├── scripts/train.py          학습 시작 버튼
├── scripts/play.py           학습된 애 구경하기
├── scripts/export.py         학습 결과 ONNX로 뽑기
├── logs/<task>/<날짜>/       학습 기록 (체크포인트 .pt)
└── export/<task>/            로봇에 쓸 모델 (.onnx)
```

시뮬 로봇 = **우리 로봇 URDF** `isaac/urdf/beebot/urdf.xacro` — ROS용(`ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro`)과 같은 내용 (`isaac/urdf/check_same.sh`로 확인).

## 3. 강화학습 5단어

| 단어 | 뜻 | 우리 예시 |
|---|---|---|
| 환경(env) | 연습장 | 빈 바닥 + 모터가 늦고 좌우가 다르고 WiFi가 늦는 가짜 로봇 수백 대 |
| 관측(obs) | 로봇이 보는 것 | `[명령 v, 명령 w, 엔코더 v, 엔코더 w, 자이로 w, 명령 바뀐 뒤 시간]` 6개 — **실제 로봇에서도 보이는 것만** |
| 행동(action) | 로봇이 내는 답 | `[Δv, Δw]` 보정량 2개 |
| 보상(reward) | 칭찬·꾸중 점수 | 진짜 몸체 속도가 명령에 가까우면 + , 보정을 크게·출렁이게 하면 − |
| 에피소드 | 한 판 | 10초 (그 안에서 명령이 0.5~2초마다 바뀜) |

**PPO**는 점수 잘 받는 방향으로 정책을 조금씩 고치는 알고리즘이고, 우리는 그냥 씁니다 (만들 필요 없음).

## 4. 전체 흐름 (외울 것)

```
① Sim에 로봇 수백 대 띄우기 → ② 멋대로 움직이며 점수 따기 → ③ 잘하는 쪽으로 정책 갱신
→ ④ ONNX 파일로 뽑기 → ⑤ PC rl-only가 ONNX를 불러옴 → ⑥ ./run-cli route rl 로 실주행에 더하기
```

## 5. 명령어 (복붙용)

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

# S1. 로봇 + 방을 창에 띄워 눈으로 확인 (먼저 이걸로)
./steps/run.sh s01_robot_in_world.py
#   통과: [s1] base 높이: 0.010 m · 링크 6개

# S2. 키보드로 직접 몰기 (W/A/S/D · Shift · X정지 · R리셋 · N 다음 방)
./steps/run.sh s02_drive_robot.py
#   단계 전체·통과 기준: steps/README.md

# L1. Isaac 개념 — 로봇 해부 (1=prim 트리, 2=joint 상태, 3/4/5=바퀴)
./steps/run.sh l01_robot_anatomy.py
#   개념 목록: steps/LEARN.md

# L2. 물리 시간 — 1/2/4/8로 steps/frame 바꾸기 (sim time / wall time)
./steps/run.sh l02_physics_time.py

# L3. 센서 — IMU + 360° LiDAR (벽에 빨간 hit 점, 2=상세)
./steps/run.sh l03_sensors.py

# L4. Env API — obs/action/reward/done/reset (1=보정없음, 2/4=보정)
./steps/run.sh l04_env_api.py

# L5. 병렬 환경 — 16대가 한 번에 (1=전진, 2/3=회전)
./steps/run.sh l05_vectorize.py --envs 16

# URDF 고친 뒤 USD만 다시 생성
./convert_usd.sh

# 물리 확인 (오차 0이면 명령대로 가나) — 고친 뒤 먼저
cd /home/pank/pa-course/IsaacLab-6.1 && ./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/smoke_rl_correct.py; cd -

# 학습 — 창 보면서: 뒤에 --visualizer kit
./train_rl_correct.sh 300

# 체크포인트 → ONNX
./export_rl.sh logs/rl_correct/<날짜_시간>/model_299.pt rl_correct_v2.onnx

# 구경
./play_rl_correct.sh export/rl_correct/rl_correct_v2.onnx 4
```

## 6. 로봇(ROS) 연결

- 결과물은 ONNX 하나: `[1,6]` → `[1,2]` (물리 보정량 m/s, rad/s).
- PC에서 `ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=<onnx>` → `./run-cli route rl` (PC에 `onnxruntime` 필요).
- 로봇 `/imu` 20 Hz 동작 확인 (2026-10-30) — OpenCR `I` 줄. 정지 시 gz≈0, az≈9.98.
- **Isaac sim 토픽 계약**: `/isaacros/...`, `ROS_DOMAIN_ID=26` — [ROS2_TOPICS.md](ROS2_TOPICS.md). real(25)과 분리.
- Before/After: `rl_eval/eval_recorder` CSV 두 개 → `rl_eval/compare_csv.py`.

## 7. 자주 묻는 것

- **Q. 왜 가짜 로봇으로 연습하나요?** A. 진짜 로봇으로 수천 번 하면 오래 걸리고 위험해서. 시뮬에서 수백 대로 병렬 연습합니다.
- **Q. 무엇을 고치나요?** A. 모터 반응 지연·좌우 차이·저속에서 안 도는 구간·회전 미끄러짐 (엔코더·자이로로 보이는 것). 직진 거리 오차는 못 고칩니다 (실제에서 보이지 않음).
- **Q. 창이 안 떠요.** A. `--visualizer kit`을 붙이세요. 안 붙이면 창 없이 돕니다.
