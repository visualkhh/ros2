# isaac — RL 이동 보정 학습 (lab PC 전용, Pi에 안 올라감)

목적·설계: **[../documents/isaac/task-spec.md](../documents/isaac/task-spec.md)** (v2) · 처음이면 [START_HERE.md](START_HERE.md).

## 환경 (2026-10-29 실측)

- GPU RTX 5060 Laptop 8GB · Isaac Sim 6.1 standalone · Isaac Lab 6.1 (`/home/pank/pa-course/IsaacLab-6.1`, `.venv`)
- 주의: Lab 6.1 쿼터니언은 **(x, y, z, w)** — 손으로 풀지 말고 `root_*_b`·`quat_from_euler_xyz` 사용

## 구조

| 경로 | 내용 |
|---|---|
| `urdf/beebot/urdf.xacro` | **학습용 로봇 URDF** — ROS용 `ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro`와 의도적으로 나눈 파일, **내용은 글자까지 같게** (`urdf/check_same.sh`) |
| `learn/` | **Isaac 개념 실습 L1~L7** — 하드웨어 없이 Stage/Prim/Sensor/Env/학습 ([learn/README.md](learn/README.md)) |
| `steps/` | **프로젝트 단계 S0~S8** — URDF·센서·실기기와 하나씩 맞추는 작업 ([steps/README.md](steps/README.md)) |
| `ROS2_TOPICS.md` | **Isaac sim ↔ ROS 2 토픽 계약** — `/isaacros/...`, domain 26 ([ROS2_TOPICS.md](ROS2_TOPICS.md)) |
| `urdf/beebot/meshes/` | 몸통·바퀴 메시 — 저장소에서 직접 관리 (tb3_bringup/meshes와 같은 파일) |
| `usd/` | URDF→USD 변환 결과 (`tb3_waffle_pi/tb3_waffle_pi.usda`). 매 실행마다 새로 생성, git 제외 — URDF가 정본 |
| `tasks/common/robot_spec.py` | 우리 로봇 값 한 곳 (바퀴 0.033/0.287 · 0.26 m/s · 1.82 rad/s · 바퀴 8 rad/s · 20 Hz) + 학습용 URDF → USD |
| `tasks/common/onnx_export.py` | actor → ONNX (출력 = 물리 보정량, train·export 공용) |
| `rl/` | **바퀴 드리프트 보정 RL** — env·train·export·play ([rl/README.md](rl/README.md)) |
| `rl/task/` | v2 환경: 모터 지연·좌우 비대칭·데드밴드 · WiFi 지연 · 계단식 명령 · 엔코더+자이로 관측 |
| `rl/smoke.py` | 물리 확인: 오차 0일 때 명령대로 가나 (직진 0.20 · 회전 1.0 · 사선) |
| `scripts/load_urdf.py` | URDF만 띄워 보기 |
| `rl/train.py · export.py · play.py` | 학습 · ONNX 뽑기 · 구경 |
| `logs/` · `export/` | 산출물 (git 제외) |

## USD 생성

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./convert_usd.sh          # URDF → USD만 (Isaac headless, ~30 s)
# → usd/tb3_waffle_pi/tb3_waffle_pi.usda
```

- 정본: `urdf/beebot/urdf.xacro` (ROS용 `tb3_bringup/urdf/robot.urdf.xacro`와 같은 내용)
- 변환: `tasks/common/robot_spec.py::robot_usd()` — xacro → URDF → Isaac `UrdfConverter`
- S1/L1/S2/S5 스크립트도 시작할 때 같은 함수를 자동 호출 — 별도 변환 없이 바로 실행해도 됨
- `usd/`는 git 제외 (파생물). 이전 변환물(`tb3_waffle_pi_1`, `_2`…)은 변환 전에 자동 삭제

## 흐름

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./rl/train.sh 300                     # 창 보기: 뒤에 --visualizer kit (기본은 창 없음)
./rl/export.sh logs/rl_correct/<날짜_시간>/model_299.pt rl_correct_v2.onnx
./rl/play.sh export/rl_correct/rl_correct_v2.onnx 4
# 로봇: PC에서 rl-only.launch.py enable:=true model:=... → Before/After (ros2_ws README)
```

ONNX: 입력 `obs` `[1, 6]` = `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` → 출력 `actions` `[1, 2]` = `[dV m/s, dW rad/s]`.
