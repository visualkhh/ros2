# isaac — RL 이동 보정 학습 (lab PC 전용, Pi에 안 올라감)

목적·설계: **[../documents/isaac/task-spec.md](../documents/isaac/task-spec.md)** (v2) · 처음이면 [START_HERE.md](START_HERE.md).

## 환경 (2026-10-29 실측)

- GPU RTX 5060 Laptop 8GB · Isaac Sim 6.1 standalone · Isaac Lab 6.1 (`/home/pank/pa-course/IsaacLab-6.1`, `.venv`)
- 주의: Lab 6.1 쿼터니언은 **(x, y, z, w)** — 손으로 풀지 말고 `root_*_b`·`quat_from_euler_xyz` 사용

## 구조

| 경로 | 내용 |
|---|---|
| `urdf/beebot/urdf.xacro` | **학습용 로봇 URDF** — ROS용 `ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro`와 의도적으로 나눈 파일, **내용은 글자까지 같게** (`urdf/check_same.sh`) |
| `steps/` | **단계별 실험 노트** — S1 창에 로봇 띄우기부터 S8 검증까지, 눈으로 확인하며 한 단계씩 ([steps/README.md](steps/README.md)) |
| `learn/` | **Isaac 개념 실습** — L1 로봇 해부(Stage/Prim/Joint)부터 L7 학습·export까지 ([learn/README.md](learn/README.md)) |
| `urdf/beebot/meshes/` | 몸통·바퀴 메시 — 저장소에서 직접 관리 (tb3_bringup/meshes와 같은 파일) |
| `tasks/common/robot_spec.py` | 우리 로봇 값 한 곳 (바퀴 0.033/0.287 · 0.26 m/s · 1.82 rad/s · 바퀴 8 rad/s · 20 Hz) + 학습용 URDF → USD |
| `tasks/common/onnx_export.py` | actor → ONNX (출력 = 물리 보정량, train·export 공용) |
| `tasks/rl_correct/` | v2 환경: 모터 지연·좌우 비대칭·데드밴드 · WiFi 지연 · 계단식 명령 · 엔코더+자이로 관측 |
| `scripts/smoke_rl_correct.py` | 물리 확인: 오차 0일 때 명령대로 가나 (직진 0.20 · 회전 1.0 · 사선) |
| `scripts/load_urdf.py` | URDF만 띄워 보기 |
| `scripts/train.py · export.py · play.py` | 학습 · ONNX 뽑기 · 구경 |
| `logs/` · `export/` | 산출물 (git 제외) |

## 흐름

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./train_rl_correct.sh 300                     # 창 보기: 뒤에 --visualizer kit (기본은 창 없음)
./export_rl.sh logs/rl_correct/<날짜_시간>/model_299.pt rl_correct_v2.onnx
./play_rl_correct.sh export/rl_correct/rl_correct_v2.onnx 4
# 로봇: PC에서 rl-only.launch.py enable:=true model:=... → Before/After (ros2_ws README)
```

ONNX: 입력 `obs` `[1, 6]` = `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` → 출력 `actions` `[1, 2]` = `[dV m/s, dW rad/s]`.
