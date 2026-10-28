# isaac — RL 학습 워크스페이스 (lab PC 전용, Pi에 안 올라감)

TurtleBot3 RL 2종(`rl_correction`, `nav2_rl_controller`) 정책 학습용.
스펙: [../documents/isaac/task-spec.md](../documents/isaac/task-spec.md)

## 환경 (2026-10-28 실측)

- GPU: RTX 5060 Laptop 8GB · driver 595.91
- Isaac Sim: 6.1 standalone (`/home/pank/install/isaac-sim-standalone-6.1.0-linux-x86_64`)
- Isaac Lab: 6.1 (`/home/pank/pa-course/IsaacLab-6.1`, `.venv`, `import isaaclab` OK) — 확인됨
- torch: 2.14.1+cpu (학습용 CUDA torch는 Lab 설치 시 같이 옴)

## 구조

- `tasks/` — 커스텀 Task (rl_correct · nav2_rl, task-spec 기준)
- `scripts/` — 학습 실행 + ONNX export 스크립트
- `export/` — 산출물 (`*.onnx`, git 제외) → 로봇 `~/rl_models/`로 scp

## 흐름

```bash
# 1. 학습 (Lab 설치 후)
./scripts/train.sh rl_correct
# 2. export
./scripts/export.sh export/rl_correct_v1.onnx
# 3. Pi 벤치 (로봇에서)
scp export/rl_correct_v1.onnx pak@pak.local:~/rl_models/
# Pi: python3 ~/ros2/ros2_ws/src/rl_eval/benchmark_onnx.py ~/rl_models/rl_correct_v1.onnx
```
