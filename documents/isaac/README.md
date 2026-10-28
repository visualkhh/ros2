# Isaac Sim / Isaac Lab — RL 정책 만들기

TurtleBot3 Waffle Pi용 RL 2종(`rl_correction`, `nav2_rl_controller`)의 학습 쪽 문서.
학습은 lab PC(x86_64 + RTX)에서, 추론은 로봇(Pi 4)에서 ONNX로.

## 파이프라인

```
Isaac Sim/Lab (lab PC) → 학습 → policy.onnx → Pi로 복사
→ ros2_ws 노드에 model_path 지정 → benchmark_onnx.py로 레이턴시 확인 → Before/After 계측
```

## 정책 2종 (입력 → 출력)

| 노드 | 입력 (obs) | 출력 (action) | 학습 환경 |
|---|---|---|---|
| `rl_correction/corrector` | `[v_cmd, w_cmd]` (+odom 오차) | `[Δv, Δw]` 보정량 | 평지 직진·회전 트래킹 (운동학 오차 보정) |
| `nav2_rl_controller/controller_corrector` | `[v_nav, w_nav, 장애물 거리]` | `[Δv, Δw]` 보정량 | 장애물+운동학 제약 속도 다듬기 |

공통: 출력은 노드에서 TurtleBot3 리밋으로 clip (`max_lin 0.22`, `max_ang 2.84`).
인터페이스가 바뀌면 `ros2_ws/src/*/***_node.py`의 `_infer_delta` 입출력부터 맞춘다.

## Lab PC 준비

- Isaac Sim 6.1 standalone (`/home/pank/install/isaac-sim-standalone-6.1.0-linux-x86_64`, RTX 5060) — 확인됨
- [ ] Isaac Lab 설치 (버전 고정 후 `isaac/README.md`에 기록)
- [ ] Task 스펙 ([task-spec.md](task-spec.md)) — obs/action/reward 확정됨, Task 코드로 옮기기 (`isaac/tasks/`)
- [ ] Task 스펙 (obs/action 차원, reward 식)
- [ ] 학습 스크립트 위치 + 시드·하이퍼파라미터
- [ ] ONNX export 명령 (`torch.onnx.export`, opset, input shape `[1, N]`)
- [ ] Pi 복사 경로 + `benchmark_onnx.py` 측정값

## 로봇 쪽 연결

- 모델 파일: 로봇 `~/rl_models/<이름>.onnx` (git 제외, scp로 복사)
- 실행: `corrector --ros-args -p enable:=true -p model_path:=~/rl_models/xxx.onnx`
- 계측: `rl_eval/eval_recorder` → `/tmp/before.csv` vs `/tmp/after.csv`

관련: [ros2_ws README](../../ros2_ws/README.md) · [Lyrical 세팅](../ros/lyrical/lyrical-setup-ubuntu26.04.md)
