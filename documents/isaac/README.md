# Isaac Sim / Isaac Lab — RL 정책 만들기

TurtleBot3 Waffle Pi 개조 로봇의 **RL 이동 보정** 학습 쪽 문서. 목적: 같은 경로를 RL 끄고/켜고 달려 트래킹이 좋아짐을 수치로 (Before/After).
학습은 lab PC(x86_64 + RTX), 추론도 PC(`rl-only`, onnxruntime) — 바퀴 명령이 cmd_manager → PC → cmd_manager로 돎.

## 파이프라인

```
Isaac Lab (우리 URDF) → 학습 → rl_correct_v2.onnx → PC rl-only (corrector) → ./run-cli route rl
→ eval_recorder CSV (before / after) → compare_csv.py
```

## 정책 (v2 — 이동 보정 1종)

| 노드 | 입력 (obs 6) | 출력 | 학습 환경 |
|---|---|---|---|
| `rl_correction/corrector` (PC, rl-only) | `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change]` | `[Δv, Δw]` 물리 보정량 | 우리 URDF · 모터 지연/비대칭/데드밴드 · WiFi 지연 · 계단식 명령 |

한계 0.26 m/s · 1.82 rad/s (Waffle Pi). Nav2 컨트롤러 단계 보정(`nav2_rl_controller`)은 삭제(2026-10-29) — [task-spec.md](task-spec.md) §0.
설계·이유·평가 기준: **[task-spec.md](task-spec.md)**. 실행: [../../isaac/README.md](../../isaac/README.md).

## 로봇 쪽 연결

- 모델 파일: `isaac/export/rl_correct/<이름>.onnx` (git 제외). Pi에서 직접 돌릴 땐 `~/rl_models/`로 scp + `benchmark_onnx.py`
- 실행: PC `ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=<onnx>` (PC에 onnxruntime) → `./run-cli route rl`
- 계측: `rl_eval/eval_recorder` → `/tmp/before.csv` vs `/tmp/after.csv` → `rl_eval/compare_csv.py`

관련: [ros2_ws README](../../ros2_ws/README.md) · [Lyrical 세팅](../ros/lyrical/lyrical-setup-ubuntu26.04.md)
