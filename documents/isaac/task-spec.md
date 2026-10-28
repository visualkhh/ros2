# RL Task 스펙 (2종)

노드 인터페이스와 1:1로 맞춘다. 바꾸면 양쪽(학습·노드) 같이 바꾼다.

## 1. 이동 보정 (`rl_correction/corrector`)

| 항목 | 스펙 |
|---|---|
| obs (N=4) | `[v_cmd, w_cmd, v_odom, w_odom]` — 명령 vs 실제 |
| action (2) | `[Δv, Δw]` — 노드가 더한 뒤 clip (0.22 / 2.84) |
| reward | `-(\|v_cmd - v_odom\| + 0.5·\|w_cmd - w_odom\|) - 0.01·\|Δ\|²` |
| 종료 | 10초 경과 · 전복(사용 안 함, 평지) |
| 커리큘럼 | 직진 → 제자리 회전 → S자 곡선 |

## 2. Controller 보정 (`nav2_rl_controller/controller_corrector`)

| 항목 | 스펙 |
|---|---|
| obs (N=5) | `[v_nav, w_nav, d_front, d_left, d_right]` (m, 최대 8m clip) |
| action (2) | `[Δv, Δw]` — 노드가 더한 뒤 clip |
| reward | `-충돌(-10) - 급감속 패널티 - 목표방향 정렬 보너스` |
| 종료 | 충돌 · 30초 경과 · 목표 도달 |
| 커리큘럼 | 빈 방 → 정적 장애물 → 좁은 통로 |

## ONNX export 규칙

- 입력 shape `[1, N]`, float32, opset 17
- 파일명: `rl_correct_v1.onnx`, `nav2_rl_v1.onnx` → 로봇 `~/rl_models/`
- 검증: `benchmark_onnx.py` 평균 50ms 이하 (Pi 실측)
