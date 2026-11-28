# rl_maze policy

Isaac Lab `isaac/training/rl_maze`에서 학습한 주행 정책이다.

```text
원본: isaac/artifacts/rl_maze/training/2026-11-19_11-38-08_nodyn1k/exported/
  policy.onnx
  policy.onnx.data
```

- 관측 52 → 행동 2
- 20 Hz 정책 (Isaac decimation 24)
- `rl_nav_controller`가 `max_lin`/`max_ang`로 스케일한다.
- ONNX Runtime이 `.onnx.data`를 같이 읽으므로 **두 파일을 항상 같이** 둔다.
