# RL navigation models

`rl_nav_controller`가 고르는 ONNX 정책 폴더다. 모델 하나가 폴더 하나다.

```text
artifacts/models/rl_nav_controller/
├── rl_maze/                     # 36 rays + inflation (obs 52)
└── rl_maze_beebot_path500/      # Isaac export, 360 rays (obs 375)
    ├── policy.onnx
    └── policy.onnx.data
```

- 폴더 안에 `*.onnx`가 있으면 목록에 뜬다 (`policy.onnx`를 우선 사용).
- ONNX Runtime이 외부 데이터(`*.onnx.data`)를 같이 읽으므로 **두 파일을 항상 같이** 둔다.
- `rl_nav_model_list`(String, latch)에 이름들이 한 줄에 하나씩 나가고,
  Viz에서 고르면 `rl_nav_model`(String)로 노드가 그 자리에서 다시 로드한다.
- 폴더는 2초마다 다시 스캔한다. 새 폴더를 넣으면 재시작 없이 목록에 뜬다.
- **관측 크기는 정책 입력에서 역산한다**: `2*lookahead + rays + 5(+inflation)`.
  Isaac 정책(360 rays, obs 375)과 기존 rl_maze(36 rays, obs 52)를 설정 없이 같이 쓸 수 있다.
- Isaac 학습 결과 복사:
  ```bash
  cp isaac/artifacts/rl_maze_beebot/training/<run>/exported/policy.onnx{,.data} \
     krosc_ws/artifacts/models/rl_nav_controller/<이름>/
  ```
