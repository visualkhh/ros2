# RL navigation models

`rl_nav_controller`가 고르는 ONNX 정책 폴더다. 모델 하나가 폴더 하나다.

```text
artifacts/models/rl_nav_controller/
└── rl_maze/
    ├── policy.onnx
    └── policy.onnx.data
```

- 폴더 안에 `*.onnx`가 있으면 목록에 뜬다 (`policy.onnx`를 우선 사용).
- ONNX Runtime이 외부 데이터(`*.onnx.data`)를 같이 읽으므로 **두 파일을 항상 같이** 둔다.
- `rl_nav_model_list`(String, latch)에 이름들이 한 줄에 하나씩 나가고,
  Viz에서 고르면 `rl_nav_model`(String)로 노드가 그 자리에서 다시 로드한다.
