# rl_maze_beebot_path500

Isaac Lab `RlMazeBeebot` 학습 결과를 ONNX로 export한 정책.

- 원본: `isaac/artifacts/rl_maze_beebot/training/2026-11-16_12-13-37_path500/exported/`
- 관측(375) = waypoint 5×2 + **lidar 360** + [v, w, gyro, a0, a1]
  - `inflation` 스칼라가 없다 (기존 `rl_maze`는 36 rays + inflation = 52).
  - `rl_nav_controller`가 정책 입력 크기에서 ray 수를 역산하므로 별도 설정 없이 로드된다.
- 학습 환경 파라미터: `lookahead 5`, `wp_scale 2.0`, `lidar_max 3.5`, `inflation_scale 0.5`,
  `max_lin 0.26`, `max_ang 1.82`, `v_min_ratio -0.2` — ROS 노드 기본값과 동일.
- `policy.onnx`와 `policy.onnx.data`는 항상 같이 둔다.
