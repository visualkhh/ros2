# krosc/viz/plugins

코어 Viz에 기본 포함되는 플러그인을 이 디렉토리로 옮긴다.

예정:

- `displays` — Grid/TF/Scan/Map/Costmap 토글
- `topics` — 토픽 테이블
- `slam` — SLAM/Map 로드/저장, 2D Pose Estimate
- `navigation` — Goal/Multi Goal/Preview/Cancel
- `teleop` — 키보드/패드, 속도
- `manipulator` — 관절 슬라이더, Torque, Reset
- MoveIt은 `krosc_ws/viz/plugins/moveit_plugin.cpp` 프로젝트 플러그인으로 이동 완료

플러그인은 `krosc::viz::VizPlugin`을 상속하고 `initialize(VizHost&)`만 구현하면 된다.
