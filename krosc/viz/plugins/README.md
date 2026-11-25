# krosc/viz/plugins

코어 Viz 패널은 `VizWindow`에 하드코딩되지 않는다. 필요한 플러그인만 골라 붙인다.

구현됨:

- `teleop` — 주행 방향 패드 + 속도 + 도크 포커스에서 W/A/S/D 전달
- `slam` — SLAM start/stop, map 목록 로드/저장/삭제/unload, collision 토글
- `navigation` — 2D Pose Estimate, 2D Goal Pose, Multi Goal/Send/Clear/Cancel, Preview/Go
- `manipulator` — 관절 슬라이더, 그리퍼, Torque/STOW/HOME/E-STOP/Reset

프로젝트 플러그인 예:

- `krosc_ws/viz/plugins/moveit_plugin.cpp`
- `krosc_ws/viz/plugins/rl_nav_plugin.cpp`
- `krosc_ws/viz/plugins/project_panel.cpp`

쓰는 법:

```cpp
krosc::viz::VizPluginRegistry registry;
krosc::viz::register_core_plugins(registry, {"slam", "navigation"});
registry.add("my_plugin", [] { return std::make_unique<MyPlugin>(); });

krosc::viz::VizOptions options;
bool help = false;
std::string error;
krosc::viz::parse_viz_options(argc, argv, defaults, options, help, error);

krosc::viz::Viz viz(std::move(options), std::move(registry));
return viz.run();
```

`VizWindow`은 고정 프레임/Follow/Reset/Displays/Topics/TF 같은 **기반 UI만** 만든다.
SLAM/지도, 목표, 텔레옵, 팔 패널은 전부 플러그인이다.
`Viz`는 Context/Client/Window/플러그인 로드/타이머/스크린샷/종료를 소유하고,
`main.cpp`는 외부 파라미터만 `VizOptions`로 넘긴다.
