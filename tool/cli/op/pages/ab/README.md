# op/pages/ab — A/B 테스트

`rl_nav_controller`(Nav2 `FollowPath`)의 RL ON/OFF를 같은 조건으로 비교한다.

```text
ab/
├── __init__.py          # A/B 메뉴
├── core.py              # toggle · rosbag · guide
└── rl_nav_controller/
    └── command.json     # 자동 A/B 시작 자세·route·반복
```

## CLI

```bash
./run-cli ab nodes
./run-cli ab rl_nav_controller status
./run-cli ab rl_nav_controller on
./run-cli ab rl_nav_controller off
./run-cli ab rl_nav_controller record --label A
./run-cli ab rl_nav_controller stop
./run-cli ab rl_nav_controller guide --label ab1
./run-cli ab rl_nav_controller history
```

- `on`: RL 정책 ON (`/controller_server/FollowPath/toggle` true)
- `off`: inner controller(DWB/MPPI)로 바이패스
- `record`: `~/bags/ab/<시각>_<라벨>`에 rosbag2 기록
- `guide`: `rl_nav_controller/command.json`의 route를 A(RL) → B(inner)로 자동 주행

## command.json

| 키 | 뜻 |
|---|---|
| `frame_id` | goal 좌표계 (기본 `map`) |
| `start` | `{x, y, yaw_deg}` 또는 `null` (현재 자세) |
| `route` | `{x, y, yaw_deg}` 목록 |
| `repeats` | A/B 쌍 반복 횟수 |
| `goal_timeout_s` | goal 하나당 제한 시간 |
| `settle_s` | goal 도착 뒤 대기 |
| `auto_mode` | 시작 전에 NAV_PLAN으로 전환 |
| `move_to_start` · `return_to_start` | 시작 자세로 이동/복귀 |

새 A/B 노드는 `AB_NODES`와 `<노드>/command.json`을 추가한다.
