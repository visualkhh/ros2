# op/pages/ab — A/B 테스트

`rl_nav_controller`(Nav2 `FollowPath`)의 RL ON/OFF를 같은 조건으로 비교한다.

```text
ab/
├── __init__.py          # A/B 메뉴 (① 경로 정하기 · ② 자동 A/B · ③ 리포트 · ④ 수동)
├── core.py              # toggle · rosbag · guide · route.json
└── rl_nav_controller/   # 노드별 — 명령 파일과 리포트가 같은 폴더에
    ├── command.json     # 자동 A/B 시작 자세·route·반복
    └── report.py        # A/B bag → 리포트
```

## TUI 흐름 (`./run-cli` → `b) A/B 테스트` → `rl_nav_controller`)

| 메뉴 | 하는 일 |
|---|---|
| ① 경로 정하기 | `route.json`(start·route)을 만든다 — 주행·녹화는 안 함. 아래 표 |
| ② 자동 A/B 실행 | `guide` — A(RL) 녹화·주행 → 시작점 복귀 → B(inner) 녹화·주행. 주행 중 `s` = 이 구간 중지 · `q` = 전체 중단 |
| ③ 리포트 만들기 | 최근 A/B → HTML |
| ④ 수동 (고급) | 상태 · 정책 ON/OFF · 녹화 시작/종료 · 녹화 목록 — 직접 몰면서 기록할 때만 |

경로 편집 (`tool/bags/ab/route.json`에 저장, 있으면 `command.json`의 start·route보다 우선 — `command.json`은 안 건드림):

| 항목 | 방법 |
|---|---|
| 1) 지도에서 찍기 | **터미널 지도 화면**(`/map`)이 열린다 — RViz 없이. 중앙 = 지도(처음에 포커스), 왼쪽 = 같은 동작 메뉴(`Tab`/`Esc`로 오감). 화면 한가운데 **십자선(`+`)** 이 찍을 자리, 로봇은 **빨간 점(●)**. 지도 포커스 키: `↑↓←→` 지도 밀기(로봇 따라가기가 풀림) · `+`/`-` 십자선 기준 확대·축소 · `f` 전체 보기 · `r` 로봇 따라가기 · `[` `]` 방향 ∓15° · `Enter` 십자선 위치에 goal 추가(벽 위는 거부) · `Backspace` 마지막 취소. 왼쪽 메뉴 9) 저장하고 나가기 · Esc = 저장 않고 나가기(고친 게 있으면 확인) |
| 2) RViz로 지정 | `./run-rviz ab`로 열고 2D Goal Pose로 순서대로 찍음 (토픽 `/ab/goal_pose` — Nav2가 안 들어 로봇은 안 움직임). Enter 저장 · Backspace 마지막 취소 |
| 3) 지금 위치를 출발점으로 | amcl_pose의 현재 자세 |
| 4) 텔레오퍼로 이동하여 goal 추가 | 텔레오퍼 창에서 몰고 가 `g`/Enter → "G{n}로 추가할까요?" → `y`. Esc로 끝 |
| 5) 좌표 입력 | `x y yaw°` |
| 6) 마지막 goal 삭제 · 7) 경로 지우기 | goal이 0개가 되면 `route.json` 삭제 → `command.json` 기본 경로 |

> 터미널 지도는 색(24bit)이 되면 반블록 `▀`(한 글자에 두 칸)로, 아니면 `# .` 문자로 그린다. 로봇 `R` · 출발 `S` · goal `1 2 …` · 커서는 방향 화살표 · 경로는 노란 점.

시작점 복귀는 ②가 한다 (`move_to_start`·`return_to_start`) — 경로 편집 뒤 로봇을 직접 옮길 필요 없음.

## CLI

```bash
./run-cli ab nodes
./run-cli ab rl_nav_controller status
./run-cli ab rl_nav_controller on
./run-cli ab rl_nav_controller off
./run-cli ab rl_nav_controller record --label A
./run-cli ab rl_nav_controller stop
./run-cli ab rl_nav_controller guide --label ab1
./run-cli ab rl_nav_controller report --label ab1
./run-cli ab rl_nav_controller history
```

- `on`: RL 정책 ON (`/controller_server/FollowPath/toggle` true)
- `off`: inner controller(DWB/MPPI)로 바이패스
- `record`: `tool/bags/ab/<시각>_<라벨>`에 rosbag2 기록 (`-a`)
- `guide`: route(`route.json`이 있으면 그것, 없으면 `rl_nav_controller/command.json`)를 A(RL) → B(inner)로 자동 주행. 주행 중 `s`/`q`
- `report`: A/B bag → `tool/bags/ab/report_<label>_<시각>.html` (인라인 SVG, matplotlib 불필요)

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
| `pause_between` | A(RL) 끝나고 B(inner) 전에 "시작 위치로 놓고 Y" 대기 |
| `move_to_start` · `return_to_start` | 시작 자세로 이동/복귀 |

새 A/B 노드는 `AB_NODES`와 `<노드>/command.json`을 추가한다.
