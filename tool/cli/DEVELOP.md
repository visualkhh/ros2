# DEVELOP — robot_tool(CLI) 개발자용

CLI를 고치거나 화면·메뉴를 추가하는 개발자용 문서입니다. 쓰는 법(명령·키)은 [../README.md](../README.md)를 봅니다.

- Python 3 · rclpy만 씀 (curses·textual 같은 TUI 라이브러리 없음)
- 화면은 **stdout에 글자 + ANSI escape code**를 쓰고, 키는 **stdin에서 바이트를 직접** 읽어서 만든다
- ROS 없이 시험: `../cli-test/` (가짜 ROS층 + pty 키 입력 시나리오 `scenarios.py`)

## 1. 구조

```text
cli/
├── robot_tool.py   # RobotTool(Node): 구독 배선 + argparse + main. 화면 이름 → Page (screen()) · window 신호 연결
└── op/
    ├── terminal.py    # Terminal — 크기 · 색 · 표시 너비(fit). ROS·메뉴를 모름
    ├── keys.py        # KeyReader(키 읽기) · set_cbreak
    ├── component.py   # Component — 화면 부품 최상위 부모 (focusable · on_key · activate)
    ├── window/        # Window — 상태바·메뉴 막대·중앙·힌트 조합 · 키 루프 · 신호 · 키 규약
    ├── menu_bar/      # MenuBar → LeftSideMenuBar · ChoiceItem
    ├── map_view/      # 터미널 지도 — MapGrid(벽 우선 피라미드) · Viewport(중심·확대) · MapView(포커스 가능 Component)
    ├── state_bar/     # 상단 상태바 — State(칸) 하위 클래스들 + StateBar(줄로 그림) + TopStateBar(기본 구성)
    ├── dialog/        # 창 — Dialog(dialog.py) 최상위 부모 + 하위 클래스 한 파일에 하나 (Input·Key·Pager·Busy·Teleop·RouteRviz)
    ├── ui.py          # Ui — 터미널 모드 · print 가로채기 · confirm/modal_input/pager/select 진입점 · ui.window
    ├── state.py       # StateStore — 구독한 ROS 상태 보관 (화면은 여기만 읽음)
    ├── procs.py       # Procs — launch 프로세스(로컬/SSH 원격) · 지도 파일
    ├── bus.py         # Bus — 발행·대기·모드 게이트 · cmd_* (한 번 실행 명령과 메뉴가 같이 씀)
    ├── defs.py        # BACK/MAIN/QUIT/STAY/TIMEOUT · MENU_REFRESH_S
    └── pages/         # Page → Menu → 각 화면 (MenuItem 객체)
        ├── nodes/     # 노드별 추가 상태 (feature.py + <node>.py)
        └── ab/        # A/B 화면 + core.py + <node>/{command.json, report.py}
```

`./run-cli status`처럼 인자가 있으면 `Bus.cmd_*`만 돌고 화면은 없습니다. 인자가 없고 TTY면 `MainMenu`가 뜹니다.

## 2. 화면을 만드는 부품

### 2.1 Component — 포커스와 activate

```text
Component (op/component.py)  — focusable · on_key(k) → 처리했으면 True · activate() → STAY/BACK/MAIN/QUIT 또는 값
 ├─ MenuItem (pages/base.py)    activate() = window.perform(label, run) — '처리 중' 창 · print 가로채 결과창 · run()
 │   └─ ChoiceItem (menu_bar/choice_item.py)  (key, label) 고르기 — activate()가 key를 돌려줌 (ui.select)
 ├─ State (state_bar/state.py)  focusable 기본 False — True로 하면 Tab으로 상태바에 들어가 ←→로 고르고 Enter
 ├─ StateBar → TopStateBar      focusable 칸이 있으면 포커스 가능 · activate() = 그 칸의 activate()
 └─ MenuBar (menu_bar/menu_bar.py)  MenuItem들을 세로로 · ↑↓/숫자/글자 포커스
     └─ LeftSideMenuBar         왼쪽 열 (폭 = 라벨에 맞춤, 최소 24 · 최대 화면 절반)

Window (op/window/window.py) — ui.window
  top_bar(TopStateBar) + menu_bar(LeftSideMenuBar) + 중앙(center_fn) + 힌트 줄을 조합해 draw()
  navigate(name, **params)                     이름으로 화면 열기 — 웹 페이지 이동처럼 파라미터를 넘김 (앱의 screen(name, **params)가 받음)
  run(title, items_fn, center_fn, back, main)  화면 하나의 키 루프 (Menu.show가 부름)
     Tab = 메뉴 막대 ↔ 상태바 포커스 · Enter = 포커스된 부품 activate() · Esc/b/m/q = 이동
  run(…, center_comp=지도, focus_center=True)  중앙에 포커스 가능한 부품을 올림 — Tab으로 메뉴 막대와 오감 (예: MapPage의 MapView)
  select(title, [(key, label)], center)        한 번 고르기 (ChoiceItem으로 run(once=True))
  perform(label, fn, busy)                     실행 + 끝나면 이 화면의 메뉴 막대·중앙·포커스 복원
  신호 connect/emit — RobotTool이 연결:
     'tick'(seconds)  ROS 폴링 (메뉴 대기 · 창이 떠 있는 동안)
     'navigate'(name, params) screen(name, **params).show() — 상태 칸·다른 화면에서 화면 열기 (파라미터 전달)
```

소스에서 직접 실행하려면 Enter와 같은 길로 `item.activate()` / `state.activate()`를 부른다.
지금 focusable 상태 칸은 없다 — 새로 만들 때 `focusable = True` + `activate()` (예: `self.ui.window.navigate('4')`).

### 2.2 Dialog — 모든 창은 Dialog를 상속

```text
Dialog (op/dialog/dialog.py)   박스 그리기 · 크기 · 지우기 · 키 루프 run()
   content() 박스 안 줄 · on_key(k) → CONTINUE 또는 결과 · on_tick() 매 주기 · fallback() TTY 아닐 때
 ├─ InputDialog     한 줄 입력 · validate               ← ui.modal_input · read_float
 ├─ KeyDialog       키 하나 (y/n)                       ← ui.confirm · modal_key
 ├─ PagerDialog     결과창 스크롤                       ← ui.pager
 ├─ BusyDialog      처리 중 창 (키 루프 없음)           ← ui.busy · step · progress
 ├─ TeleopDialog    텔레오퍼                            ← bus.cmd_teleop (A/B 경로 goal 찍기는 on_mark로 같은 창)
 └─ RouteRvizDialog A/B RViz goal 찍기                  ← pages/ab 경로 편집
```

새 창은 `op/dialog/<이름>_dialog.py`에 `Dialog`를 상속해 위 메서드만 채우고 `dialog/__init__.py`에 넣는다.
쓰는 곳에서는 `XxxDialog(ui, ...).run()`의 결과로 다음 동작을 정한다.

### 2.3 상단 상태바 — State 칸 + StateBar

```text
State (op/state_bar/state.py)  — text()만 정함 (빈 문자열 = 그 칸 안 보임)
 ├─ ProcState(이름·라벨)  서비스(베이스·SLAM·Nav…)마다 ON/off                   (줄 1)
 ├─ RateState(속성·라벨)  odom · scan 수신 Hz — MapState가 이어받아 지도 이름 추가   (줄 2)
 ├─ GoalState             Nav2 목표 진행 (없으면 빈 칸)                           (줄 2)
 ├─ RobotState            opencr_driver 진단 — 연결·모터 에러·온도·전류           (줄 3)
 └─ DomainState           제어 · 운행 · 비상정지                                  (줄 4)
StateBar — State를 줄(row)로 받아 ' | '로 이어 그림 · add(state, row)로 칸·줄 추가
 └─ TopStateBar — 위 구성. Window.top_bar, Window.top_lines()가 그린다
```

새 칸: `op/state_bar/<이름>_state.py`에 `State`를 상속해 `text()`를 만들고 `TopStateBar` 구성에 넣거나 `ui.window.top_bar.add(XxxState(ui), row=1)`.

### 2.4 키 입력 — cbreak + 직접 읽기

`KeyReader`(op/keys.py)가 cbreak(줄 단위 X · echo X)로 키 즉시 1바이트를 읽고, 방향키(`ESC [ A`)·Tab·UTF-8을 `'up'`·`'tab'`·`'가'` 같은 이름으로 돌려줍니다.
`tcflush`(입력 버퍼 비우기)는 확인창·입력창·결과창을 열 때만 하고, 메뉴(`Window._wait_key`)는 `flush=False`입니다 (2초마다 다시 그릴 때 키가 사라지지 않게).
실행 중 `print()`는 `Ui.capture`가 가로채 결과창에 모읍니다 (`Window.perform`). 화면 그리기는 진짜 stdout(`Ui.console`)에 씁니다.

## 3. 화면 흐름 규약

키 규약은 `op/window/window.py`·`op/dialog/dialog.py` 맨 위, 흐름 규약은 `op/pages/base.py` 맨 위에 있습니다. 새 화면도 이걸 지킵니다.

- `MenuItem.run()`은 `BACK/MAIN/QUIT/STAY`만 돌려줍니다. 화면 이동은 반환값으로만. 하위 화면의 `BACK`은 "그 화면이 닫힘"이라 부모는 남습니다.
- 결정 전에 알아야 할 경고·현재값은 `print`가 아니라 `ui.confirm(body=[...])` / `ui.modal_input(body=[...])`에 넣습니다.
- 값 검사는 `modal_input(validate=...)`로 — 거부 사유 문자열을 돌려주면 창이 안 닫히고 사유를 보여 줍니다.
- 오래 걸리는 명령은 `ui.progress('무엇을 … N/M초')` · 여러 단계면 `ui.step('1/3 …')`.
- 새 메뉴: `Menu`를 상속해 `items()`(MenuItem 목록)와 `center()`(오른쪽 내용)를 만들고, 부모 메뉴 `items()`에 `GoMenu`로 넣는다. 최상위면 `root = True`.

## 4. 시험

```bash
./run-cli-test menu                  # 가짜 ROS로 TUI를 직접 써 보기
python3 cli-test/scenarios.py        # pty 키 입력 시나리오 (메뉴 · 텔레오퍼 · 입력 검증 · A/B 경로 편집 · 지도에서 찍기)
python3 cli-test/test_map_view.py    # map_view 단위 시험 (벽 우선 축소 · 중심 확대 · 로봇 따라가기 · 그리기)
# 가짜 /map = cli-test/map/map.pgm · map.yaml (isaac_maze 지도 복사본)
```

시나리오는 `Session.step(키, 기대 문자열)`로 실제 키를 넣고 화면에 나온 글자를 확인합니다. 새 화면을 만들면 시나리오를 하나 더합니다.

## 5. 지도 화면 (MapPage) — 파라미터로 쓰임이 바뀌는 화면

`pages/map_page.py` 하나가 **보기 · A/B 경로 goal 찍기 · 로봇 위치 지정**을 다 한다. 웹 페이지처럼 이름과 파라미터로 연다:

```python
self.ui.window.navigate('map', mode='view')                                   # 지금 /map 보기 (로봇 ● 표시)
self.ui.window.navigate('map', mode='view', source=map_yaml)                  # 저장된 지도 파일 보기
self.ui.window.navigate('map', mode='goals', start=…, goals=…, on_save=cb)    # A/B 경로 — 저장하면 cb(goals)
self.ui.window.navigate('map', mode='pose', on_pose=fn)                       # 위치 지정 — Enter → fn(x, y, yaw_deg)
```

- 중앙 = `MapView`(포커스 가능 Component), 왼쪽 = 같은 동작 메뉴. Tab/Esc로 오간다. 화면 한가운데 십자선(+)이 찍을 자리, 로봇은 빨간 점(●), 찍는 모드는 십자선에서 방향선이 나간다 (`[` `]` 15° · `{` `}` 1°).
- 로봇 따라가기(기본): 중심이 로봇을 따라간다. 지도가 한 화면에 다 보이면 지도는 고정되고 로봇 점이 움직인다. 방향키로 지도를 밀면 풀리고 `r`로 다시 켠다.
- 새 쓰임은 mode를 늘리지 말고, 먼저 파라미터(`on_pose` 같은 콜백)로 되는지 본다.
