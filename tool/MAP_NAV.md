# 지도 만들기 → 지도로 주행 → 특정 위치로 보내기

SLAM·Nav는 로봇(Pi)에서 돌고, 조작은 **토픽이 보이는 곳 어디서나** `./run-cli`로 합니다 (보통 화면이 있는 PC에서 RViz와 같이).
명령 전체는 [README.md](README.md), 노드 구성은 [../ros2_ws/ARCH.md](../ros2_ws/ARCH.md).

- 켜기/끄기·지도 저장은 로봇의 프로세스·파일이라, 로봇이 아닌 곳에서는 tool이 SSH로 로봇에 넘깁니다 (`[pak.local] …`). 지도는 항상 로봇 `~/ros2/ros2_ws/map/`.
- 텔레오퍼·목표·초기 위치·모드는 토픽이라 어디서나 바로.
- 아래 명령은 PC든 Pi든 `tool/` 폴더에서 그대로 실행.

## 0. 준비 (매번)

```bash
./run-cli status
#   ● 베이스 · 로봇 ok · 모드 … · odom 20Hz · scan 10Hz 이면 OK
#   ○ 베이스면:  ./run-cli base start
```

`✗`·`수신 없음`이면 [README 문제 확인](README.md#문제-확인).

## 1. 지도 만들기 (SLAM)

```bash
# ① SLAM 시작 (로봇에서 돔)
./run-cli slam start

# ② 터미널 1 — 지도 커지는 거 보기
./run-rviz slam

# ③ 터미널 2 — 천천히 돌아다니기
./run-cli teleop        # w 전진 · x 후진 · a/d 제자리 회전 · s 정지 · e/c 속도 · q 종료
```

요령:

- **천천히.** 0.10 m/s, 회전은 `a`/`d` 짧게 → `s`. 빨리 돌면 스캔 정합이 깨져 지도가 겹치거나 휩니다.
- 벽을 따라 한 바퀴 → **출발점으로 돌아오기**(루프가 닫히며 지도가 바로잡힘) → 방 가운데.
- 라이다가 1층(바닥에서 ~3 cm)이라 책상 상판 같은 높은 것은 다리만 보입니다.
- CLI 상단 `map 10.4x9.6m` · 상태 화면의 `탐색 N%`로 진행 확인.

## 2. 지도 저장

**SLAM을 끄기 전에** 저장 (끄면 만든 지도가 사라짐).

```bash
./run-cli map save office    # 이름 생략 = 날짜_시각
./run-cli map list           # 확인
./run-cli slam stop          # 저장 후 SLAM 끄기
```

- 저장 위치 (항상 로봇): `~/ros2/ros2_ws/map/office/map.yaml` + `map.pgm`
- 메뉴: `./run-cli` → `3) SLAM 맵핑` → `3) 지도 저장` / 목록·내용은 `6) 지도 파일`
- `/map 수신 없음`으로 거부되면 SLAM이 지도를 안 내는 중 → `./run-cli slam log`, 라이다 `scan` 확인.

## 3. 저장한 지도로 주행 (Nav)

```bash
# ① Nav 시작 (SLAM이 켜져 있으면 거부 → 먼저 slam stop)
./run-cli nav start office   # 이름 생략 = 가장 최근 지도

# ② 주행 화면
./run-rviz nav
```

### ③ 초기 위치 — **반드시 먼저**

Nav2는 로봇이 지도 어디에 있는지 알아야 켜집니다. 초기 위치를 주기 전에는 목표를 찍어도 **조용히 무시**됩니다.

- RViz: 위쪽 **2D Pose Estimate** → 지도에서 로봇 자리를 클릭하고 **앞쪽 방향으로 드래그**
- 또는 CLI (map 좌표, 각도는 °):

  ```bash
  ./run-cli initpose 0 0 0          # 지도 만들기 시작한 자리 = (0, 0, 0°)
  ```

  메뉴: `./run-cli` → `4) Nav` → `3) 초기 위치 (x y yaw°)`. 어디서 해도 됨 (`/initialpose` 토픽 → 로봇 amcl).

  출력이 `→ amcl 반영: x=… y=…` 이면 성공. RViz에서 초록 화살표 구름(파티클)이 로봇 주변에 모이고,
  빨간 스캔 점이 지도 벽과 겹치면 맞은 것. 어긋나면 다시 찍습니다.

초기 위치 후 Nav 주행부가 켜지기까지 로봇에서 수십 초 걸릴 수 있습니다 (`./run-cli nav log`에
`lifecycle_manager_navigation … Managed nodes are active`가 뜨면 준비 완료).

## 4. 특정 위치로 보내기

모드: Nav2 명령은 `/state_mode`가 **AUTO**일 때만 바퀴로 갑니다. RViz Nav2 Goal·`./run-cli goal`은 자동으로 AUTO로 바꾸고,
텔레오퍼·`drive`는 MANUAL로 바꿉니다 (마지막에 한 조작이 모드를 정함). 상단 상태바 `모드 …`로 확인.

### RViz에서 찍기

- **Nav2 Goal**(2D Goal Pose) → 갈 곳 클릭 + 도착했을 때 바라볼 방향으로 드래그.
- 경로(초록 선)가 그려지고 출발. 다른 곳을 찍으면 목표가 바뀝니다.

### CLI로 좌표 보내기

```bash
./run-cli goal 1.0 0.5 90           # x=1.0 m, y=0.5 m, 도착 후 90°(왼쪽)를 보게 — 도착까지 기다림
./run-cli goal 1.0 0.5 90 --no-wait # 보내고 바로 끝냄
./run-cli goal --cancel             # 이 CLI가 보낸 목표 취소 (RViz 목표는 RViz에서 다시 찍기)
```

- 결과: `도착` / `취소됨` / `실패(ABORTED)` (막혀서 경로 없음 등).
- 메뉴: `./run-cli` → `4) Nav` → `4) 목표 보내기 (x y yaw°)` · `5) 목표 취소` · `3) 초기 위치`.
- 좌표 찾기: RViz 아래쪽 상태줄에 마우스가 가리키는 map 좌표가 나옵니다. 또는 **Publish Point**로 찍으면 `/clicked_point`에 나옴.

좌표계 (`map`): 원점(0,0) = 지도 만들기를 **시작한 자리**, +x = 그때 로봇이 보던 앞쪽, +y = 왼쪽, 각도 0° = +x 방향, 반시계 +.

### 여러 곳 순서대로

```bash
for p in "1.0 0.0 0" "1.0 1.0 90" "0 0 180"; do ./run-cli goal $p || break; done   # 실패하면 멈춤
```

## 5. 지도 넓히며 주행 (SLAM 중 Nav)

지도를 아직 다 안 만든 상태에서 목표를 찍어 움직이며 지도를 넓힐 때. 초기 위치 불필요 (SLAM이 위치를 냄).

```bash
./run-cli slam start && ./run-cli explore start
./run-rviz slam                      # 2D Goal Pose로 찍기 (또는 ./run-cli goal x y yaw)
# 끝나면: map save → explore stop → slam stop
```

## 6. 끄기

```bash
./run-cli nav stop        # 또는 slam stop / explore stop
./run-cli stop-all        # 바퀴 정지 → 베이스까지 전부
```

## 7. 문제

| 증상 | 확인 |
|---|---|
| RViz에 지도가 안 보임 | 상단 `map ✗`? → `./run-cli status`에서 SLAM/Nav 켜졌는지 · RViz는 `run-rviz`로 켰는지 (DDS 피어 설정이 들어감) |
| 목표를 찍어도 안 움직임 | 초기 위치를 줬는지 · Nav 준비 완료됐는지 (`nav log`) · 텔레오퍼가 같이 켜져 있으면 끄기 (명령이 섞임) |
| `실패(ABORTED)` | 목표가 벽 안/지도 밖 · 좁은 통로(로봇 폭 31 cm + 여유) · `nav log` |
| 베이스를 재시작한 뒤 위치가 튐 | 베이스(드라이버)를 다시 켜면 **odom이 0으로 초기화**됨 → Nav면 2D Pose Estimate로 다시, SLAM이면 저장 후 `slam restart` |
| 위치가 점점 틀어짐 | RViz 2D Pose Estimate로 다시 · 바퀴 미끄러짐 · 지도가 실제와 다름(다시 만들기) |
| 지도가 겹침·휨 | 맵핑 때 너무 빠름 → 다시 천천히, 출발점으로 돌아와 루프 닫기 |

## 파일

| 파일 | 내용 |
|---|---|
| `config/slam_toolbox.yaml` | SLAM 설정 (`base_frame: base_footprint` · 지도 갱신 2초 · 최소 사거리 0.12 m) |
| `config/nav2_params.yaml` | Nav2 설정 (Waffle Pi footprint 다각형 · Twist cmd_vel · amcl `base_footprint`) |
| `rviz/slam.rviz` · `rviz/nav.rviz` | RViz 화면 |
| 로봇 `~/ros2/ros2_ws/map/<이름>/` | 저장한 지도 (`map.yaml` · `map.pgm`) |
| 로봇 `~/ros2/tool/.run/<slam·nav·…>.log` | 실행 로그 (`./run-cli slam log`) |
