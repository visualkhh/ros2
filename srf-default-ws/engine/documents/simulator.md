# Simulator: 퍽 드래그, 높이, D435 카메라

> Qt "truth" 창은 시뮬레이터의 실제 상태를 보여준다.
> 여기서 퍽을 직접 옮겨 pick/place를 시험할 수 있다.

## 1. 실행

```bash
cd /Users/hyunhakim/workspace/visualkhh/source/ros2
./srf-default-ws/run-mac.sh                 # truth 창 + 웹 Viz(:8080)
./srf-default-ws/build-mac/srf_default_ws_simulator --pucks 10 --no-noise
```

- `--pucks N`: 흩뿌리는 퍽 수 (기본 10). `0`이면 노란 graspable 퍽만 남는다.
- `--no-noise`: 센서 잡음 끔.
- `--slam`: SLAM mapping 시작.

## 2. Alt+드래그 — 바닥 x/y 이동

1. 퍽 위에 마우스를 올린다.
2. **Alt(Option)** 를 누른 채 왼쪽 버튼으로 드래그한다.
3. 커서가 지나는 바닥면으로 퍽이 따라온다.

```text
Alt + drag  →  ray를 z=퍽 높이 평면으로 역투영
            →  SimulationRunner::post(...)
            →  시뮬레이션 스레드가 world object pose 변경
```

- 월드는 **시뮬레이션 스레드만** 만진다. UI 스레드가 직접 `pose`를 바꾸면 경쟁한다.
  그래서 `SimulationRunner::post()` 큐를 거친다.
- 그리퍼가 든 퍽(`held`)과 상판 payload(`on_robot`)는 드래그하지 않는다.

## 3. Alt+Shift+드래그 — 높이 z

1. 퍽 위에서 **Alt+Shift**를 누른다.
2. 위로 드래그하면 퍽이 올라가고, 아래로 드래그하면 내려간다.
3. x/y는 그대로 유지된다.

```text
Alt + Shift + drag  →  delta.y * 0.0015 m
                    →  z = clamp(z, 0, 1.0)
```

용도:

- D435 시야에서 퍽을 들어 올리기
- pick 전에 퍽 높이 맞추기
- 상판/선반 위 물체 흉내

## 4. 왜 "다른 퍽이 없어졌어?" 

테스트 중 `--pucks 0`으로 띄우면 노란 graspable 퍽 하나만 보인다.
평소에는 `--pucks 10`(기본)이라 10개가 흩어져 있다.

```bash
./srf-default-ws/run-mac.sh                 # 기본 10개
./srf-default-ws/run-mac.sh --pucks 0       # 노란 퍽만
```

## 5. D435 카메라

`REALSENSE D435 (SIM)` 패널은 160×120 color/depth를 5Hz로 그린다.

### 자기 몸체 제외

카메라 원점은 로봇 base/deck/mount **안**에 있다. 그대로 렌더하면
자기 몸체가 화면을 가득 채운다. 그래서 `SimulatorRealSense::render()`는
카메라 원점을 품는 로봇 visual을 렌더 목록에서 뺀다.

```cpp
// 카메라 원점이 visual의 2D footprint 안이면 skip
if (camera_inside_visual(visual)) continue;
```

- 베이스/상판/마운트/카메라 하우징은 빠진다.
- 팔/그리퍼는 카메라 밖에 있으므로 그대로 남는다.
- 팔이 카메라 앞으로 들어오면 color/depth에 보인다.

### 팔이 안 보일 때

- 팔이 카메라 **뒤/위**에 접혀 있으면 안 보이는 게 정상이다.
- 웹 `Manipulator` 탭에서 joint2를 움직여 팔을 앞으로 뻗어 본다.
- `Camera` 뷰에서 color와 depth를 같이 본다. depth에 가까운 물체(노랑)가
  color에 어두운 회색으로 보일 수 있다.

## 6. pick/place 테스트 순서

```text
1. Alt+드래그로 노란 퍽을 로봇 앞 0.3m에 둔다.
2. 웹 AI 패널 "앞에 있는 노란색 퍽을 집어 올려"
3. pick/status: approach → descend → close → lift → done
4. "내려놔" → place floor
5. 다시 pick → "내 몸에 올려놔" → body place
```

- body place는 `sim/deck_free`가 상판 빈 자리를 준다.
- 퍽이 그리퍼보다 크면(12cm 대형) 집히지 않는다. 4cm 노란 퍽을 쓴다.
- 시뮬 grasp는 kinematic attach다. 마찰/미끄러짐 모델이 아니다.

## 7. 문제 해결

| 증상 | 확인 |
|---|---|
| Alt+드래그가 안 됨 | Alt(Option)를 누른 채 왼쪽 드래그, 퍽 위 26px 이내 |
| 높이가 안 바뀜 | **Alt+Shift**+드래그 (Alt만은 x/y) |
| 퍽이 사라짐 | `--pucks 0`인지, held/on_robot인지 |
| D435가 까맣다 | 카메라 패널 running, `camera/color/image_raw` Hz |
| D435에 몸체가 보임 | self-body skip 목록, 카메라 마운트 위치 |
| 팔이 안 보임 | 팔이 카메라 FOV 밖(뒤/위)인지 |
