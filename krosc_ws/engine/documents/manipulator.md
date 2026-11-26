# Manipulator: pick / place / on-body

## 1. 소스

| 파일 | 역할 |
|---|---|
| `krosc_ws/engine/nodes/pick_place_node.cpp` | nav2 접근, MoveIt 시퀀스, gripper, floor/body |
| `krosc_ws/simulator/robot/waffle_pi_arm.cpp` | TCP FK, grasp attach, on_robot payload |
| `krosc_ws/simulator/worlds/maze_world.cpp` | 노란 graspable 퍽 (r=0.02, h=0.12) |
| `krosc_ws/simulator/main.cpp` | `sim_graspable` Path 발행 |
| `krosc_ws/simulator/hardwares/realsense/simulator_realsense.cpp` | D435에 로봇 팔/그리퍼 렌더 |
| `krosc/simulator/include/krosc/simulator/world.hpp` | `graspable`, `held`, `on_robot`, `z` |
| `krosc_ws/engine/nodes/llm_input.cpp` | pick/place intent → `pick/command` |

## 2. 그리퍼와 물체

| 항목 | 값 |
|---|---|
| 그리퍼 최대 벌림 | 약 6.2 cm (finger gap 2.44 cm + 2×1.9 cm) |
| 잡는 퍽 | 지름 4 cm (r=0.02), 높이 12 cm |
| 기존 큰 퍽 | 지름 12 cm — **못 잡음**, 장애물로 남김 |
| 닫힘 명령 | `grip=8 mm` (4 cm 퍽; 0은 과폐쇄) |
| 열림 명령 | `grip=19 mm` |

`MazeWorld`가 시작 위치 `(0.28, 0.0)`에 노란 graspable 퍽을 추가한다.

```cpp
auto graspable = std::make_unique<Puck>(
    PuckShape::Circle, std::array<double,3>{0.95, 0.85, 0.15});
graspable->radius = 0.02;
graspable->height = 0.12;
graspable->graspable = true;
graspable_ = static_cast<Puck*>(add(std::move(graspable),
                                    msgs::Pose2D{0.28, 0.0, 0.0}));
```

`sim main`이 4 Hz로 `sim_graspable` Path를 발행한다:

```cpp
krosc::msgs::Path path;
path.header = {context.now(), "odom"};
pose.pose.position = {object->pose.x, object->pose.y,
                      object->z + object->height/2.0};
```

## 3. pick 흐름

`llm_input` → `pick/command` → `PickPlaceNode::worker_loop` → `pick()`.

1. `object_odom()`: `sim_graspable`에서 퍽 중심 (odom).
2. `approach()`:
   - 거리 > 0.42 m → `navigate_to_puck()`: nav2 `NavigateToPose` goal을
     퍽 앞 `0.34 m` standoff로. **map + localization 필요.**
   - 거리 ≤ 0.42 m → 로컬 전진/회전. 3초 정체 시 `approach stalled`.
3. `object_in_base()`: odom → `base_link`.
4. 작업영역 검사: `0.12 < x < 0.40`, `|y| < 0.22`.
5. `switch_arm()`: `controller_manager/switch_controller`로
   `joint_trajectory_controller` 활성화 (strictness 0; best effort).
6. MoveIt `move_to()` joint-space:
   - `approach`: `x,y, center+0.13`, pitch 20°, grip 19 mm
   - `descend`: `x,y, center+0.015`, pitch 45°
   - `close`: 같은 pose, grip 8 mm
   - `lift`: `x,y, center+0.16`, pitch 20°, grip 8 mm
7. `move_to()`는 action status뿐 아니라 **MoveGroupResult.error_code**까지 검사한다.
   `PLANNING_FAILED`/`INVALID_GOAL`인데 done으로 넘어가던 버그를 막는다.

`goal.cartesian = false` — joint-space 계획으로 카메라/차체에 막히는 직선 계획을 피한다.

## 4. place 흐름

`place`는 `target` 파라미터로 분기한다. **별도 intent가 아니다.**

| target | 의미 | 시퀀스 |
|---|---|---|
| `floor` (기본) | 바닥에 내려놓기 | lower `z=0.06` → open → retreat |
| `body` | 로봇 상판에 올려놓기 | body_approach `z=0.26` → body_lower `z=0.17` → open → retreat |

- `내려놔/바닥/다시 내려놔` → floor
- `내 몸/몸 위/상판` → body
- floor `z=0.06`은 퍽 바닥을 0으로 만든다(이전 0.10은 4cm 떠 있었음).
- body는 `sim/deck_free`가 준 빈 `(x, y)`를 쓴다. 서비스가 없으면 `(0.05, 0)` 폴백.
  `(0.05, 0)`은 팔 베이스(-0.092)에서 14cm로, `(-0.03, 0)`(6cm)은 IK 최소 도달거리 밖이었다.

### body 빈 자리 (`sim/deck_free`)

D435는 상판보다 낮고 앞에 있어서 상판을 볼 수 없다. 그래서 **로봇 모델 + payload**로
빈 자리를 계산한다. 계산은 시뮬레이션 스레드(listener)에서 하고, 서비스는 캐시만 읽는다.
(서비스 핸들러가 `world.objects()`를 직접 읽으면 시뮬레이션 스레드와 경쟁한다.)

```text
후보: x=0.05..-0.05, y=0,±0.04,±0.08,±0.10
제외: 팔 베이스에서 너무 가까움(<0.125m), LDS 박스(<0.069m),
      이미 on_robot인 payload와 겹침
선택: 첫 번째 free cell (앞쪽 x=0.05,y=0부터)
```

- payload는 여러 개를 들 수 있다. `WafflePiArm::payloads_`가 각자의 상대 pose를 들고
  로봇을 따라간다.
- body에 놓인 퍽은 `on_robot=true`가 되고 월드 충돌/라이다 레이에서 빠진다.

## 5. pick/status 타임라인

```text
switching          → joint_trajectory_controller 활성화
approach/navigating→ nav2 standoff 또는 로컬 접근
descend            → z=center+0.015, pitch 45°
close              → grip 8mm
lift               → z=center+0.16, pitch 20°
done               → holding_=true

place:
body_approach → body_lower → body_open → body_retreat → done
floor: lower → open → retreat → done
```

실패하면 `stage:"error"` + `error` 필드가 `pick/status`에 남는다.

## 6. 시뮬레이터 grasp / payload

`WafflePiArm::tcp_pose()`가 URDF 체인으로 TCP를 계산한다:

```text
base_footprint -> base_link(0.010) -> arm_base(-0.092,0,0.091)
 -> joint1(0.012,0,0.017,Rz q1) -> joint2(0,0,0.0595,Ry q2)
 -> joint3(0.024,0,0.128,Ry q3) -> joint4(0.124,0,Ry q4)
 -> end_effector(0.126,0,0)
```

`update_grasp()`:

- gripper ≤ 12 mm이고 TCP가 graspable 퍽 중심 5 cm 이내 → `held=true`.
- gripper > 15 mm로 열리면:
  - TCP가 base_link `x∈[-0.20,0.07]`, `|y|<0.14`, `z>0.11` → **on_robot payload**.
    `payload_dz=0.104`(상판), 로봇 pose를 따라 매 tick 갱신.
  - 아니면 바닥: `z = max(0, tcp.z + held_dz)`.
- `World::resolve_objects/walls/ray`는 `held`/`on_robot` 물체를 건너뛴다.

## 7. D435 카메라에 팔 보이기

`SimulatorRealSense::render()`는 원래 월드 벽/퍽만 레이캐스트했다.
로봇 자신의 `RobotState.visuals`도 같은 2.5D 레이로 넣는다.

```cpp
const krosc::simulator::RobotState robot_state = robot_.state();
for (const auto& visual : robot_state.visuals) {
  // Cylinder: ray_hit_circle, Box: ray_box_2d(yaw)
  // z = camera_z + t*tan_v 가 visual의 z 범위 안이면 hit
}
```

이제 D435 color/depth에 팔·그리퍼가 잡힌다.
VLA/VLM이 팔 상태를 카메라로 볼 수 있는 전제다.

카메라 원점은 로봇 base/deck/mount **안**에 있다. 그대로 렌더하면 자기 몸체가
화면을 가득 채운다. 그래서 원점을 품는 visual은 렌더에서 뺀다.

```cpp
if (camera_inside_visual(visual)) continue;  // base/deck/mount/housing
```

## 8. 디버깅

- `pick/status`를 AI 패널 `action` 줄에서 본다.
  `switching → approach/navigating → descend → close → lift → done/error`.
- `IK 도달 불가 (pos err ...)` → 목표가 작업영역 밖/최소 도달거리.
  body는 `x=0.05`, pick은 `x≈0.28`을 기준으로 조정.
- 팔이 안 움직이면 `controller_manager/switch_controller` strictness와
  MoveGroupResult.error_code를 본다.
- 시뮬 grasp는 kinematic attach다. 실제 마찰/미끄러짐 모델이 아니다.

## 9. 예시 (20개는 commands.md)

전체 매니퓰레이터 명령 20개는 [commands.md](commands.md#1-매니퓰레이터-20)에 있다.

```text
"앞에 있는 노란색 퍽을 집어 올려"
  → intent=pick, target=yellow
  → nav2(0.34 m standoff) 또는 로컬 접근
  → approach/descend/close/lift
  → pick/status done, holding_=true

"바닥에 내려놔" / "다시 내려놔"
  → intent=place, target=floor
  → lower z=0.06 / open / retreat

"잡은 물건을 내 몸에 올려놔"
  → intent=place, target=body
  → sim/deck_free로 빈 (x,y) 선택
  → body_lower z=0.17 / open / retreat
  → 시뮬 on_robot payload, 로봇과 함께 이동
```
