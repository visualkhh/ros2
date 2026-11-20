# kros

ROS를 파이썬으로 직접 만들며 학습하는 프로젝트. 타입 힌트 필수. **kros 코어**(Node, 토픽, 서비스, 액션)는 순수 파이썬(표준 라이브러리만)이라 라즈베리파이 같은 실제 하드웨어에서 그대로 돈다. 시뮬레이션(`worlds/`, `viewer/`)은 numpy를 쓰고(없으면 순수 파이썬으로 내려간다), GPU가 있으면 큰 계산에 GPU를 쓴다.

## 구조

```
kros/
├── kros.py            Kros: 부트스트랩. Context를 만들고 node_create 콜백에 넘겨 노드들을 받아 보관. 파라미터 시작 값 주입, 실행 중 get/set_parameter
├── context.py         Context: 노드들이 공유하는 묶음 (topic_manager, service_manager, action_manager, timer_manager, parameter_manager) + 노드 이름 중복 검사. 매니저끼리의 연결은 여기서
├── node/node.py       Node(context, name=None, parameters=None): publish/subscribe, 서비스, 액션, 타이머, 파라미터 (모두 타입 지정). 이름 기본값은 클래스 이름
├── msgs/              메시지 타입(순수 데이터). geometry_msgs: Vector3, Twist, Pose2D / sensor_msgs: LaserScan / std_msgs: Header / diagnostic_msgs: DiagnosticStatus, DiagnosticArray ... (ROS의 msg 흉내)
├── timer/             TimerManager(타이머 스케줄), Timer(손잡이), TimeSource(SystemTimeSource 실제 시간 / SimulatedTimeSource 시뮬레이션 시간)
├── diagnostics/       DiagnosticUpdater(노드가 주기적으로 상태를 diagnostics 토픽에 냄), DiagnosticAggregator(모아서 STALE 판단), DiagnosticStatusWrapper
├── parameter/         ParameterManager(노드별 파라미터 저장소), Parameter[T](현재 값 손잡이)
├── action/            ActionManager(서버 레지스트리, 서버/클라이언트 수명), ActionServer/ActionClient/ActionType(.action 흉내). 서비스 2개 + 토픽 2개 위에 구현
├── service/           ServiceManager(이름->서버 레지스트리), ServiceServer, ServiceType(.srv 흉내)
├── topic/             TopicManager(토픽별 구독자 목록, 메시지 전달), Subscription(구독 하나: 전용 스레드+큐), QoS(depth, durability) (한 프로세스 안)
├── interactions/      [현실] 사람(조종자)이 넣는 입력 장치. interaction.py: Interaction 베이스, keyboard.py: Keyboard(KeyState). 월드 바깥의 것이고 로봇이 receive()로 받는다. kros와 무관
├── worlds/            [현실] 외부 환경 + 그 안의 객체들. kros와 무관. world.py: World(환경, 시계), world_object.py: WorldObject, clock.py: Clock,
│   │                  simulation_runner.py: SimulationRunner(월드를 화면과 따로 고정 간격으로 돌린다. 월드만 알고 로봇은 모른다)
│   ├── collision.py   충돌/레이 도형 Circle, Box와 계산 (ray_hit, circle_vs)
│   ├── maze/          MazeWorld(World): isaac rl_maze 트랙(track.py의 벽 데이터) -> Wall 20개 + 시작할 때 퍽 10개를 흩뿌림
│   ├── objects/       로봇이 아닌 WorldObject들. wall.py: Wall(static 상자), puck.py: Puck(모양 PuckShape, 색)
│   ├── hardwares/     부품 하드웨어. hardware.py: Hardware/Sensor/Actuator(최상위), lidar.py/motor.py: 종류별 인터페이스,
│   │                  lds_01.py(LDS-01 라이다), xl430_w250.py(XL430-W250 모터 한 개): 모델 번호별 구현 (시뮬레이션/더미, 진짜도 여기에)
│   └── robots/        로봇들. robot.py: Robot(Hardware, WorldObject) 베이스
│       └── turtlebot3/  TurtleBot3(Robot): diff_drive.py(DiffDrive: XL430-W250 바퀴 모터 둘을 묶은 구동부) + LDS-01 라이다를 달고, 바퀴 치수와 라이다 장착 위치는 이 로봇이 정한다. description/ (ROBOTIS URDF·STL 메시)
├── viewer/            [현실] 3D 화면: Panda3D(GPU) 창(panda_viewer.py: Viewer)과 URDF/STL 로더(urdf_model.py, stl.py, shapes.py). 스텝마다 그 순간을 불변 데이터 Snapshot(snapshot.py)으로 만드는 SnapshotRecorder(snapshot_recorder.py: 월드와 로봇을 아는 곳)가 러너의 리스너로 달리고, Viewer는 그 최신 스냅샷만 그린다. snapshot/recorder는 panda3d 없이 돈다. kros와 무관
├── examples/turtlebot3/  [kros 쪽] 예제: kros 노드 RobotBridge로 위 현실 쪽과 토픽(cmd_vel, scan)을 잇는다 (main.py, run.sh)
├── tests/             unittest
├── setting-venv.sh    .venv 생성
└── test.sh            테스트 실행 (.venv 없으면 자동 생성)
```

## 사용

```python
from kros import Kros
from kros.node import Node

kros = Kros(lambda ctx: [Node(ctx, "pub"), Node(ctx, "sub")])
pub, sub = kros.nodes
sub.subscribe("chat", str, print)   # 받을 타입을 선언
pub.publish("chat", "hello")   # -> hello
```

- Node는 `Context`를 반드시 받는다 (`Node(context, name=None, parameters=None)`: 이름은 기본이 클래스 이름이고, 같은 Context에서 겹치면 `ValueError`라서 같은 클래스를 여럿 만들 때는 `name=`을 준다). 직접 만들지 않고 Kros가 주입한다 (Spring식 조합). `ctx.topic_manager`, `ctx.service_manager`.
- 구독자 없는 토픽에 publish해도 에러 없음.
- publish는 즉시 반환. 구독마다 전용 스레드+큐에서 콜백이 동시에 실행되고(구독 안에서는 순서 보장), 콜백 예외는 로그만 남기고 다른 구독자에 영향 없음.
- `subscribe`는 `Subscription`을 돌려주고 `unsubscribe(sub)`로 해제. `join()`은 전달 완료 대기, `Kros.shutdown()`은 정리 후 종료.
- 토픽은 처음 쓰인 타입으로 고정된다. 다른 타입으로 publish/subscribe하면 `TypeError`.
- 서비스: `node.create_service(이름, ServiceType, handler)` / `node.call_service(이름, ServiceType, request, timeout=)` / `call_service_async` (Future). 서비스 이름당 서버 1개(중복 `ValueError`), 서버 없으면 `LookupError`, 타입 다르면 `TypeError`, 핸들러 예외는 호출자에게 전달. 서버마다 스레드 1개라 요청은 순서대로 하나씩 처리. 자기 서비스를 블로킹 호출하면 데드락(timeout 권장).
- 액션: `node.create_action_server(이름, ActionType, execute, goal_callback=None)` / `node.create_action_client(이름, ActionType)` (모두 `Context.action_manager`를 거친다. 내릴 때는 `node.destroy_action_server(서버)` / `node.destroy_action_client(클라이언트)`. 액션 이름당 서버는 하나, 중복은 `ValueError`. 서버를 내리면 실행 중인 goal은 취소 요청을 받고 시작도 못 한 goal은 CANCELED로 끝나며, 클라이언트를 내리면 결과를 기다리던 핸들은 ABORTED로 풀린다. 그래서 영원히 기다리는 일이 없다) -> `client.send_goal(goal, feedback_callback, timeout)` -> `ClientGoalHandle`(`result()`, `cancel()`, `status`). execute는 `handle.publish_feedback()`으로 진행을 알리고 `handle.is_cancel_requested`로 취소에 응한다. 정상 반환 SUCCEEDED(취소 요청 중이었으면 CANCELED), 예외/잘못된 결과 타입 ABORTED, goal_callback이 False면 REJECTED. goal은 받은 순서대로 하나씩 실행. feedback과 result는 같은 토픽(순서 보장)으로 간다.
- Kros 콜백은 `Context` 하나를 받는다. `Kros.shutdown()`은 `ctx.shutdown()`: 타이머, 액션, 서비스, 토픽 순서로 정리한다 (타이머 콜백은 토픽/서비스/액션을 부를 수 있고, 액션은 서비스와 토픽 위에서 돌고, 서비스 핸들러는 publish할 수 있으므로). 이때 결과를 기다리던 액션 클라이언트의 핸들은 ABORTED로 풀린다.

## QoS (서비스 품질)

토픽의 `publish`/`subscribe`는 `qos`를 받는다 (`QoS(depth, durability)`, 기본 `QOS_DEFAULT`: 무제한 큐, 휘발성이라 이전과 똑같이 동작).

```python
node.publish("tf_static", msg, QOS_LATCHED)              # 마지막 메시지를 기억한다 (latch)
node.subscribe("tf_static", Msg, callback, QOS_LATCHED)  # 늦게 구독해도 기억된 마지막 메시지를 먼저 받는다
node.subscribe("scan", LaserScan, callback, QOS_SENSOR_DATA)  # 느리면 오래된 것을 버리고 최신을 본다 (depth=5)
```

- **`durability`**: `VOLATILE`(기본)은 지금 구독 중인 쪽에만 전달한다. `TRANSIENT_LOCAL`로 **발행하면 마지막 `depth`개를 기억**하고, `TRANSIENT_LOCAL`로 **나중에 구독한 쪽**에게 새 메시지보다 먼저 전달한다. 발행 쪽과 구독 쪽이 **둘 다** `TRANSIENT_LOCAL`이어야 받는다 (ROS와 같은 규칙). 한 번만 발행하는 정보(`tf_static`, `robot_description`, 지도)를 늦게 뜬 노드도 받게 하는 용도다. `TRANSIENT_LOCAL`은 `depth`가 필요하다(무제한으로 쌓이지 않게).
- **`depth`**: 구독에서는 처리를 기다리는 메시지의 최대 개수다. 넘치면 가장 오래된(아직 실행 전인) 것을 버리고 `Subscription.dropped`로 센다 (사실만 노출한다). 버려도 `join()`은 안 막힌다. 기본 `None`은 무제한(아무것도 안 버림). 느린 구독자 때문에 큐가 무한히 쌓이는 것을 막는다.
- **함정 하나**: `QOS_LATCHED`(`depth=1`)를 **구독에** 쓰면 처리 대기 큐도 1개라서, 메시지가 연달아 오면 오래된 것은 버려지고 최신만 남는다 (ROS의 `keep_last(1)`과 같다). 하나도 놓치면 안 되면 구독에는 더 깊은 `depth`를 준다 (`QoS(depth=10, durability=TRANSIENT_LOCAL)`).
- 지금 없는 것: `reliability`(재전송 정책), `deadline`, `lifespan`, `liveliness`. 한 프로세스 안에서는 전달이 항상 신뢰할 수 있어서 필요해지는 때는 네트워크를 붙일 때다.

## 진단 (diagnostics)

ROS의 `diagnostics`와 같은 구조다. 하드웨어 드라이버가 "나는 이렇다"는 **사실**을 올리고, 심각한지는 **보는 쪽이 판단**한다.

- **메시지**: `DiagnosticStatus(level, name, message, hardware_id, values)`와 이를 묶은 `DiagnosticArray(header, status)`가 `diagnostics` 토픽으로 나온다. `level`은 `OK < WARN < ERROR < STALE`(숫자가 클수록 나쁨, STALE은 "소식이 없어 상태를 모른다").
- **`DiagnosticUpdater(node, period, hardware_id)`**: `add(이름, task)`로 진단 작업을 등록하면 `period`마다 모든 작업을 돌려 한 `DiagnosticArray`로 발행한다. 작업은 `DiagnosticStatusWrapper`에 `summary(level, 메시지)`와 `add(키, 값)`을 적는다(안 적으면 OK). 진단 이름은 `"노드이름: 작업이름"`. 작업이 예외를 내면 그 진단만 ERROR가 되고 나머지는 그대로 나간다. `set_period`로 실행 중 주기를 바꿀 수 있다.
- **`DiagnosticAggregator`(노드)**: `diagnostics`를 구독해 이름별 최신 진단을 모으고, `stale_timeout`(파라미터, 기본 5초, 실행 중 변경 가능) 넘게 소식이 없는 것은 `STALE`로 본다. `statuses()`, `worst_level()`(아직 아무 소식도 없으면 OK가 아니라 STALE: 모르는 것이지 정상이 아니다). 시각은 kros 시간(시뮬레이션이면 시뮬레이션 시간)이다.
- **하드웨어와의 연결(예제)**: 하드웨어(`worlds/`)는 kros를 모르므로 진단도 모른다. `examples/turtlebot3`의 `RobotBridge`가 하드웨어 트리의 센서/구동기마다 진단 작업을 등록하고(`hardware_status.py`), **하드웨어가 남긴 사실을 수준으로 바꾸는 판단**을 여기서 한다: 전원이 꺼져 있으면 WARN, 센서가 한 번도 값을 못 냈으면 WARN, 최신 값이 `data_timeout`(파라미터, 기본 2초)보다 오래됐으면 ERROR, 지난 진단 이후 새 실패가 있으면 WARN, 그 밖에는 OK. 문제가 여럿이면 수준은 가장 나쁜 것을 따르고 설명에는 모두 적는다(가장 나쁜 것이 맨 앞). `values`에는 모델(`hardware_id`), 전원, 주기, 실패 횟수, 마지막 실패, 데이터 나이가 들어간다. 진단 이름은 로봇부터 이은 경로다(`turtlebot3_waffle_pi/drive/left_wheel`).
- `diagnostics_period`(진단 발행 주기)도 `RobotBridge`의 파라미터라 실행 중에 바꿀 수 있다.

## 시간과 타이머

- **시간 원천(`TimeSource`)**: `Kros(node_create, time_source=...)`로 정한다. 기본은 `SystemTimeSource`(실제 시간)이고, 시뮬레이션이면 `SimulatedTimeSource`를 주고 시뮬레이션이 `set(t)`/`advance(dt)`로 밀어 준다 (ROS의 `use_sim_time`/`/clock`과 같은 역할; 프로세스를 나누게 되면 이 자리가 토픽이 된다). 시간이 되돌아가면 `ValueError`. `node.now()`가 현재 시각(초)이다.
- **타이머**: `node.create_timer(period, callback)` -> `Timer`, `node.destroy_timer(timer)`/`timer.cancel()`. 첫 호출은 만든 뒤 `period`가 지났을 때다. 콜백은 **그 타이머 전용 워커 스레드 하나**에서 순서대로 돌고(구독과 같은 결), 예외는 로그만 남기고 타이머는 계속 돈다.
- **사실만 노출한다**: `timer.calls`(실제로 불린 횟수), `timer.missed`(주기가 됐지만 부르지 못한 횟수). 지난 콜백이 아직 실행 중이면 새 호출은 쌓지 않고 버리며 `missed`로 센다. 시간이 한꺼번에 건너뛰면(예: 시뮬레이션 `dt`가 큼) 한 번만 부르고 나머지는 `missed`로 세되, 박자는 유지한다.
- **동작 방식**: 실제 시간이면 스케줄 스레드 하나가 가장 가까운 주기까지 기다렸다 깨운다(타이머가 없으면 스레드도 없다). 시뮬레이션 시간이면 시간을 `set`하는 쪽이 **그 스레드에서 바로** 타이머를 판정하므로 스레드 없이 결정적으로 동작한다.
- 이 예제(`examples/turtlebot3`)에서는 `SimulationRunner.add_listener`로 매 스텝 `world.time`을 `SimulatedTimeSource`에 밀어 준다. 러너는 월드만 알고, 스텝마다 월드를 읽어야 하는 쪽(스냅샷 기록기, kros 시간)이 리스너로 달린다.

## 파라미터

노드가 파라미터를 선언해서 가진다 (ROS2식). 처리는 `Context.parameter_manager`가 하고, `Kros`는 시작 값을 넘기기만 한다.

```python
class Driver(Node):
    def __init__(self, context: Context) -> None:
        super().__init__(context)   # 노드 이름은 "Driver"
        self.speed = self.declare_parameter("speed", 0.26, validator=lambda v: v > 0, on_change=self.on_speed)

    def command(self) -> float:
        return self.speed.value * 2   # 쓸 때마다 .value를 읽는다: 바뀐 값이 바로 반영된다

kros = Kros(lambda ctx: [Driver(ctx)], parameters={"Driver": {"speed": 0.3}})   # 시작 값
kros.set_parameter("Driver", "speed", 0.1)    # 실행 중에 바꾼다 (노드 객체나 이름). 노드 안에서도 node.set_parameter(...) / node.speed.set(...)
```

- **우선순위**(아래가 낮고 위가 이긴다): 선언한 기본값 < `Node(parameters=...)`(그 인스턴스의 값) < `Kros(parameters=...)`(실행할 때 바깥에서 주는 값) < 실행 중 `set`.
- **타입은 기본값으로 고정**된다(`bool`, `int`, `float`, `str`, 그 튜플). 다른 타입으로 set하면 `TypeError`(int는 float이 아니고 bool은 int가 아니다). 시작 값이 타입이나 검증에 안 맞으면 선언이 실패한다.
- **검증과 반응**: `validator(새 값) -> bool`이 False면 `ParameterRejectedError`(값은 그대로), 바뀐 뒤에는 `on_change(새 값)`이 불린다. 선언하지 않은 이름은 `ParameterNotDeclaredError`, 두 번 선언하면 `ParameterAlreadyDeclaredError`.
- **동적 반영**: `Parameter.value`는 호출할 때마다 현재 값을 읽는다(락으로 스레드 안전). 다른 스레드에서 바꾼 값을 구독 콜백이 바로 읽는다. 단 `x = param.value`로 변수에 복사해 두면 따라 바뀌지 않는다. 값에서 파생된 상태를 다시 만들어야 하면 `on_change`를 쓴다.
- **변화 알림 (`parameter_events` 토픽)**: 선언하거나 값이 바뀌면 `ParameterEvent(node, name, kind, old, new)`가 `parameter_events` 토픽(`PARAMETER_EVENTS_TOPIC_NAME`) 하나로 나온다(ROS2의 `/parameter_events`와 같은 구조, 단 이전 값 `old`도 담는다). 다른 노드가 구독해서 그 노드의 변화를 알 수 있고, `Kros.set_parameter`처럼 노드를 거치지 않는 변경도 알린다. 거절됐거나 타입이 틀린 변경, 같은 값으로의 set은 알리지 않는다.
- **교통정리는 `Context`가 한다**: 매니저(`topic`, `service`, `parameter`)끼리는 서로를 import하지 않는다(테스트가 검사). `ParameterManager`는 변화를 콜백으로 알리기만 하고, 그 콜백을 토픽 발행에 연결하는 건 `Context`다. 새 기능이 매니저끼리 엮여야 하면 매니저 안이 아니라 `Context`에서 연결한다.
- **오타 점검**: 누가 값을 줬는데 선언된 파라미터가 없으면(`Kros(parameters={"Driver": {"spede": 1}})`) 경고 로그를 남긴다.
- 파라미터는 kros 노드의 개념이다. 현실 쪽(`worlds/` 등)의 `Robot`, `World`, 하드웨어는 생성자 인자로 값을 받는다.

## 실행

저장소 루트(`ros2/`)에서:

```bash
python3 -m kros
```

## venv

```bash
kros/setting-venv.sh            # 이 기기에 맞는 프로필을 자동으로 골라 설치 (auto)
kros/setting-venv.sh core       # 직접 고르기: core | sim | gpu
kros/setting-venv.sh --detect   # 자동 감지 결과만 보기
source kros/.venv/bin/activate
```

| 프로필 | 설치하는 것 | auto가 고르는 때 |
|---|---|---|
| `core` | 없음 (kros 코어만) | 라즈베리파이 (`/proc/device-tree/model`에 raspberry) |
| `sim` | + Panda3D, numpy (`examples/turtlebot3/requirements.txt`) | 그 밖의 기기 |
| `gpu` | + cupy (`examples/turtlebot3/requirements-gpu.txt`) | `nvidia-smi`가 CUDA 13 드라이버를 보여 줄 때 |

환경변수 `KROS_PROFILE`로도 고정할 수 있다. 같은 명령을 다시 돌리면 갱신된다. `test.sh`와 `run.sh`는 venv가 없으면 이 스크립트를 부른다(`run.sh`는 시뮬레이터라 최소 `sim`). 설치 후 cupy가 GPU를 못 쓰면 경고만 하고 CPU로 동작한다. `core` 프로필에서도 전체 테스트가 돈다(numpy/Panda3D가 필요한 9개는 건너뜀). CUDA 13이 아닌 NVIDIA 환경(예: 12)의 cupy 설치는 지원하지 않아서 `sim`으로 설치된다.

## 테스트

```bash
kros/test.sh                          # 전부 + 타입 검사(mypy --strict, mypy.ini)
kros/test.sh -v                       # 이름까지 자세히
kros/test.sh -k pub_sub               # 이름에 pub_sub 들어간 테스트
kros/test.sh -v -k TestKros           # 클래스
kros/test.sh -v -k TestKros.test_pub  # 클래스.메서드
kros/test.sh -v -p test_kros.py       # 파일 하나
```

인자를 주면(특정 테스트만) 타입 검사는 건너뛴다. `mypy`는 `sim`/`gpu` 프로필 venv에 설치된다(`requirements-dev.txt`). 코어(`node`/`topic`/...)와 현실 쪽 모두 `--strict`를 통과한다(panda3d/numpy/cupy는 타입 정보가 없어 설정에서 제외).


`-k`는 전체 이름(`kros.tests.test_kros.TestKros.test_...`)에서 부분 문자열로 찾고 대소문자를 구분한다. 여러 개는 `-k a -k b`(OR). `Ran 0 tests`면 패턴 확인.

## 설계 메모

- 중앙 매니저(ROS1 master 방식)부터 시작. 이후 소켓으로 프로세스 분리 -> 분산 디스커버리(ROS2/DDS 방식) 순으로 확장 예정.
- 메시지는 dataclass(ROS의 msg 흉내). `subscribe(topic, 타입, callback)`으로 받을 타입을 선언하고, publish는 값의 타입을 쓴다.

## 현실 쪽과 kros 쪽

두 층은 완전히 분리된다.

- **현실 쪽 (`worlds/`, `interactions/`, `viewer/`)**: 진짜 하드웨어와 세계를 시뮬레이션할 뿐이다. kros의 `Node`, `Context`, 토픽을 전혀 모르고, 바깥과는 **순수 데이터**(`Twist` 명령, `LaserScan` 센서 값)로만 주고받는다. (`tests/test_maze_world.py`가 이를 검사한다.)
  - **`World`**는 `WorldObject`들(벽, 로봇, 퍽)이 놓이는 공간이고 시계를 쥔다. 객체가 무엇인지(벽, 로봇, 센서, 명령)는 모른다. `world.add(obj)`로 놓고(기본 `start_pose`), `world.step(dt)`가 시계를 진행시켜 모든 객체에 `Clock`(시각, dt) 신호를 주고 충돌을 푼다. 두 가지 질문에 답한다: `ray_distance(x, y, angle, height)`(그 방향으로 쏜 선이 처음 닿는 객체까지 거리), `push_out(x, y, radius)`(원이 벽을 파고들었으면 밀어낸 위치). `MazeWorld`는 isaac `rl_maze` 트랙의 벽 데이터(`track.py`)로 `Wall` 객체 20개를 만든다.
  - **`WorldObject`**는 월드 안에 놓이는 모든 것의 최상위다: `pose`, `tick(clock, world)`(시계 신호를 받으면 스스로 움직인다), `collider`(충돌 도형: `Circle` 또는 `Box`), 그리고 물리 속성 `radius`, `height`(m), `mass`(kg), `friction`(이 객체 재질의 마찰 계수), `static`(월드 지형: 절대 안 움직임), `movable`(월드 물리가 움직이는가), `velocity`(미끄러지는 속도). `Wall`은 `static`(질량 무한)이고 상자, `Puck`은 `movable`이고 원, `Robot`은 둘 다 아니다(모터가 스스로 움직임).
  - **월드의 성질**: `gravity`(중력가속도, 기본 9.81, 달이면 1.62), `ground_friction`(바닥 재질의 마찰 계수, 기본 0.3, 얼음이면 0.05). `world.weight(obj)` = 질량 x 중력(같은 질량도 월드가 다르면 무게가 다르다). 객체와 바닥 사이의 실제 마찰은 두 계수를 기하평균으로 합친다: `world.friction_with_ground(obj)` = sqrt(객체 계수 x 바닥 계수). 중력만으로는 멈추지 않는다: 중력은 누르는 힘(무게)을 정하고 감속은 마찰 계수 x 중력이다.
  - **물리(월드가 `step`에서)**: 객체끼리 겹치면 밀어낸다. `movable`이 아닌 객체(로봇)는 안 밀리고, `movable`끼리는 질량이 작은 쪽이 더 밀린다. 밀린 객체는 실제로 움직인 만큼 속도를 얻어 미끄러지다가 마찰(합친 마찰 계수 x 월드 중력)로 감속해 멈춘다(정지거리 = v²/(2μg)). 벽(static)에는 박히지 않고(이동을 2cm 이하로 쪼개 벽을 건너뛰지 않음), 퍽이 벽에 눌려 못 물러나면 로봇이 대신 물러난다. 레이(`ray_distance`)는 객체의 `collider`와 만나되, 바닥에서 `height` 높이로 수평하게 쏘므로 그보다 낮은 객체(`WorldObject.height`)는 위로 지나간다. 라이다는 바닥에서 0.132m 높이라 높이 4cm 퍽은 못 보고, 벽(0.25m)은 본다. 출발점이 도형 안이면 그 객체는 무시한다. 퍽의 충돌은 원으로 근사한다.
  - **`Hardware`**는 물리 장치의 최상위(모터, 센서, 로봇)다. 부품(`parts`)을 `add`로 달 수 있고(부품은 달린 곳을 `owner`로 안다), 시계 신호 `tick(clock, world)`를 받으면 부품에게 내려준다.
    - **주기와 훅**: 장치마다 `period`(초, `None`이면 매 시계 신호)를 따로 설정하고, 주기가 되면 장치 안에서 **훅**(`on_period`)이 불린다. 데이터를 시뮬레이션으로 만들든 더미로 보내든 진짜 장치에서 읽든 그 훅을 구현한 하위 클래스가 정한다. 첫 신호에 바로 한 번 일하고 이후 주기마다 일한다(`clock.dt`는 지난 호출 이후 지난 시간).
    - **`Sensor[T]`**(데이터를 내보냄): `read()`(최신값), `subscribe(콜백)`(새 값이 나올 때마다), `stamp`(최신값의 시각). 새 값은 `sample()` 훅이 만든다. **`Actuator[T]`**(명령을 받음): `write(명령)`, `command`(마지막 명령). 명령은 `apply()` 훅이 실행한다.
    - **종류 -> 모델 -> 구현**: `Lidar`/`Motor`는 종류별 인터페이스(`hardwares/lidar.py`, `motor.py`: 회전 모터 한 개, 목표 속도 rad/s를 받고 `position`/`velocity`를 읽음)이고, 모델 번호 파일이 구현을 가진다: `hardwares/lds_01.py`(`Lds01` 사양: 360샘플, 0.12~3.5m, 5Hz / `SimulatedLds01`: 월드에 레이를 쏨, 기본 / `DummyLds01`: 월드와 상관없이 고정 스캔), `hardwares/xl430_w250.py`(`SimulatedXl430W250`: 모터 한 개, 목표 속도로 돌며 위치를 누적, `max_velocity`로 제한 가능. 바퀴에도 매니퓰레이터 관절에도 쓸 수 있다). 진짜 장치는 같은 인터페이스를 상속해 `sample`/`apply`에서 장치와 통신하는 구현을 모델 번호 파일로 추가하면 된다. `TurtleBot3(left_motor=..., right_motor=..., lidar=...)`로 끼운다. 어디에 달렸는지(`mount_x`, `mount_height`)는 부품이 아니라 로봇이 정해 넘긴다(라이다 사양은 모델, 장착 위치는 로봇).
    - **전원과 실패 (진짜 하드웨어처럼)**: `power_on()`/`power_off()`는 실제 명령이다(부모가 꺼지면 부품도 꺼진다. 꺼진 동안엔 일하지 않고, 켜면 꺼져 있던 시간이 한꺼번에 흘러오지 않는다). 하드웨어는 "고장 상태"를 알려 주지 않으므로 on/off/error 같은 상태 머신은 없다. 실패는 데이터에 나타난다: 훅이 `HardwareError`를 일으키면 `error_count`, `last_error`에 기록하고 새 값을 만들지 않는다(구독자는 안 불리고 `latest`는 옛 값 그대로, `stamp`는 그대로). 오래됐는지(`world.time - stamp`), 고장인지는 **쓰는 쪽이 판단한다**. 라이다의 `inf`(범위 밖) 같은 값도 그대로 데이터다. 시험용으로 `sensor.fault = lambda clock: ...`로 고장을 넣을 수 있다.
  - **`Interaction[T]`**(`interactions/`)는 사람이 넣는 입력 장치(키보드, 게임패드 ...)의 베이스다. 입력을 상태(`state`)로 들고, `subscribe(콜백)`으로 알리고, 입력을 넣는 쪽(3D 창, 진짜 장치 드라이버)이 `feed(상태)`를 부른다. `Keyboard`는 눌린 키 집합(`KeyState`)이 **바뀔 때만** 내보낸다. 입력을 무슨 뜻으로 해석할지는 받는 쪽(로봇)이 정한다.
  - **`Robot(Hardware, WorldObject)`**는 부품을 가진 하드웨어이면서 월드에 놓이는 객체다: `command`(`Twist`, 바깥에서 넣는다). 월드의 시계 신호가 `Robot.tick` -> 부품 순으로 내려간다. `receive(interaction)`으로 입력 장치를 받으면 입력이 올 때마다 `on_interaction(interaction, state)` 훅이 불리고(기본은 무시), `TurtleBot3`는 `KeyState`를 직접 해석해 `command`를 낸다(w/s 전후진, a/d 회전, 반대 키 상쇄, q/z 속도 배율). `TurtleBot3`는 `Motor`(`motor.py`, 2륜 차동구동, 벽은 `world.push_out`이 밀어냄)와 `Lidar`(`lidar.py`, LDS-01, 5Hz)를 조립한다. 센서 값은 `lidar.subscribe(콜백)`으로 바깥에 나온다.
- **kros 쪽 (`examples/turtlebot3/`)**: kros 노드는 `RobotBridge` 하나다: `cmd_vel` 토픽 -> `robot.command`(다른 프로그램이 로봇을 움직일 때), `lidar.subscribe` -> `scan` 토픽. 실제 로봇의 하드웨어 드라이버 자리에 해당하는 유일한 접착제다. 사람의 조종은 kros를 거치지 않고 로봇이 직접 받는다(아래).
- 새 로봇은 `Robot`, 새 환경은 `World`, 그 밖의 객체(장애물 등)는 `WorldObject`를 상속해 `worlds/`에 추가한다. 화면 조작 키(`c`, `f`, `l`, 마우스)는 `Viewer`가 처리하고, 조종 키는 `Keyboard`로 넘어간다.

## 계산 백엔드 (GPU -> CPU -> 순수 파이썬)

라이다처럼 독립적인 계산이 많은 부분(`World.ray_distances`: 레이 x 도형)은 `worlds/compute.py`가 환경에 맞는 계산 방법을 고른다.

| 백엔드 | 조건 | 비고 |
|---|---|---|
| GPU (cupy) | NVIDIA GPU + CUDA 드라이버 + `pip install -r examples/turtlebot3/requirements-gpu.txt`, 그리고 작업량(레이 수 x 도형 수)이 5만 이상 | 이보다 작으면 CPU가 더 빨라서 안 쓴다 |
| CPU (numpy) | numpy 설치됨 | 기본 |
| 순수 파이썬 | 둘 다 없음 (라즈베리파이 등) | 느리지만 결과는 같다 |

- 실패하면 조용히 내려간다: GPU 계산이 예외를 내면 경고 로그를 한 번 남기고 이후 CPU로 계산한다. cupy가 import는 돼도 GPU가 없으면 쓰지 않는다.
- 환경변수 `KROS_COMPUTE=auto|gpu|cpu|python`으로 고정할 수 있다 (기본 `auto`). 시작할 때 `compute: ...`가 찍힌다.
- 이 PC(RTX 5060)에서 측정(레이 수, 도형 31개): 360개 numpy 0.22ms / GPU 0.60ms(CPU 승), 3600개 1.10 / 0.61ms, 36000개 24.5 / 1.2ms, 360000개 317 / 14.5ms. 지금 라이다(360개)는 CPU가 더 빨라서 GPU를 쓰지 않는다. 순수 파이썬은 360개에 약 6.8ms.
- 세 백엔드가 같은 결과를 내는지 `tests/test_compute.py`, `tests/test_maze_world.py`(`TestBatchRays`)가 확인한다.
- 렌더링(Panda3D)의 GPU는 이와 별개다.

## 예제: TurtleBot3 + MazeWorld

```bash
kros/examples/turtlebot3/run.sh
```

`run.sh`가 venv와 Panda3D(`requirements.txt`)를 준비하고 실행한다. `python3 -m kros.examples`(ros2/ 에서)로도 되고, IDE에서 `examples/__main__.py`를 직접 실행해도 된다. NVIDIA 드라이버가 있으면 NVIDIA GPU로 렌더링하도록(PRIME offload) 환경변수를 설정하고, 시작할 때 `GPU renderer: ...`가 찍힌다.

**조작**

| 키/마우스 | 동작 |
|---|---|
| `w/s`, `↑/↓` | 전진/후진 |
| `a/d`, `←/→` | 좌/우 회전 (반대 키는 서로 상쇄) |
| `q` / `z` | 속도 배율 올리기/내리기 (×1.25, 0.25~4배, 누를 때마다 한 번) |
| 왼쪽 드래그 | 카메라 회전 |
| 오른쪽 드래그 | 화면 이동 |
| 휠 | 확대/축소 |
| `f` | 로봇 추적 켜기/끄기 (끄면 지금 시점 그대로 고정) |
| `c` | 트랙 전체 보기 (추적 꺼짐) |
| `l` | 라이다 점 표시 켜기/끄기 (처음엔 꺼짐) |
| `Esc` | 종료 |

**데이터 흐름**: 3D 창이 받은 키 -> `Keyboard.set_keys` -> (입력이 바뀌면) `TurtleBot3.on_interaction`(키 해석) -> `robot.command`(`Twist`) -> `DiffDrive`가 바퀴 모터 속도로 바꿈. 다른 프로그램이 `cmd_vel` 토픽에 `Twist`를 publish하면 `RobotBridge`가 같은 `robot.command`에 넣는다(마지막 명령이 이긴다). 월드는 `SimulationRunner` 스레드에서 따로 돈다: `world.step`이 시계 신호를 `TurtleBot3.tick`으로 내려 `DiffDrive`(바퀴 모터가 돌고 몸체가 움직임)와 라이다가 일하고, 라이다 측정(시뮬 시간 5Hz, 1도 간격 360개 레이, 범위 0.12~3.5m, 밖은 `inf`)은 `lidar.subscribe` -> `RobotBridge` -> `scan`(`LaserScan`). 화면은 `runner.latest`(`Snapshot`)만 읽는다.

**월드**: isaac `rl_maze` 트랙(바깥 4.48x6.37m, 벽 두께 5cm)이다. 벽 선분은 `worlds/maze/track.py`의 `WALLS`에 내장했고(`MazeWorld`가 `Wall` 상자로 바꾼다) 원본 격자는 `worlds/maze/track_source/grid.npz`에 복사해 뒀다(출처: 그 폴더의 `README.md`). 라이다 레이는 벽을 두께 있는 상자로 보고 앞면까지 거리를 구하고(`ray_distance`), 로봇은 반지름 원으로 벽에 밀려난다(`push_out`, 로봇의 이동은 5cm 단위로 쪼개 벽을 건너뛰지 않음). 벽은 축에 평행하다고 가정한다.

**화면**: Panda3D(GPU, 조명/그림자 셰이더). 로봇은 ROBOTIS TurtleBot3 waffle pi URDF/메시(`worlds/robots/turtlebot3/description/`, 출처는 그 안의 `README.md`)를 `viewer/urdf_model.py`가 읽어 만들고(STL은 `viewer/stl.py`), 바닥/벽은 `viewer/shapes.py`가 코드로 만든다. 바퀴가 속도에 맞춰 돈다. 라이다 점은 빨간 점(그림자 없음)이고 HUD에 위치, 속도, 가장 가까운 거리(`nearest`)가 나온다.

**테스트**: 3D 창 없이 `tests/test_maze_world.py`(월드 질문, 시계 신호, 월드가 로봇/센서를 모르는지, 현실 쪽이 kros와 무관한지), `tests/test_turtlebot3.py`(하드웨어 주기/전원/실패, 구동부, 라이다, 퍽 밀기), `tests/test_interaction.py`(입력 장치, 로봇이 입력을 받아 해석, `RobotBridge`), `tests/test_diagnostics.py`(진단 메시지, 업데이트, 집계, 하드웨어 사실 -> 수준), `tests/test_simulation_runner.py`(러너: 고정 스텝, 리스너, 스레드), `tests/test_snapshot.py`(스냅샷 기록기), `tests/test_viewer.py`(오프스크린으로 화면을 몇 프레임 돌려 보는 스모크 테스트, Panda3D가 있을 때), `tests/test_compute.py`(GPU/CPU/순수 파이썬), `tests/test_urdf_model.py`(Panda3D 있을 때)로 로직을 확인한다. `Viewer`의 그림 자체는 자동으로 검증하지 못하고(스크린샷으로 수동 확인), 오프스크린 스모크 테스트가 프레임이 죽지 않는지만 본다.

## TODO

[TODO.md](TODO.md) 참고.
