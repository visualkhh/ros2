# TODO / Checklist

레드팀 점검(2026-11-19) 결과. 위에서부터 우선순위 순.

## 지금 할 것

- [ ] **토픽 타입 잠김 규칙 정하기**: 잘못된 타입으로 먼저 publish하면 토픽이 그 타입으로 고정되어 맞는 구독자가 `TypeError`를 맞는다. publish로는 타입을 못 정하게 할지(subscribe/advertise만 허용), 먼저 선언하게 할지 결정
- [ ] **에러 경로 테스트 추가**: 타입 잠김, 순환 구독 (예외 격리/unsubscribe/동시성은 완료)
- [ ] **`test_start` 이름 정리**: `main.py`의 `test_start`가 pytest에서 중복 수집될 수 있음. `main.py`의 `assert isinstance`도 `-O`에서 사라지므로 교체
- [ ] **`Node` 단독 pub/sub 테스트**: 지금은 `test_kros.py`/`test_node.py`에 흩어져 있음
- [ ] **README 사용 예 실제 실행으로 검증**
- [ ] **git 커밋**: `kros/`는 아직 untracked

## 다음 단계

- [ ] 진짜 하드웨어 구현: `Motor`/`Lidar` 인터페이스를 상속해 `sample`/`apply`에서 실제 장치(OpenCR 시리얼, LDS-01)와 통신. 아직 없음 (더미/시뮬레이션만). 실제 통신 실패를 `HardwareError`로 바꾸는 부분 포함
- [ ] 실제 하드웨어(라즈베리파이)에 올려 실행해 보기: kros 코어가 numpy 없이 도는 것은 확인했지만(`sys.modules` 차단으로 흉내) 실제 파이 기기에서는 해 보지 못함. 실제 로봇은 `RobotBridge` 자리에 하드웨어 드라이버 노드가 들어가야 함
- [ ] 충돌 계산(`_resolve_collisions`의 객체 쌍 반복)은 아직 순수 파이썬 루프: 객체가 수백 개가 되면 numpy 벡터화/공간 분할 필요
- [ ] 객체 물리 보강: 퍽 모양(삼각형/사각형)을 충돌에 반영(지금은 원 근사), `movable` 상자(Box)/회전하는 상자 충돌(지금 Box는 static 벽 전용, 축에 평행), 회전/각운동량, 반발(탄성), 로봇이 무거운 객체를 밀 때 모터 힘 한계, 줍기 같은 상호작용
- [ ] `Viewer`가 `TurtleBot3`/`MazeWorld`에 묶여 있음(바퀴 조인트 이름, 라이다 점 표시, 트랙 벽): 다른 로봇/월드가 생기면 일반화
- [ ] 키 입력을 3D 창이 아닌 별도 입력 장치(터미널, 게임패드 `Interaction`)로도 받기: `Interaction`을 상속하고 `robot.receive()`로 달면 된다
- [ ] 재사용할 kros 쪽 노드(RobotBridge 등)가 늘면 `examples/`에서 `kros/nodes/` 같은 곳으로 올리기
- [ ] 시뮬레이터 충돌: 로봇을 반지름 0.2 원으로 근사 중(waffle pi는 앞 0.077/뒤 0.197/좌우 0.153). 회전 중 뒤 모서리가 벽을 파고듦 -> 직사각형 충돌로
- [ ] 라이다는 벽만 본다: 장애물(상자 등) 추가 + 로봇/라이다 충돌, 로봇 프로그램 쪽에서 `scan`을 쓰는 노드(장애물 회피 등), 카메라(`camera_link`) 시야
- [ ] TurtleBot3 시뮬레이터: Odometry 메시지(`msgs/`), 맵 표시, 시뮬 시계를 GUI에서 분리
- [ ] `Viewer`(Panda3D)는 그림 내용을 자동으로 검증하지 못함 (오프스크린 스모크 테스트가 프레임이 도는지만 본다). 화면/키 확인은 수동
- [ ] `Kros` 역할 보강: `run()` / 종료, 노드 소유권
- [ ] 타입 비교 규칙: 서브클래스/`bool` 허용 여부 (지금은 정확히 같은 타입만)
- [ ] 보정 노드에 각속도 처리 + 테스트, 정확한 `repr` 문자열 비교 대신 필드 비교
- [ ] 서비스 예제 늘리기 (운영자 노드가 모터 노드 서비스 호출 등)
- [ ] 모터 노드가 주기적으로 읽기 (`read()` 직접 호출 대신)
- [ ] 메시지 패키지 추가: `std_msgs`, `sensor_msgs`, `nav_msgs`

## 나중에 (설계가 필요)

- [x] **오도메트리**: `DiffDriveOdometry`(바퀴 각 적분, `odom` 토픽 + `odom -> base_footprint` TF). 남은 것: IMU를 섞은 보정, 시뮬레이션에 바퀴 슬립 넣기 (`map -> odom`은 SLAM/위치추정이 낸다)
- [ ] TF: 벡터/PointCloud 변환 편의 함수, `viewer`에서 TF 좌표축 그리기, URDF 파서와 `viewer/urdf_model.py`(panda3d용)의 중복 정리 (뷰어가 `kros/urdf` 파서를 쓰도록)
- [ ] `RobotStatePublisher`: 움직이는 관절 값이 안 오면 아무것도 안 낸다 (ROS도 같음). mimic 관절, floating/planar 관절은 미지원
- [ ] **bag(기록/재생)**: 보류. 방향은 ROS의 rosbag과 같다 (토픽 메시지를 시각과 함께 파일에 기록, 나중에 원래 간격으로 재생). `rosout`(로그)도 같이 기록된다. 필요한 것: 메시지 직렬화(JSON + 타입 이름 레지스트리: **네트워크와 같은 부품이라 같이 설계**), `TopicManager`의 토픽 목록 조회와 "전부 엿보기" 훅, 재생 시각은 `SimulatedTimeSource`로 원래 간격 맞추기, 재생이 만드는 `/clock`
- [ ] 로깅: 노드별이 아니라 전체 기본 수준(`Kros(log_level=...)`), 같은 로그를 N초에 한 번만 내는 throttle, `rosout`을 `TRANSIENT_LOCAL`로 내서 늦게 뜬 모니터가 최근 로그를 받기, 파일로 남기기
- [ ] 진단: `DiagnosticAggregator`가 계층/그룹 분석(ROS의 analyzer)과 `diagnostics_agg` 재발행을 하지 않음, 진단 작업이 느리면 타이머 워커를 막음(콜백이 길면 `missed`로 셈)
- [ ] **네트워크(여러 프로세스/호스트 연결)**: 보류 중. 방향: 허브(별 구조, TCP 아웃바운드)로 시작 -> 원격 토픽 -> 서비스 -> 액션(토픽+서비스 위라 거의 공짜). 매니저는 원격 훅만 열고 연결은 `Context`가 한다. 직렬화는 JSON + 타입 이름 레지스트리(`pickle`은 안 씀), 기본은 아무것도 안 내보내고 허용 목록만 교차. 먼저 필요한 것: `ServiceType`/`ActionType`의 문자열 이름 (토픽 latch와 큐 길이 제한 QoS는 완료), 하트비트/끊김 처리, `/clock` 전달. 규모는 약 1300줄 + 테스트 약 800줄. 첫 사용처는 라즈베리파이(실제 하드웨어) <-> 노트북. 보안은 1단계에서 "신뢰할 수 있는 네트워크 전용"으로 명시
- [ ] 타이머마다 워커 스레드 1개: 타이머가 많아지면 부담 (구독과 같은 문제). executor 공유 방식 검토
- [ ] 시간: 프로세스를 나눌 때 시뮬레이션 시간을 `/clock` 토픽으로 전달, 실제/시뮬레이션 시간 혼용 방지(`use_sim_time` 파라미터)
- [ ] 파라미터: 서비스로 get/set 열기(ROS2의 `/node/get_parameters`), 파일(TOML)에서 시작 값 읽기, `ParameterEvent`에 시각(stamp, 시간 원천이 생기면)
- [ ] 월드와 로봇 프로그램을 별도 프로세스로 분리: kros가 아직 한 프로세스 안이라 지금은 같은 프로세스의 스레드로 같이 뜸 (프로세스 간 통신이 필요)
- [ ] 액션: goal 상태 토픽(실행 중/취소 중 등 중간 상태), 동시 goal 정책(지금은 서버당 직렬 실행), 서버가 **비정상 종료**하면 클라이언트 handle이 영원히 대기(타임아웃/생존 확인; 정상적인 destroy/shutdown은 해결됨)
- [ ] 액션 취소의 `goal_callback`/`cancel_callback`(취소 거절) 분리
- [ ] 서비스: 서버 대기(wait_for_service), 서버 스레드 안에서 자기 호출 시 데드락 감지
- [ ] 서비스 타입 검사는 `ServiceType` 동일성(`==`) 비교: 같은 이름/타입의 다른 정의를 어떻게 볼지
- [ ] 노드 재진입: 서비스 핸들러가 다른 서비스를 부르는 체인(A->B)은 되지만 순환(A->B->A)은 데드락
- [ ] 순환 구독(A->B->A)은 RecursionError 대신 무한 전달: 기본 큐가 무제한이라 쌓인다 (QoS depth를 주면 제한됨. 기본값을 바꿀지, 순환을 감지할지 결정)
- [ ] 구독마다 스레드 1개: 구독이 많아지면 부담. executor 공유 방식 검토 (이제 스레드는 첫 메시지가 올 때 만든다)
- [ ] shutdown 이후 publish는 조용히 버려짐: 에러로 할지 결정
- [ ] `Subscription`/`TopicManager` 스트레스 테스트 (publish/unsubscribe 동시 호출)
- [ ] 프로세스 분리 (소켓) -> 분산 디스커버리
- [ ] QoS: `reliability`, `deadline`, `lifespan`, `liveliness` (depth와 durability는 있음). 네트워크를 붙일 때 필요

- [x] viz: rviz 같은 두 번째 창 (격자, TF, 로봇 모델, 라이다, IMU/odom 글자)
- [ ] viz 확장: IMU 화살표, odom 궤적, 오른쪽 클릭 목표, 디스플레이 on/off 패널, scan_frame을 토픽 header에서 읽기 (지금은 LaserScan에 header가 없어 파라미터), 눈으로 확인은 아직 못 했다 (스크린샷)
