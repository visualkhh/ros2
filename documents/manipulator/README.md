# 매니퓰레이터 (팔)

**기준 문서: <https://docs.robotis.com/docs/systems/openmanipulator_x/overview/>** (ROBOTIS OpenMANIPULATOR-X) — 4축 + 그리퍼, 손목 roll 없음. 조립·배선·소프트웨어는 이 문서에 맞춘다.

## 연결 (2026-11-19)

- OpenCR의 **3번째 핀(포트)** 에 꽂음 — 정확히 어느 커넥터(Dynamixel TTL 3번 / 전원 / 확장 핀)인지는 실제 확인 후 여기에 적을 것.
- 펌웨어 `tb3_wheels`에 팔 구동(`A`·`AT`·`AR` 명령, `M`·`F` 상태)을 추가했고 **OpenCR에 업로드·실행 확인**했다. [firmware/opencr](../../firmware/opencr/README.md)
- C++ `krosc_ws`가 `M`/`F`를 파싱해 `joint_states`에 joint1~4+그리퍼를 넣고, `arm_joint_command`(`sensor_msgs/JointState`)로 `A`를 보낸다. URDF는 `robot/description/urdf/turtlebot3_real.urdf`.

## 할 일

- [x] ROBOTIS 문서의 배선·전원·Dynamixel ID/보레이트 확인 (ID 11~15 응답, ID16 없음)
- [x] 팔 서보 ID 11~15 (joint1~4=11~14, gripper=15) · 펌웨어 업로드
- [ ] 관절 부호(`ARM_DIR`)·그리퍼 전류 실측 (현재 모두 +1)
- [x] Pi C++: `M`·`F` 파싱 / `A` 전송, `joint_states`·`arm_joint_command`
- [ ] Viz 팔 슬라이더/그리퍼 UI
