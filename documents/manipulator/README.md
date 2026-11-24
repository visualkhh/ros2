# 매니퓰레이터 (팔)

**기준 문서: <https://docs.robotis.com/docs/systems/openmanipulator_x/overview/>** (ROBOTIS OpenMANIPULATOR-X) — 4축 + 그리퍼, 손목 roll 없음. 조립·배선·소프트웨어는 이 문서에 맞춘다.

## 연결 (2026-11-19)

- OpenCR의 **3번째 핀(포트)** 에 꽂음 — 정확히 어느 커넥터(Dynamixel TTL 3번 / 전원 / 확장 핀)인지는 실제 확인 후 여기에 적을 것.
- 펌웨어 `tb3_wheels`에 팔 구동(`A`·`AT`·`AR` 명령, `M`·`F` 상태)을 추가함 — Lv2 `beebot_opencr` 참고, [firmware/opencr](../../firmware/opencr/README.md). **아직 OpenCR에 올리지 않았고(컴파일만 확인), Pi 쪽 ROS 노드는 팔을 모른다.**

## 할 일

- [ ] ROBOTIS 문서의 배선·전원·Dynamixel ID/보레이트 확인
- [ ] 팔 서보 ID를 11~15로 설정 (joint1~4=11~14, gripper=15) · 펌웨어 업로드 · 관절 부호(`ARM_DIR`)·그리퍼 전류 실측
- [ ] Pi 쪽: `opencr_driver_node`에 `M`·`F` 파싱 / `A` 전송, 관절 상태·명령 토픽, CLI 화면
