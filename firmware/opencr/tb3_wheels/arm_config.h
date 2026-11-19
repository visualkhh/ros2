#pragma once
// 팔 설정 — OpenMANIPULATOR-X(RM-X52-TNM) 5축 + 그리퍼. 값은 Lv2_bee_Assignment/lv2_module6/firmware/beebot_opencr/config.h 에서 가져옴.
//   참고 문서: https://docs.robotis.com/docs/systems/turtlebot3/manipulation/
#include <stddef.h>
#include <stdint.h>

// 모터 ID — DYNAMIXEL Wizard로 미리 설정 (바퀴는 1·2). OM-X 기본은 그리퍼가 15인데, 손목 roll을 끼우며 16으로 옮김
constexpr uint8_t ARM_IDS[] = {11, 12, 13, 14, 15, 16};   // joint1~5, 그리퍼(마지막)
constexpr size_t ARM_N = sizeof(ARM_IDS) / sizeof(ARM_IDS[0]);
constexpr size_t ARM_GRIPPER = ARM_N - 1;
// 관절 부호 (0 rad = 서보 중앙 2048). TODO: 실기에서 관절마다 확인 — 영점 오프셋은 SBC URDF 쪽
constexpr float ARM_DIR[ARM_GRIPPER] = {1, 1, 1, 1, 1};

// 그리퍼 — 프로토콜 값은 손가락 하나 [m] (0 = 닫힘, + = 열림). 링크 구조라 서보각과 비선형 → 실측표 구간별 직선 (Lv2 2026-10-24 실측)
constexpr size_t GRIPPER_TABLE = 5;
constexpr float GRIPPER_FINGER_M[GRIPPER_TABLE] = {0.0f, 0.0035f, 0.005f, 0.009f, 0.0275f};    // 오름차순
constexpr float GRIPPER_SERVO_RAD[GRIPPER_TABLE] = {1.0068f, 0.6f, 0.4998f, 0.12f, -1.637f};   // 서보각이 클수록 닫힘
constexpr float GRIPPER_GOAL_CURRENT_MA = 600.0f;   // 쥐는 힘 한계 기본값 (전류 기반 위치 모드). TODO 실측
constexpr float GRIPPER_CURRENT_MIN_MA = 2.69f;     // Goal Current 1단위
constexpr float GRIPPER_CURRENT_MAX_MA = 1200.0f;

constexpr uint32_t ARM_CHECK_MS = 500;              // 에러·온도·토크 점검 주기
constexpr uint32_t ARM_RETRY_MS = 5000;             // 팔이 없을 때 다시 찾는 간격 (ping이 loop를 잠깐 막으니 드물게)
constexpr uint32_t ARM_CMD_TIMEOUT_MS = 500;        // 마지막 A 후 이 시간이 지나면 현재 자세 유지
constexpr uint32_t ARM_RECOVER_PROFILE_VELOCITY = 20;   // 토크 복구 후 명령 자세로 돌아갈 때 속도 한계 (×0.229 rpm)
constexpr float ARM_GOAL_HYSTERESIS_RAW = 0.8f;     // 목표를 바꾸는 최소 변화 [tick] — 반올림 경계 떨림 방지

// XM430 컨트롤 테이블 (바퀴와 겹치는 주소는 tb3_wheels.ino에 이미 있음)
constexpr uint16_t ADDR_RETURN_DELAY = 9;
constexpr uint16_t ADDR_GOAL_CURRENT = 102;
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint8_t OP_POSITION = 3;
constexpr uint8_t OP_CURRENT_POSITION = 5;
