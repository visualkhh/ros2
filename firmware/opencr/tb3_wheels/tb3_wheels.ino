// tb3_wheels — TurtleBot3 style 2-wheel OpenCR firmware (minimal).
//
// Hardware: ROBOTIS TurtleBot3 Waffle Pi — OpenCR 1.0 + DYNAMIXEL XM430-W210 (wheel ID 2 = left, ID 1 = right), 1 Mbps, Protocol 2.0.
// Pair with SBC driver over USB CDC serial (/dev/opencr).
// SBC side: ros2_ws/src/tb3_bringup/opencr_driver (C -> wheels, S -> odom/TF/joint_states).
//
// Protocol (ASCII lines, '\\n' terminated, SI units):
//   SBC -> OpenCR : C,<left rad/s>,<right rad/s>   (empty field = keep current)
//   SBC -> OpenCR : T,0 | T,1                       (torque off/on, both wheels)
//   SBC -> OpenCR : R | R,<l 0/1>,<r 0/1>           (reboot DYNAMIXEL; bare R = both,
//                                                    empty/0 field = leave that wheel)
//                   Reboot clears hardware error; wheel restarts in velocity mode,
//                   goal 0, torque restored to current T state. Its multi-turn
//                   position restarts within one turn (expect a jump in S pos).
//   SBC -> OpenCR : B | B,<Hz>,<ms>                 (buzzer beep; bare B = 2000 Hz 150 ms, non-blocking)
//   OpenCR -> SBC : S,<l_pos rad>,<l_vel rad/s>,<r_pos rad>,<r_vel rad/s>,<uptime ms>,
//                     <l_cur A>,<r_cur A>,<l_temp C>,<r_temp C>,<battery V>
//                   at STATUS_HZ. Positions are continuous multi-turn radians.
//                   Current/temp are appended after uptime so fields 0-5 keep their index.
//   OpenCR -> SBC : I,<gx>,<gy>,<gz rad/s>,<ax>,<ay>,<az m/s^2>,<qw>,<qx>,<qy>,<qz>
//                   on-board IMU (OpenCR MPU9250/ICM20648), right after each S line. Board frame
//                   (TurtleBot3 imu_link). Gyro bias is calibrated at boot -> keep the robot still ~2 s.
//   OpenCR -> SBC : E,<l_err>,<r_err>
//                   Hardware Error Status (addr 70) per wheel, decimal bitmask:
//                     1=input voltage 4=overheating 8=motor encoder 16=electrical shock
//                     32=overload, 0=ok, -1=no response. Sent on change, then every
//                     ERR_REPEAT_MS while any wheel is non-zero. Clear with R.
//
// Safety: no C command for CMD_TIMEOUT_MS -> wheels stop (0, 0).
//
// Arm (OpenMANIPULATOR-X, IDs 11~15 + gripper 16; additive — wheel lines above are unchanged, arm absent = wheels still work):
//   SBC -> OpenCR : A,<j1>,<j2>,<j3>,<j4>,<j5 rad>,<gripper m>[,<gripper A>]   (empty = keep)
//   SBC -> OpenCR : AT,0 | AT,1 | AT,<6 fields 0/1>     (arm torque off/on; empty = keep)
//   SBC -> OpenCR : AR,1 | AR,<6 fields 0/1>            (reboot arm DYNAMIXELs, clears hardware error)
//   OpenCR -> SBC : M,<6 x (pos rad|m, vel, current A)>,<6 temps C>   (nan = not read yet, temp -1 = unknown)
//   OpenCR -> SBC : F,<6 hardware errors>               (like E: on change, then every ERR_REPEAT_MS while non-zero; -1 = no response)
//   Arm: no A for ARM_CMD_TIMEOUT_MS -> hold current pose. Details: arm_config.h, firmware/opencr/README.md
#include <DynamixelSDK.h>
#include <IMU.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "arm_config.h"

// ---------------- config ----------------
// 이 로봇 배선: 1번 = 오른쪽, 2번 = 왼쪽 (2026-10-27 실기 확인). C·S·R·E의 l/r 순서는 이 매핑을 따른다.
#define WHEEL_LEFT_ID   2
#define WHEEL_RIGHT_ID  1
#define DXL_BUS_NAME    "3"          // OpenCR Serial3 = DYNAMIXEL bus
#define DXL_BAUD        1000000
#define DXL_PROTOCOL    2.0f

#define ADDR_OPERATING_MODE    11
#define ADDR_HW_ERROR_STATUS   70   // 1 byte, bitmask
#define ADDR_TORQUE_ENABLE     64
#define ADDR_GOAL_VELOCITY     104
#define ADDR_PRESENT_CURRENT   126  // 2 bytes, signed
#define ADDR_PRESENT_VELOCITY  128
#define ADDR_PRESENT_POSITION  132
#define ADDR_PRESENT_TEMP      146  // 1 byte, degC
#define OP_VELOCITY            1

#define VEL_UNIT_RPM     0.229f     // 1 unit = 0.229 rpm (X series velocity)
#define POS_UNIT_RAD     (2.0f * 3.14159265f / 4096.0f)
#define CUR_UNIT_A       0.00269f   // 1 unit = 2.69 mA (XM430)
#define VEL_LIMIT_RAD_S  8.0f       // Waffle Pi spec 0.26 m/s / r 0.033 = 7.9 rad/s (DYNAMIXEL Velocity Limit also caps)
#define STATUS_HZ        20
#define GYRO_FACTOR      0.0010642f       // ADC -> rad/s  (+-2000 deg/s, TurtleBot3 turtlebot3_sensor.h)
#define ACCEL_FACTOR     0.000598550415f  // ADC -> m/s^2  (+-2 g)
#define CMD_TIMEOUT_MS   500
#define ERR_REPEAT_MS    1000       // E line repeat period while in error
#define REBOOT_WAIT_MS   1000       // max wait for DYNAMIXEL to answer ping after reboot

// ---------------- state ----------------
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
cIMU imu;

float cmd_l = 0.0f, cmd_r = 0.0f;
uint32_t last_cmd_ms = 0;
uint32_t last_status_ms = 0;
bool torque_on = false;
int16_t err_l = 0, err_r = 0;       // last reported hardware error (-1 = no response)
uint32_t last_err_ms = 0;

static int32_t radps_to_unit(float v) {
  if (v > VEL_LIMIT_RAD_S) v = VEL_LIMIT_RAD_S;
  if (v < -VEL_LIMIT_RAD_S) v = -VEL_LIMIT_RAD_S;
  return (int32_t)(v * 60.0f / (VEL_UNIT_RPM * 2.0f * 3.14159265f));
}

static float unit_to_radps(int32_t u) {
  return (float)u * VEL_UNIT_RPM * 2.0f * 3.14159265f / 60.0f;
}

static float unit_to_rad(int32_t u) {
  return (float)u * POS_UNIT_RAD;
}

static bool dxl_write_goal(float vl, float vr) {
  uint8_t err = 0;
  int r1 = packetHandler->write4ByteTxRx(portHandler, WHEEL_LEFT_ID,
                                         ADDR_GOAL_VELOCITY, (uint32_t)radps_to_unit(vl), &err);
  int r2 = packetHandler->write4ByteTxRx(portHandler, WHEEL_RIGHT_ID,
                                         ADDR_GOAL_VELOCITY, (uint32_t)radps_to_unit(vr), &err);
  return r1 == COMM_SUCCESS && r2 == COMM_SUCCESS;
}

static bool dxl_set_torque(bool on) {
  uint8_t err = 0;
  int r1 = packetHandler->write1ByteTxRx(portHandler, WHEEL_LEFT_ID,
                                         ADDR_TORQUE_ENABLE, on ? 1 : 0, &err);
  int r2 = packetHandler->write1ByteTxRx(portHandler, WHEEL_RIGHT_ID,
                                         ADDR_TORQUE_ENABLE, on ? 1 : 0, &err);
  if (r1 == COMM_SUCCESS && r2 == COMM_SUCCESS) {
    torque_on = on;
    return true;
  }
  return false;
}

// Hardware Error Status of one wheel; -1 if it does not answer.
static int16_t dxl_read_hw_error(uint8_t id) {
  uint8_t err = 0, hw = 0;
  if (packetHandler->read1ByteTxRx(portHandler, id, ADDR_HW_ERROR_STATUS, &hw, &err) != COMM_SUCCESS)
    return -1;
  return hw;
}

// Reboot one wheel and restore velocity mode + torque state. Blocks up to REBOOT_WAIT_MS.
static bool dxl_reboot(uint8_t id) {
  uint8_t err = 0;
  if (packetHandler->reboot(portHandler, id, &err) != COMM_SUCCESS) return false;
  uint32_t t0 = millis();
  uint16_t model = 0;
  delay(100);
  while (packetHandler->ping(portHandler, id, &model, &err) != COMM_SUCCESS) {
    if (millis() - t0 > REBOOT_WAIT_MS) return false;
    delay(20);
  }
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, OP_VELOCITY, &err);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, torque_on ? 1 : 0, &err);
  return true;
}

// ---------------- arm ----------------
struct __attribute__((packed)) ArmPresent { int16_t current; int32_t velocity; int32_t position; };  // addr 126·128·132 연속 10 byte

dynamixel::GroupSyncWrite *arm_sync;
bool arm_ready = false;                 // 6개 전부 세팅 끝 — 아니면 팔 쪽 읽기·쓰기를 건너뛴다 (응답 대기로 loop가 멈추므로)
bool arm_set_up[ARM_N] = {};
bool arm_torque[ARM_N];                 // 켜야 하는 상태 (setup에서 true)
bool arm_seen[ARM_N] = {};
bool arm_wd_tripped = true;             // 첫 A 전에는 자세 유지를 건드리지 않음
int32_t arm_goal[ARM_N] = {};
uint32_t arm_recover_since[ARM_N] = {};
int16_t arm_err[ARM_N], arm_last_err[ARM_N];
int16_t arm_temp[ARM_N];
ArmPresent arm_present[ARM_N] = {};
float gripper_ma = GRIPPER_GOAL_CURRENT_MA;
uint32_t arm_cmd_ms = 0, arm_check_ms = 0, arm_try_ms = 0, arm_err_ms = 0;

static const float RAW_PER_RAD = 4096.0f / (2.0f * 3.14159265f);
static float rad_to_raw_f(float rad, float dir) { return 2048.0f + dir * rad * RAW_PER_RAD; }
static float raw_to_rad(int32_t raw, float dir) { return dir * (raw - 2048) / RAW_PER_RAD; }

// 단조 표 구간별 직선 (오름·내림 둘 다, 표 밖은 끝값)
static float interp(float x, const float *xs, const float *ys, size_t n) {
  const bool up = xs[n - 1] > xs[0];
  if (up ? x <= xs[0] : x >= xs[0]) return ys[0];
  for (size_t i = 1; i < n; ++i)
    if (up ? x <= xs[i] : x >= xs[i])
      return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[n - 1];
}
static float gripper_m_to_rad(float m) { return interp(m, GRIPPER_FINGER_M, GRIPPER_SERVO_RAD, GRIPPER_TABLE); }
static float gripper_rad_to_m(float r) { return interp(r, GRIPPER_SERVO_RAD, GRIPPER_FINGER_M, GRIPPER_TABLE); }

static int32_t arm_read_position(size_t i) {
  uint8_t err = 0;
  uint32_t v = 0;
  packetHandler->read4ByteTxRx(portHandler, ARM_IDS[i], ADDR_PRESENT_POSITION, &v, &err);
  return (int32_t)v;
}

static void arm_write_goal_current() {
  uint8_t err = 0;
  packetHandler->write2ByteTxRx(portHandler, ARM_IDS[ARM_GRIPPER], ADDR_GOAL_CURRENT,
                                (uint16_t)(gripper_ma / 2.69f + 0.5f), &err);
}

// 토크 켜기 — 목표 = 지금 위치(튀지 않게). recover면 명령 자세로 돌아갈 때까지 속도 한계
static void arm_enable(size_t i, bool recover) {
  uint8_t err = 0;
  const uint8_t id = ARM_IDS[i];
  if (i == ARM_GRIPPER) arm_write_goal_current();
  arm_goal[i] = arm_read_position(i);
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, (uint32_t)arm_goal[i], &err);
  if (recover && i != ARM_GRIPPER) {
    packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, ARM_RECOVER_PROFILE_VELOCITY, &err);
    arm_recover_since[i] = millis() | 1;
  }
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &err);
  arm_torque[i] = true;
}

static bool arm_setup_one(size_t i, bool recover) {
  uint8_t err = 0, v = 0;
  const uint8_t id = ARM_IDS[i];
  uint16_t model = 0;
  if (packetHandler->ping(portHandler, id, &model, &err) != COMM_SUCCESS) return false;
  const uint8_t mode = i == ARM_GRIPPER ? OP_CURRENT_POSITION : OP_POSITION;
  // EEPROM은 토크가 꺼져야 쓸 수 있다 — 다를 때만 쓰고 그때만 끈다 (팔이 툭 떨어지지 않게, 반복 복구에도 닳지 않게)
  const bool mode_ok = packetHandler->read1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, &v, &err) == COMM_SUCCESS && v == mode;
  const bool delay_ok = packetHandler->read1ByteTxRx(portHandler, id, ADDR_RETURN_DELAY, &v, &err) == COMM_SUCCESS && v == 0;
  if (!mode_ok || !delay_ok) packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &err);
  if (!mode_ok) packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, mode, &err);
  if (!delay_ok) packetHandler->write1ByteTxRx(portHandler, id, ADDR_RETURN_DELAY, 0, &err);  // 기본 250 µs가 응답마다 붙음
  arm_enable(i, recover);
  return true;
}

static bool arm_setup_all() {
  bool ok = true;
  for (size_t i = 0; i < ARM_N; ++i) {
    if (!arm_set_up[i]) arm_set_up[i] = arm_setup_one(i, false);
    ok = arm_set_up[i] && ok;
  }
  return ok;
}

// "a,b,,c" → out[] (빈칸·모자란 뒷칸 = NAN = 유지). 너무 많거나 숫자가 아니면 false
static bool parse_fields(char *text, float *out, size_t count) {
  size_t i = 0;
  while (true) {
    if (*text == ',' || *text == '\0') {
      out[i] = NAN;
    } else {
      char *end = nullptr;
      out[i] = strtof(text, &end);
      if (end == text || !isfinite(out[i])) return false;
      text = end;
    }
    ++i;
    if (*text == '\0') break;
    if (*text != ',') return false;
    ++text;
    if (i == count) return false;
  }
  for (; i < count; ++i) out[i] = NAN;
  return true;
}

static bool arm_any_torque() {
  for (size_t i = 0; i < ARM_N; ++i) if (arm_torque[i]) return true;
  return false;
}

static void arm_command(char *text) {
  if (!arm_ready) return;
  float v[ARM_N + 1];   // + 7번째 그리퍼 전류 A
  if (!parse_fields(text, v, ARM_N + 1)) return;
  if (!isnan(v[ARM_N])) {
    const float ma = constrain(v[ARM_N] * 1000.0f, GRIPPER_CURRENT_MIN_MA, GRIPPER_CURRENT_MAX_MA);
    if (fabsf(ma - gripper_ma) >= 1.0f) {   // 바뀔 때만 씀 (매 주기 쓰면 버스 시간)
      gripper_ma = ma;
      if (arm_torque[ARM_GRIPPER] && arm_err[ARM_GRIPPER] == 0) arm_write_goal_current();
    }
  }
  bool any = false;
  for (size_t i = 0; i < ARM_N; ++i) {
    if (isnan(v[i]) || !arm_torque[i]) continue;
    const float f = i == ARM_GRIPPER ? rad_to_raw_f(gripper_m_to_rad(v[i]), 1.0f) : rad_to_raw_f(v[i], ARM_DIR[i]);
    if (fabsf(f - (float)arm_goal[i]) < ARM_GOAL_HYSTERESIS_RAW) continue;   // 같은 목표를 계속 쓰면 1 tick 떨림
    arm_goal[i] = lroundf(f);
    uint8_t data[4] = {(uint8_t)(arm_goal[i] & 0xFF), (uint8_t)((arm_goal[i] >> 8) & 0xFF),
                       (uint8_t)((arm_goal[i] >> 16) & 0xFF), (uint8_t)((arm_goal[i] >> 24) & 0xFF)};
    arm_sync->addParam(ARM_IDS[i], data);
    any = true;
  }
  if (any) { arm_sync->txPacket(); arm_sync->clearParam(); }   // 한 패킷으로 동시에 반영
  arm_cmd_ms = millis();
  if (arm_any_torque()) arm_wd_tripped = false;
}

// AT,0 / AT,1 = 전체 · AT,<6칸> = 관절별 (빈칸 = 유지). 켤 때는 지금 자세가 목표 — 손으로 옮긴 팔이 튀지 않게
static void arm_torque_line(char *text) {
  if (!arm_ready) return;
  const bool whole = strchr(text, ',') == nullptr;
  float v[ARM_N];
  if (!parse_fields(text, v, ARM_N)) return;
  if (whole && isnan(v[0])) return;
  for (size_t i = 0; i < ARM_N; ++i) {
    const float x = whole ? v[0] : v[i];
    if (isnan(x)) continue;
    if (x != 0.0f) {
      arm_enable(i, true);
    } else {
      uint8_t err = 0;
      packetHandler->write1ByteTxRx(portHandler, ARM_IDS[i], ADDR_TORQUE_ENABLE, 0, &err);
      arm_torque[i] = false;
    }
  }
  arm_wd_tripped = !arm_any_torque();
  arm_cmd_ms = millis();
}

static void arm_read_error(size_t i) {
  uint8_t err = 0, hw = 0;
  arm_err[i] = packetHandler->read1ByteTxRx(portHandler, ARM_IDS[i], ADDR_HW_ERROR_STATUS, &hw, &err) == COMM_SUCCESS ? hw : -1;
}

static void arm_print_errors() {
  Serial.print("F");
  for (size_t i = 0; i < ARM_N; ++i) { Serial.print(","); Serial.print(arm_err[i]); }
  Serial.println();
}

// AR,1 = 전체 · AR,<6칸> = 고른 관절만 DYNAMIXEL Reboot → 응답하면 세팅·토크 (느린 복귀)
static void arm_reboot_line(char *text) {
  if (!arm_ready) return;
  const bool whole = strchr(text, ',') == nullptr;
  float v[ARM_N];
  if (!parse_fields(text, v, ARM_N)) return;
  bool pick[ARM_N];
  for (size_t i = 0; i < ARM_N; ++i) {
    const float x = whole ? v[0] : v[i];
    pick[i] = !isnan(x) && x != 0.0f;
    if (pick[i]) { uint8_t err = 0; packetHandler->reboot(portHandler, ARM_IDS[i], &err); }
  }
  const uint32_t t0 = millis();
  for (size_t i = 0; i < ARM_N; ++i) {
    if (!pick[i]) continue;
    uint8_t err = 0;
    uint16_t model = 0;
    delay(100);
    while (packetHandler->ping(portHandler, ARM_IDS[i], &model, &err) != COMM_SUCCESS && millis() - t0 < REBOOT_WAIT_MS) delay(20);
    arm_setup_one(i, true);
    arm_read_error(i);
  }
  arm_print_errors();
  arm_cmd_ms = millis();
}

// 100 ms마다가 아니라 0.5 s마다: 에러·온도 읽기, 속도 한계 해제, 12V를 껐다 켜 풀린 토크 복구, 팔이 없으면 5 s마다 찾기
static void arm_check() {
  const uint32_t now = millis();
  if (!arm_ready) {
    if (now - arm_try_ms < ARM_RETRY_MS) return;
    arm_try_ms = now;
    uint8_t err = 0;
    uint16_t model = 0;
    if (packetHandler->ping(portHandler, ARM_IDS[0], &model, &err) == COMM_SUCCESS)   // joint1만 먼저 (없는 모터마다 응답 대기가 붙음)
      arm_ready = arm_setup_all();
    return;
  }
  if (arm_wd_tripped == false && now - arm_cmd_ms > ARM_CMD_TIMEOUT_MS) {   // 명령이 끊김 — 지금 자세 유지 (한 번)
    for (size_t i = 0; i < ARM_N; ++i) {
      uint8_t err = 0;
      arm_goal[i] = arm_read_position(i);
      packetHandler->write4ByteTxRx(portHandler, ARM_IDS[i], ADDR_GOAL_POSITION, (uint32_t)arm_goal[i], &err);
    }
    arm_wd_tripped = true;
  }
  if (now - arm_check_ms < ARM_CHECK_MS) return;
  arm_check_ms = now;
  for (size_t i = 0; i < ARM_N; ++i) {
    uint8_t err = 0, t = 0, en = 1;
    const uint8_t id = ARM_IDS[i];
    arm_read_error(i);
    if (arm_err[i] < 0) { arm_temp[i] = -1; continue; }   // 응답 없음
    arm_temp[i] = packetHandler->read1ByteTxRx(portHandler, id, ADDR_PRESENT_TEMP, &t, &err) == COMM_SUCCESS ? t : -1;
    if (arm_recover_since[i]) {
      const bool near = labs(arm_present[i].position - arm_goal[i]) < (long)(0.05f * RAW_PER_RAD);
      if (near || now - arm_recover_since[i] > 10000) {
        packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, 0, &err);   // 0 = 무제한
        arm_recover_since[i] = 0;
      }
    }
    if (!arm_torque[i] || arm_err[i] != 0) continue;   // Hardware Error는 AR로만 풀린다
    if (packetHandler->read1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, &en, &err) == COMM_SUCCESS && en == 0)
      arm_setup_one(i, true);   // 재부팅된 모터 — 모드·Return Delay·전류 다시, 느린 복귀
  }
  bool changed = false, in_error = false;
  for (size_t i = 0; i < ARM_N; ++i) { changed |= arm_err[i] != arm_last_err[i]; in_error |= arm_err[i] != 0; }
  if (changed || (in_error && now - arm_err_ms >= ERR_REPEAT_MS)) {
    memcpy(arm_last_err, arm_err, sizeof(arm_err));
    arm_err_ms = now;
    arm_print_errors();
  }
}

static void arm_print_state() {
  if (!arm_ready) return;   // 팔이 없으면 M 줄을 보내지 않는다
  Serial.print("M");
  for (size_t i = 0; i < ARM_N; ++i) {
    uint8_t err = 0;
    ArmPresent p;
    if (packetHandler->readTxRx(portHandler, ARM_IDS[i], ADDR_PRESENT_CURRENT, sizeof(p), (uint8_t *)&p, &err) == COMM_SUCCESS) {
      arm_present[i] = p;
      arm_seen[i] = true;
    }
    float pos = NAN, vel = NAN, cur = NAN;
    if (arm_seen[i]) {
      const ArmPresent &q = arm_present[i];
      const float vel_rad_s = q.velocity * VEL_UNIT_RPM * 2.0f * 3.14159265f / 60.0f;
      cur = q.current * CUR_UNIT_A;
      if (i == ARM_GRIPPER) {
        const float servo = raw_to_rad(q.position, 1.0f);
        const float slope = (gripper_rad_to_m(servo + 0.01f) - gripper_rad_to_m(servo - 0.01f)) / 0.02f;   // m/rad
        pos = gripper_rad_to_m(servo);
        vel = slope * vel_rad_s;
      } else {
        pos = raw_to_rad(q.position, ARM_DIR[i]);
        vel = ARM_DIR[i] * vel_rad_s;
        cur *= ARM_DIR[i];
      }
    }
    Serial.print(","); Serial.print(pos, 5);
    Serial.print(","); Serial.print(vel, 5);
    Serial.print(","); Serial.print(cur, 4);
  }
  for (size_t i = 0; i < ARM_N; ++i) { Serial.print(","); Serial.print(arm_temp[i]); }
  Serial.println();
}

// Parse "C,vl,vr" / "T,x" / "R[,l,r]" / "B[,hz,ms]". Empty numeric field = keep previous value.
static void handle_line(char *line) {
  if (strncmp(line, "AT,", 3) == 0) {
    arm_torque_line(line + 3);
  } else if (strncmp(line, "AR,", 3) == 0) {
    arm_reboot_line(line + 3);
  } else if (strncmp(line, "A,", 2) == 0) {
    arm_command(line + 2);
  } else if (line[0] == 'C' && line[1] == ',') {
    char *p = line + 2;
    char *comma = strchr(p, ',');
    float vl = cmd_l, vr = cmd_r;
    if (comma != NULL) {
      *comma = '\0';
      if (p[0] != '\0') vl = atof(p);
      if (comma[1] != '\0') vr = atof(comma + 1);
    } else if (p[0] != '\0') {
      vl = atof(p);
    }
    cmd_l = vl;
    cmd_r = vr;
    last_cmd_ms = millis();
    dxl_write_goal(cmd_l, cmd_r);
  } else if (line[0] == 'T' && line[1] == ',') {
    if (line[2] == '1') dxl_set_torque(true);
    else if (line[2] == '0') dxl_set_torque(false);
  } else if (line[0] == 'R' && (line[1] == '\0' || line[1] == ',')) {
    bool rl = true, rr = true;  // bare "R" = both
    if (line[1] == ',') {
      char *p = line + 2;
      char *comma = strchr(p, ',');
      if (comma != NULL) *comma = '\0';
      rl = p[0] != '\0' && atoi(p) != 0;
      rr = comma != NULL && comma[1] != '\0' && atoi(comma + 1) != 0;
    }
    // Rebooted wheel comes back with goal 0; keep cmd in sync.
    if (rl && dxl_reboot(WHEEL_LEFT_ID)) cmd_l = 0.0f;
    if (rr && dxl_reboot(WHEEL_RIGHT_ID)) cmd_r = 0.0f;
  } else if (line[0] == 'B' && (line[1] == '\0' || line[1] == ',')) {
    long hz = 2000, ms = 150;
    if (line[1] == ',') {
      char *comma = strchr(line + 2, ',');
      if (line[2] != '\0' && line[2] != ',') hz = atol(line + 2);
      if (comma != NULL && comma[1] != '\0') ms = atol(comma + 1);
    }
    if (hz < 100) hz = 100;
    if (hz > 8000) hz = 8000;
    if (ms < 10) ms = 10;
    if (ms > 2000) ms = 2000;
    tone(BDPIN_BUZZER, (unsigned int)hz, (unsigned long)ms);  // timer-driven, returns at once
  }
}

static void poll_serial() {
  static char buf[128];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handle_line(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // overflow: drop line
    }
  }
}

void setup() {
  Serial.begin(115200);
  portHandler = dynamixel::PortHandler::getPortHandler(DXL_BUS_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL);
  portHandler->openPort();
  portHandler->setBaudRate(DXL_BAUD);

  uint8_t err = 0;
  packetHandler->write1ByteTxRx(portHandler, WHEEL_LEFT_ID, ADDR_OPERATING_MODE, OP_VELOCITY, &err);
  packetHandler->write1ByteTxRx(portHandler, WHEEL_RIGHT_ID, ADDR_OPERATING_MODE, OP_VELOCITY, &err);
  dxl_set_torque(true);

  // 팔 — 없어도(뽑음 · 12V 꺼짐) 바퀴는 그대로 동작, arm_check()가 5 s마다 다시 찾는다
  arm_sync = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
  for (size_t i = 0; i < ARM_N; ++i) { arm_torque[i] = true; arm_err[i] = arm_last_err[i] = 0; arm_temp[i] = -1; }
  arm_ready = arm_setup_all();

  imu.begin();
  imu.SEN.gyro_cali_start();
  while (!imu.SEN.gyro_cali_get_done()) imu.update();  // gyro bias at rest (~2 s)
  last_cmd_ms = millis();
}

void loop() {
  poll_serial();
  imu.update();  // rate-limited inside (200 Hz)

  // Watchdog: stop on command loss
  if (torque_on && (millis() - last_cmd_ms > CMD_TIMEOUT_MS)) {
    if (cmd_l != 0.0f || cmd_r != 0.0f) {
      cmd_l = 0.0f;
      cmd_r = 0.0f;
      dxl_write_goal(0.0f, 0.0f);
    }
  }

  arm_check();

  // Status at STATUS_HZ
  if (millis() - last_status_ms >= 1000 / STATUS_HZ) {
    last_status_ms = millis();
    uint8_t err = 0;
    int32_t lp = 0, lv = 0, rp = 0, rv = 0;
    packetHandler->read4ByteTxRx(portHandler, WHEEL_LEFT_ID, ADDR_PRESENT_POSITION, (uint32_t *)&lp, &err);
    packetHandler->read4ByteTxRx(portHandler, WHEEL_LEFT_ID, ADDR_PRESENT_VELOCITY, (uint32_t *)&lv, &err);
    packetHandler->read4ByteTxRx(portHandler, WHEEL_RIGHT_ID, ADDR_PRESENT_POSITION, (uint32_t *)&rp, &err);
    packetHandler->read4ByteTxRx(portHandler, WHEEL_RIGHT_ID, ADDR_PRESENT_VELOCITY, (uint32_t *)&rv, &err);
    int16_t lc = 0, rc = 0;
    uint8_t lt = 0, rt = 0;
    packetHandler->read2ByteTxRx(portHandler, WHEEL_LEFT_ID, ADDR_PRESENT_CURRENT, (uint16_t *)&lc, &err);
    packetHandler->read2ByteTxRx(portHandler, WHEEL_RIGHT_ID, ADDR_PRESENT_CURRENT, (uint16_t *)&rc, &err);
    packetHandler->read1ByteTxRx(portHandler, WHEEL_LEFT_ID, ADDR_PRESENT_TEMP, &lt, &err);
    packetHandler->read1ByteTxRx(portHandler, WHEEL_RIGHT_ID, ADDR_PRESENT_TEMP, &rt, &err);
    Serial.print("S,");
    Serial.print(unit_to_rad(lp), 5); Serial.print(",");
    Serial.print(unit_to_radps(lv), 5); Serial.print(",");
    Serial.print(unit_to_rad(rp), 5); Serial.print(",");
    Serial.print(unit_to_radps(rv), 5); Serial.print(",");
    Serial.print(millis()); Serial.print(",");
    Serial.print(lc * CUR_UNIT_A, 3); Serial.print(",");
    Serial.print(rc * CUR_UNIT_A, 3); Serial.print(",");
    Serial.print(lt); Serial.print(",");
    Serial.print(rt); Serial.print(",");
    Serial.println(getPowerInVoltage(), 2);  // battery [V] (OpenCR core API)

    Serial.print("I");
    for (int i = 0; i < 3; i++) { Serial.print(","); Serial.print(imu.SEN.gyroADC[i] * GYRO_FACTOR, 5); }
    for (int i = 0; i < 3; i++) { Serial.print(","); Serial.print(imu.SEN.accADC[i] * ACCEL_FACTOR, 4); }
    for (int i = 0; i < 4; i++) { Serial.print(","); Serial.print(imu.quat[i], 5); }
    Serial.println();
    arm_print_state();

    // Error report: on change, then repeat while any wheel is in error
    int16_t el = dxl_read_hw_error(WHEEL_LEFT_ID);
    int16_t er = dxl_read_hw_error(WHEEL_RIGHT_ID);
    bool changed = (el != err_l || er != err_r);
    bool in_error = (el != 0 || er != 0);
    if (changed || (in_error && millis() - last_err_ms >= ERR_REPEAT_MS)) {
      err_l = el;
      err_r = er;
      last_err_ms = millis();
      Serial.print("E,");
      Serial.print(el); Serial.print(",");
      Serial.println(er);
    }
  }
}
