// tb3_wheels — TurtleBot3 style 2-wheel OpenCR firmware (minimal).
//
// Hardware: OpenCR 1.0 + DYNAMIXEL XM430-W350 (wheel ID 1, 2), 1 Mbps, Protocol 2.0.
// Pair with SBC driver over USB CDC serial (/dev/opencr).
// Future SBC side: ros2_control hardware plugin replacing mock_base (same topics).
//
// Protocol (ASCII lines, '\\n' terminated, SI units):
//   SBC -> OpenCR : C,<left rad/s>,<right rad/s>   (empty field = keep current)
//   SBC -> OpenCR : T,0 | T,1                       (torque off/on, both wheels)
//   SBC -> OpenCR : R | R,<l 0/1>,<r 0/1>           (reboot DYNAMIXEL; bare R = both,
//                                                    empty/0 field = leave that wheel)
//                   Reboot clears hardware error; wheel restarts in velocity mode,
//                   goal 0, torque restored to current T state. Its multi-turn
//                   position restarts within one turn (expect a jump in S pos).
//   OpenCR -> SBC : S,<l_pos rad>,<l_vel rad/s>,<r_pos rad>,<r_vel rad/s>,<uptime ms>,
//                     <l_cur A>,<r_cur A>,<l_temp C>,<r_temp C>
//                   at STATUS_HZ. Positions are continuous multi-turn radians.
//                   Current/temp are appended after uptime so fields 0-5 keep their index.
//   OpenCR -> SBC : E,<l_err>,<r_err>
//                   Hardware Error Status (addr 70) per wheel, decimal bitmask:
//                     1=input voltage 4=overheating 8=motor encoder 16=electrical shock
//                     32=overload, 0=ok, -1=no response. Sent on change, then every
//                     ERR_REPEAT_MS while any wheel is non-zero. Clear with R.
//
// Safety: no C command for CMD_TIMEOUT_MS -> wheels stop (0, 0).
// Arduino.h의 min/max 매크로가 gcc 14 newlib <map>(DynamixelSDK가 포함)의 std::min/max와 충돌 → 먼저 해제.
// 이 스케치는 min/max를 쓰지 않는다.
#undef min
#undef max
#include <DynamixelSDK.h>

// ---------------- config ----------------
#define WHEEL_LEFT_ID   1
#define WHEEL_RIGHT_ID  2
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
#define VEL_LIMIT_RAD_S  6.0f       // XM430-W350 no-load ~61 rpm ~= 6.4 rad/s
#define STATUS_HZ        20
#define CMD_TIMEOUT_MS   500
#define ERR_REPEAT_MS    1000       // E line repeat period while in error
#define REBOOT_WAIT_MS   1000       // max wait for DYNAMIXEL to answer ping after reboot

// ---------------- state ----------------
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

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

// Parse "C,vl,vr" / "T,x" / "R[,l,r]". Empty numeric field = keep previous value.
static void handle_line(char *line) {
  if (line[0] == 'C' && line[1] == ',') {
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
  }
}

static void poll_serial() {
  static char buf[64];
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
  last_cmd_ms = millis();
}

void loop() {
  poll_serial();

  // Watchdog: stop on command loss
  if (torque_on && (millis() - last_cmd_ms > CMD_TIMEOUT_MS)) {
    if (cmd_l != 0.0f || cmd_r != 0.0f) {
      cmd_l = 0.0f;
      cmd_r = 0.0f;
      dxl_write_goal(0.0f, 0.0f);
    }
  }

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
    Serial.println(rt);

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
