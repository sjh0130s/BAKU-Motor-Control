/* Bare-metal AVR C port of MotorController.ino; no Arduino core/libraries.
 * Target: ATmega328P-AU, external 16 MHz, 5 V; source schematic 2026-09-20.
 * Polling CAN/UDP, Timer0 1 ms tick, UART 115200 8N1.
 */
#include "hal.h"
#include <stdlib.h>
#include <string.h>
static const uint32_t MCU_ID = 0x1801D0EFUL;
static const uint32_t VCU_ID = 0x0C01EFD0UL;
static bool ethernet_fault;

// 실제 RPM 피드백: MCU message I, byte 6~7, little endian, raw - 32000.
// UDP 목적지 10.177.21.1:5001, 20ms 주기(목표 50Hz), 8바이트
// <int32_t actual_motor_rpm, uint32_t sample_timestamp_ms>, little endian.
// timestamp는 모터 CAN 데이터를 파싱한 순간의 PCB millis() (부팅 이후 ms).
// 실제 CAN 업데이트가 더 느리면 동일 샘플/타임스탬프가 반복될 수 있음.
static const uint8_t SPEED_PC_IP[4] = {10, 177, 21, BAKU_PC_IP_LAST_OCTET};
static const uint32_t SPEED_SEND_PERIOD_MS = 20;
static const uint32_t SPEED_FEEDBACK_TIMEOUT_MS = 500;
static int32_t actual_speed_rpm;
static uint32_t lastSpeedReceiveTime;
static uint32_t lastSpeedSampleTimeMs;
static uint32_t lastSpeedSendTime;
static bool hasSpeedFeedback;
static void processPcSpeedFeedback(void);
// ===============================
// 조이스틱 설정
// ===============================
const int16_t JOY_DEADBAND = 120;
const float JOY_MAX_ABS = 2048.0f;

// 조이스틱을 앞으로 밀었을 때
// joy_x가 음수로 들어오므로 true
const bool JOY_FORWARD_IS_NEGATIVE = true;

// ===============================
// 모터 속도 설정
// ===============================

// 조이스틱을 끝까지 밀었을 때 최대 RPM
const int16_t SPEED_MAX_RPM = 1500;

// AUTO speed 입력 범위
const int32_t AUTO_SPEED_MIN = -100;
const int32_t AUTO_SPEED_MAX = 100;

// ===============================
// 모터 전류 설정
// ===============================

// EZ-Tune의 Phase Current Reference에 대응하는 값
// 출발과 주행 모두 항상 같은 값 사용
const float PHASE_CURRENT_REF_A = 500.0f;

// ===============================
// 조향 설정
// 현재 코드에서는 계산만 하고 전송하지 않음
// ===============================
const int STEER_CENTER_CMD = 90;
const int STEER_LEFT_CMD   = 60;
const int STEER_RIGHT_CMD  = 120;

// ===============================
// UDP 안전 설정
// ===============================

// ASMS 모드/수동 패킷과 PC 자동 패킷의 수신 시간을 각각 검사
const uint32_t UDP_TIMEOUT_MS = 1000;

uint32_t lastAsmsReceiveTime = 0;
uint32_t lastPcReceiveTime = 0;
bool hasAsmsPacket = false;
bool hasPcAutoPacket = false;

// ===============================
// 모터 명령 송신 설정
// ===============================

// 50ms마다 모터 CAN 명령 반복 송신
const uint32_t MOTOR_SEND_PERIOD_MS = 50;

uint32_t lastMotorSendTime = 0;

// ===============================
// 상태 변수
// ===============================
bool handshake_ok = false;

uint8_t life = 0;
uint8_t receivedMode = 0;

// 조이스틱으로 계산된 목표 RPM
int16_t target_speed_rpm = 0;

// true  : RUNNING
// false : HALT
bool motor_enable_cmd = false;

/* Final PCB: external pull-ups on PD3/PD4, active-low switch.
 * 30ms debounce; transition immediately invalidates remote drive commands.
 */
enum { HW_THROTTLE = 0, HW_MANUAL = 1, HW_AUTO = 2, HW_INVALID = 3 };
static uint8_t hw_mode = HW_INVALID;
static uint8_t mode_candidate = 0xFF;
static uint32_t mode_candidate_since;
static bool estop_latched;
static const uint32_t MODE_DEBOUNCE_MS = 30;
void sendMotorStop_CAN(void);

static void clear_remote_control(void) {
  target_speed_rpm = 0;
  motor_enable_cmd = false;
  receivedMode = 0;
  hasAsmsPacket = false;
  hasPcAutoPacket = false;
}

static void update_hardware_mode(void) {
  uint8_t raw = read_mode_pins();
  uint32_t now = millis();
  if (raw > HW_INVALID) raw = HW_INVALID;
  if (raw != mode_candidate) {
    mode_candidate = raw;
    mode_candidate_since = now;
    hw_mode = HW_INVALID;
    clear_remote_control();
    if (handshake_ok) sendMotorStop_CAN();
  }
  if (hw_mode == HW_INVALID && mode_candidate != HW_INVALID &&
      (uint32_t)(now - mode_candidate_since) >= MODE_DEBOUNCE_MS) {
    hw_mode = mode_candidate;
    clear_remote_control();
    if (hw_mode == HW_THROTTLE) log_text("[SW] THROTTLE (CAN commands inhibited)\r\n");
    else if (hw_mode == HW_MANUAL) log_text("[SW] MANUAL\r\n");
    else log_text("[SW] AUTO\r\n");
  }
}

// ===============================
// 함수 선언
// ===============================

void processCANHandshake();
void sendHandshakeResponse();

void processUdpPacket();
void enforceControlTimeout();

void handleManualMode(int16_t joy_x, int16_t joy_y);
void handleAutoMode(int32_t steer, int32_t speed, uint8_t misc);
void handleEstop();

uint16_t enc_current(float current_A);
uint16_t enc_speed(int16_t rpm);

void sendMotorCommand_CAN(
  int16_t speed_rpm,
  float phase_current_A,
  bool enable_cmd
);

void sendMotorStop_CAN();

void enforceControlTimeout() {
  uint32_t now = millis();
  if (estop_latched || hw_mode == HW_INVALID || hw_mode == HW_THROTTLE ||
      (hw_mode == HW_MANUAL && receivedMode != 2) ||
      (hw_mode == HW_AUTO && receivedMode != 1)) {
    target_speed_rpm = 0;
    motor_enable_cmd = false;
    return;
  }
  if (!hasAsmsPacket || (uint32_t)(now - lastAsmsReceiveTime) > UDP_TIMEOUT_MS) {
    receivedMode = 0;
    hasPcAutoPacket = false;
    target_speed_rpm = 0;
    motor_enable_cmd = false;
    return;
  }

  if (receivedMode == 1 &&
      (!hasPcAutoPacket || (uint32_t)(now - lastPcReceiveTime) > UDP_TIMEOUT_MS)) {
    hasPcAutoPacket = false;
    target_speed_rpm = 0;
    motor_enable_cmd = false;
  }
}

void handleManualMode(
  int16_t joy_x,
  int16_t joy_y
) {
  int32_t absJoy = joy_x < 0 ? -(int32_t)joy_x : (int32_t)joy_x;

  // ===============================
  // 조이스틱 중립
  // ===============================
  if (absJoy <= JOY_DEADBAND) {
    target_speed_rpm = 0;

    // HALT가 아닌 RUNNING + 0rpm
    motor_enable_cmd = true;
  }

  // ===============================
  // 전진 또는 후진
  // ===============================
  else {
    float ratio =
      ((float)absJoy - (float)JOY_DEADBAND) /
      (JOY_MAX_ABS - (float)JOY_DEADBAND);

    if (ratio < 0.0f) {
      ratio = 0.0f;
    }

    if (ratio > 1.0f) {
      ratio = 1.0f;
    }

    // 최소 RPM 강제 기능 없음
    //
    // 조이스틱을 조금 밀면 낮은 RPM
    // 조이스틱을 끝까지 밀면 1500rpm
    int16_t speed_rpm = (int16_t)(
      ratio * (float)SPEED_MAX_RPM
    );

    bool forward_cmd = false;
    bool reverse_cmd = false;

    // 조이스틱을 앞으로 밀면 joy_x 음수
    if (JOY_FORWARD_IS_NEGATIVE) {
      forward_cmd =
        joy_x < -JOY_DEADBAND;

      reverse_cmd =
        joy_x > JOY_DEADBAND;
    } else {
      forward_cmd =
        joy_x > JOY_DEADBAND;

      reverse_cmd =
        joy_x < -JOY_DEADBAND;
    }

    // =============================================
    // 실제 차량 방향
    //
    // 음수 RPM → 차량 전진
    // 양수 RPM → 차량 후진
    // =============================================
    if (forward_cmd) {
      // 조이스틱 앞으로 → 차량 전진
      target_speed_rpm = -speed_rpm;
      motor_enable_cmd = true;
    }

    else if (reverse_cmd) {
      // 조이스틱 뒤로 → 차량 후진
      target_speed_rpm = speed_rpm;
      motor_enable_cmd = true;
    }

    else {
      target_speed_rpm = 0;
      motor_enable_cmd = true;
    }
  }

  // ===============================
  // 조향값 계산
  // 현재는 실제로 전송하지 않음
  // ===============================
  int steer_cmd = STEER_CENTER_CMD;

  if (joy_y < -JOY_DEADBAND) {
    steer_cmd = STEER_LEFT_CMD;
  }

  else if (joy_y > JOY_DEADBAND) {
    steer_cmd = STEER_RIGHT_CMD;
  }

  else {
    steer_cmd = STEER_CENTER_CMD;
  }

  // 현재는 조향 전송 기능이 없으므로
  // 사용하지 않는 변수 경고 방지
  (void)steer_cmd;
}

// =====================================================
// AUTO 모드
// 9바이트 speed를 -100~100으로 제한하고 실제 모터 RPM으로 변환
// =====================================================
void handleAutoMode(
  int32_t steer,
  int32_t speed,
  uint8_t misc
) {
  // 조향과 misc는 모터 명령에 사용하지 않음
  (void)steer;
  (void)misc;

  int32_t speedLimited = speed;
  if (speedLimited > AUTO_SPEED_MAX) {
    speedLimited = AUTO_SPEED_MAX;
  }
  if (speedLimited < AUTO_SPEED_MIN) {
    speedLimited = AUTO_SPEED_MIN;
  }

  // 양수 speed: 전진(음수 RPM), 음수 speed: 후진(양수 RPM)
  target_speed_rpm = (int16_t)(
    -(speedLimited * (int32_t)SPEED_MAX_RPM) / AUTO_SPEED_MAX
  );
  motor_enable_cmd = true;

  // AUTO 패킷 수신 시 원본 speed와 적용값을 10진수로 출력
  log_auto(speed, speedLimited, target_speed_rpm);
}

// =====================================================
// ESTOP
// =====================================================
void handleEstop() {
  /* Software ESTOP stays latched until reboot; external E-STOP required. */
  estop_latched = true;
  clear_remote_control();
  if (handshake_ok) sendMotorStop_CAN();
}

// =====================================================
// Protocol 2 Current 인코딩
//
// raw = (A + 3200) / 0.1
// =====================================================
uint16_t enc_current(float current_A) {
  int32_t raw = (int32_t)(
    (current_A + 3200.0f) / 0.1f
  );

  if (raw < 0) {
    raw = 0;
  }

  if (raw > 65535) {
    raw = 65535;
  }

  return (uint16_t)raw;
}

// =====================================================
// Protocol 2 Speed 인코딩
//
// raw = RPM + 32000
// =====================================================
uint16_t enc_speed(int16_t rpm) {
  int32_t raw = (int32_t)rpm + 32000L;

  if (raw < 0) {
    raw = 0;
  }

  if (raw > 65535) {
    raw = 65535;
  }

  return (uint16_t)raw;
}

// =====================================================
// 모터 CAN 명령 송신
//
// 실제 차량 방향:
// - 음수 RPM, 음수 Current → 차량 전진
// - 양수 RPM, 양수 Current → 차량 후진
//
// Byte 0~1: Signed Phase Current Reference
// Byte 2~3: Target Rotor Speed
// Byte 4   : 제어모드
// Byte 7   : Life Signal
// =====================================================
void sendMotorCommand_CAN(
  int16_t speed_rpm,
  float phase_current_A,
  bool enable_cmd
) {
  float signed_current_A = 0.0f;

  if (enable_cmd) {
    if (speed_rpm > 0) {
      // 실제 차량 후진
      signed_current_A = phase_current_A;
    }

    else if (speed_rpm < 0) {
      // 실제 차량 전진
      signed_current_A = -phase_current_A;
    }

    else {
      signed_current_A = 0.0f;
    }
  } else {
    signed_current_A = 0.0f;
  }

  uint16_t currentRaw =
    enc_current(signed_current_A);

  uint16_t speedRaw =
    enc_speed(speed_rpm);

  unsigned char commandData[8] = {0};

  // Byte 0~1: Current, Little Endian
  commandData[0] =
    (uint8_t)(currentRaw & 0xFF);

  commandData[1] =
    (uint8_t)(currentRaw >> 8);

  // Byte 2~3: Speed, Little Endian
  commandData[2] =
    (uint8_t)(speedRaw & 0xFF);

  commandData[3] =
    (uint8_t)(speedRaw >> 8);

  // Byte 4
  if (enable_cmd) {
    // RUNNING + Speed Control Mode
    commandData[4] = 0x03;
  } else {
    // HALT
    commandData[4] = 0x00;
  }

  commandData[5] = 0;
  commandData[6] = 0;

  // Byte 7: Life Signal
  commandData[7] = life++;

  if (!can_send(VCU_ID, commandData)) {
    log_text("[MOTOR TX FAIL]\r\n");
  }
}

// =====================================================
// 모터 정지
// =====================================================
void sendMotorStop_CAN() {
  target_speed_rpm = 0;
  motor_enable_cmd = false;

  sendMotorCommand_CAN(
    0,
    PHASE_CURRENT_REF_A,
    false
  );
}

static int16_t get_i16(const uint8_t *p) {
  uint16_t u = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  return u <= 0x7FFF ? (int16_t)u : (int16_t)(-1 - (int32_t)(0xFFFFU - u));
}
static int32_t get_i32(const uint8_t *p) {
  uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
             | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  return u <= INT32_MAX ? (int32_t)u : -1 - (int32_t)(UINT32_MAX - u);
}
void processUdpPacket(void) {
  uint8_t ip[4], data[9];
  uint16_t len;
  if (ethernet_fault) return;
  int8_t result = udp_receive(ip, data, sizeof(data), &len);
  if (result < 0) {
    ethernet_fault = true;
    hasAsmsPacket = false;
    hasPcAutoPacket = false;
    receivedMode = 0;
    sendMotorStop_CAN();
    log_text("W5500 fault; reset required\r\n");
    return;
  }
  if (result == 0) return;
  const uint8_t asms[4] = {10,177,21,5};
  const uint8_t pc[4] = {10,177,21,BAKU_PC_IP_LAST_OCTET};
  /* ASMS ESTOP accepts 3B or 5B; it must originate from ASMS IP. */
  if (memcmp(ip, asms, 4) == 0 &&
      (len == 3 || len == 5) && data[0] == 3) {
    handleEstop();
    return;
  }
  if (estop_latched) return;
  if (memcmp(ip, asms, 4) == 0 && len == 5) {
    /* Physical selector is authoritative over UDP mode. */
    if ((hw_mode == HW_MANUAL && data[0] != 2) ||
        (hw_mode == HW_AUTO && data[0] != 1) ||
        (hw_mode != HW_MANUAL && hw_mode != HW_AUTO)) {
      clear_remote_control();
      return;
    }
    lastAsmsReceiveTime = millis();
    hasAsmsPacket = true;
    uint8_t previousMode = receivedMode;
    receivedMode = data[0];
    if (receivedMode != 1) hasPcAutoPacket = false;
    int16_t joy_x = get_i16(data+1), joy_y = get_i16(data+3);
    switch (receivedMode) {
      case 1:
        if (previousMode != 1) {
          hasPcAutoPacket = false;
          sendMotorStop_CAN();
          log_text("[MODE] AUTO: waiting for PC speed\r\n");
        }
        break;
      case 2: handleManualMode(joy_x, joy_y); break;
      case 3: handleEstop(); break;
      default: sendMotorStop_CAN(); break;
    }
  } else if (hw_mode == HW_AUTO && memcmp(ip, pc, 4) == 0 && len == 9) {
    uint32_t now = millis();
    if (receivedMode != 1 || !hasAsmsPacket ||
        (uint32_t)(now-lastAsmsReceiveTime) > UDP_TIMEOUT_MS) return;
    lastPcReceiveTime = now;
    hasPcAutoPacket = true;
    handleAutoMode(get_i32(data), get_i32(data+4), data[8]);
  }
}
void processCANHandshake(void) {
  CanFrame frame;
  if (!can_receive(&frame) || frame.id != MCU_ID || !frame.extended ||
      frame.remote || frame.len != 8) return;

  // 핸드셰이크와 속도 피드백은 같은 ID를 사용한다.
  // 첫 바이트만 검사하면 정상 BUS Voltage도 0x55일 수 있으므로
  // 매뉴얼대로 8바이트 전체가 0x55인지 확인한다.
  bool isHandshake = true;
  for (uint8_t i = 0; i < 8; ++i) {
    if (frame.data[i] != 0x55) { isHandshake = false; break; }
  }
  if (isHandshake) {
    hasSpeedFeedback = false;
    sendHandshakeResponse();
    return;
  }

  /* Ignore telemetry before the controller handshake completes. */
  if (!handshake_ok) return;

  uint16_t raw = (uint16_t)frame.data[6] | ((uint16_t)frame.data[7] << 8);
  // 매뉴얼 범위 -32000~32000 RPM. 범위 밖 값은 유효한 속도로 보내지 않음.
  if (raw > 64000U) {
    hasSpeedFeedback = false;
    return;
  }
  actual_speed_rpm = (int32_t)raw - 32000L;
  lastSpeedReceiveTime = millis();
  lastSpeedSampleTimeMs = lastSpeedReceiveTime;
  hasSpeedFeedback = true;
}

static void processPcSpeedFeedback(void) {
  if (ethernet_fault) return;
  // UDP 전송 완료/ARP 응답을 기다리지 않고 다음 제어 루프로 진행한다.
  udp_speed_poll();
  uint32_t now = millis();
  if (hasSpeedFeedback &&
      (uint32_t)(now - lastSpeedReceiveTime) > SPEED_FEEDBACK_TIMEOUT_MS) {
    hasSpeedFeedback = false;
  }
  if ((uint32_t)(now - lastSpeedSendTime) < SPEED_SEND_PERIOD_MS) return;
  lastSpeedSendTime = now;

  // valid 필드를 제거했으므로 피드백이 없거나 오래되면 송신하지 않는다.
  // PC는 수신 타임아웃으로 피드백 손실을 감지해야 한다.
  if (!hasSpeedFeedback) return;
  uint32_t value = (uint32_t)actual_speed_rpm;
  uint32_t timestamp = lastSpeedSampleTimeMs;
  uint8_t data[8] = {
    (uint8_t)value, (uint8_t)(value >> 8),
    (uint8_t)(value >> 16), (uint8_t)(value >> 24),
    (uint8_t)timestamp, (uint8_t)(timestamp >> 8),
    (uint8_t)(timestamp >> 16), (uint8_t)(timestamp >> 24)
  };
  // 송신 중이라면 이번 주기를 생략한다. CAN 명령 처리를 막지 않는다.
  (void)udp_speed_try_send(SPEED_PC_IP, data, sizeof(data));
}
void sendHandshakeResponse(void) {
  const uint8_t data[8] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
  handshake_ok = can_send(VCU_ID, data);
  log_text(handshake_ok ? ">>> Handshake OK! Motor Control Start!\r\n" : "Handshake Send Error\r\n");
}
void control_step(void) {
  processCANHandshake();
  update_hardware_mode();
  enforceControlTimeout();
  processUdpPacket();
  enforceControlTimeout();
  uint32_t now = millis();
  if ((uint32_t)(now-lastMotorSendTime) >= MOTOR_SEND_PERIOD_MS) {
    lastMotorSendTime = now;
    if (handshake_ok && (hw_mode != HW_THROTTLE || estop_latched)) {
      sendMotorCommand_CAN(target_speed_rpm, PHASE_CURRENT_REF_A, motor_enable_cmd);
    }
  }
  processPcSpeedFeedback();
}
#ifndef UNIT_TEST
int main(void) {
  board_init();
  log_text("=== UDP JOYSTICK SPEED CONTROL START ===\r\n");
  if (!ethernet_init()) {
    log_text("W5500 / UDP Init Fail\r\n");
    for (;;) {}
  }
  log_text("Local IP: 10.177.21.2\r\n");
  if (!can_init()) {
    log_text("CAN Init Fail\r\n");
    for (;;) {}
  }
  log_text("CAN Init OK\r\nWaiting Handshake...\r\n");
  for (;;) control_step();
}
#endif
