// =====================================================
// BAKU Motor Control - Arduino UNO 3-Shield Test
// Version: 4500 RPM + Actual Motor RPM CAN Feedback -> UDP 20Hz
// Date: 2026-10-22
// Board: Arduino UNO + Ethernet Shield + MCP2515 CAN Shield
// Status: Source only; Arduino build and vehicle test NOT verified
// Change summary: Target RPM limit +/-4500; measured motor RPM UDP <=20Hz
// PC AUTO packet: <ffB> (float32 steering, float32 speed -100..100, uint8 misc)
// NOTE: 500 A current reference remains unchanged from previous code.
// Safety: Verify physical speed/current limits and brakes before driving.
// =====================================================

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <mcp2515_can.h>
#include <math.h>
#include <string.h>



// ===============================
// SPI CS 핀
// ===============================
const int ETH_CS_PIN = 10;
const int CAN_CS_PIN = 9;
const int SD_CS_PIN  = 4;

// ===============================
// Ethernet 설정
// ===============================
IPAddress localIp(10, 177, 21, 2);

byte mac[] = {
  0xA8, 0x61, 0x0A,
  0xAE, 0x21, 0x68
};

IPAddress pcIP(10, 177, 21, 1);
IPAddress motorIP(10, 177, 21, 2);
IPAddress brakeIP(10, 177, 21, 3);
IPAddress steerIP(10, 177, 21, 4);
IPAddress asmsIP(10, 177, 21, 5);

const unsigned int localPort = 5000;

EthernetUDP Udp;     // 명령 수신: 5000
EthernetUDP RpmUdp;  // RPM 송신: 5001

// ===============================
// CAN 설정
// ===============================
mcp2515_can CAN(CAN_CS_PIN);

// 모터 컨트롤러 → 아두이노
const unsigned long MCU_ID = 0x1801D0EF;

// 아두이노 → 모터 컨트롤러
const unsigned long VCU_ID = 0x0C01EFD0;

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
const int16_t SPEED_MAX_RPM = 4500;

// AUTO speed 입력 범위
const float AUTO_SPEED_MIN = -100.0f;
const float AUTO_SPEED_MAX = 100.0f;

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
const unsigned long UDP_TIMEOUT_MS = 1000;

unsigned long lastAsmsReceiveTime = 0;
unsigned long lastPcReceiveTime = 0;
bool hasAsmsPacket = false;
bool hasPcAutoPacket = false;

// ===============================
// 모터 명령 송신 설정
// ===============================

// 50ms마다 모터 CAN 명령 반복 송신
const unsigned long MOTOR_SEND_PERIOD_MS = 50;

unsigned long lastMotorSendTime = 0;

// ===============================
// 실제 모터 RPM CAN 수신 -> PC UDP 송신 (20Hz)
// ===============================
const unsigned int RPM_UDP_SOURCE_PORT = 5001;
const unsigned int RPM_UDP_DEST_PORT = 5001;
const unsigned long RPM_UDP_SEND_PERIOD_MS = 50;

unsigned long lastRpmUdpSendTime = 0;
int32_t actual_motor_rpm = 0;          // CAN 피드백으로 측정된 실제 모터 RPM
uint32_t actual_rpm_timestamp_ms = 0; // 피드백을 수신하고 RPM을 계산한 millis()
bool hasNewActualRpm = false;          // 새로운 실제 RPM 수신 여부

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

// ===============================
// 함수 선언
// ===============================
void spiSelectNone();
void spiSelectEthernet();
void spiSelectCAN();

void processCANHandshake();
void sendHandshakeResponse();
void sendActualRpmUdp();

void processUdpPacket();
void enforceControlTimeout();

void handleManualMode(int16_t joy_x, int16_t joy_y);
void handleAutoMode(float steer, float speed, uint8_t misc);
void handleEstop();

uint16_t enc_current(float current_A);
uint16_t enc_speed(int16_t rpm);

void sendMotorCommand_CAN(
  int16_t speed_rpm,
  float phase_current_A,
  bool enable_cmd
);

void sendMotorStop_CAN();

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  Serial.println(
    "=== UDP JOYSTICK SPEED CONTROL START ==="
  );

  pinMode(ETH_CS_PIN, OUTPUT);
  pinMode(CAN_CS_PIN, OUTPUT);
  pinMode(SD_CS_PIN, OUTPUT);

  // 모든 SPI 장치 선택 해제
  spiSelectNone();

  // ===============================
  // Ethernet 시작
  // ===============================
  spiSelectEthernet();

  Ethernet.begin(mac, localIp);
  Udp.begin(localPort);
  RpmUdp.begin(RPM_UDP_SOURCE_PORT);

  spiSelectNone();

  Serial.print("Local IP: ");
  Serial.println(Ethernet.localIP());

  // ===============================
  // CAN 시작
  // ===============================
  spiSelectCAN();

  if (CAN.begin(CAN_250KBPS, MCP_16MHz) == CAN_OK) {
    Serial.println("CAN Init OK");
  } else {
    Serial.println("CAN Init Fail");

    spiSelectNone();

    while (1) {
      delay(1000);
    }
  }

  spiSelectNone();

  Serial.println("Waiting Handshake...");

  // ASMS 모드 지시를 받기 전에는 HALT 상태 유지
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  // 1. CAN 핸드셰이크 확인
  processCANHandshake();

  // 2. 만료된 모드를 먼저 해제한 뒤 UDP 패킷 처리
  enforceControlTimeout();
  processUdpPacket();

  // 3. 수신 처리 중 경과한 시간까지 반영
  enforceControlTimeout();

  // 4. 모터 명령을 50ms마다 반복 전송
  if (
    millis() - lastMotorSendTime >=
    MOTOR_SEND_PERIOD_MS
  ) {
    lastMotorSendTime = millis();

    if (handshake_ok) {
      sendMotorCommand_CAN(
        target_speed_rpm,
        PHASE_CURRENT_REF_A,
        motor_enable_cmd
      );
    }
  }

  // 5. 새로 수신한 실제 RPM만 PC로 최대 20Hz UDP 전송
  sendActualRpmUdp();
}

// =====================================================
// 수신 시간 초과 처리
// 자동 모드에서는 ASMS와 PC가 모두 500ms 이내에 수신되어야 함
// =====================================================
void enforceControlTimeout() {
  unsigned long now = millis();
  if (!hasAsmsPacket || now - lastAsmsReceiveTime > UDP_TIMEOUT_MS) {
    receivedMode = 0;
    hasPcAutoPacket = false;
    target_speed_rpm = 0;
    motor_enable_cmd = false;
    return;
  }

  if (receivedMode == 1 &&
      (!hasPcAutoPacket || now - lastPcReceiveTime > UDP_TIMEOUT_MS)) {
    hasPcAutoPacket = false;
    target_speed_rpm = 0;
    motor_enable_cmd = false;
  }
}

// =====================================================
// SPI 선택 함수
// =====================================================
void spiSelectNone() {
  digitalWrite(ETH_CS_PIN, HIGH);
  digitalWrite(CAN_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
}

void spiSelectEthernet() {
  digitalWrite(CAN_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(ETH_CS_PIN, LOW);
}

void spiSelectCAN() {
  digitalWrite(ETH_CS_PIN, HIGH);
  digitalWrite(SD_CS_PIN, HIGH);
  digitalWrite(CAN_CS_PIN, LOW);
}

// =====================================================
// UDP 패킷 처리
// =====================================================
void processUdpPacket() {
  spiSelectEthernet();

  int packetSize = Udp.parsePacket();

  if (packetSize <= 0) {
    spiSelectNone();
    return;
  }

  IPAddress senderIP = Udp.remoteIP();

  // ASMS는 모드/수동 입력, PC는 자동 속도 입력을 전송
  if (senderIP != asmsIP && senderIP != pcIP) {
    while (Udp.available()) {
      Udp.read();
    }

    spiSelectNone();
    return;
  }

  // ===============================
  // ASMS의 5바이트 모드 / MANUAL 패킷
  //
  // Byte 0   : mode
  // Byte 1~2 : joy_x Little Endian
  // Byte 3~4 : joy_y Little Endian
  // ===============================
  if (senderIP == asmsIP && packetSize == 5) {
    uint8_t buffer[5];

    int readLength = Udp.read(
      buffer,
      sizeof(buffer)
    );

    spiSelectNone();

    if (readLength != 5) {
      return;
    }

    // ASMS 패킷으로만 모드를 변경하며 PC 수신 시간은 갱신하지 않음
    lastAsmsReceiveTime = millis();
    hasAsmsPacket = true;
    uint8_t previousMode = receivedMode;
    receivedMode = buffer[0];
    if (receivedMode != 1) {
      hasPcAutoPacket = false;
    }

    int16_t joy_x = (int16_t)(
      ((uint16_t)buffer[1]) |
      ((uint16_t)buffer[2] << 8)
    );

    int16_t joy_y = (int16_t)(
      ((uint16_t)buffer[3]) |
      ((uint16_t)buffer[4] << 8)
    );

    switch (receivedMode) {
      // AUTO 진입 시 기존 명령을 정지하고 새 PC 패킷을 기다림
      // 같은 mode 1의 반복 수신은 현재 PC 속도 명령을 유지
      case 1:
        if (previousMode != 1) {
          hasPcAutoPacket = false;
          sendMotorStop_CAN();
          Serial.println(F("[MODE] AUTO: waiting for PC speed"));
        }
        break;

      // MANUAL
      case 2:
        handleManualMode(joy_x, joy_y);
        break;

      // ESTOP
      case 3:
        handleEstop();
        break;

      // 그 외 모드는 정지
      default:
        sendMotorStop_CAN();
        break;
    }

    return;
  }

  // ===============================
  // PC의 9바이트 AUTO 패킷 (ASMS가 자동 모드를 지정한 경우만 적용)
  //
  // Byte 0~3 : steer float32 Little Endian
  // Byte 4~7 : speed float32 Little Endian (-100..100)
  // Byte 8   : misc
  // ===============================
  if (senderIP == pcIP && packetSize == 9) {
    uint8_t autoBuffer[9];

    int readLength = Udp.read(
      autoBuffer,
      sizeof(autoBuffer)
    );

    spiSelectNone();

    if (readLength != 9) {
      return;
    }

    // 수동/ESTOP/ASMS 시간 초과 상태에서는 PC 명령을 무시
    unsigned long now = millis();
    if (receivedMode != 1 || !hasAsmsPacket ||
        now - lastAsmsReceiveTime > UDP_TIMEOUT_MS) {
      return;
    }
    // Arduino UNO: 4-byte IEEE-754 float, Little Endian.
    float steer;
    float speed;
    memcpy(&steer, autoBuffer, sizeof(steer));
    memcpy(&speed, autoBuffer + 4, sizeof(speed));

    uint8_t misc = autoBuffer[8];

    if (!isfinite(speed)) {
      return;
    }

    lastPcReceiveTime = now;
    hasPcAutoPacket = true;

    handleAutoMode(steer, speed, misc);

    return;
  }

  // 잘못된 크기의 패킷은 모두 버림
  while (Udp.available()) {
    Udp.read();
  }

  spiSelectNone();
}

// =====================================================
// CAN 핸드셰이크 처리
// =====================================================
void processCANHandshake() {
  spiSelectCAN();

  unsigned char len = 0;
  unsigned char buf[8] = {0};

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);

    unsigned long canId = CAN.getCanId();

    // 핸드셰이크는 0x55 8개 모두 일치할 때만 인정
    // (RPM 피드백의 첫 바이트가 우연히 0x55인 경우 구분)
    bool isHandshakeFrame = (canId == MCU_ID && len == 8);
    if (isHandshakeFrame) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (buf[i] != 0x55) {
          isHandshakeFrame = false;
          break;
        }
      }
    }

    if (isHandshakeFrame) {
      sendHandshakeResponse();
    }
    // Golden Motor Protocol 2: MCU -> VCU 피드백 Part I
    // CAN ID: 0x1801D0EF, 8바이트
    // Byte 6: Speed Raw Low / Byte 7: Speed Raw High
    // 실제 RPM = (Byte7 << 8 | Byte6) - 32000
    else if (canId == MCU_ID && len == 8 && handshake_ok) {
      uint16_t speedRaw =
        (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

      actual_motor_rpm = (int32_t)speedRaw - 32000L;
      actual_rpm_timestamp_ms = (uint32_t)millis();
      hasNewActualRpm = true;
    }
  }

  spiSelectNone();
}

// =====================================================
// 핸드셰이크 응답
// =====================================================
void sendHandshakeResponse() {
  unsigned char handshakeData[8] = {
    0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA
  };

  spiSelectCAN();

  byte status = CAN.sendMsgBuf(
    VCU_ID,
    1,
    0,
    8,
    handshakeData
  );

  spiSelectNone();

  if (status == CAN_OK) {
    handshake_ok = true;

    Serial.println(
      ">>> Handshake OK! Motor Control Start!"
    );
  } else {
    handshake_ok = false;

    Serial.println("Handshake Send Error");
  }
}

// =====================================================
// MANUAL 모드
// =====================================================
void handleManualMode(
  int16_t joy_x,
  int16_t joy_y
) {
  long absJoy = labs((long)joy_x);

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
    // 조이스틱을 끝까지 밀면 4500rpm
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
// 9바이트 float32 speed를 -100~100으로 제한하고 최종 RPM만 반올림
// =====================================================
void handleAutoMode(
  float steer,
  float speed,
  uint8_t misc
) {
  // 조향과 misc는 모터 명령에 사용하지 않음
  (void)steer;
  (void)misc;

  float speedLimited = speed;
  if (speedLimited > AUTO_SPEED_MAX) {
    speedLimited = AUTO_SPEED_MAX;
  }
  if (speedLimited < AUTO_SPEED_MIN) {
    speedLimited = AUTO_SPEED_MIN;
  }

  // 양수 speed: 전진(음수 RPM), 음수 speed: 후진(양수 RPM)
  target_speed_rpm = (int16_t)lroundf(
    -speedLimited * (float)SPEED_MAX_RPM / AUTO_SPEED_MAX
  );
  motor_enable_cmd = true;

  // AUTO 패킷 수신 시 원본 speed와 적용값을 10진수로 출력
  Serial.print(F("[AUTO] speedRaw="));
  Serial.print(speed, 6);
  Serial.print(F(" speedLimited="));
  Serial.print(speedLimited, 6);
  Serial.print(F(" targetRPM="));
  Serial.println(target_speed_rpm, DEC);
}

// =====================================================
// ESTOP
// =====================================================
void handleEstop() {
  sendMotorStop_CAN();
}

// =====================================================
// Protocol 2 Current 인코딩
//
// raw = (A + 3200) / 0.1
// =====================================================
uint16_t enc_current(float current_A) {
  long raw = (long)(
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
  long raw = (long)rpm + 32000L;

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
// - 양수 RPM, 양수 Current → ㅁ 후진
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

  spiSelectCAN();

  byte status = CAN.sendMsgBuf(
    VCU_ID,
    1,
    0,
    8,
    commandData
  );

  spiSelectNone();

  // 실패할 때만 출력
  if (status != CAN_OK) {
    Serial.println("[MOTOR TX FAIL]");
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

// =====================================================
// 실제 모터 RPM PC로 UDP 전송 (최대 20Hz)
// 패킷: int32_t RPM 4B + uint32_t timestamp_ms 4B
// Little Endian, 합계 8바이트, valid 필드 없음
// 새 CAN RPM 피드백이 들어온 경우에만 전송 (이전값 반복 송신 안 함)
// =====================================================
void sendActualRpmUdp() {
  if (!hasNewActualRpm) {
    return;
  }

  unsigned long now = millis();
  if (now - lastRpmUdpSendTime < RPM_UDP_SEND_PERIOD_MS) {
    return;
  }
  lastRpmUdpSendTime = now;

  uint32_t rpmBits = (uint32_t)actual_motor_rpm;
  uint32_t timeBits = actual_rpm_timestamp_ms;
  uint8_t packet[8];

  for (uint8_t i = 0; i < 4; ++i) {
    packet[i] = (uint8_t)(rpmBits >> (i * 8));
    packet[i + 4] = (uint8_t)(timeBits >> (i * 8));
  }

  spiSelectEthernet();
  if (RpmUdp.beginPacket(pcIP, RPM_UDP_DEST_PORT)) {
    RpmUdp.write(packet, sizeof(packet));
    if (RpmUdp.endPacket() == 1) {
      hasNewActualRpm = false;
    }
  }
  spiSelectNone();
}
