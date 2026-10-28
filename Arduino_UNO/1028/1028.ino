// =====================================================
// BAKU Motor Control - Arduino UNO 3-Shield Test
// Version: ASMS-or-PC independent control + non-blocking RPM UDP + CAN independent
// Date: 2026-10-28 (ASMS/PC either source independent control)
// Based on: e7112d3 (float32 PC AUTO + +/-4500 RPM + actual RPM UDP <=20Hz)
// Board: Arduino UNO + Ethernet Shield + MCP2515 CAN Shield
// Status: SOURCE ONLY; compile and actual vehicle tests NOT verified
//
// ================== [동작 조건] =======================
// ASMS MANUAL(mode=2) 유효 명령만으로 구동 가능 (PC 불필요).
// PC AUTO(9B) 유효 명령만으로 구동 가능 (ASMS 선행 불필요).
// 둘 다 1000ms 이상 유효한 구동 명령이 없으면 HALT.
// ASMS MANUAL 입력이 유효하면 MANUAL이 PC보다 우선한다.
// MANUAL 입력이 1000ms 넘게 끊기면, 새 PC AUTO 패킷으로
// PC 단독 제어를 재개할 수 있다.
// ASMS AUTO(mode=1)는 선택 상태만 유지하며, 실제 속도 구동에는
// PC AUTO 속도 패킷이 반드시 필요하다.
// ASMS ESTOP(mode=3) 이후 PC로는 구동할 수 없다.
// ESTOP 해제는 ASMS에서 새 mode 1 또는 mode 2 수신 시에만 허용.
// ASMS가 mode=0/알 수 없는 값을 보내면 즉시 HALT.
// 이 경우 새 ASMS 모드 1 또는 2가 오기 전에는 PC 자동진입 차단.
// CAN 수신/핸드셰이크 및 PC로 RPM 50Hz UDP 전송 시도는
// 위 구동 허용 상태와 독립적으로 계속 수행한다.
// =====================================================

// ================== [데이터 포맷] ====================
// PC -> UNO UDP 10.177.21.2:5000 = 9 bytes (Little Endian)
//   float32 steer [4B] + float32 speed(실제 RPM, +/-891.387) [4B] + misc [1B]
//   speed 양수 -> 목표 RPM 음수, speed 음수 -> 목표 RPM 양수 (기존 부호 유지)
//   PC 송신부도 실제 RPM 단위 계약으로 함께 전환해야 함.
//   steer/misc는 모터 제어에 사용하지 않음.
// UNO -> PC UDP 10.177.21.1:5001 = 8 bytes (50Hz 목표, 20ms 간격)
//   GNSS 기준 보정 RPM int32 [4B] + 마지막 CAN 수신 millis() uint32 [4B]
//   모드/E-STOP/핸드셰이크와 무관하게 송신 시도; 동일한 RPM 반복 가능.
//   CAN RPM을 한번도 수신하지 않았다면 RPM=INT32_MIN(-2147483648) 전송.
//   한번이라도 수신했다면 CAN 피드백이 끊겨도 마지막 측정 RPM 반복 전송.
//   timestamp는 마지막 실제 CAN RPM 수신 시각이며 새 수신이 없으면 갱신하지 않음.
//   PC는 오래된 timestamp의 RPM을 최신 측정값으로 취급하지 말 것.
//   PC 전원 OFF/랜선 분리 시 패킷은 실제로 전달될 수 없으나 전송 시도는 유지.
//   UDP 전송 완료를 기다리지 않아 모터 CAN 처리 루프를 막지 않음.
//   ASMS MANUAL은 PC 불필요. PC 속도 명령을 쓰는 AUTO는 PC가 필요.
//
// ================== [안전/유지 항목] =================
// 물리 E-STOP/브레이크/전류 및 속도 제한 별도 검증 필수.
// 실제 목표 RPM +/-891.387은 기존 CAN 명령 +/-4500에 대응.
// 2026-10-23 합의: 실제 RPM = 컨트롤러 RPM * 0.198086.
// 같은 비율을 명령/피드백에 적용. Phase Current Reference 500A는 기존 설정 유지.
// CAN handshake가 성공해야 모터 CAN 제어 명령 전송.
// 실제 차량 대신 바퀴를 지면에서 띄운 통제 환경에서 먼저 검증.
// =====================================================

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <utility/w5100.h>  // W5100 SEND 상태를 대기 없이 확인 (Ethernet 2.x)
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

EthernetUDP Udp;  // 명령 수신: 5000

// 일반 EthernetUDP.endPacket()은 상대 PC가 없을 때 ARP/UDP 실패를
// 기다리는 동안 CAN 루프를 오래 막을 수 있다.
// W5100에 SEND 명령만 시작하고, SEND_OK/TIMEOUT은 다음 loop에서 확인한다.
// 이 클래스는 Arduino 공식 Ethernet 라이브러리 2.x API를 사용한다.
class NonBlockingRpmUdp : public EthernetUDP {
 public:
  bool socketReady() const {
    return sockindex < MAX_SOCK_NUM;
  }

  // true = 전송 '시작' 성공 (PC 수신 성공을 의미하지 않음)
  bool startSend(IPAddress dest, uint16_t port,
                 const uint8_t *data, size_t dataLen) {
    if (!socketReady()) return false;
    if (beginPacket(dest, port) != 1) return false;
    if (write(data, dataLen) != dataLen) return false;

    // Ethernet 라이브러리의 endPacket()을 호출하지 않는다.
    // 아래 SEND 명령만 시작한 뒤 즉시 돌아온다.
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    W5100.writeSnIR(sockindex, (uint8_t)(SnIR::SEND_OK | SnIR::TIMEOUT));
    W5100.execCmdSn(sockindex, Sock_SEND);
    SPI.endTransaction();
    return true;
  }

  // 0 = 아직 전송 중, 1 = 송신 완료, -1 = timeout, -2 = 소켓 없음
  int8_t pollSendResult() {
    if (!socketReady()) return -2;
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    uint8_t ir = W5100.readSnIR(sockindex);
    const uint8_t doneMask = (uint8_t)(SnIR::SEND_OK | SnIR::TIMEOUT);
    uint8_t done = ir & doneMask;
    if (done != 0) W5100.writeSnIR(sockindex, done);
    SPI.endTransaction();

    if (done & SnIR::TIMEOUT) return -1;
    if (done & SnIR::SEND_OK) return 1;
    return 0;
  }
};

NonBlockingRpmUdp RpmUdp;  // 실제 RPM 송신: 5001

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

// 수동/AUTO 공통 최대 실제 RPM (GNSS 기준)
const float SPEED_MAX_RPM = 891.387f;

// PC AUTO 입력은 백분율이 아닌 실제 목표 RPM
const float AUTO_SPEED_MIN = -SPEED_MAX_RPM;
const float AUTO_SPEED_MAX =  SPEED_MAX_RPM;

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

// PC 독립 제어: ASMS 이력이 있더라도 MANUAL 타임아웃 이후 허용.
// ESTOP / ASMS 정지 명령은 PC 단독 명령으로 우회하지 못한다.
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
// CAN 실제 RPM 피드백 -> GNSS 기준 보정 -> PC UDP 50Hz
// 제조사 프로토콜상 실제 CAN 피드백 주기는 기본 50ms (20Hz).
// CAN 피드백이 중단되어도 마지막 RPM을 유지해서 송신함.
// ===============================
const unsigned int RPM_UDP_SOURCE_PORT = 5001;
const unsigned int RPM_UDP_DEST_PORT = 5001;
const unsigned long RPM_UDP_SEND_PERIOD_MS = 20;  // 목표 50Hz
const int32_t RPM_UNAVAILABLE_SENTINEL = (-2147483647L - 1L);

// 통신 실패가 모터 제어를 막지 않도록 전송 상태만 추적한다.
const unsigned long RPM_TX_STUCK_MS = 150;       // W5100 송신 상태 비정상 지속 시 소켓 재시작
const unsigned long RPM_SOCKET_RETRY_MS = 1000;  // 소켓 확보 실패 후 재시도
bool rpmUdpSendPending = false;
unsigned long rpmUdpSendStartTime = 0;
unsigned long lastRpmUdpSocketRetryTime = 0;
unsigned long lastRpmUdpSendTime = 0;
int32_t actual_motor_rpm = 0;          // 최근 CAN에서 측정한 보정 RPM
uint32_t actual_rpm_timestamp_ms = 0;  // 마지막 RPM 피드백 CAN 수신시각
bool hasActualRpm = false;            // 실제 피드백을 받은 적 있는지

// ===============================
// 상태 변수
// ===============================
bool handshake_ok = false;
bool canInitialized = false;
const unsigned long CAN_INIT_RETRY_PERIOD_MS = 1000;
unsigned long lastCanInitAttemptTime = 0;
const uint8_t CAN_MAX_RX_FRAMES_PER_LOOP = 8;

uint8_t life = 0;
uint8_t receivedMode = 0;
bool asmsStopLatched = false;  // ASMS mode 0/기타 명시적 정지 상태

// 수동/AUTO 공통 실제 목표 RPM. CAN 인코딩 직전까지 소수 유지.
float target_speed_rpm = 0.0f;

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
void retryCANInitialization();
void sendHandshakeResponse();
void sendActualRpmUdp();

void processUdpPacket();
void enforceControlTimeout();

void handleManualMode(int16_t joy_x, int16_t joy_y);
void handleAutoMode(float steer, float speed, uint8_t misc);
void handleEstop();

uint16_t enc_current(float current_A);
uint16_t enc_speed(float rpm);

void sendMotorCommand_CAN(
  float speed_rpm,
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
  // PC가 꺼져도 기본 재시도(최대 약 1.6초)가 CAN을 방해하지 않도록.
  // 송신 자체는 별도 non-blocking 상태 머신으로 처리한다.
  Ethernet.setRetransmissionTimeout(20);
  Ethernet.setRetransmissionCount(1);
  Udp.begin(localPort);
  RpmUdp.begin(RPM_UDP_SOURCE_PORT);

  spiSelectNone();

  Serial.print("Local IP: ");
  Serial.println(Ethernet.localIP());

  // ===============================
  // CAN 시작
  // ===============================
  spiSelectCAN();

  canInitialized = (CAN.begin(CAN_250KBPS, MCP_16MHz) == CAN_OK);
  lastCanInitAttemptTime = millis();
  if (canInitialized) {
    Serial.println("CAN Init OK");
  } else {
    // CAN 실패 시에도 50Hz UDP 송신 루프는 계속 진행.
    Serial.println("CAN Init Fail: UDP active, CAN retry enabled");
  }

  spiSelectNone();

  Serial.println("Waiting Handshake...");

  // 첫 유효 PC AUTO 또는 ASMS MANUAL 명령 전에는 HALT
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  // 1. AUTO/MANUAL/ESTOP과 상관없이 CAN 수신 처리
  processCANHandshake();

  // 2. 실제 CAN 신호/PC 연결 유무와 무관하게 RPM 송신 시도 (대기 없음)
  sendActualRpmUdp();

  // 3. 만료된 입력을 먼저 해제한 뒤 UDP 패킷 처리
  enforceControlTimeout();
  processUdpPacket();

  // 4. 수신 처리 중 경과한 시간까지 반영
  enforceControlTimeout();

  // 5. 모터 명령을 50ms마다 반복 전송
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

  // 6. 초기 CAN 연결 실패 시, 정상 운용을 막지 않고 재초기화
  retryCANInitialization();

  // 7. 이전 UDP SEND 완료 상태를 다시 확인 (대기 없음)
  sendActualRpmUdp();
}

// =====================================================
// 독립 제어 입력 타임아웃 (1초)
// - 초기 상태 또는 ASMS AUTO: PC 유효 속도패킷이 없으면 HALT.
// - ASMS MANUAL: 조이스틱 패킷이 새로 들어오는 동안 우선 제어.
// - MANUAL 끊김: 안전하게 HALT 후 PC의 새 속도패킷 수락.
// - ESTOP, ASMS mode 0: PC 자동 구동 차단/모터 HALT.
// - CAN 송수신과 RPM UDP 송신은 모드와 무관하게 계속 실행.
// =====================================================
void enforceControlTimeout() {
  const unsigned long now = millis();

  if (receivedMode == 2) {
    // ASMS MANUAL 명령이 최신이면 PC보다 우선한다.
    if (hasAsmsPacket &&
        now - lastAsmsReceiveTime <= UDP_TIMEOUT_MS) {
      return;
    }
    // MANUAL 입력이 끊어졌음. 이전 조이스틱 값을 즉시 제거하고
    // 다음 PC 새 패킷을 기다린다 (과거 PC RPM 재사용 금지).
    receivedMode = 0;
    hasPcAutoPacket = false;
    target_speed_rpm = 0;
    motor_enable_cmd = false;
    return;
  }

  // ESTOP 및 ASMS의 명시적 STOP(mode 0/기타)은 우회 금지.
  // receivedMode==0은 초기 상태 또는 MANUAL 타임아웃에서도 쓰므로
  // 별도로 정지 래치를 확인한다.
  if (receivedMode == 3 || asmsStopLatched) {
    hasPcAutoPacket = false;
    target_speed_rpm = 0;
    motor_enable_cmd = false;
    return;
  }

  // ASMS AUTO(mode 1) 또는 ASMS 명령 없이 PC만 사용하는 상태,
  // 혹은 MANUAL이 끊어진 뒤의 PC 단독 모드.
  if (!hasPcAutoPacket ||
      now - lastPcReceiveTime > UDP_TIMEOUT_MS) {
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
    // ASMS가 명시적으로 AUTO/MANUAL을 새로 보낼 때만 STOP 래치 해제.
    if (receivedMode == 1 || receivedMode == 2) {
      asmsStopLatched = false;
    } else if (receivedMode != 3) {
      asmsStopLatched = true;
    }
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
  // PC의 9바이트 AUTO 패킷
  // ASMS 선행 이력이 없어도 단독 허용.
  // ASMS MANUAL / ESTOP / 명시적 STOP(mode 0) 우선.
  //
  // Byte 0~3 : steer float32 Little Endian
  // Byte 4~7 : speed float32 Little Endian (실제 목표 RPM)
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

    // ASMS 이력과 무관하게 PC 단독 속도명령을 허용한다.
    // 단, ASMS MANUAL이 살아 있거나, ESTOP/STOP 중에는 차단한다.
    // MANUAL 타임아웃 이후 PC가 새 패킷을 보내면 제어 재개 가능.
    if (receivedMode == 2 || receivedMode == 3 || asmsStopLatched) {
      return;
    }
    unsigned long now = millis();
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
// CAN 수신: 모드, E-STOP, handshake 상태와 상관없이 읽기.
// MCP2515 수신 버퍼 오버런 완화를 위해 최대 8프레임 처리.
// =====================================================
void processCANHandshake() {
  if (!canInitialized) {
    return;
  }

  for (uint8_t frame = 0; frame < CAN_MAX_RX_FRAMES_PER_LOOP; ++frame) {
    spiSelectCAN();

    if (CAN_MSGAVAIL != CAN.checkReceive()) {
      spiSelectNone();
      break;
    }

    unsigned char len = 0;
    unsigned char buf[8] = {0};
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();
    spiSelectNone();

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
    // Golden Motor Protocol 2 피드백 Part I, ID 0x1801D0EF:
    // Byte 6~7 LE = controller_rpm + 32000
    // handshake_ok와 관계없이 수신 가능한 피드백을 저장한다.
    else if (canId == MCU_ID && len == 8) {
      uint16_t speedRaw =
        (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

      const int32_t controller_rpm = (int32_t)speedRaw - 32000L;
      actual_motor_rpm = (int32_t)lroundf(
        (float)controller_rpm * 0.198086f
      );
      actual_rpm_timestamp_ms = (uint32_t)millis();
      hasActualRpm = true;
    }
  }
}

// =====================================================
// CAN 초기화에 실패했더라도 UDP 서비스를 중지하지 않고 재시도.
// CAN 버스 장애로 인한 수신 중단 자체는 하드웨어 점검 필요.
// =====================================================
void retryCANInitialization() {
  if (canInitialized) {
    return;
  }

  unsigned long now = millis();
  if (now - lastCanInitAttemptTime < CAN_INIT_RETRY_PERIOD_MS) {
    return;
  }
  lastCanInitAttemptTime = now;

  spiSelectCAN();
  if (CAN.begin(CAN_250KBPS, MCP_16MHz) == CAN_OK) {
    canInitialized = true;
    handshake_ok = false;
    Serial.println(F("CAN Init Recovered; waiting handshake"));
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
    // 조이스틱을 끝까지 밀면 실제 SPEED_MAX_RPM
    float speed_rpm = ratio * SPEED_MAX_RPM;

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
// 9바이트 float32 speed는 실제 RPM. 범위를 제한하고 CAN 인코딩 때 반올림.
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

  // PC가 보낸 실제 목표 RPM에 기존 부호 변환만 적용
  target_speed_rpm = -speedLimited;
  motor_enable_cmd = true;

  // AUTO 패킷 수신 시 원본 speed와 적용값을 10진수로 출력
  Serial.print(F("[AUTO] speedRaw="));
  Serial.print(speed, 6);
  Serial.print(F(" speedLimited="));
  Serial.print(speedLimited, 6);
  Serial.print(F(" targetRPM="));
  Serial.println(target_speed_rpm, 3);
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
// raw = round(실제 목표 RPM / 0.198086) + 32000
// =====================================================
uint16_t enc_speed(float rpm) {
  long raw = lroundf(rpm / 0.198086f) + 32000L;

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
// - 양수 RPM, 양수 Current → 후진
//
// Byte 0~1: Signed Phase Current Reference
// Byte 2~3: Target Rotor Speed
// Byte 4   : 제어모드
// Byte 7   : Life Signal
// =====================================================
void sendMotorCommand_CAN(
  float speed_rpm,
  float phase_current_A,
  bool enable_cmd
) {
  // CAN 장치 초기화가 안 된 상태에선 CAN 송신을 시도하지 않음.
  if (!canInitialized) {
    return;
  }

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
// GNSS 기준 보정 모터 RPM: 비블로킹 UDP (목표 50Hz)
//  - PC IP: 10.177.21.1, 목적 포트: 5001
//  - 페이로드: int32_t actual_rpm + uint32_t timestamp, 8B LE
//  - CAN 피드백이 없으면 마지막 측정값/시각 유지
//  - 첫 CAN 수신 이전에는 INT32_MIN, timestamp=0
//  - MANUAL / AUTO / ESTOP / CAN handshake 상태와 관계없이 송신 시도
//  - PC가 없으면 물리적인 수신은 불가능하므로 TIMEOUT 후 재시도
//  - EthernetUDP.endPacket()을 호출하지 않아서 CAN 주기를 막지 않음
// =====================================================
void sendActualRpmUdp() {
  const unsigned long now = millis();

  // 1) 이전 SEND의 결과를 짧은 레지스터 조회 한 번으로만 확인.
  if (rpmUdpSendPending) {
    spiSelectEthernet();
    const int8_t sendResult = RpmUdp.pollSendResult();
    spiSelectNone();

    if (sendResult != 0) {
      // SEND_OK와 TIMEOUT 모두 송신 대기 상태를 해제하고 다음 기회에 재시도.
      rpmUdpSendPending = false;
    } else if (now - rpmUdpSendStartTime >= RPM_TX_STUCK_MS) {
      // 이더넷 하드웨어가 SEND_OK/TIMEOUT을 내지 않는 드문 상태 복구.
      // PC OFF에 대한 일반 처리는 위 TIMEOUT에서 자동으로 진행됨.
      spiSelectEthernet();
      RpmUdp.stop();
      RpmUdp.begin(RPM_UDP_SOURCE_PORT);
      spiSelectNone();
      rpmUdpSendPending = false;
      lastRpmUdpSocketRetryTime = now;
      lastRpmUdpSendTime = now;
      return;
    }
  }

  // 송신 완료 확인 전에는 TX 버퍼를 덮어쓰지 않는다.
  if (rpmUdpSendPending) return;

  // 2) 송신 주기 제한: PC 연결 시 약 50Hz 목표.
  if (now - lastRpmUdpSendTime < RPM_UDP_SEND_PERIOD_MS) return;
  lastRpmUdpSendTime = now;

  // 3) 소켓이 없는 경우 1초 간격으로만 재시도, CAN은 계속 처리.
  if (!RpmUdp.socketReady()) {
    if (now - lastRpmUdpSocketRetryTime >= RPM_SOCKET_RETRY_MS) {
      lastRpmUdpSocketRetryTime = now;
      spiSelectEthernet();
      RpmUdp.begin(RPM_UDP_SOURCE_PORT);
      spiSelectNone();
    }
    if (!RpmUdp.socketReady()) return;
  }

  // 4) 이전 코드의 실제 RPM/타임스탬프 포맷을 그대로 유지.
  const int32_t rpmToSend =
    hasActualRpm ? actual_motor_rpm : RPM_UNAVAILABLE_SENTINEL;
  const uint32_t timeBits = hasActualRpm ? actual_rpm_timestamp_ms : 0;
  const uint32_t rpmBits = (uint32_t)rpmToSend;
  uint8_t packet[8];

  for (uint8_t i = 0; i < 4; ++i) {
    packet[i] = (uint8_t)(rpmBits >> (8 * i));
    packet[i + 4] = (uint8_t)(timeBits >> (8 * i));
  }

  // 5) 패킷을 W5100 TX 버퍼에 쓰고 SEND 명령만 시작.
  // ARP 대상 PC가 없더라도 모터 CAN 처리 루프를 기다리게 하지 않는다.
  spiSelectEthernet();
  const bool started = RpmUdp.startSend(
    pcIP, RPM_UDP_DEST_PORT, packet, sizeof(packet));
  spiSelectNone();

  if (started) {
    rpmUdpSendPending = true;
    rpmUdpSendStartTime = now;
  }
}
