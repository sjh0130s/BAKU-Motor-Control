# Arduino 모터 실측 RPM → PC UDP 전송 추가 (2026-10-21)

## 파일과 목적
- [BAKU_Arduino_MotorSpeedFeedback_20261021.ino](BAKU_Arduino_MotorSpeedFeedback_20261021.ino): Arduino UNO + Ethernet Shield + MCP2515 적층 테스트용 모터 제어 코드
- 기존 사용자 제공 버전에서 **실측 모터 RPM의 CAN 수신 및 PC UDP 송신 기능만 추가**한 별도 버전입니다.
- 기존 루트의 `MotorController.ino` 및 PCB AVR 코드를 교체하지 않습니다.

## 추가한 기능
1. 모터 컨트롤러 → Arduino: 확장 CAN ID `0x1801D0EF`, DLC 8의 속도 피드백 프레임을 수신합니다. 기존의 `0x55` 핸드셰이크 처리 분기는 유지합니다.
2. 속도 필드: 수신 바이트 **6~7** (0부터 시작, Little Endian)를 `uint16_t`로 읽어 `실측 RPM = raw - 32000`으로 변환합니다. 이 프레임 레이아웃은 **Golden Motor/EZkontrol Protocol 2 설정에 대한 가정**이므로 실제 컨트롤러의 CAN 사양과 실수신 덤프를 확인해야 합니다.
3. `millis()` 값을 속도 피드백을 해석한 시점의 `uint32_t` timestamp로 기록합니다. Unix 시간이 아니라 **아두이노 기동 후 경과 시간(ms)** 입니다.
4. CAN SPI 사용을 종료한 뒤 Ethernet으로 `10.177.21.1:5000`에 UDP 패킷을 전송합니다. 수신된 해당 CAN 속도 프레임 하나당 전송을 시도하며 별도 고정 50Hz 타이머는 추가하지 않았습니다.

## UDP 피드백 패킷 (8바이트)
| 바이트 | 자료형 | 내용 |
| --- | --- | --- |
| 0–3 | `int32_t`, Little Endian | 실측 모터 회전속도 (RPM, 부호 포함) |
| 4–7 | `uint32_t`, Little Endian | 수신 시점 `millis()` timestamp (ms) |

- `valid` 필드는 없습니다.
- **모터 회전자 RPM**이며 감속기 출력축/바퀴 RPM이나 차량 속도(km/h)가 아닙니다. 바퀴 속도 산출에는 별도 감속비 및 타이어 정보가 필요합니다.
- PC 수신부 예시 (Python):
```python
import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5000))
while True:
    data, sender = sock.recvfrom(2048)
    if len(data) == 8 and sender[0] == "10.177.21.2":
        rpm, timestamp_ms = struct.unpack("<iI", data)
        print("motor_rpm=", rpm, "time_ms=", timestamp_ms)
```

## 변경하지 않은 동작
- ASMS `10.177.21.5`와 PC `10.177.21.1`의 수신 형식 및 자동 모드 진입 조건
- 수동/자동/ESTOP 로직, `UDP_TIMEOUT_MS=1000`
- 목표 최대 속도 `SPEED_MAX_RPM=1500` (전/후진 부호 등 기존 코드 설정 유지)
- `PHASE_CURRENT_REF_A=500.0f` 설정 (실측 전류가 아니라 전류 지령)
- CAN 제어 명령 50ms 송신, CAN 250kbps / MCP2515 16MHz
- Shield SPI CS: Ethernet D10, CAN D9, SD D4

## 검증 및 주의사항
- **실차/모터/PC 네트워크에서 아직 검증하지 않았습니다.** PC 수신 프로그램, MCU 펌웨어 빌드, 컨트롤러의 CAN 속도 응답 형식을 확인해야 합니다.
- 실제 피드백 주기는 컨트롤러가 내보내는 CAN 메시지 빈도와 MCU 처리 부하에 종속됩니다. **50Hz 송신 보장 코드가 아닙니다.**
- 8바이트 CAN 프레임에 대한 프로토콜 및 필드 검증 전에는 수신 RPM을 안전 제어의 유일한 근거로 사용하지 마세요.
- 기존 500A 전류 지령은 하드웨어 허용치 확인이 필요하며, 소프트웨어 ESTOP는 독립 하드웨어 비상정지를 대체하지 않습니다.
- 이 파일은 **Arduino 적층 보드용**입니다. 전용 PCB AVR용 핀맵/빌드와 혼동하지 마세요.
