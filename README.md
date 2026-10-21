# BAKU Motor Control

## 코드 버전 구분

| 코드 | 목적 | 자료 |
| --- | --- | --- |
| **3단 적층 테스트 (Arduino)** | Arduino 보드 + CAN BUS Shield + Ethernet Shield로 수행한 초기 시험용 코드 | [MotorController.ino](MotorController.ino) |
| **Arduino 실측 RPM 피드백 (2026-10-21, 실차 미검증)** | 기존 Arduino 적층 모터 제어에 CAN 실측 RPM 수신 → PC UDP 8바이트(rpm + timestamp) 전송만 추가 | [코드 및 설명](BAKU_Arduino_MotorSpeedFeedback_20261021/README.md) |
| **통합 PCB용 AVR C** | ATmega328P 기반 제작 PCB에서 최종 테스트를 위해 준비한 Arduino 비의존 펌웨어 | [PCB_AVR/avr](PCB_AVR/avr) |
| **PCB AVR RPM 피드백 추가 (실물 미검증)** | 실측 모터 RPM의 PC UDP 전송 기능을 추가한 별도 소스코드 (새 HEX 미포함) | [엔코더값·속도값 PC 전송 추가](PCB_AVR_엔코더값_속도값_PC전송추가/README.md) |
| **PCB AVR RPM 50Hz + timestamp (실물 미검증)** | 조향팀 요청 반영: UDP 20ms 목표, RPM+측정 시각 8바이트, valid 삭제 | [50Hz + timestamp 추가 버전](PCB_AVR_RPM_50Hz_timestamp_추가/README.md) |

**PCB용 자료:** [소스 설명](PCB_AVR/avr/README_KO.md) · [굽기용 HEX](PCB_AVR/avr/firmware/motor_controller.hex) · [ELF](PCB_AVR/avr/firmware/motor_controller.elf) · [검증 결과](PCB_AVR/검증결과.md) · [전달받은 ZIP 원본](PCB_AVR/AVR_result_original.zip)

PCB용 코드는 Arduino 버전과 통신/제어 개념을 공유하지만 **드라이버 및 핀맵이 다릅니다.** 특히 PCB는 Ethernet CS가 **PD7 (D7)** 이고 CAN CS는 **PB1 (D9)** 입니다. 아래 본문에서 Arduino 라이브러리와 D10 Ethernet CS에 관한 내용은 **기존 적층 테스트 버전** 설명입니다.

**검증 상태:** 첨부된 검증 자료에는 2026-09-20 기준 AVR 빌드 및 모의 시험 성공이 기록되어 있으나, 그 과정에서 **실제 PCB 업로드·CAN 버스·모터 구동은 시험하지 않았습니다.** 최종 실차 검증 여부는 별도로 기록해야 합니다. 파일에 유지된 **500 A 전류 지령**과 보드 핀맵, 클록/퓨즈/전원/하드웨어 비상정지는 구동 전에 반드시 확인하세요.

---


BAKU 자율주행 차량의 **구동 모터 제어 펌웨어**입니다.

이 저장소에는 ASMS/PC에서 수신한 UDP 명령을 해석하고, ATmega328P와 MCP2515를 통해
모터 컨트롤러에 CAN 제어 명령을 전달하는 소스코드가 포함되어 있습니다.

> **공유 목적:** 모터팀 ↔ 조향팀 간 통신 형식과 제어 흐름 확인  
> **상태:** 전달받은 소스코드 원본 기준. 컴파일, PCB 업로드, 최신 실차 동작 여부는 이 저장소 작성 과정에서 검증하지 않았습니다.

## 파일

| 파일 | 내용 |
| --- | --- |
| `MotorController.ino` | Arduino/C++ 기반 모터 제어 펌웨어 |
| `README.md` | 통신 규격 및 팀 간 연동 참고사항 |

## 시스템 구성

```text
ASMS (10.177.21.5) ── UDP:5000 ─┐
                                  ├── ATmega328P / Ethernet ── MCP2515 (CAN) ── 모터 컨트롤러 ── 구동 모터
PC   (10.177.21.1) ── UDP:5000 ─┘
                         수신 측 IP: 10.177.21.2
```

코드에 정의된 네트워크 설정입니다. 실제 장비의 IP 주소와 일치하는지 확인해야 합니다.

| 항목 | 코드 설정 |
| --- | --- |
| 모터팀 수신 IP | `10.177.21.2` |
| UDP 포트 | `5000` |
| ASMS 송신 IP | `10.177.21.5` |
| PC 송신 IP | `10.177.21.1` |
| CAN 속도 / MCP2515 오실레이터 | `250 kbps` / `16 MHz` |
| CAN CS / Ethernet CS / SD CS | `9` / `10` / `4` |
| CAN 수신 ID (MCU → VCU) | `0x1801D0EF` |
| CAN 송신 ID (VCU → MCU) | `0x0C01EFD0` |

## UDP 수신 데이터 형식

정수 데이터는 **Little Endian** 방식입니다. 코드는 **송신자 IP와 패킷 크기를 모두 검사**합니다.

### 1) ASMS → 모터팀: 5바이트

| 바이트 | 형식 | 의미 |
| --- | --- | --- |
| 0 | `uint8_t` | 모드: `1=AUTO`, `2=MANUAL`, `3=ESTOP`; 기타 값은 정지 |
| 1–2 | `int16_t` | `joy_x` |
| 3–4 | `int16_t` | `joy_y` |

- **MANUAL:** `joy_x`를 목표 모터 RPM으로 변환합니다.
- `joy_x` 음수 방향이 전진 입력으로 설정되어 있습니다.
- **AUTO:** ASMS가 모드 `1`을 지정해야 PC의 AUTO 패킷이 적용됩니다.
- **ESTOP (`mode=3`):** 코드에서 모터 HALT 명령을 보냅니다. 하드웨어 비상정지와 동일하지 않습니다.

### 2) PC → 모터팀: 9바이트 (AUTO 전용)

| 바이트 | 형식 | 의미 |
| --- | --- | --- |
| 0–3 | `int32_t` | `steer` |
| 4–7 | `int32_t` | `speed` |
| 8 | `uint8_t` | `misc` |

- `speed`는 코드에서 `-100 ~ +100`으로 제한해 목표 속도 `-1500 ~ +1500 RPM`으로 변환합니다.
- **speed 양수 → 차량 전진(목표 RPM 음수)**, **speed 음수 → 차량 후진(목표 RPM 양수)** 로 작성되어 있습니다.
- `steer`와 `misc`는 수신하지만 **현재 모터 제어에 사용하지 않습니다**.
- PC의 AUTO 명령은 ASMS의 AUTO 모드가 유효할 때만 적용됩니다.

## CAN 동작

1. 모터 컨트롤러가 `0x1801D0EF` ID로 `0x55` 시작 바이트를 보내면,
   모터팀은 `0x0C01EFD0` ID로 `0xAA` 8바이트를 응답합니다.
2. 핸드셰이크 성공 후 목표 RPM, 전류 지령 및 동작 모드를 포함한
   **8바이트 CAN 명령**을 **50 ms 간격**으로 반복 전송합니다.
3. 송신 명령의 주요 필드:
   - Byte 0–1: 전류 지령 (`raw = (A + 3200) / 0.1`, Little Endian)
   - Byte 2–3: 목표 모터 속도 (`raw = RPM + 32000`, Little Endian)
   - Byte 4: 동작 모드 (`0x03`: RUNNING+Speed Control, `0x00`: HALT)
   - Byte 7: Life Signal

### 주요 설정값 (현재 코드 기준)

| 변수 | 값 | 의미 |
| --- | --- | --- |
| `SPEED_MAX_RPM` | `1500` | 목표 모터 RPM 한계 |
| `JOY_DEADBAND` | `120` | 조이스틱 중립 범위 |
| `PHASE_CURRENT_REF_A` | `500.0f` | **전류 지령 설정값** (실제 측정 전류 아님) |
| `MOTOR_SEND_PERIOD_MS` | `50` | CAN 명령 전송 간격 |
| `UDP_TIMEOUT_MS` | `1000` | 유효한 UDP 입력의 타임아웃 기준 |

> 코드 내부의 타임아웃 설명 주석에 `500ms`라고 적힌 부분이 있으나,
> **실제 상수 값은 `1000ms`** 입니다.

## 조향팀 연동 참고사항

- 본 코드는 BAKU 자율주행 차량의 **구동 모터 제어**를 담당합니다.
- ASMS/PC에서 **UDP** 주행 명령을 수신하고, **CAN** 통신으로 모터를 제어합니다.
- 조향 제어는 별도 시스템에서 수행하며, **본 코드에서는 조향 명령을 전송하지 않습니다**.
- **실측 모터 RPM의 PC 전송 기능**은 [별도 PCB AVR 수정 버전](PCB_AVR_엔코더값_속도값_PC전송추가/README.md)에 추가했습니다. **기존 Arduino 및 PCB AVR 원본에는 해당 기능이 없습니다.** 수정 버전은 소프트웨어 모의 테스트까지만 완료했으며, 실제 PCB·모터 동작은 아직 검증되지 않았습니다.

## 개발 환경 및 주의사항

- 코드에 `SPI.h`, `Ethernet.h`, `EthernetUdp.h`, `mcp2515_can.h`를 사용합니다.
- **CAN 컨트롤러 오실레이터 16 MHz** 및 **CAN 250 kbps** 설정에 맞는지 확인하세요.
- 사용하는 ATmega328P/PCB 보드, 라이브러리 버전 및 하드웨어 배선을 기준으로
  컴파일과 실차 동작을 다시 검증해야 합니다.
- 전류 지령(`PHASE_CURRENT_REF_A`)은 **500.0 A로 작성**되어 있으므로
  모터 드라이버·모터의 허용값과 맞는지 실차 구동 전에 별도 확인이 필요합니다.
- `ESTOP` 처리는 코드상의 정지 명령이며, **독립적인 하드웨어 비상정지를 대체하지 않습니다**.
- 팀 내부 하드웨어 설정이 포함되므로 공개 저장소로 바꾸기 전에 팀과 공유 범위를 합의하세요.

## 라이선스

팀 소유 코드이므로 라이선스는 별도로 지정하지 않았습니다.
외부 공개나 재배포 시 BAKU 팀의 승인을 확인하세요.
