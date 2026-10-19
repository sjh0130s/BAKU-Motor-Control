# PCB AVR - 엔코더값(속도값) PC 전송 기능 추가

> **테스트용 신규 버전 / 기존 PCB 펌웨어와 구별 보관**  
> **상태: PC 기반 모의 테스트 통과, AVR 실제 컴파일·HEX 생성·실제 PCB/모터 주행 시험 미실시. 차량에 바로 굽지 마세요.**

원본 `PCB_AVR/avr/`은 **변경하지 않았습니다**. 이번 폴더는 기존 PCB용 ATmega328P C 펌웨어에 실제 모터 속도 보고 기능을 더한 별도 프로젝트입니다. `main.c`, `hal.h`, `w5500.c` 이외 원본의 `board.c`, `mcp2515.c`, `spi_bus.h`, `Makefile`은 같은 내용을 유지합니다.

## 무엇이 추가됐나

```
Golden Motor EZkontrol (actual motor RPM feedback)
    └─ CAN ID 0x1801D0EF, Part I, 8-byte extended frame
         └─ Byte[6..7], little-endian: raw - 32000 = motor RPM
              └─ PCB ATmega328P + MCP2515 + W5500
                   └─ UDP → PC 10.177.21.1:5001, 약 100 ms마다 5 bytes
```

- **MCU → VCU CAN Part I**의 Byte 6~7에서 측정 RPM을 추출합니다 (Golden Motor EZkontrol MCU to VCU CAN Protocols V1.0, p.3).
- 핸드셰이크는 동일 ID에 `0x55` 8바이트가 들어온 경우로 구분합니다 (동 문서 p.5).
- 500 ms 동안 새 실제 RPM 메시지가 없다면 `valid=0`으로 전송합니다. 이때 값이 `0`인 것은 **모터 정지의 증거가 아닙니다**.
- 기존 **명령 수신 UDP 5000 포트(소켓 0)**와 분리하여 **속도 송신 소켓 1**을 구성합니다. PC의 **수신 포트 5001**을 사용합니다.
- UDP는 연결 없는 전송이므로 `udp_speed_try_send()`가 `true`를 반환해도 **PC 수신을 보장하지 않습니다**. BUSY, ARP 미해결/타임아웃 시 샘플을 생략할 수 있습니다.
- 이 값은 **모터 컨트롤러가 보고하는 회전속도(RPM)**이지 엔코더 펄스 원시값이나 감속기 이후 바퀴 RPM은 아닙니다.

## PC 수신 패킷 (5바이트)

| 위치 | 값 | 자료형 |
|---|---|---|
| Byte 0~3 | 실측 **모터** RPM, signed Little Endian | `int32_t` |
| Byte 4 | `1`: 유효 / `0`: 무효 | `uint8_t` |

- PCB `10.177.21.2` → PC `10.177.21.1:5001`
- 송신 목표 주기 100ms. 실제 전송 성공률과 주기는 네트워크/MCU 처리량에 따라 달라집니다.
- PC 쪽 테스트 도구: `pc_receiver.py` (`python pc_receiver.py`). 포트 `5001`의 방화벽/수신 허용을 확인하세요.
- MCU 동작 시 CAN에서 받는 ID는 `0x1801D0EF`입니다. **Part II `0x1802D0EF`를 RPM으로 파싱하지 않습니다.**
- 실제 정지 여부를 판단하려면 `valid=1`인 데이터만 사용하고, PC 측에서도 수신 시각을 확인하세요.

## 기존 기능 유지

- 기존 ASMS `10.177.21.5` → PCB UDP `5000`: 5B 모드/조이스틱
- 기존 PC `10.177.21.1` → PCB UDP `5000`: 9B AUTO
- CAN `0x0C01EFD0` 명령: 기존 속도/전류, Enable, 생명 신호, 50ms 송신
- ESTOP 모드: **소프트웨어 HALT** (독립 하드웨어 비상정지 아님)
- `PHASE_CURRENT_REF_A = 500.0f`는 원본 값을 유지했습니다. 실제 모터/제어기 제한과 일치하는지 반드시 확인하세요.

## 수정 파일

| 파일 | 변경 사항 |
|---|---|
| `main.c` | CAN 피드백 파싱, `valid` 타임아웃, 5B 패킷 생성, 주기적 송신 |
| `hal.h` | W5500 속도 송신용 API 함수 선언 |
| `w5500.c` | 송신 소켓 1, W5500 TX 버퍼 관리, SEND_OK/TIMEOUT 폴링 |
| `pc_receiver.py` | PC에서 실제 RPM 수신 결과를 출력하는 테스트 스크립트 |
| `tests/` | PC 환경 mock 시험 (실제 HW 시험 아님) |

## 빌드하기 (AVR-GCC 설치된 PC에서)

```sh
cd PCB_AVR_엔코더값_속도값_PC전송추가
make clean
make
```

대상: `atmega328p`, `F_CPU=16000000UL`, `avr-gcc`, `avr-libc`, `avr-objcopy`, `avr-size` 필요. 출력은 **`build/motor_controller.hex`** (빌드 성공 시). 실제 ISP와 PCB 하드웨어/클럭/퓨즈 설정을 먼저 검증해야 합니다. **기존 `PCB_AVR/avr/firmware/motor_controller.hex`를 사용하면 이 새 기능이 포함되지 않습니다.**

**이 폴더에는 새 HEX가 없습니다.** 현재 실행 환경에 AVR 컴파일러가 없어 확인 없이 이전 HEX를 복사하지 않았습니다.

## PC 로직·드라이버 모의 테스트

```sh
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_main_host.c -o test_main_host
./test_main_host
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_w5500_host.c w5500.c -o test_w5500_host
./test_w5500_host
```

`tests/test_main_host.c`: handshake, 다른 CAN ID 무시, ±RPM, 유효 플래그, 타임아웃, ASMS 수동/ESTOP, CAN 50ms 제어 경로. `tests/test_w5500_host.c`: 송수신 분리, UDP5000 유지, TX5001 패킷, SEND_OK/TIMEOUT/busy 및 stale 재초기화.

## 실제 적용 전 필수 검증

1. AVR-GCC를 이용해 `-Wall -Wextra -Werror` 빌드하고 ELF/HEX 해시 및 플래시 용량 확인.
2. 프로그래머와 PCB 전원, ATmega328P 클럭/퓨즈/핀 연결 검증 후 바이너리 기록.
3. **모터 구동 없이** CAN 프레임을 주입해 RPM → PC UDP 5001 전송과 `valid` 상태 확인.
4. 기존 UDP 5000 명령 수신과 CAN 50ms 송신의 지터, 네트워크 불량 시 동작 확인.
5. 차륜을 지면에서 분리하고, 독립 비상정지가 준비된 감독 환경에서 제한된 동작 시험.
6. 피드백 RPM의 부호와 모터 실제 회전 방향, 감속비 일치 여부 측정.

**주의:** 이 저장소의 소프트웨어 모의 테스트는 실제 차량 안전성을 증명하지 않습니다. 실제 하드웨어 구동·주행 전에 담당자가 별도 검증해야 합니다.

참고: [Golden Motor EZkontrol MCU to VCU CAN Protocols V1.0 (2022-10-01)](https://device.report/m/63ccdb79415c3f907cb2218494bf178d1526734d1e953d00fa0c4d870c7b1c43)
