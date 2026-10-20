# 검증 결과 — RPM 50Hz 목표 + timestamp 버전

**검증 범위:** Linux 호스트 GCC와 모의 CAN, MCU 시계, W5500 SPI 레지스터 테스트. 실제 PCB 및 모터 테스트 아님.

| 검사 | 결과 |
|---|---|
| GCC `-std=c11 -Wall -Wextra -Werror -pedantic` (호스트 테스트) | PASS |
| ±RPM int32 / uint32 timestamp little endian, 8B 송신 | PASS (mock) |
| CAN 수신 시각 캡처와 동일 샘플 timestamp 반복 | PASS (mock) |
| 20ms 간격 UDP 전송 시뮬레이션 | PASS (mock) |
| 1ms polling / 50Hz CAN 프레임: 1초간 50회 UDP 송신 함수 호출 | PASS (mock) |
| 1ms polling / 20Hz CAN 프레임: 1초간 50회 UDP 송신 함수 호출, timestamp 20개 | PASS (mock) |
| 500ms 초과 피드백 손실시 UDP 미전송 | PASS (mock) |
| 기존 ASMS MANUAL/ESTOP와 CAN 모터 명령 | PASS (mock) |
| W5500 UDP5000 제어 수신 / UDP5001 속도 전송과 TX busy 처리 | PASS (mock) |
| Python `pc_receiver.py` 문법 검사 | PASS |
| **실제 AVR 컴파일 및 신규 HEX 생성** | **NOT RUN** (도구체인 없음) |
| **PCB/실모터/실제 네트워크 전송률 측정** | **NOT RUN** (장비 없음) |

**중요:** 50Hz는 **목표 전송 주기**이며 실제 MCU에서 20ms마다 송신 완료되는지, 모터 속도 피드백이 몇 Hz 들어오는지는 측정하지 않았습니다. UDP 프레임 타임스탬프는 MCU 부팅 기준이며 PC 절대 시간과 동기화되어 있지 않습니다.


## 2026-10-20 최종 회로도 대응 로컬 호스트 테스트
- 기본 PC 목적지 .1: `gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_main_host.c` **PASS**
- 실험실 PC 목적지 .50: 같은 명령에 `-DBAKU_PC_IP_LAST_OCTET=50` 추가 **PASS**
- W5500 모의 테스트: `gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_w5500_host.c w5500.c` **PASS**
- 추가 항목: 디바운스, 물리 모드 전환, CAN 명령 중지, UDP 모드 일치, ESTOP 래치 및 3B 처리.
- **미실시:** 실제 AVR GCC 교차 컴파일, 새 HEX 생성, PCB 재업로드, 차량 네트워크 및 모터 검증.
