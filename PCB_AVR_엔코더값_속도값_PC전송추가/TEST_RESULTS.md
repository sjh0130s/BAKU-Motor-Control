# 엔코더값(속도값) PC 전송 기능 추가 — 검증 결과

2026-10-19, **실제 하드웨어 미연결**인 PC 환경에서 수행한 검증입니다.

| 검증 항목 | 결과 |
| --- | --- |
| `main.c`, `w5500.c` 호스트 GCC 경고 오류 검사 (`-Wall -Wextra -Werror -pedantic`) | PASS |
| CAN handshake + 0x1801D0EF 속도 프레임 처리 | PASS (mock) |
| ±RPM 값/5바이트 패킷/500ms stale → valid=0 | PASS (mock) |
| 기존 ASMS MANUAL/ESTOP + CAN 50ms 명령 경로 | PASS (mock) |
| W5500 소켓 0 UDP5000 수신, 소켓1 UDP5001 송신 | PASS (mock SPI) |
| TX BUSY, SEND_OK, TIMEOUT, stale socket 처리 | PASS (mock SPI) |
| Python PC 수신 스크립트 구문 검사 | PASS |
| **AVR GCC 교차 컴파일 및 새 HEX 생성** | **NOT RUN** (해당 컴파일러 미설치) |
| **W5500/MCP2515/실제 PCB/실제 모터 테스트** | **NOT RUN** (장비 미연결) |

```
$ gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_main_host.c -o /tmp/BAKU_test_main
$ /tmp/BAKU_test_main
PASS: handshake, frame ID filter, +1000/-1500 RPM, stale flag, 5-byte UDP, manual/ESTOP, CAN cadence

$ gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_w5500_host.c w5500.c -o /tmp/BAKU_test_w5500
$ /tmp/BAKU_test_w5500
PASS: W5500 TX separate socket, 5-byte UDP, busy/SEND_OK/TIMEOUT, RX5000 preserved, stale reset
```

**주요 한계:** 테스트 드라이버는 W5500 레지스터의 기본 동작만 모의합니다. 실제 W5500 송신 지연, ARP, SPI 물리 배선, CAN 수신 부하, 제어 명령 지터는 측정하지 못했습니다. 시뮬레이션 통과를 실차 안전 보증으로 간주하지 마세요.
