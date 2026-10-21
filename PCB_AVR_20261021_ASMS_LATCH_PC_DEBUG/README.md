# PCB AVR — 2026-10-21 ASMS mode latch / optional PC-only debug

> **상태: 소스 수정본 (차량 및 실제 AVR/PCB에서 미검증).** 기존 2026-10-20 폴더와 HEX 파일은 변경하지 않았습니다. 실제 차량에서 바로 사용하지 마세요.

## 변경 사항
1. **물리적 3단 모드 스위치 의존성 제거**: MCU PD3/PD4는 모드 선택에 사용하지 않습니다. 별도로 제작된 독립 하드웨어 E-STOP은 반드시 유지해야 합니다.
2. **ASMS AUTO(5바이트, 첫 byte=1)는 상태 래치**: 한 번 설정하면 다른 ASMS 모드/비상정지/MCU 재시작까지 유지합니다. ASMS 신호가 1초 이상 없다고 AUTO 모드를 취소하지 않습니다.
3. **선택적 PC 단독 디버깅**: `DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS` 기본값 **0(꺼짐)**. **1로 빌드한 경우에만**, ASMS 모드를 아직 선택하지 않은 상태(`receivedMode=0`)에서 PC의 9바이트 AUTO 명령을 수신할 수 있습니다. ASMS MANUAL은 디버깅보다 우선하며 비상정지를 우회하지 않습니다.
4. **안전 타임아웃 유지**: AUTO에서 PC 속도 명령이 1000ms 넘게 없으면 CAN HALT 상태. AUTO 모드는 유지되므로 새 유효한 PC 명령이 오면 자동 복귀. MANUAL에서는 ASMS 조이스틱 패킷이 1000ms 넘게 없으면 HALT하되 MANUAL 모드는 유지합니다.
5. **CAN 핸드셰이크 독립**: ASMS 유무와 무관하게 CAN 수신/핸드셰이크를 수행하고, 성공 후 50ms마다 RUN/HALT CAN 프레임을 송신합니다. CAN 연결 오류 경고음의 발생 여부만으로 통신 정상 여부를 판단할 수 없습니다.
6. **유지**: ASMS IP `10.177.21.5`, PC IP 기본 `10.177.21.1`, PCB `10.177.21.2:5000`, CAN 250kbps / 16MHz MCP2515 / 50ms 모터 프레임, MCU 프레임에서 RPM 수신 후 PC UDP `:5001` 8바이트 속도+타임스탬프 목표 20ms, ASMS 3/5바이트 E-STOP 재부팅 전까지 래치.

## 모드 동작
| 상황 | 모드 | 구동 명령 |
|---|---|---|
| 전원 직후 ASMS 미수신 / debug=0 | 0 | PC 명령 거절, CAN HALT |
| 전원 직후 ASMS 미수신 / debug=1 | 0 | 신뢰된 PC의 9B AUTO만 허용, 1초 타임아웃 |
| ASMS 5B mode=1 한 번 수신 | AUTO(1) 유지 | 새 PC 9B 명령 필요, PC 명령 지속 필요 |
| ASMS 5B mode=2 | MANUAL(2) 유지 | 같은 패킷의 joystick 적용, 1초 동안 조이스틱 갱신 없으면 HALT |
| ASMS 3B 또는 5B mode=3 | E-STOP 래치 | 재부팅까지 RUN 금지 |
| ASMS의 알 수 없는 모드 | 미선택(0) | 현재 구동 정지. **debug=1일 때는 PC 단독 AUTO가 다시 허용될 수 있음** |

## 빌드 옵션
```bash
# 일반 ASMS 모드 래치 (권장 기본값)
make clean && make

# 제한된 벤치 진단에서만 PC 단독 AUTO 허용
make clean && make DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS=1

# PC 주소를 실험용 노트북 .50으로 변경하는 경우
make clean && make DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS=1 PC_IP_LAST_OCTET=50
```

Microchip Studio에서는 AVR/GNU C Compiler의 Symbols에 `DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS=1` 추가 (PC 단독 디버깅을 선택했을 때만), 다른 IP의 PC를 사용할 경우 `BAKU_PC_IP_LAST_OCTET=50`도 추가하세요. **이 설정은 컴파일 시 정해지며, 실행 중 변경되지 않습니다.** MCU 16MHz, CAN/ETH SPI 핀은 기존 최종 회로도를 유지합니다.

## 테스트 (호스트 mock — 실제 PCB 시험 아님)
```bash
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_main_host.c -o /tmp/test_latch && /tmp/test_latch
gcc -std=c11 -Wall -Wextra -Werror -pedantic -DDEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS=1 -I. tests/test_main_host.c -o /tmp/test_debug && /tmp/test_debug
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_w5500_host.c w5500.c -o /tmp/test_w5500 && /tmp/test_w5500
```
테스트 파일은 포함하지만 이 GitHub 업로드에서 위 명령을 실제로 실행하지 않았으며, AVR 바이너리/HEX와 실제 주행은 검증되지 않았습니다.

## 중요 안전 점검
- 기존 설정 `PHASE_CURRENT_REF_A=500.0f`는 **검증된 안전 전류 한계가 아님**. 전류/토크 한계를 장비 사양 및 시험 절차에 맞게 검증하기 전에는 실차 구동 금지.
- **물리적 독립 E-STOP과 구동 전원 차단**을 별도로 확인. 소프트웨어의 UDP ESTOP은 네트워크가 끊기면 전달되지 않음.
- 예전 `Flash Verify failed` 상태라면 업로드 성공을 가정하지 말 것. MCU UART의 `CAN Init OK` / `Waiting Handshake` / `Handshake OK` 로그, 실제 CAN TX/RX를 구동 비활성 상태에서 먼저 검증.
- 기존 하드웨어 감사의 **MCP2515 발진기 연결, CAN 종단저항, W5500 초기화 실패 가능성**은 해결된 것이 아닙니다. 소프트웨어 변경만으로 PCB의 초기 비프음 문제 원인 확정 불가.
- PC 주소는 **송신자 IP를 정확히 비교**합니다. 기본 PC는 10.177.21.1, 테스트 노트북이 10.177.21.50이면 그 주소로 별도 컴파일해야 합니다.

기준 코드: 2026-10-20 `PCB_AVR_20261020_50Hz_최종수정본`. 기존 50Hz UDP는 목표 주기일 뿐 실측 보장 아님.
