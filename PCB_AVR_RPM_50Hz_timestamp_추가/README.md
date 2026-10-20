# BAKU 모터팀 — RPM 50Hz 목표 + timestamp 추가 버전

**기존 모터 제어 코드와 이전 RPM 피드백 버전은 변경하지 않았습니다.** 이 디렉터리는 조향팀의 통신 인터페이스 변경 요청(10Hz → 목표 50Hz, `valid` 제거, `uint32_t timestamp` 추가)을 반영한 **별도 실험용 소스코드**입니다.

> **상태:** 호스트 GCC/모의 SPI·CAN·UDP 테스트 통과. 실제 AVR-GCC 교차 컴파일, 새 HEX 생성, PCB 업로드, W5500 실송수신, 모터 실측 테스트는 **미실시**. 실제 차량에 적용하기 전에 반드시 장비 테스트를 수행하세요.

## 기존 버전과 달라진 점

| 항목 | 이전 RPM 피드백 버전 | 이번 버전 |
|---|---|---|
| 목표 UDP 전송 간격 | 100ms (10Hz) | **20ms (50Hz)** |
| 목적지 | `10.177.21.1:5001` | **동일** |
| 페이로드 | 5B: `<int32 RPM, uint8 valid>` | **8B: `<int32 RPM, uint32 timestamp_ms>`** |
| timestamp | 없음 | **PCB가 CAN 속도를 파싱한 시각의 `millis()`** |
| 유효성/끊김 처리 | 500ms stale 시 valid=0 | **500ms 초과 새 CAN 속도값이 없으면 전송 중단**, PC에서 수신 타임아웃 확인 |
| 기존 CAN 모터 제어 주기 | 50ms | **그대로 유지** |

### 8바이트 UDP 패킷 (Little Endian)

| Byte | 타입 | 의미 |
|---|---|---|
| 0~3 | `int32_t` (signed) | 컨트롤러가 보고한 **실제 모터 RPM**. 전진 방향은 음수가 될 수 있음 |
| 4~7 | `uint32_t` | **CAN 속도 샘플 수신 및 RPM 계산 당시**의 MCU 부팅 후 경과 시간 `millis()` 단위 ms |

- **timestamp는 PC 절대 시각이나 UNIX 시간(ms)이 아닙니다.** PC와 시간 동기화되어 있지 않으므로 PC의 시각과 직접 비교하면 안 됩니다.
- 32비트 ms 카운터는 약 **49.7일**마다 0으로 순환합니다.
- 동일한 timestamp가 반복되면 동일한 CAN 샘플을 재전송했다는 뜻입니다. RPM 값이 같아도 timestamp가 다르면 새로운 CAN 샘플입니다.
- 모터 컨트롤러가 새로운 속도 프레임을 20Hz로만 보내면 **50Hz UDP 송신에서도 고유 속도 샘플은 약 20Hz**입니다. 50Hz *송신 목표*는 실측 RPM 갱신 50Hz 보장이 아닙니다.
- UDP는 비신뢰 전송 방식이며, BUSY/SEND_OK 대기·ARP·수신 누락·MCU 작업 지연 등에 따라 실물 전송률은 50Hz보다 낮을 수 있습니다.
- 새 CAN 피드백이 **500ms 넘게 없으면 UDP를 보내지 않습니다**. 따라서 RPM=0과 피드백 없음이 혼동되지 않게 PC 쪽에서 UDP 수신 타임아웃을 처리하세요.
- `valid`가 없어졌으므로 **옛 PC 수신기(`<iB>`, 5바이트)는 호환되지 않습니다.** `pc_receiver.py` 또는 동일한 `<iI>` 8바이트 파서로 변경해야 합니다.

## 통신 흐름

```text
ASMS (10.177.21.5) -- 5B UDP:5000 --+
                                     +--> ATmega328P (10.177.21.2) --> CAN VCU CMD 50ms --> Golden Motor
PC   (10.177.21.1) -- 9B UDP:5000 --+
                                                 CAN RPM feedback --> PCB
                                                                    |
                                 8B UDP:5001, TARGET 20ms ----------+--> PC (10.177.21.1)
```

- CAN RPM 피드백: 확장 ID `0x1801D0EF`, MCU Part I byte[6:7] little-endian, RPM=`raw-32000`.
- 기존 CAN VCU 명령은 ID `0x0C01EFD0`, 50ms 주기 유지.
- `SPEED_SEND_PERIOD_MS=20`, `SPEED_FEEDBACK_TIMEOUT_MS=500`.
- 조향 steer 값을 여기서 전송하지 않으며, RPM은 **모터 회전속도**로 감속기 이후 바퀴 RPM이나 엔코더 원시 펄스가 아닙니다.

## PC 수신 테스트

```bash
python pc_receiver.py
```

PC에서 UDP `5001` 포트를 허용하고 `10.177.21.1`을 사용해야 합니다. 수신 스크립트는 RPM, PCB 샘플 timestamp, `NEW/REPEATED`, 약 1초 단위로 관찰된 UDP 수신 Hz/고유 CAN 샘플 Hz를 표시합니다. **실제 Hz는 PC에서 측정해야** 합니다.

## 호스트 모의 테스트

```bash
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_main_host.c -o test_main
./test_main
gcc -std=c11 -Wall -Wextra -Werror -pedantic -I. tests/test_w5500_host.c w5500.c -o test_w5500
./test_w5500
python -m py_compile pc_receiver.py
```

- 모의 1ms 제어 루프 + 50Hz CAN 데이터: 1초간 **50개 UDP 전송 호출** PASS
- 모의 1ms 제어 루프 + 20Hz CAN 데이터: 1초간 **50개 UDP 전송 호출, 고유 timestamp 20개** PASS
- 500ms 넘게 CAN 피드백이 없으면 UDP 중단, ±RPM 부호, 8바이트 인코딩, 기존 CAN 제어·MANUAL·ESTOP, W5500 송수신 분리 테스트 PASS

**주의:** 위 50Hz 결과는 실제 SPI·W5500·CAN 장비가 아닌 mock 함수 실행 결과이며, 하드웨어에서 50Hz를 달성했다는 의미가 아닙니다. 모의 CAN 송신과 MCU 명령 지터에 대한 실제 측정은 별도 수행해야 합니다.

## 빌드 및 실물 검증

프로젝트의 `Makefile`은 `avr-gcc`, `avr-libc`, `avr-objcopy`와 ATmega328P/16MHz를 필요로 합니다.

```bash
make clean && make
```

성공하면 `build/motor_controller.hex`가 생성됩니다. **이 버전의 신규 HEX는 제공하지 않습니다**(해당 빌드를 수행할 수 있는 AVR 도구체인이 없는 환경). 이전 폴더의 HEX는 새 변경사항을 포함하지 않습니다.

1. AVR 컴파일·플래시 용량 및 핀맵/클록/퓨즈 확인
2. **모터 구동 없이** CAN 피드백 생성기 + PC UDP5001을 이용해 timestamp/부호/패킷 길이/전송률 확인
3. 기존 UDP5000 제어 수신과 50ms CAN 제어 명령이 지연·누락되지 않는지 확인
4. 별도 하드웨어 비상정지와 안전 제한이 있는 감독 환경에서 실제 모터 시험

이전 코드에서 유지한 `PHASE_CURRENT_REF_A=500.0f`는 모터 드라이버 및 실제 하드웨어 허용 범위가 검증된 값이라는 뜻이 아닙니다.
