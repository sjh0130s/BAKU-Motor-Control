# Arduino 비의존 AVR C 변환본

대상: **ATmega328P-AU, 외부 16 MHz, 5 V**. Arduino 코어/라이브러리 없이 AVR 레지스터와 avr-libc로 구현했다.

| 파일 | 기능 |
|---|---|
| main.c | UDP 모드 처리, 시간 초과, 핸드셰이크, 속도/전류 계산, CAN 명령 |
| board.c | GPIO, SPI, Timer0 1 ms, UART 115200 8N1 |
| w5500.c | W5500 초기화 및 UDP 수신, 소켓 0 |
| mcp2515.c | MCP2515 초기화, 250 kbit/s 송수신, 확장 ID |
| hal.h / spi_bus.h | 하드웨어 인터페이스 |
| Makefile | ELF/HEX 생성 |

## 빌드

```sh
make
```

`avr-gcc`, `avr-objcopy`, `avr-size`와 avr-libc가 PATH에 필요하다.
검증 도구는 Arduino 배포 AVR GCC **7.3.0-atmel3.6.1-arduino7**이다.
예를 들어 이 컴퓨터에서는 아래처럼 Arduino에 포함된 도구를 지정할 수 있다.

```sh
export PATH="$HOME/Library/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin:$PATH"
make
```

생성물: `build/motor_controller.elf`, `build/motor_controller.hex`.
`firmware/`에는 검증 빌드의 ELF/HEX가 있다. 애플리케이션만 포함하며 부트로더/퓨즈는 포함하지 않는다.
16 MHz 외부 클록과 실제 실행 클록이 일치해야 한다. ISP 장비·기존 부트로더·퓨즈가 확인되지 않아 업로드나 퓨즈 변경은 실행하지 않았다.

## 하드웨어 설정

- ETH_CS PD7, CAN_CS PB1, 하드웨어 SS PB2는 HIGH 출력 초기화.
- MOSI PB3, MISO PB4, SCK PB5. SPI mode 0, 4 MHz.
- CAN_INT PD2, ETH_INT PB0, SW_MANUAL PD3, SW_AUTO PD4는 입력. 폴링 방식.
- Timer0 CTC, 분주 64, OCR0A=249 → 1 ms. 밀리초 읽기는 원자적 처리.
- UART 115200 8N1 요청, U2X=1/UBRR=16. 실제 117647 baud로 Arduino 16 MHz 설정과 동일.
- W5500 RST는 공통 RESET 네트이므로 GPIO가 아닌 소프트웨어 리셋 사용.
- W5500 소켓 0, UDP 5000, RX 16 KiB/TX 2 KiB. 나머지 소켓 비활성화.
- 10.177.21.2/24, gateway 10.177.21.1, MAC A8:61:0A:AE:21:68.
- MCP2515: 16 MHz/250 kbit/s, CNF1=0x41, CNF2=0xF1, CNF3=0x85.

## 유지한 동작 및 드라이버 차이

ASMS 5바이트/PC 9바이트 Little Endian, AUTO/MANUAL/ESTOP, 1000 ms 타임아웃,
50 ms 주기 송신, 최대 1500 RPM, 전류 기준 500.0 A, 원본 CAN ID/8바이트 데이터 구성을 유지했다.
AUTO 전환 때는 HALT 후 새 PC 패킷을 기다린다. 같은 AUTO 모드 반복 패킷은 기존 PC 명령을 유지한다.
중립은 원본처럼 RUNNING+0 RPM이다. 정지 패킷은 핸드셰이크 전에도 전송될 수 있고 주기 명령은 핸드셰이크 뒤에만 전송된다.
물리 스위치로 모드를 바꾸는 기능, 조향/브레이크 UDP 송신은 추가하지 않았다.

AVR 드라이버는 단일 TX 버퍼로 CAN 전송 완료를 최대 10 ms 기다리고, 초과하면 해당 전송 요청을 취소한다.
Arduino 버전은 포함된 Seeed 라이브러리의 버퍼·재시도·시간 제한을 사용한다. 오류 상황의 재시도 시점까지 동일한 것은 아니다.
W5500 소켓/명령 오류 검출 시 AVR은 HALT를 요청하고 네트워크 고장을 래치한다. 다시 운전하려면 리셋이 필요하다.
원본 Arduino처럼 재접속 자동화나 CAN 핸드셰이크 만료 기능은 추가하지 않았다.
CAN 자체가 끊겼을 때 HALT가 실제 모터에 전달됨을 보장하지는 않는다.

50 ms/1000 ms는 메인 루프에서 검사한다. SPI/UART/CAN 처리 시간에 따른 지연은 있으며 실제 최악 지연은 계측하지 않았다.
실제 모터 프로토콜 사양서가 제공되지 않아 전류/속도 해석은 원본 코드의 규칙을 따른다.

## 구현 참고

- [WIZnet W5500 데이터시트](https://docs.wiznet.io/Product/Chip/Ethernet/W5500/datasheet)
- [WIZnet UDP 처리 절차](https://docs.wiznet.io/Product/Chip/Ethernet/W5500/Application/udp)
- [Microchip MCP2515 데이터시트](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP2515-Family-Data-Sheet-DS20001801K.pdf)
- [Microchip ATmega328P 데이터시트](https://ww1.microchip.com/downloads/en/devicedoc/atmel-7810-automotive-microcontrollers-atmega328p_datasheet.pdf)
- [Seeed 16 MHz / 250 kbit/s 설정](https://github.com/Seeed-Studio/Seeed_Arduino_CAN/blob/master/src/mcp2515_can_dfs.h)
