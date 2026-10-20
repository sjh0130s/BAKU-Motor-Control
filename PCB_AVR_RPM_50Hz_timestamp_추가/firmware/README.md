# HEX 출력 관련

이 버전에 맞는 AVR HEX는 아직 생성하지 않았습니다.

- `main.c` 등에 50Hz 목표 UDP / 8B RPM+timestamp 로직이 들어 있습니다.
- `avr-gcc` 환경에서 `make clean && make` 실행 후 `build/motor_controller.hex`가 생성됩니다.
- 기존 `PCB_AVR/avr/firmware/motor_controller.hex` 및 `PCB_AVR_엔코더값_속도값_PC전송추가`의 이전 바이너리를 신규 펌웨어로 착각해 기록하지 마세요.
- 실제 PCB에 기록하기 전 호환 클록, 핀, 전원 및 안전 기능을 검증해야 합니다.
