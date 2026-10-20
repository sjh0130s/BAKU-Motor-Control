# PCB AVR 속도 피드백 50Hz 목표 / timestamp 버전 (한국어 요약)

이 프로젝트는 BAKU 자율주행 차량의 ATmega328P 통합 PCB용 **별도 실험 버전**입니다. 기존 모터 구동 CAN/UDP 제어 경로를 유지하며 실측 모터 RPM을 PC에 전달하는 UDP 패킷을 8바이트로 변경했습니다.

- 모터 컨트롤러 → PCB: CAN 확장 ID `0x1801D0EF`, Byte 6~7에서 RPM 계산 (`raw - 32000`)
- PCB → PC: `10.177.21.1:5001`, 목표 **20ms 간격 = 50Hz**
- 데이터: `int32_t rpm` (바이트 0~3, **signed** little endian) + `uint32_t timestamp_ms` (바이트 4~7, little endian)
- Timestamp: **CAN 데이터를 처리한 PCB 부팅 후 ms 시각**, 중복 샘플이면 동일한 timestamp로 전송
- 유효성 표시 1바이트 삭제: CAN 피드백이 500ms 넘게 오지 않으면 송신 중지 → PC에서 수신 타임아웃으로 감지
- 기존 UDP 5000 모드·수동·자동 명령과 CAN 모터 명령 50ms 송신 주기는 그대로 유지

**주의:** 50Hz UDP 송신 목표와 실제 RPM 신규 샘플 50Hz는 다릅니다. 실제 RPM 수신이 20Hz라면 50Hz 송신 패킷 중에는 중복 timestamp가 존재할 수 있습니다. 현 버전은 호스트 모의 검증만 통과했고 AVR HEX 빌드·PCB·모터 시험은 미수행입니다. 자세한 내용은 `README.md`와 `TEST_RESULTS.md`를 참고하세요.
