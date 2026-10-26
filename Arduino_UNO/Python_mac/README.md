# macOS 모터 튜닝 도구

대상 환경은 **13형 MacBook Air M3(Apple Silicon), macOS 26.5.2, UGREEN CM639 유선 LAN**입니다. Windows 버전과 동일한 모터 제어/시나리오/피드백/기록/그래프 로직을 사용하고 설치·네트워크 설정만 macOS에 맞췄습니다. 실제 Mac/CM639/모터에서 확인한 범위는 마지막 검증 항목과 구분합니다.

모터 저장소 기준 위치는 `Arduino_UNO/Python_mac/`입니다. 실행 Python은 `motor_test.py` 하나이며 다른 `Python` 폴더를 import하지 않습니다. ROS, Docker, Arduino IDE 없이 실행합니다. **조향 입력은 없고 `joy_y`, `steer`, `misc`는 0으로 고정**합니다.

기준 펌웨어는 [1025_0100 스케치](../1025_0100_변경코드_PC에_모터값_계속_보내/1025_0100_변경코드_PC에_모터값_계속_보내.ino)입니다. 펌웨어 변경/업로드 기능은 없습니다. 기존 모터 저장소의 이전 README/이전 수신 예제와 포트·데이터 형식을 혼동하지 마세요.

## 1. Python과 최초 설치

[Python 공식 macOS 다운로드](https://www.python.org/downloads/macos/)에서 **일반 Python 3.13 계열 macOS 64-bit universal2 installer(.pkg)**를 설치합니다. 기본 설치에 포함된 Tcl/Tk를 사용합니다. 실험적인 free-threaded 빌드와 Rosetta로 실행하는 Intel Python은 사용하지 않습니다. universal2 설치본은 Apple Silicon에서 네이티브로 실행할 수 있습니다. [Python의 macOS 설치 안내](https://docs.python.org/3.13/using/mac.html)

설치 후 `/Applications/Python 3.13/Install Certificates.command`를 실행해 인증서 설치도 완료합니다. pip에서 인증서 오류가 발생하면 이 단계와 네트워크 연결을 확인하세요.

Terminal을 열고 다음을 확인합니다.

```bash
python3.13 --version
python3.13 -c 'import platform; print(platform.machine())'
python3.13 -m tkinter
```

각각 `Python 3.13.x`, `arm64`, Tk 테스트 창이 나와야 합니다. `x86_64`면 Terminal의 Rosetta 실행 여부와 Python 설치본을 확인하세요. macOS 시스템 Python이나 Homebrew Python을 사용할 필요는 없습니다.

팀 모터 저장소를 clone/pull한 뒤 `Arduino_UNO/Python_mac` 폴더로 이동합니다. **저장소 경로와 폴더 이름은 자유**이며 한글·공백 경로도 허용합니다. 아래 `/Users/사용자/...`는 예시일 뿐입니다.

```bash
cd "/Users/사용자/팀 저장소/motor/Arduino_UNO/Python_mac"
bash install.command
bash run.command --dry-run
```

`install.command`는 현재 폴더의 `.venv`에 Matplotlib 3.10.7과 prompt_toolkit 3.0.52를 설치합니다. 하위 의존성은 pip가 설치하고 실제 버전은 로그에 기록합니다. 하위 의존성까지 고정한 lock 파일은 아닙니다. 설치 및 프로그램 실행에 `sudo`를 붙이지 마세요.

CMD 대신 `.command` 스크립트를 사용합니다. 스크립트 위치를 기준으로 이동하므로 다른 작업 디렉터리에서 절대 경로로 실행해도 됩니다. 실행 권한이 유지된 Git clone에서는 Finder 더블 클릭으로 실행할 수도 있습니다. 권한 오류가 있으면 위처럼 `bash`로 실행하거나 다음을 한 번 실행합니다.

```bash
chmod +x install.command run.command
```

다른 위치의 Python 3.13을 지정하려면:

```bash
MOTOR_PYTHON="/Library/Frameworks/Python.framework/Versions/3.13/bin/python3.13" bash install.command
```

업데이트 후에는 `git pull`하고 `bash run.command`로 실행합니다. `.venv`가 없거나 requirements 내용이 바뀌면 설치를 수행하므로 그때는 인터넷이 필요합니다. Python 자체를 자동 설치하지는 않습니다. 가상환경 활성화는 필요 없습니다. 가상환경을 만든 뒤 폴더를 옮겼거나 Python 버전/아키텍처를 바꿨으면 **이 폴더의 `.venv`만** 지우고 `bash install.command`로 다시 만드세요. [venv 안내](https://docs.python.org/3.13/library/venv.html)

## 2. 장비 없이 모의 실행

```bash
bash run.command --dry-run
```

IP 변경, UDP 소켓, 장비 송신 없이 내부 모델이 피드백을 생성합니다. 메뉴·시나리오·실시간 상태·로그·결과 창을 먼저 확인하세요. 모의 모델은 실제 컨트롤러의 전류·회생 제동·물리 동작을 검증하는 모델은 아닙니다.

시험 실행 중 다음 명령으로 경고와 복구를 확인할 수 있습니다.

```text
fault can       CAN 측정 시각 정체, UDP만 반복
fault udp       UDP 수신 중단
fault sentinel  CAN 측정 이력 없음(INT32_MIN)
fault none      정상 모의 피드백으로 복구
```

모의 실행도 로그를 저장하며 `metadata.json`의 `options.dry_run=true`로 구분합니다. `.command` 실행기는 Mac 전용이지만 Python의 `--dry-run`은 소프트웨어 검증용 Linux에서도 사용할 수 있습니다.

## 3. CM639 연결과 인터페이스 선택

CM639를 Mac의 USB-C에 연결하고 허브의 RJ45 포트를 시험망/모터 보드에 연결합니다. 같은 시험망에서 `.1`을 쓰는 기존 PC와 `.5`를 쓰는 실제 ASMS는 분리합니다. CM639가 macOS 네트워크 장치로 인식되어야 하며, 인식 문제는 먼저 `시스템 설정 → 네트워크`에서 확인하세요.

```bash
bash run.command --list-adapters
```

출력 예시:

```text
en5: USB 10/100/1000 LAN | active | IPv4=169.254.x.x
```

`en5`는 예시입니다. 포트나 연결 순서에 따라 달라질 수 있으므로 고정하지 않습니다. 유선 어댑터가 하나면 자동 선택하고, 여러 개면 번호를 입력받습니다. Wi-Fi/블루투스/iPhone USB/브리지는 선택 목록에서 제외합니다. `--list-adapters` 자체는 IP를 바꾸거나 UDP를 보내지 않습니다.

## 4. 실제 통신과 임시 IP

| 용도 | 로컬 주소 | 상대 주소 | 패킷 |
|---|---|---|---|
| PC 모터 명령 | `10.177.21.1:임시포트` | `10.177.21.2:5000` | `<ffB`, 9바이트 |
| ASMS 모터 명령 | `10.177.21.5:임시포트` | `10.177.21.2:5000` | `<Bhh`, 5바이트 |
| RPM 피드백 수신 | `10.177.21.1:5001` | `10.177.21.2:5001`에서 수신 | `<iI`, 8바이트 |

펌웨어가 출발지 IP로 PC/ASMS를 구분하므로 선택한 유선 인터페이스에 `.1`, `.5` 두 주소가 모두 필요합니다. ASMS 모드에서도 피드백은 `.1:5001`로 들어옵니다.

추천 실행:

```bash
bash run.command --setup-network
```

인터페이스를 확인한 경우 명시적으로 지정할 수도 있습니다.

```bash
bash run.command --setup-network --adapter en5
```

프로그램은 선택한 어댑터에 **없는 주소만 `/24` alias로 추가**합니다. 이때만 `sudo` 인증이 필요하며 Terminal에서 Mac 로그인 암호를 입력합니다. 입력 문자가 표시되지 않는 것이 정상입니다. 암호는 프로그램 로그에 저장하지 않습니다. 전체 프로그램/GUI는 일반 사용자로 실행합니다.

기존 주소/DHCP/DNS/기본 게이트웨이를 직접 변경하지 않고 임시 alias를 사용합니다. 해당 서브넷의 연결 경로는 OS가 추가할 수 있습니다. 시작 전 인터페이스/IP 목록과 사용 명령·경로 조회 결과를 로그에 보존합니다. 이미 다른 인터페이스에 같은 로컬 IP가 있거나 보드 경로가 선택한 유선 어댑터를 가리키지 않으면 송신을 시작하지 않습니다.

정상 종료 시 **이번 실행이 추가한 주소만 삭제**하고 기존 주소는 남깁니다. 장시간 실행 후 sudo 인증이 만료되었으면 삭제 시 다시 암호를 요청할 수 있습니다. 강제 종료/허브 분리/인증 취소로 정리가 끝나지 않으면 안내된 인터페이스를 확인해 수동 정리합니다. [Apple ifconfig 매뉴얼 소스](https://github.com/apple-oss-distributions/network_cmds/blob/main/ifconfig.tproj/ifconfig.8)

수동 설정을 선호하면 아래처럼 한 번 추가한 뒤 `bash run.command --adapter en5`로 실행할 수 있습니다. 이 경우 주소는 프로그램이 추가한 것이 아니므로 자동 삭제하지 않습니다.

```bash
sudo /sbin/ifconfig en5 inet 10.177.21.1 netmask 255.255.255.0 alias
sudo /sbin/ifconfig en5 inet 10.177.21.5 netmask 255.255.255.0 alias
bash run.command --adapter en5
```

수동 삭제(해당 인터페이스에서 삭제해도 되는 시험용 주소만):

```bash
sudo /sbin/ifconfig en5 inet 10.177.21.5 -alias
sudo /sbin/ifconfig en5 inet 10.177.21.1 -alias
```

기존 `10.177.21.0/24` 네트워크가 Wi-Fi/VPN/다른 유선망에도 있으면 경로가 충돌할 수 있습니다. 프로그램이 다른 인터페이스의 IP나 경로를 강제로 지우지는 않습니다.

macOS에서 로컬 네트워크 접근 허용 창이 뜨면 허용합니다. 수신/송신이 막히면 `시스템 설정 → 개인정보 보호 및 보안 → 로컬 네트워크`에서 실제 실행에 사용한 Terminal/Python 등의 항목을 확인하세요. 방화벽을 사용하는 경우 관련 수신 허용도 확인합니다. 프로그램이 개인정보 권한이나 방화벽을 자동 변경하지는 않습니다. [Apple 로컬 네트워크 접근 안내](https://support.apple.com/guide/mac-help/control-access-to-your-local-network-on-mac-mchla4f49138/mac)

## 5. PC / ASMS 모터 입력

시작 메뉴는 `PC / ASMS / QUIT`입니다. 메뉴/시나리오 편집 중에는 구동 명령을 주기 전송하지 않고 피드백만 수신합니다.

**PC → 1=수동 RPM:** 시작 Enter 후 숫자와 Enter로 현재 목표를 변경합니다. 입력 대기 중에도 현재 값을 목표 50Hz로 반복 송신합니다. PC의 수동 입력은 보드 관점에서는 AUTO 명령입니다.

```text
rpm> 500
rpm> 250.5
rpm> 0
rpm> s
```

범위는 `-891.387 ~ +891.387 RPM`입니다. 범위 초과/NaN/무한대는 재입력받고 기존 유효 목표를 유지합니다. 모든 시험은 시작 후 첫 1초 동안 0으로 고정하며 그동안 입력한 목표는 1초 후 적용됩니다.

**ASMS → 1=모터 joy_x 입력:** 실물 조이스틱 없이 정수 `-2048 ~ 2047`을 입력합니다. 중립은 `0`, deadband는 `±120`입니다. 펌웨어 기준 음수 joy_x는 전진, 양수는 후진입니다. 조향용 입력은 없습니다. 현재 조이스틱 값을 MANUAL(`mode=2`)로 반복 전송합니다.

**ASMS AUTO / ESTOP:** ASMS 메뉴 `2=AUTO`, `3=ESTOP`, PC 메뉴 `a=ASMS AUTO`를 제공합니다. 같은 모드를 3회 전송하고 구동 명령 주기 송신은 대기합니다. 실제 모드 수락 ACK는 없습니다.

기준 펌웨어의 `PC_STANDALONE_DEBUG=1`은 부팅 후 ASMS 이력이 없을 때만 PC 단독을 허용합니다. MANUAL/ESTOP 사용 후 PC로 바꾸려면 **PC 메뉴 a로 ASMS AUTO를 명시적으로 전송**하세요. 도구가 보낸 MANUAL/ESTOP 이력이 있으면 AUTO 전송 전 PC 시작을 막습니다. 프로그램 재시작 전의 보드 모드는 UDP로 확인할 수 없으므로 필요하면 `a`를 사용합니다. ESTOP 뒤 새 ASMS MANUAL 명령으로 운전하는 것도 가능합니다.

## 6. 자동 RPM과 그래프 편집

PC 메뉴 `2=그래프 편집`, `3=질문식 자동`은 같은 시나리오 계산을 사용합니다. 그래프에서 확정한 뒤 CLI에서 빈 입력으로 Enter를 눌러야 실행 시각 0이 시작됩니다.

표의 목표 RPM/목표 도달 시각/유지 시간을 수정하고 **선택 수정**을 누릅니다. 단계 추가·삭제와 JSON 저장·불러오기를 지원합니다. 그래프의 원은 목표 도달점, 사각형은 유지 종료점으로 마우스로 끌 수 있습니다. Pan/Zoom을 해제하면 점을 끌 수 있습니다. step/0초 유지/겹친 점은 숫자 표로 정확히 입력하세요.

기본 예제(`scenarios/example_500_to_zero.json`):

| 시간 | Reference |
|---|---|
| 0~1초 | 0 RPM |
| 1~2초 | 0 → 500 선형 ramp |
| 2~4초 | 500 유지 |
| 4~6초 | 500 → 0 선형 ramp |
| 6초 이후 | 기록 종료까지 0 유지 |

- 첫 목표 도달 시각은 1초 이상입니다.
- 다음 도달 시각은 이전 `도달 시각 + 유지 시간` 이상입니다.
- 같으면 step, 더 늦으면 두 값 사이 선형 ramp입니다.
- 유지 시간은 0 이상이며 소수 시간/RPM을 허용합니다.
- **마지막 RPM은 기록 종료까지 유지**합니다. 마지막 유지 시간 칸은 사용하지 않습니다. 마지막 RPM이 0이 아니면 그 값의 송신도 계속합니다.
- 첫 1초 0 구간은 제거할 수 없습니다. 실행 중 시나리오는 변경하지 않습니다.

질문식 입력은 마지막 단계의 유지 시간 질문을 생략하고 마지막 값을 무기한 유지합니다. 미리보기 창에서 사용/취소를 선택합니다. Windows 버전의 시나리오 JSON을 그대로 교환할 수 있습니다.

## 7. 종료와 HALT

| 시험 중 명령 | 동작 |
|---|---|
| `s`, `stop`, `b`, `q` | 0 명령 2초 추가 전송·기록 → 송신 중단 → 로그 마감·결과 창 |
| `e`, `estop` | 즉시 ASMS ESTOP 요청 → 로그 마감·결과 창 |
| `status` | 현재 상태 한 번 출력 |

ASMS MANUAL의 일반 종료는 조이스틱 중립을 2초 전송합니다. 일반 종료에서는 ESTOP을 보내지 않습니다. 송신 중단 후 보드의 기존 **1초 초과 타임아웃**이 HALT를 요청합니다. PC AUTO 상태의 타임아웃은 AUTO를 유지하므로 다음 PC 시험을 다시 시작할 수 있습니다. ASMS MANUAL 타임아웃 뒤 PC로 전환할 때는 AUTO 명령이 필요합니다.

결과 창을 닫으면 메뉴로 복귀합니다. 초기 메뉴 `QUIT`, CLI Ctrl+C/Ctrl+D(EOF)도 진행 중 기록을 일반 종료하고 프로그램을 닫습니다. 그래프 편집 창 닫기는 편집 취소, 결과 창 닫기는 메뉴 복귀입니다. Terminal 강제 종료/활성 상태에서의 전원 차단은 정리와 최종 그래프 저장을 보장하지 않습니다.

HALT는 목표 0 RPM/전류 지령 0 A/RUNNING 해제 요청입니다. 물리 브레이크나 전원 차단, 실제 정지 확인과 다릅니다. 현재 펌웨어는 RUNNING 0 RPM에서도 전류 지령을 0 A로 보내므로 2초는 반응 관측 시간이며 능동 제동이나 실제 정지를 보장하는 시간이 아닙니다. HALT 중에도 UDP 수신/RPM 피드백은 계속됩니다.

## 8. 실시간 상태와 RPM 단위

입력창 아래 상태는 약 5Hz로 갱신하고, 그래프가 열린 동안을 포함해 CLI에 약 1초마다 상태/경고를 출력합니다. 입력 중 문자열은 유지합니다.

`REF`, `ACT`는 모두 **RPM**입니다. 이 도구는 GPS 장치나 실시간 GPS 속도를 직접 수신하지 않습니다. 펌웨어가 CAN 컨트롤러 RPM에 보정계수 `0.198086`을 적용한 RPM을 받습니다. 코드의 “GNSS 기준 보정”은 그 환산 기준이며 현재 GPS 측정값이라는 뜻이 아닙니다.

```text
PC/running t=3.2s REF=500.0 ACT=498 RPM [OK]
UDP=50Hz CAN=20Hz age UDP/CAN=0.01/0.03s TX=... errors=0 badRX=0 maxGap=20ms
```

| 상태 | 의미 |
|---|---|
| `WAIT_UDP` | 유효한 UDP를 아직 못 받음 |
| `WAIT_CAN_ADVANCE` | 첫 측정 후 MCU 시각 갱신 확인 중 |
| `NO_CAN_SAMPLE` | INT32_MIN: 보드의 CAN RPM 수신 이력 없음 |
| `STALE_CAN` | UDP는 오지만 CAN 측정 시각 정체 |
| `NO_UDP` | 유효 UDP 수신 중단 |
| `OK` | 최근 UDP 수신과 CAN 측정 시각 갱신 확인 |

경고 기준은 기본 0.5초(`--warning-age 0.8` 등으로 변경 가능)이며 **시험 자동 종료 타임아웃이 아닙니다.** 오류가 지속되면 경고를 반복하고 복구 시 정상으로 돌아옵니다. 보드의 기존 제어 명령 타임아웃 1초는 별개이며 변경하지 않습니다.

UDP 목표 50Hz와 새 CAN 측정 약 20Hz를 구분합니다. 같은 RPM도 MCU 측정 시각이 진행하면 새 측정입니다. 반복 시각은 새 측정으로 세지 않습니다. MCU uint32 순환은 허용하고 역행은 재부팅/재정렬로 추정해 갱신 확인 전 정상으로 표시하지 않습니다. PC 단조 증가 시계로 경과 시간을 계산합니다.

표시 부호는 `REF=+500 → UDP speed=+500 → 보드 내부 목표=-500 → raw 피드백 약 -500 → ACT=+500`입니다. 보정계수를 Python에서 다시 적용하지 않습니다. 원본 raw는 로그에 보존합니다. 전진/후진 표기는 펌웨어 기준이며 실제 방향은 저속 장비 시험으로 확인합니다.

현재 UDP에는 모드/핸드셰이크/HALT 상태/명령 수락 ACK가 없습니다. 송신 성공과 실제 수락·정지 확인은 구분합니다. macOS 스케줄링의 50Hz 실시간 보장은 없으므로 실제 송신 간격/최대 간격/주기 지연도 기록합니다.

## 9. 로그와 Git

기본 로그는 이 폴더의 `logs/`이고 `--logs "/다른 위치/시험 기록"`으로 바꿀 수 있습니다. 실행 전체 `*_session`, 실제 연결 준비 `*_network_setup`, 시험별 `*_pc_manual`, `*_asms_manual`, `*_auto` 폴더를 생성합니다.

```text
logs/날짜_시간_auto/
  metadata.json  Git/소스 해시/Python/패키지/macOS·CPU/실행 인수/종료 상태
  scenario.json  자동 시나리오(자동 시험만)
  commands.csv  송신 reference/float32 wire/joy_x/성공·오류/간격
  feedback.csv  raw·표시 RPM/MCU 시각/새 측정 여부/유효 상태
  events.csv    입력·모드 요청·상태 변화·오류·종료
  result.png    종료 후 reference/actual 그래프
```

CSV는 UTF-8 BOM이고 PC 상대 시각/epoch를 함께 남깁니다. 기록 시작과 실제 제어 시작의 미세한 차이는 `control_start_elapsed_s`로 확인합니다. 원본 CSV는 그대로 보존하고 긴 기록의 그래프 데이터만 간추립니다. 새 측정만 actual로 표시하며 오래된 구간을 정상 측정처럼 연결하지 않고 경고 구간은 붉은 음영으로 표시합니다. 실행 중 0.5초마다 flush하고 디스크 오류/로그 누락은 화면과 메타데이터에 표시합니다.

기준 펌웨어 소스 해시는 기록하지만 실제 보드 업로드 버전의 검증을 의미하지는 않습니다. 이 폴더만 별도로 복사해도 실행할 수 있고 그때 펌웨어 소스가 없으면 해시는 null로 남깁니다.

`.venv/`, `logs/`, 캐시는 `.gitignore`로 제외합니다. 코드/설치 스크립트/문서/의존성 명세/예제 시나리오만 Git에 올립니다. 시나리오는 `scenarios/`의 JSON으로 공유할 수 있습니다. 스크립트는 commit/push를 자동 수행하지 않습니다.

## 10. 검증 범위

제어/기록/GUI 로직이 기존 Windows 코드와 동일한지 비교하고 격리 Linux 임시 컨테이너에서 시나리오·피드백·로컬 UDP·모의 실행·한글/공백 경로·그래프 편집을 검증합니다. macOS 네트워크 명령은 모의 응답으로 선택/alias 추가·정리/실패 복구를 확인합니다. macOS arm64/Python 3.13용 의존성 wheel 다운로드 가능 여부도 별도로 확인합니다.

위 검증은 **실제 macOS 26.5.2의 GUI/권한/CM639 인식/IP 경로/모터 동작 확인을 대신하지 않습니다.** 처음에는 `--dry-run`, 다음에는 장비 연결 후 낮은 RPM으로 단위·방향·모드 전환·종료 반응을 확인하세요.
