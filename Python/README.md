# Windows 모터 튜닝 도구

Windows PC 한 대에서 기존 PC와 ASMS의 **모터 명령만** 모사하고 UNO의 RPM 피드백을 기록합니다. ROS, Docker, Arduino IDE, USB CAN 장치 없이 실행합니다. 실행 Python 파일은 `motor_test.py` 하나입니다. 조향 입력/조향 보드 송신 기능은 없습니다.

기준 펌웨어는 [`1025_0100_변경코드_PC에_모터값_계속_보내.ino`](../Arduino_UNO/1025_0100_변경코드_PC에_모터값_계속_보내/1025_0100_변경코드_PC에_모터값_계속_보내.ino)입니다. 이 도구는 펌웨어를 수정하거나 업로드하지 않습니다. 저장소 루트의 이전 `README.md`, 이전 수신 예제의 포트/정수 형식과 혼동하지 마세요.

## 1. 최초 설치

대상은 Windows 10/11, **일반 CPython 3.13 64비트**입니다. [Python 공식 Windows 다운로드](https://www.python.org/downloads/windows/)에서 3.13 계열의 Windows installer (64-bit)를 설치합니다. 설치 옵션에서 **pip, Tcl/Tk(IDLE), Python launcher**를 포함합니다. `Add python.exe to PATH`도 선택하면 편리합니다. Embeddable ZIP이나 free-threaded 실험 빌드 대신 일반 설치 프로그램을 사용하세요.

Git이 없으면 [Git for Windows](https://git-scm.com/download/win)를 설치합니다. 팀의 모터 저장소를 clone/pull한 뒤 **모터 저장소의 `Python` 폴더**에서 CMD를 엽니다. 현재 통합 워크스페이스에서 경로는 `docs\firmware\motor\Python`입니다.

**팀원 PC의 저장 경로는 자유입니다.** CMD는 자기 위치(`%~dp0`), Python은 자기 파일 위치(`__file__`)를 기준으로 실행/로그 경로를 정합니다. 예를 들어 `D:\Motor Team\motor\Python`처럼 드라이브·공백·한글 경로가 달라도 사용할 수 있습니다. 이 문서의 `C:\모터저장소\Python` 등은 예시이며 그대로 맞출 필요가 없습니다. `--logs "D:\시험 기록"`으로 로그 위치도 바꿀 수 있습니다. 가상환경 생성 후 폴더 자체를 이동했다면 `.venv`는 새 위치에서 다시 생성하세요.

```bat
py -3.13 --version
install.cmd
run.cmd --dry-run
```

`install.cmd`는 이 폴더에 `.venv`를 생성하고 `requirements.txt`의 Matplotlib 3.10.7, prompt_toolkit 3.0.52를 설치합니다. NumPy 등 하위 의존성은 pip가 설치하며, 실제 설치 버전은 매 실행 로그에 기록합니다. 하위 의존성까지 완전히 고정한 lock 파일은 아닙니다.

`.venv`는 PC마다 생성합니다. Git에 넣거나 다른 사람의 `.venv`를 복사하지 않습니다. [Python venv 문서](https://docs.python.org/3.13/library/venv.html)

설치/실행을 직접 하려면:

```bat
py -3.13 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe motor_test.py --dry-run
```

활성화 스크립트나 PowerShell 실행 정책 변경은 필요 없습니다. Tcl/Tk 설치 확인은 `.venv\Scripts\python.exe -m tkinter`로 합니다.

이후 업데이트는 `git pull` 후 `run.cmd`로 실행합니다. 가상환경이 없거나 requirements 내용이 바뀌면 `run.cmd`가 설치를 수행합니다(이때 인터넷 필요). Python 자체는 자동 설치하지 않습니다. 이미 설치한 패키지가 손상되었으면 `install.cmd`를 다시 실행하세요. 다른 Python 버전으로 만들어진 `.venv`는 지우고 재생성합니다.

## 2. 장비 없는 모의 실행

```bat
run.cmd --dry-run
```

IP 설정/UDP 소켓/장비 송신 없이 내부 모델이 RPM 피드백을 만듭니다. 실제 통신 지연, 전류, 제동, 컨트롤러의 물리 동작을 검증하는 모델은 아닙니다. 처음에는 아래 자동 시나리오를 실행하고 로그와 결과 창을 확인하세요.

시험 실행 중 CLI에 다음을 입력하면 오류 표시와 복구를 확인할 수 있습니다.

```text
fault can       CAN 타임스탬프 정체, UDP 반복 수신
fault udp       UDP 수신 중단
fault sentinel  CAN 측정 이력 없음(INT32_MIN)
fault none      정상 모의 피드백으로 복구
```

모의 실행 기록도 `logs/`에 저장되며 메타데이터의 `dry_run`으로 구분됩니다.

## 3. 실제 장비 연결과 IP

시험용 유선 이더넷을 모터 보드에 연결합니다. 같은 네트워크의 기존 PC(`.1`)와 실제 ASMS(`.5`)는 분리해 IP 충돌/명령 충돌을 피합니다. 모터 보드는 `10.177.21.2`입니다.

| 소켓 | 로컬 주소 | 상대 주소 | 형식 |
|---|---|---|---|
| PC 송신 | `10.177.21.1:임시포트` | `10.177.21.2:5000` | `<ffB`, 9B |
| ASMS 송신 | `10.177.21.5:임시포트` | `10.177.21.2:5000` | `<Bhh`, 5B |
| RPM 수신 | `10.177.21.1:5001` | `10.177.21.2:5001`에서 수신 | `<iI`, 8B |

보드가 **출발지 IP**를 검사하므로 이더넷에 `.1`, `.5`를 모두 설정해야 합니다. 마스크는 `255.255.255.0`(`/24`)이고 시험망에는 기본 게이트웨이가 필요 없습니다. 피드백 수신은 ASMS 모드에서도 `.1:5001`입니다. 다른 수신 프로그램이 5001을 점유하면 먼저 종료하세요.

### 방법 A: Windows에서 고정 IP를 설정하고 실행

`설정 → 네트워크 및 인터넷 → 고급 네트워크 설정 → 기타 네트워크 어댑터 옵션`(Windows 버전에 따라 명칭 차이)에서 시험용 이더넷의 IPv4 속성을 엽니다. 기본 IP `.1`, 마스크 `/24`를 설정하고 고급 설정에서 `.5`를 추가합니다. 설정 변경에 관리자 권한이 필요할 수 있습니다. 주소 설정 후 도구 자체는 일반 CMD에서 실행할 수 있습니다.

```bat
run.cmd
```

### 방법 B: 관리자 CMD에서 임시 주소 자동 추가

`시작 → cmd 검색 → 관리자 권한으로 실행` 후 이 폴더로 이동합니다. 어댑터 이름은 아래 명령으로 확인합니다.

```bat
netsh interface ipv4 show interfaces
cd /d "C:\모터저장소\Python"
run.cmd --setup-network --adapter "이더넷"
```

`--adapter`에는 실제 시험용 어댑터 이름(예: `Ethernet`, `이더넷 2`)을 넣습니다. 프로그램은 바인딩 가능한 기존 주소를 재사용하고, 없는 주소만 `netsh ... add address ... store=active`로 추가합니다. 이번 실행에서 추가한 주소만 종료 때 삭제합니다. 추가 전 `netsh interface ipv4 dump`도 로그에 저장합니다.

**전용 시험 어댑터에서 사용하세요.** 정적 주소 추가는 DHCP 설정에 영향을 줄 수 있습니다. 프로그램이 DHCP/DNS/기본 게이트웨이를 통째로 복원하지는 않습니다. 인터넷용 어댑터나 기존 네트워크를 자동 변경하려는 용도로 사용하지 마세요. [Microsoft netsh 문서](https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/netsh-interface)

강제 종료로 임시 주소가 남으면 관리자 CMD에서 이번에 추가한 주소만 수동 삭제합니다.

```bat
netsh interface ipv4 delete address name="이더넷" address=10.177.21.1
netsh interface ipv4 delete address name="이더넷" address=10.177.21.5
```

Windows 방화벽에서 수신이 막히면 관리자 CMD에서 보드의 UDP 5001만 허용합니다. 아래 규칙은 별도 삭제 전까지 유지됩니다.

```bat
netsh advfirewall firewall add rule name="BAKU Motor RPM UDP 5001" dir=in action=allow protocol=UDP localip=10.177.21.1 localport=5001 remoteip=10.177.21.2 remoteport=5001 profile=any
```

필요 없어진 규칙 삭제:

```bat
netsh advfirewall firewall delete rule name="BAKU Motor RPM UDP 5001"
```

## 4. 모드와 수동 명령

시작 시 `PC / ASMS / QUIT`를 선택합니다. 시작 메뉴/시나리오 편집 중에는 구동 명령을 주기 전송하지 않으며 피드백만 수신합니다. 피드백 오류 때문에 프로그램/시험이 자동 종료되지는 않습니다.

### PC → 1: 수동 RPM

시작 Enter 후 기록을 생성하고 0 RPM 전송을 시작합니다. 이후 숫자와 Enter로 목표 RPM을 바꾸면 현재 값을 **목표 50Hz**로 반복 전송합니다. 입력 대기 중에도 송신은 계속합니다. 모든 시험은 시작 후 첫 1초 동안 0으로 고정되며 그동안 입력한 목표는 1초 이후 적용됩니다.

```text
rpm> 500
rpm> 250.5
rpm> 0
rpm> s
```

입력 범위는 `-891.387 ~ +891.387 RPM`입니다. 범위 초과/NaN/무한대는 오류 후 재입력하며 마지막 유효 목표는 유지됩니다. **수동 RPM 입력도 보드 관점에서는 PC AUTO 패킷**입니다.

### ASMS → 1: 모터 joy_x 입력

실물 조이스틱 없이 CLI 숫자로 입력합니다. 범위는 정수 `-2048 ~ 2047`, 중립은 `0`이고 `±120`까지 deadband입니다. `joy_x` 음수는 펌웨어 기준 전진, 양수는 후진입니다. 숫자를 입력하면 MANUAL(`mode=2`)을 계속 전송합니다. `joy_y=0`, PC의 `steer=0`, `misc=0`은 고정입니다.

### ASMS AUTO / ESTOP

ASMS 메뉴 `2=AUTO`, `3=ESTOP`, PC 메뉴 `a=ASMS AUTO`를 제공합니다. 명령은 같은 모드를 3회 송신하며 구동 명령 송신은 대기 상태로 전환됩니다. 송신 성공은 보드 수락 ACK가 아닙니다.

기준 펌웨어는 부팅 후 ASMS 수신 이력이 없으면 PC 단독을 허용합니다(`PC_STANDALONE_DEBUG=1`). ASMS MANUAL/ESTOP 사용 후 PC 시험을 하려면 **PC 메뉴 a로 ASMS AUTO를 명시적으로 전송**하세요. 이 도구가 보낸 MANUAL/ESTOP 이력이 있으면 AUTO 전송 전 PC 실행을 막습니다. 프로그램 재시작 전의 실제 보드 모드는 UDP 규격으로 확인할 수 없으므로, 그 경우에도 필요하면 `a`를 사용합니다.

ESTOP 뒤에 ASMS MANUAL로 운전하는 것도 가능합니다. HALT가 명령 수신을 중단하거나 재부팅을 요구하는 것은 아닙니다.

## 5. 자동 시나리오

PC 메뉴 `2=그래프 편집`을 추천합니다. `3=질문식 자동`도 같은 시나리오 계산을 사용합니다.

그래프 편집 창의 숫자 표에서 행을 선택하고 RPM/도달 시각/유지 시간을 입력한 뒤 **선택 수정**을 누릅니다. 단계 추가/삭제와 JSON 저장/불러오기도 가능합니다. 그래프의 **원은 목표 도달점**, **사각형은 유지 종료점**이며 마우스로 끌어 수정할 수 있습니다. 드래그는 이전 구간 종료/다음 도달 시각 안으로 제한됩니다. 겹친 점/0초 유지/step은 숫자 표로 정확히 지정하세요. Pan/Zoom을 끄면 점을 끌 수 있습니다.

| 구간 | 예제 reference |
|---|---|
| 0~1초 | 0 RPM 고정 |
| 1~2초 | 0 → 500 선형 ramp |
| 2~4초 | 500 유지 |
| 4~6초 | 500 → 0 선형 ramp |
| 6초 이후 | 기록 종료까지 0 유지 |

위 예제는 기본값이며 `scenarios/example_500_to_zero.json`에도 있습니다.

- 첫 도달 시각은 1초 이상입니다.
- 다음 도달 시각은 이전 `도달 시각 + 유지 시간` 이상이어야 합니다.
- 같으면 step, 더 늦으면 두 목표 사이의 선형 ramp입니다.
- 유지 시간 0초와 소수 시간/RPM을 허용합니다.
- **마지막 RPM은 기록 종료까지 유지**합니다. 마지막 유지 시간 칸은 사용하지 않습니다. 마지막 RPM이 0이 아니면 그 값으로 계속 명령을 보냅니다.
- 첫 1초는 그래프 편집으로 제거할 수 없습니다.

`이 그래프 사용`으로 확정한 뒤 CLI에서 Enter를 누르면 실행 시각 0이 시작됩니다. 그래프를 보거나 수정하는 시간에는 시나리오가 진행되지 않습니다. 실행 중에는 시나리오 변경을 받지 않으며 마지막 구간이 끝나도 자동 종료하지 않습니다. 질문식 입력에서도 마지막 단계는 유지 시간 질문 없이 무기한 유지하고 미리보기 창에서 확인합니다.

## 6. 기록 종료 / ESTOP / 프로그램 종료

시험 실행 중 공통 명령:

| 입력 | 동작 |
|---|---|
| `s`, `stop`, `b`, `q` | **0 명령 2초 추가 전송·기록 → 송신 중단 → 로그 마감·결과 창** |
| `e`, `estop` | 즉시 ASMS ESTOP(`mode=3`) 요청 → 기록 마감·결과 창 |
| `status` | 현재 상태 한 번 출력 |

ASMS MANUAL 시험의 일반 종료에서는 중립 조이스틱 명령을 2초 보냅니다. 일반 종료는 ESTOP을 보내지 않습니다. 송신 중단 뒤 보드의 기존 **1초 초과 타임아웃**으로 HALT가 요청됩니다. PC AUTO 상태에서 발생한 타임아웃은 AUTO를 유지하므로 다음 PC 시험을 다시 시작할 수 있습니다. ASMS MANUAL 타임아웃 뒤 PC로 바꾸려면 ASMS AUTO가 필요합니다.

결과 창을 닫으면 메뉴로 돌아갑니다. 초기 메뉴 `QUIT` 또는 CLI의 Ctrl+C/Ctrl+Z(EOF)도 진행 중인 시험을 일반 종료한 뒤 프로그램을 닫습니다. 그래프 창을 닫는 것은 편집 취소/결과 닫기입니다. CMD의 X 버튼, 작업 관리자 강제 종료, 전원 차단은 Python 정리 절차와 최종 그래프 저장을 보장할 수 없습니다. 실행 중 CSV는 0.5초 간격으로 flush합니다.

HALT는 목표 0 RPM, 전류 지령 0 A, RUNNING 해제 명령입니다. 전원 차단/기계식 브레이크/실제 정지 확인과는 다릅니다. 현재 펌웨어는 RUNNING 상태의 0 RPM에서도 전류 지령을 0 A로 전송합니다. 따라서 위의 2초는 **정지 명령 후 반응을 관측하는 시간**이며 2초 이내 실제 정지나 능동 제동을 보장하지 않습니다. HALT의 실제 제동/관성 동작과 방향은 최초 저속 시험에서 확인해야 합니다.

## 7. 실시간 상태와 단위

입력창 아래 상태는 약 5Hz로 갱신하고, 그래프가 열린 동안을 포함해 약 1초마다 CLI에 상태/경고를 출력합니다. 입력 중인 문자열은 prompt_toolkit이 유지합니다.

```text
PC/running t=3.2s REF=500.0 ACT=498 RPM [OK]
UDP=50Hz CAN=20Hz age UDP/CAN=0.01/0.03s TX=... errors=0 badRX=0 maxGap=20ms
```

| 표시 | 의미 |
|---|---|
| `WAIT_UDP` | 유효 UDP를 아직 수신하지 못함 |
| `WAIT_CAN_ADVANCE` | 첫 측정 후 MCU 시각이 갱신되는지 확인 중 |
| `NO_CAN_SAMPLE` | 보드가 INT32_MIN 전송: CAN RPM 수신 이력 없음 |
| `STALE_CAN` | UDP는 들어오지만 새 CAN 측정 시각이 갱신되지 않음 |
| `NO_UDP` | 유효한 UDP 수신 중단 |
| `OK` | UDP 수신과 CAN 측정 시각 갱신이 최근 확인됨 |

경고 기준은 기본 0.5초이며 `--warning-age 0.8`처럼 변경할 수 있습니다. 이 값은 **경고 표시 기준이며 자동 종료 타임아웃이 아닙니다.** 보드의 제어 명령 수신 타임아웃 1초는 별개이고 변경하지 않습니다. 내부 소켓의 짧은 대기는 프로그램 종료/상태 갱신을 위한 것입니다.

UDP 목표 50Hz와 새 CAN 측정 약 20Hz는 다릅니다. 같은 RPM이라도 MCU 시각이 진행하면 새 측정이고, 값과 시각이 반복되면 새 측정으로 세지 않습니다. MCU uint32 시각 순환은 허용하며 역행(재부팅/재정렬 추정)은 기준을 다시 잡고 갱신 확인 전 정상으로 표시하지 않습니다. PC 시각과 MCU 부팅 시각을 직접 빼지 않습니다.

**입력/그래프 부호 기준:** `REF=+500` → PC UDP speed `+500` → 보드 내부 목표 `-500` → raw 피드백 약 `-500` → 그래프 `ACT=+500`입니다. Python에서 이미 보정된 RPM에 0.198086을 다시 곱하거나 나누지 않습니다. 원본 raw는 로그에 함께 남습니다. `+전진` 표기는 펌웨어 기준이며 실제 방향 일치는 장비에서 확인합니다.

기준 UDP에는 실제 모드·HALT 상태·명령 수락 ACK·핸드셰이크 상태가 없습니다. 따라서 `TX 성공`, `RPM 피드백 정상`, `실제 명령 수락/정지 확인`은 서로 다릅니다. Windows 타이밍은 실시간 보장이 없으므로 송신 간격/최대 간격/주기 지연도 기록합니다.

## 8. 로그와 Git

기본 저장 위치는 이 폴더의 `logs/`입니다. 실행 전체 `*_session`과 시험별 `*_pc_manual`, `*_asms_manual`, `*_auto` 폴더를 생성합니다. 실제 네트워크 시작 시 `*_network_setup`도 생성합니다.

```text
logs/날짜_시간_auto/
  metadata.json   코드 해시/Git 상태/Python/패키지/실행 인수/네트워크/종료 상태
  scenario.json   자동 시나리오(자동 시험만)
  commands.csv   송신 reference, float32 wire 값, joy_x, 성공/오류/간격
  feedback.csv   raw/표시 RPM, MCU 시각, 새 측정 여부, 유효 상태
  events.csv     입력, 모드 요청, 피드백 상태 변화, 종료, 오류
  result.png     기록 종료 후 결과 그래프
```

CSV는 UTF-8 BOM으로 Excel에서 열기 쉽게 저장합니다. 시간은 PC 상대 시각과 epoch를 함께 기록합니다. 시나리오의 0시각은 `run_start` 이벤트이며 `metadata.json`의 `control_start_elapsed_s`로 CSV 기록 시작과의 미세한 차이를 확인할 수 있습니다. 기준 펌웨어 소스의 해시를 저장하지만 실제 보드 업로드 버전이 검증되었다는 의미는 아닙니다.

결과 그래프는 예정 reference(자동 시험), 실제 **송신 성공한** reference, 새 측정 actual을 보여줍니다. 오래된 측정을 새 actual로 연결하지 않으며 경고 구간은 붉은 음영으로 표시합니다. 긴 기록은 그래프 데이터만 간추리고 원본 CSV는 보존합니다. 디스크 오류/로그 큐 누락은 CLI와 메타데이터에 표시합니다.

`.gitignore`는 `.venv/`, `logs/`, Python 캐시를 제외합니다. 실행 파일/의존성 명세/CMD/README/예제 시나리오만 Git에 올리므로 가상환경·패키지·시험 로그 용량이 저장소에 포함되지 않습니다. 필요한 시나리오는 `scenarios/`에 JSON으로 저장하고 커밋할 수 있습니다.

```bat
git status --short
git add Python
git commit -m "Add Windows motor tuning tool"
git push
```

위 Git 명령은 **모터 저장소 루트** 기준입니다. 도구 자체가 commit/push를 자동 수행하지는 않습니다.

## 9. 검증 범위

장비에 연결하기 전 `--dry-run`에서 메뉴/그래프/ramp/step/로그/오류 복구를 확인하고 실제 보드에서는 최초 저속으로 단위·방향·모드 전환·종료 반응을 확인합니다. 이 저장소 작업 환경에서 가능한 소프트웨어 검증과 Windows 실장비 검증은 구분합니다. Linux 임시 컨테이너에서 검증해도 Windows CMD, 방화벽, IP 추가/복구, 실제 모터 동작까지 검증한 것은 아닙니다.
