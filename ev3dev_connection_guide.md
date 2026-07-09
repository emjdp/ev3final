# EV3 dev SSH 접속 및 개발 환경 설정 가이드

본 문서는 EV3 브릭(ev3dev OS)에 SSH로 접속하고, **코드를 올려서 실행하는 세션**을 여는 방법을 윈도우 및 우분투 환경 모두에 대해 설명합니다.

---

## 1. 기본 접속 정보 (Credentials)

EV3 브릭의 기본 로그인 계정 정보는 다음과 같습니다. (ID가 `ev3dev`가 아니라 `robot`임에 주의해 주세요!)

*   **아이디 (Username):** `robot`
*   **비밀번호 (Password):** `maker`
*   **기본 호스트 이름 (Hostname):** `ev3dev.local`

> **주소가 두 가지입니다.**
> *   `ev3dev.local` — mDNS(Bonjour) 이름. 보통 이걸 씁니다.
> *   `192.168.x.x` 같은 **IP 주소** — mDNS가 안 되는 환경에서 사용. 브릭 LCD 화면 왼쪽 상단에 표시됩니다. 이 IP는 연결할 때마다 **바뀔 수 있으니** 접속이 안 되면 화면에서 다시 확인하세요.

---

## 2. 네트워크 연결 및 SSH 접속 방법

### 2.1. SSH 직접 접속 (터미널 / PowerShell)

윈도우 10/11 및 우분투는 자체 터미널에서 SSH 명령어를 직접 사용할 수 있습니다.

1.  **네트워크 연결:** EV3 브릭을 USB 케이블(테더링), Wi-Fi, 혹은 이더넷을 통해 PC와 연결합니다.
2.  **터미널 실행:**
    *   **윈도우:** `PowerShell` 실행
    *   **우분투:** `터미널` 실행
3.  **SSH 명령어 입력:**
    ```bash
    ssh robot@ev3dev.local
    ```
4.  **로그인 진행:**
    *   처음 접속 시 신뢰성 확인(`Are you sure you want to continue connecting?`)에 **`yes`**를 입력합니다.
    *   비밀번호 창이 뜨면 **`maker`**를 입력합니다. (보안상 화면에 글자가 표시되지 않으니 그대로 타이핑 후 Enter.)

---

## 3. 윈도우 팀원 — 코드 올리고 세션 열기 (권장)

VS Code 확장으로 파일을 하나씩 올릴 필요 없이, **PowerShell 스크립트 한 방으로** `stages / lib / tools / config` 를 브릭에 통째로 업로드하고 실행/터널/모니터 창들을 자동으로 띄웁니다. 리눅스의 `ev3sess`와 동일한 워크플로우입니다.

### 3.1. 처음 한 번만 — 셋업

repo 폴더에서 PowerShell 을 열고 아래를 **한 번** 실행합니다.

```powershell
powershell -ExecutionPolicy Bypass -File tools\setup_ev3_windows.ps1
```

이 스크립트가 자동으로 처리합니다:

1.  **OpenSSH 클라이언트**(ssh/scp) 확인 — 윈도우 10/11엔 보통 기본 내장. 없으면 켜는 방법을 안내합니다.
2.  **python** 확인 — 없으면 설치 방법(`winget install Python.Python.3.12` 등)을 안내합니다.
3.  **`windows-curses` 설치** — dashboard 화면(curses)이 윈도우에서 돌아가게 합니다.
4.  **SSH 키 생성 + 브릭 등록** — 이때 비밀번호 `maker`를 **딱 한 번** 칩니다. 이후로는 업로드/실행 창마다 비밀번호를 다시 치지 않습니다.
5.  **`ev3sess` 명령 등록** — PowerShell 프로필에 등록되어, 이후 어느 창에서든 `ev3sess`로 실행할 수 있습니다.

> 브릭 주소가 `ev3dev.local`이 아니라 IP라면 셋업에도 주소를 넘기세요:
> ```powershell
> powershell -ExecutionPolicy Bypass -File tools\setup_ev3_windows.ps1 -RobotHost robot@192.168.137.3
> ```
> **셋업이 끝나면 PowerShell 창을 새로 열어야** `ev3sess` 명령이 인식됩니다.

### 3.2. 사용법 — 세션 열기

새 PowerShell 창에서:

```powershell
ev3sess                 # 기본 stage(final_run8) 업로드 + 세션 창 열기
ev3sess final_run8      # stage 지정
ev3sess final_run7
ev3sess -NoUpload final_run8            # 업로드 없이 실행만
ev3sess -RobotHost robot@192.168.137.3 final_run8   # IP로 접속
ev3sess -DryRun final_run8              # 실제 실행 없이 명령만 미리 보기
```

실행하면 다음 창들이 자동으로 뜹니다:

| 창 | 역할 |
|---|---|
| **EV3 stage** | 브릭에서 해당 stage 를 실행 (`ssh -t`) |
| **EV3 tunnel** | 튜닝/telemetry용 SSH 포트포워딩 (`8765`) |
| **EV3 watcher** | telemetry 를 받아 `runs/` 에 기록 |
| **EV3 dashboard** | 실시간 상태 대시보드 (curses) |
| **EV3 robotctl** | `robotctl.py` 로 수동 명령을 보내는 대기창 |

주요 옵션: `-Stage`, `-RobotHost`, `-Port`, `-NoUpload`, `-NoRobotctl`, `-DryRun`. 자세한 내용은 `tools\ev3_session.ps1` 상단 주석 참고.

### 3.3. 파일만 빠르게 올리고 싶을 때 (대안)

세션 없이 파일만 브릭에 올리려면 PowerShell 에서 직접 `scp` 를 씁니다.

```powershell
scp stages\final_run8.py robot@ev3dev.local:~/ev3final/stages/
ssh robot@ev3dev.local "cd ~/ev3final && python3 stages/final_run8.py"
```

---

## 4. (참고) 리눅스/우분투 — `ev3sess`

우분투에서는 `tools/install_ev3sess.sh` 를 한 번 실행하면 `ev3sess run_maze` 처럼 쓸 수 있습니다. 동작은 3.2와 동일하며, GUI 터미널 또는 tmux 창으로 세션을 엽니다. `~/.ssh/config` 에 `ev3` 호스트 별칭을 등록해 두고 사용합니다.

---

## 5. 접속 문제 해결 가이드 (Troubleshooting)

### Q. `ev3dev.local`을 찾을 수 없다고 나옵니다. (Hostname Resolution 에러)
*   **원인:** PC에서 `mDNS` 서비스가 제대로 작동하지 않는 경우.
*   **해결책 1 (IP로 직접 접속):** 브릭 LCD 화면 왼쪽 상단의 IP 주소를 확인해 직접 접속합니다.
    ```bash
    ssh robot@<EV3에_표시된_IP_주소>
    ```
    `ev3sess` 도 `-RobotHost robot@<IP>` 로 넘기면 됩니다.
*   **해결책 2 (윈도우 mDNS):** `ev3dev.local` 이름 해석이 필요하면 Apple Bonjour 서비스가 도움이 될 수 있으나, 보통은 위의 **IP 직접 접속**이 가장 확실합니다.

### Q. `ev3sess` 명령을 못 찾습니다.
*   셋업(`setup_ev3_windows.ps1`) 후 **PowerShell 창을 새로 열었는지** 확인하세요.
*   그래도 안 되면 스크립트를 직접 호출: `powershell -ExecutionPolicy Bypass -File tools\ev3_session.ps1 final_run8`

### Q. 창마다 비밀번호(`maker`)를 계속 묻습니다.
*   SSH 키 등록이 안 된 것입니다. `setup_ev3_windows.ps1` 을 다시 실행해 4단계(키 등록)를 완료하세요.

### Q. dashboard 창만 바로 닫히거나 오류가 납니다.
*   `windows-curses` 가 없을 때 발생합니다. `py -m pip install windows-curses` (또는 `python -m pip install windows-curses`) 로 설치하세요. 나머지 창은 이것 없이도 동작합니다.

### Q. `ssh`/`scp` 가 없다고 합니다. (윈도우)
*   관리자 PowerShell 에서 OpenSSH 클라이언트를 켭니다.
    ```powershell
    Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0
    ```
    또는 **설정 > 앱 > 선택적 기능 > OpenSSH 클라이언트 추가**.

### Q. USB 연결 시 네트워크 인식이 되지 않습니다.
*   브릭 설정(`Wireless and Networks` → `All Network Connections`)에서 USB 인터페이스가 활성화되어 있는지 확인하세요.
