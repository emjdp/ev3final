<#
============================================================================
  setup_ev3_windows.ps1  —  윈도우 팀원용 1회 설치 스크립트
============================================================================

  이 스크립트를 한 번만 실행하면 ev3sess(ev3_session.ps1) 세션을 쓸 준비가 끝납니다.
  하는 일:
    1) OpenSSH 클라이언트(ssh/scp) 확인 — 없으면 켜는 방법 안내
    2) python 확인 — 없으면 설치 방법 안내
    3) pip install windows-curses   (dashboard.py 의 curses 화면용)
    4) SSH 키 생성 + 브릭에 등록   (이후 비밀번호 'maker' 를 매번 안 침)
    5) 스크립트 실행 정책 완화(CurrentUser) + `ev3sess` 명령 등록(프로필)

  ── 실행 방법 ────────────────────────────────────────────────────────────
    PowerShell 을 열고 repo 폴더에서:

      powershell -ExecutionPolicy Bypass -File tools\setup_ev3_windows.ps1

    브릭 주소가 ev3dev.local 이 아니면 IP 로:

      powershell -ExecutionPolicy Bypass -File tools\setup_ev3_windows.ps1 -RobotHost robot@192.168.137.3

  ── 실행 후 사용법 ────────────────────────────────────────────────────────
    PowerShell 새 창에서:

      ev3sess                 # 기본 stage 업로드 + 세션 창들 열기
      ev3sess final_run8
      ev3sess -RobotHost robot@192.168.137.3 final_run8

    자세한 사용법은 repo 의 ev3dev_connection_guide.md 참고.
============================================================================
#>

[CmdletBinding()]
param(
  [string]$RobotHost = "robot@ev3dev.local"
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$SessionScript = Join-Path $RepoRoot "tools\ev3_session.ps1"

function Section($n, $t) { Write-Host "`n[$n] $t" -ForegroundColor Cyan }
function Ok($t)   { Write-Host "  OK  $t" -ForegroundColor Green }
function Warn($t) { Write-Host "  !!  $t" -ForegroundColor Yellow }

Write-Host "=== EV3 윈도우 셋업 시작 (host=$RobotHost) ===" -ForegroundColor Magenta

# ---- 1) OpenSSH 클라이언트 ------------------------------------------------
Section 1 "OpenSSH 클라이언트(ssh/scp) 확인"
$hasSsh = (Get-Command ssh -ErrorAction SilentlyContinue) -and (Get-Command scp -ErrorAction SilentlyContinue)
if ($hasSsh) {
  Ok "ssh / scp 사용 가능"
} else {
  Warn "ssh/scp 가 없습니다. 관리자 PowerShell 에서 아래를 실행해 켜세요:"
  Write-Host '     Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0' -ForegroundColor White
  Write-Host "     (또는 설정 > 앱 > 선택적 기능 > OpenSSH 클라이언트 추가)" -ForegroundColor White
  Warn "설치 후 이 스크립트를 다시 실행하세요."
}

# ---- 2) python ------------------------------------------------------------
Section 2 "python 확인"
$pyCmd = $null
foreach ($c in @("py", "python")) {
  if (Get-Command $c -ErrorAction SilentlyContinue) { $pyCmd = $c; break }
}
if ($pyCmd) {
  $ver = & $pyCmd --version 2>&1
  Ok "python 사용 가능 ($pyCmd — $ver)"
} else {
  Warn "python 이 없습니다. 아래 중 하나로 설치하세요:"
  Write-Host "     winget install Python.Python.3.12" -ForegroundColor White
  Write-Host "     또는 https://www.python.org/downloads/ 에서 설치 (Add to PATH 체크)" -ForegroundColor White
  Warn "설치 후 이 스크립트를 다시 실행하세요."
}

# ---- 3) windows-curses ----------------------------------------------------
Section 3 "windows-curses 설치 (dashboard 화면용)"
if ($pyCmd) {
  try {
    & $pyCmd -m pip install --user windows-curses
    Ok "windows-curses 준비 완료"
  } catch {
    Warn "windows-curses 설치 실패. dashboard 창은 안 뜰 수 있지만 나머지는 동작합니다."
    Warn "수동 설치: $pyCmd -m pip install windows-curses"
  }
} else {
  Warn "python 이 없어 건너뜀."
}

# ---- 4) SSH 키 생성 + 브릭 등록 -------------------------------------------
Section 4 "SSH 키 (비밀번호 없이 접속)"
$sshDir = Join-Path $env:USERPROFILE ".ssh"
$keyPath = Join-Path $sshDir "id_ed25519"
$pubPath = "$keyPath.pub"

if (-not (Test-Path $sshDir)) { New-Item -ItemType Directory -Path $sshDir | Out-Null }
if (Test-Path $pubPath) {
  Ok "키가 이미 있습니다: $pubPath"
} elseif ($hasSsh) {
  Write-Host "  키를 새로 만듭니다. 'Enter passphrase' 가 나오면 그냥 Enter 두 번(암호 비움)..." -ForegroundColor White
  & ssh-keygen -t ed25519 -f $keyPath
}

if ($hasSsh -and (Test-Path $pubPath)) {
  Write-Host "  공개키를 브릭($RobotHost)에 등록합니다. 이때 비밀번호 'maker' 를 한 번 칩니다..." -ForegroundColor White
  $pub = Get-Content $pubPath -Raw
  $remoteCmd = "mkdir -p ~/.ssh && chmod 700 ~/.ssh && grep -qxF '$($pub.Trim())' ~/.ssh/authorized_keys 2>/dev/null || echo '$($pub.Trim())' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
  try {
    & ssh $RobotHost $remoteCmd
    Ok "공개키 등록 완료. 이제 비밀번호 없이 접속됩니다."
  } catch {
    Warn "키 등록 실패(브릭 연결/주소 확인). 나중에 다시 실행하거나 수동 등록하세요."
  }
} else {
  Warn "ssh 또는 공개키가 없어 키 등록을 건너뜀."
}

# ---- 5) 실행 정책 + ev3sess 명령 등록 -------------------------------------
Section 5 "스크립트 실행 정책 + ev3sess 명령 등록"
try {
  Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned -Force
  Ok "실행 정책(CurrentUser) = RemoteSigned"
} catch {
  Warn "실행 정책 변경 실패. ev3sess 실행이 막히면 -ExecutionPolicy Bypass 로 직접 호출하세요."
}

# PowerShell 프로필에 ev3sess 함수 등록 (중복 방지)
$profilePath = $PROFILE
$profileDir = Split-Path -Parent $profilePath
if (-not (Test-Path $profileDir)) { New-Item -ItemType Directory -Path $profileDir | Out-Null }
if (-not (Test-Path $profilePath)) { New-Item -ItemType File -Path $profilePath | Out-Null }

$marker = "# >>> ev3sess >>>"
$profileText = Get-Content $profilePath -Raw -ErrorAction SilentlyContinue
if ($profileText -and $profileText.Contains($marker)) {
  Ok "ev3sess 명령이 이미 등록되어 있습니다."
} else {
  $funcBlock = @"

$marker
function ev3sess { & '$SessionScript' @args }
# <<< ev3sess <<<
"@
  Add-Content -Path $profilePath -Value $funcBlock
  Ok "ev3sess 명령 등록 완료 (프로필: $profilePath)"
  Warn "새 PowerShell 창을 열어야 ev3sess 명령이 인식됩니다."
}

Write-Host "`n=== 셋업 완료 ===" -ForegroundColor Magenta
Write-Host "새 PowerShell 창에서:  ev3sess final_run8" -ForegroundColor White
Write-Host "자세한 사용법: ev3dev_connection_guide.md" -ForegroundColor White
