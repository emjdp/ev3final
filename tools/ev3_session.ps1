<#
  ev3_session.ps1  —  Windows(PowerShell)용 EV3 실기 세션 런처.

  리눅스의 tools/ev3_session.sh (ev3sess) 와 같은 일을 한다:
    1) stages / lib / tools / config 를 scp 로 브릭 ~/ev3final/ 아래에 통째 업로드
       (hardware.py 등 나뉜 파일도 자동 전송 — 파일을 하나씩 드래그할 필요 없음)
    2) 독립 PowerShell 창 여러 개를 띄운다:
         - EV3 stage 실행     (ssh -t)
         - SSH 포트포워딩      (ssh -N -L)
         - telemetry watcher  (python)
         - dashboard          (python, windows-curses 필요)
         - robotctl 대기창

  처음 쓰기 전에 tools\setup_ev3_windows.ps1 를 한 번 실행해서
  OpenSSH / python / windows-curses / SSH 키를 준비하세요.

  예:
    ev3sess                       # 기본 stage(final_run8) 업로드+세션
    ev3sess final_run8
    ev3sess final_run7 -Port 8765
    ev3sess -RobotHost robot@192.168.137.3 final_run8
    ev3sess -NoUpload final_run8
    ev3sess -DryRun final_run8    # 실제 실행 없이 명령만 출력
#>

[CmdletBinding()]
param(
  [Parameter(Position = 0)]
  [string]$Stage = "stages/final_run8.py",

  [string]$RobotHost = "robot@ev3dev.local",
  [string]$RemoteDir = "~/ev3final",
  [int]$Port = 8765,
  [int]$LocalPort = 0,
  [int]$RemotePort = 0,
  [switch]$NoUpload,
  [switch]$NoRobotctl,
  [switch]$DryRun
)

$ErrorActionPreference = "Stop"

if ($LocalPort  -le 0) { $LocalPort  = $Port }
if ($RemotePort -le 0) { $RemotePort = $Port }

# tools\ 의 부모가 repo 루트
$RepoRoot = Split-Path -Parent $PSScriptRoot

function Die($msg) {
  Write-Host "ev3_session: $msg" -ForegroundColor Red
  exit 1
}

function Resolve-PythonCommand {
  # python.org 설치본은 py 런처가 가장 안정적. 없으면 python.
  foreach ($c in @("py", "python")) {
    if (Get-Command $c -ErrorAction SilentlyContinue) { return $c }
  }
  Die "python 을 찾을 수 없습니다. python.org 에서 설치하거나 tools\setup_ev3_windows.ps1 를 실행하세요."
}

# ---- ssh/scp 존재 확인 --------------------------------------------------
foreach ($c in @("ssh", "scp")) {
  if (-not (Get-Command $c -ErrorAction SilentlyContinue)) {
    Die "$c 를 찾을 수 없습니다. OpenSSH 클라이언트를 켜세요(tools\setup_ev3_windows.ps1)."
  }
}
$Py = Resolve-PythonCommand

# ---- stage 경로 해석 (run_maze_v12 / final_run8.py / stages\x.py 모두 허용) ----
function Resolve-StagePath([string]$name) {
  $cands = @(
    (Join-Path $RepoRoot $name),
    (Join-Path $RepoRoot "$name.py"),
    (Join-Path $RepoRoot (Join-Path "stages" $name)),
    (Join-Path $RepoRoot (Join-Path "stages" "$name.py"))
  )
  foreach ($p in $cands) {
    if (Test-Path -LiteralPath $p -PathType Leaf) { return (Resolve-Path -LiteralPath $p).Path }
  }
  return $null
}

$StagePath = Resolve-StagePath $Stage
if (-not $StagePath) { Die "stage 파일을 찾을 수 없습니다: $Stage" }

$stagesDir = Join-Path $RepoRoot "stages"
if (-not $StagePath.StartsWith((Resolve-Path -LiteralPath $stagesDir).Path)) {
  Die "stage 파일은 stages\ 아래에 있어야 합니다: $StagePath"
}
$StageFile = Split-Path -Leaf $StagePath
$StageName = [System.IO.Path]::GetFileNameWithoutExtension($StageFile)

$RemoteBase = $RemoteDir.TrimEnd("/")

# ---- 업로드 -------------------------------------------------------------
function Invoke-Upload {
  $stageFiles  = @(Get-ChildItem -Path (Join-Path $RepoRoot "stages\*.py")  -File -ErrorAction SilentlyContinue)
  $libFiles    = @(Get-ChildItem -Path (Join-Path $RepoRoot "lib\*.py")     -File -ErrorAction SilentlyContinue)
  $toolPy      = @(Get-ChildItem -Path (Join-Path $RepoRoot "tools\*.py")   -File -ErrorAction SilentlyContinue)
  $toolSh      = @(Get-ChildItem -Path (Join-Path $RepoRoot "tools\*.sh")   -File -ErrorAction SilentlyContinue)
  $configFiles = @(Get-ChildItem -Path (Join-Path $RepoRoot "config\*.json") -File -ErrorAction SilentlyContinue)

  if ($stageFiles.Count -eq 0) { Die "업로드할 stages\*.py 가 없습니다: $RepoRoot\stages" }
  if ($libFiles.Count   -eq 0) { Die "업로드할 lib\*.py 가 없습니다: $RepoRoot\lib" }

  Write-Host "[ev3-session] 업로드 대상: ${RobotHost}:${RemoteBase}" -ForegroundColor Cyan

  $mkdir = "mkdir -p $RemoteBase/stages $RemoteBase/lib $RemoteBase/tools $RemoteBase/config"
  Run-Native "ssh" @($RobotHost, $mkdir)
  Run-Native "scp" (@($stageFiles.FullName) + "${RobotHost}:$RemoteBase/stages/")
  Run-Native "scp" (@($libFiles.FullName)   + "${RobotHost}:$RemoteBase/lib/")

  $toolAll = @($toolPy.FullName) + @($toolSh.FullName)
  if ($toolAll.Count -gt 0) {
    Run-Native "scp" ($toolAll + "${RobotHost}:$RemoteBase/tools/")
  }
  if ($configFiles.Count -gt 0) {
    Run-Native "scp" (@($configFiles.FullName) + "${RobotHost}:$RemoteBase/config/")
  }
}

function Run-Native([string]$exe, [object[]]$argList) {
  Write-Host "+ $exe $($argList -join ' ')" -ForegroundColor DarkGray
  if ($DryRun) { return }
  & $exe @argList
  if ($LASTEXITCODE -ne 0) { Die "$exe 실패 (exit $LASTEXITCODE)" }
}

# ---- 창(runner) 생성 및 실행 -------------------------------------------
$TmpDir = Join-Path $env:TEMP ("ev3-session-" + $StageName + "-" + (Get-Random))
$RunnerIndex = 0

function New-Runner([string]$title, [string]$body) {
  if (-not (Test-Path $TmpDir)) { New-Item -ItemType Directory -Path $TmpDir | Out-Null }
  $script:RunnerIndex++
  $safe = ($title -replace '[^A-Za-z0-9_]', '_')
  $file = Join-Path $TmpDir ("{0:D2}-{1}.ps1" -f $RunnerIndex, $safe)
  $content = @"
`$host.UI.RawUI.WindowTitle = '$title'
Set-Location -LiteralPath '$RepoRoot'
Write-Host '[ev3-session] $title' -ForegroundColor Green
Write-Host '[ev3-session] repo: $RepoRoot'
Write-Host ''
$body
Write-Host ''
Write-Host '[ev3-session] 종료: $title. 창을 닫으려면 exit 를 입력하세요.' -ForegroundColor Yellow
"@
  Set-Content -LiteralPath $file -Value $content -Encoding UTF8
  return $file
}

function Start-RunnerWindow([string]$title, [string]$runnerFile) {
  if ($DryRun) {
    Write-Host "+ window '$title': $runnerFile" -ForegroundColor DarkGray
    return
  }
  Start-Process powershell -ArgumentList @(
    "-NoExit", "-ExecutionPolicy", "Bypass", "-File", $runnerFile
  ) | Out-Null
}

# ---- 각 창의 본문 -------------------------------------------------------
$stageBody     = "ssh -t $RobotHost `"cd $RemoteBase && python3 stages/$StageFile`""
$tunnelBody    = "ssh -N -o ExitOnForwardFailure=yes -L ${LocalPort}:127.0.0.1:${RemotePort} $RobotHost"
$watcherBody   = "Start-Sleep -Seconds 2; & $Py tools\telemetry_watcher.py --host 127.0.0.1 --port $LocalPort --stage $StageName"
$dashboardBody = "Start-Sleep -Seconds 3; & $Py tools\dashboard.py --host 127.0.0.1 --port $LocalPort"
$robotctlBody  = @"
Write-Host '자주 쓰는 명령:'
Write-Host '  $Py tools\robotctl.py describe'
Write-Host '  $Py tools\robotctl.py latest'
Write-Host '  $Py tools\robotctl.py do bench_toggle'
Write-Host '  $Py tools\robotctl.py do read_reflect'
Write-Host '  $Py tools\robotctl.py do read_color'
Write-Host '  $Py tools\robotctl.py stop'
Write-Host ''
"@

# ---- 실행 ---------------------------------------------------------------
if ($NoUpload) {
  Write-Host "[ev3-session] -NoUpload: scp 업로드를 건너뜁니다" -ForegroundColor Yellow
} else {
  Invoke-Upload
}

Write-Host "[ev3-session] stage=$StageName, host=$RobotHost, port=$LocalPort->$RemotePort" -ForegroundColor Cyan

$r1 = New-Runner "EV3 $StageName" $stageBody
$r2 = New-Runner "EV3 tunnel $LocalPort" $tunnelBody
$r3 = New-Runner "EV3 watcher" $watcherBody
$r4 = New-Runner "EV3 dashboard" $dashboardBody

Start-RunnerWindow "EV3 $StageName" $r1
Start-RunnerWindow "EV3 tunnel $LocalPort" $r2
Start-RunnerWindow "EV3 watcher" $r3
Start-RunnerWindow "EV3 dashboard" $r4

if (-not $NoRobotctl) {
  $r5 = New-Runner "EV3 robotctl" $robotctlBody
  Start-RunnerWindow "EV3 robotctl" $r5
}

if (-not $DryRun) {
  Write-Host "[ev3-session] 창을 열었습니다. runner: $TmpDir" -ForegroundColor Cyan
}
