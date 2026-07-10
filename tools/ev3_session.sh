#!/usr/bin/env bash
# EV3 실기 세션을 한 번에 준비한다.
#
# 기본 동작:
#   1) stages/old_stages/lib/tools/config 를 scp 로 브릭 ~/ev3final/ 아래에 업로드
#   2) 브릭 stage 실행 터미널
#   3) SSH 포트포워딩 터미널
#   4) telemetry watcher 터미널
#   5) dashboard 터미널
#   6) robotctl 대기 터미널
#
# 예:
#   tools/ev3_session.sh
#   tools/ev3_session.sh run_maze
#   tools/ev3_session.sh --stage stages/stage3v2_linetrace_branch.py
#   tools/ev3_session.sh --stage stage4_color --host ev3 --terminal tmux

set -euo pipefail

SOURCE="${BASH_SOURCE[0]}"
while [[ -h "${SOURCE}" ]]; do
  SOURCE_DIR="$(cd -P "$(dirname "${SOURCE}")" && pwd)"
  SOURCE_TARGET="$(readlink "${SOURCE}")"
  case "${SOURCE_TARGET}" in
    /*) SOURCE="${SOURCE_TARGET}" ;;
    *) SOURCE="${SOURCE_DIR}/${SOURCE_TARGET}" ;;
  esac
done

SCRIPT_DIR="$(cd -P "$(dirname "${SOURCE}")" && pwd)"
if [[ -n "${EV3FINAL_REPO:-}" ]]; then
  REPO_ROOT="$(cd "${EV3FINAL_REPO}" && pwd)"
else
  REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
fi

ROBOT_HOST="ev3"
REMOTE_DIR="~/ev3final"
STAGE_INPUT="stages/gg8.py"
LOCAL_PORT="8765"
REMOTE_PORT="8765"
TERMINAL="auto"
UPLOAD="1"
OPEN_ROBOTCTL="1"
DRY_RUN="0"
TMUX_SESSION=""
STAGE_FROM_ARG="0"
DASHBOARD_KIND="dashboard"
PAD_HZ="10"

usage() {
  cat <<'EOF'
Usage: tools/ev3_session.sh [options] [stage_name_or_path]

EV3 실기 세션을 여러 터미널로 한 번에 띄운다.

Examples:
  ev3sess run_maze
  ev3sess stage4v2_color_follow
  ev3sess stages/stage4_color.py --terminal tmux

Branch-safe launcher:
  tools/install_ev3sess.sh 를 한 번 실행하면 ~/.local/bin/ev3sess 로 설치된다.
  이후 다른 브랜치에 이 파일이 없어도 ev3sess run_maze 처럼 실행할 수 있다.

Options:
  -s, --stage PATH_OR_NAME   실행할 stage 파일. 기본: stages/gg8.py
                             예: stages/gg8.py, gg8, old_stages/run_maze_v12.py
  -h, --host HOST            SSH/scp 대상. 기본: ev3
  -r, --remote-dir DIR       브릭 작업 디렉토리. 기본: ~/ev3final
  -p, --port PORT            로컬/브릭 튜닝 포트. 기본: 8765
      --local-port PORT      로컬 포트만 지정
      --remote-port PORT     브릭 포트만 지정
  -t, --terminal NAME        auto, gnome-terminal, konsole, xfce4-terminal,
                             mate-terminal, kitty, alacritty, xterm, tmux 중 하나
      --tmux-session NAME    tmux 세션 이름. 기본: ev3final-<stage>
  -n, --no-upload            scp 업로드를 건너뛴다
      --dashboard KIND       조종 UI. dashboard, aplus-pad, both. 기본: dashboard
      --pad-hz HZ            aplus-pad 텔레메트리 폴링 Hz. 기본: 10
      --no-robotctl          robotctl 대기 터미널을 열지 않는다
      --dry-run              실행할 업로드/터미널 명령만 출력한다
      --help                 도움말 출력
EOF
}

die() {
  echo "ev3_session: $*" >&2
  exit 1
}

q() {
  printf '%q' "$1"
}

has_command() {
  command -v "$1" >/dev/null 2>&1
}

while (($#)); do
  case "$1" in
    -s|--stage)
      [[ $# -ge 2 ]] || die "--stage 값이 필요합니다"
      [[ "${STAGE_FROM_ARG}" == "0" ]] || die "stage 는 한 번만 지정하세요"
      STAGE_INPUT="$2"
      STAGE_FROM_ARG="1"
      shift 2
      ;;
    -h|--host)
      [[ $# -ge 2 ]] || die "--host 값이 필요합니다"
      ROBOT_HOST="$2"
      shift 2
      ;;
    -r|--remote-dir)
      [[ $# -ge 2 ]] || die "--remote-dir 값이 필요합니다"
      REMOTE_DIR="$2"
      shift 2
      ;;
    -p|--port)
      [[ $# -ge 2 ]] || die "--port 값이 필요합니다"
      LOCAL_PORT="$2"
      REMOTE_PORT="$2"
      shift 2
      ;;
    --local-port)
      [[ $# -ge 2 ]] || die "--local-port 값이 필요합니다"
      LOCAL_PORT="$2"
      shift 2
      ;;
    --remote-port)
      [[ $# -ge 2 ]] || die "--remote-port 값이 필요합니다"
      REMOTE_PORT="$2"
      shift 2
      ;;
    -t|--terminal)
      [[ $# -ge 2 ]] || die "--terminal 값이 필요합니다"
      TERMINAL="$2"
      shift 2
      ;;
    --tmux-session)
      [[ $# -ge 2 ]] || die "--tmux-session 값이 필요합니다"
      TMUX_SESSION="$2"
      shift 2
      ;;
    -n|--no-upload)
      UPLOAD="0"
      shift
      ;;
    --dashboard)
      [[ $# -ge 2 ]] || die "--dashboard 값이 필요합니다"
      DASHBOARD_KIND="$2"
      shift 2
      ;;
    --pad-hz)
      [[ $# -ge 2 ]] || die "--pad-hz 값이 필요합니다"
      PAD_HZ="$2"
      shift 2
      ;;
    --no-robotctl)
      OPEN_ROBOTCTL="0"
      shift
      ;;
    --dry-run)
      DRY_RUN="1"
      shift
      ;;
    --help)
      usage
      exit 0
      ;;
    *)
      case "$1" in
        -*)
          die "알 수 없는 옵션입니다: $1"
          ;;
      esac
      [[ "${STAGE_FROM_ARG}" == "0" ]] || die "stage 는 한 번만 지정하세요: ${STAGE_INPUT}, $1"
      STAGE_INPUT="$1"
      STAGE_FROM_ARG="1"
      shift
      ;;
  esac
done

case "${LOCAL_PORT}:${REMOTE_PORT}" in
  *[!0-9:]*|:*|*:) die "포트는 숫자여야 합니다: local=${LOCAL_PORT}, remote=${REMOTE_PORT}" ;;
esac
case "${DASHBOARD_KIND}" in
  dashboard|aplus-pad|both) ;;
  *) die "--dashboard 는 dashboard, aplus-pad, both 중 하나여야 합니다: ${DASHBOARD_KIND}" ;;
esac
case "${PAD_HZ}" in
  *[!0-9.]*|""|.*|*.) die "--pad-hz 는 숫자여야 합니다: ${PAD_HZ}" ;;
esac

resolve_stage_path() {
  local input="$1"
  local candidate

  if [[ -f "${REPO_ROOT}/${input}" ]]; then
    printf '%s\n' "${REPO_ROOT}/${input}"
    return
  fi

  if [[ "${input}" != *.py && -f "${REPO_ROOT}/${input}.py" ]]; then
    printf '%s\n' "${REPO_ROOT}/${input}.py"
    return
  fi

  candidate="stages/${input}"
  if [[ -f "${REPO_ROOT}/${candidate}" ]]; then
    printf '%s\n' "${REPO_ROOT}/${candidate}"
    return
  fi

  candidate="stages/${input}.py"
  if [[ "${input}" != *.py && -f "${REPO_ROOT}/${candidate}" ]]; then
    printf '%s\n' "${REPO_ROOT}/${candidate}"
    return
  fi

  candidate="old_stages/${input}"
  if [[ -f "${REPO_ROOT}/${candidate}" ]]; then
    printf '%s\n' "${REPO_ROOT}/${candidate}"
    return
  fi

  candidate="old_stages/${input}.py"
  if [[ "${input}" != *.py && -f "${REPO_ROOT}/${candidate}" ]]; then
    printf '%s\n' "${REPO_ROOT}/${candidate}"
    return
  fi

  return 1
}

STAGE_PATH="$(resolve_stage_path "${STAGE_INPUT}")" || die "stage 파일을 찾을 수 없습니다: ${STAGE_INPUT}"
STAGE_REL="${STAGE_PATH#"${REPO_ROOT}/"}"
STAGE_FILE="$(basename "${STAGE_PATH}")"
STAGE_NAME="${STAGE_FILE%.py}"

case "${STAGE_PATH}" in
  "${REPO_ROOT}/stages/"*) ;;
  "${REPO_ROOT}/old_stages/"*) ;;
  *) die "stage 파일은 stages/ 또는 old_stages/ 아래에 있어야 합니다: ${STAGE_PATH}" ;;
esac

case "${REMOTE_DIR}" in
  *[[:space:]]*) die "--remote-dir 에 공백은 지원하지 않습니다: ${REMOTE_DIR}" ;;
esac

if [[ -z "${TMUX_SESSION}" ]]; then
  TMUX_SESSION="ev3final-${STAGE_NAME//[^A-Za-z0-9_]/_}"
fi

choose_terminal() {
  if [[ "${TERMINAL}" != "auto" ]]; then
    printf '%s\n' "${TERMINAL}"
    return
  fi

  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    for candidate in gnome-terminal konsole xfce4-terminal mate-terminal kitty alacritty xterm; do
      if has_command "${candidate}"; then
        printf '%s\n' "${candidate}"
        return
      fi
    done
  fi

  if has_command tmux; then
    printf '%s\n' "tmux"
    return
  fi

  printf '%s\n' "none"
}

SELECTED_TERMINAL="$(choose_terminal)"
if [[ "${SELECTED_TERMINAL}" != "none" && "${SELECTED_TERMINAL}" != "tmux" ]] && ! has_command "${SELECTED_TERMINAL}"; then
  die "터미널 실행 파일을 찾을 수 없습니다: ${SELECTED_TERMINAL}"
fi
if [[ "${SELECTED_TERMINAL}" == "tmux" ]] && ! has_command tmux; then
  die "tmux 를 찾을 수 없습니다"
fi

REMOTE_BASE="${REMOTE_DIR%/}"
REMOTE_STAGE_CMD="cd ${REMOTE_BASE} && python3 ${STAGE_REL}"
TUNNEL_SPEC="${LOCAL_PORT}:127.0.0.1:${REMOTE_PORT}"
TMP_DIR=""
RUNNERS=()
RUNNER_RESULT=""

make_runner() {
  local title="$1"
  local body="$2"
  local file

  if [[ -z "${TMP_DIR}" ]]; then
    TMP_DIR="$(mktemp -d "/tmp/ev3-session-${STAGE_NAME}.XXXXXX")"
  fi
  file="${TMP_DIR}/$(printf '%02d' "$(( ${#RUNNERS[@]} + 1 ))")-${title//[^A-Za-z0-9_]/_}.sh"
  cat >"${file}" <<EOF
#!/usr/bin/env bash
set -u
cd $(q "${REPO_ROOT}")
printf '\\033]0;%s\\007' $(q "${title}")
echo "[ev3-session] ${title}"
echo "[ev3-session] repo: ${REPO_ROOT}"
echo
${body}
status=\$?
echo
echo "[ev3-session] 종료: ${title} (exit \${status})"
echo "[ev3-session] 창을 닫으려면 Ctrl-D 또는 exit 를 입력하세요."
exec bash
EOF
  chmod +x "${file}"
  RUNNERS+=("${file}")
  RUNNER_RESULT="${file}"
}

run_or_print() {
  echo "+ $*"
  if [[ "${DRY_RUN}" == "0" ]]; then
    "$@"
  fi
}

upload_files() {
  local stage_files=("${REPO_ROOT}/stages/"*.py)
  local old_stage_files=("${REPO_ROOT}/old_stages/"*.py)
  local lib_files=("${REPO_ROOT}/lib/"*.py)
  local tool_files=("${REPO_ROOT}/tools/"*.py)
  local tool_shell_files=("${REPO_ROOT}/tools/"*.sh)
  local config_files=("${REPO_ROOT}/config/"*.json)

  shopt -s nullglob
  stage_files=("${REPO_ROOT}/stages/"*.py)
  old_stage_files=("${REPO_ROOT}/old_stages/"*.py)
  lib_files=("${REPO_ROOT}/lib/"*.py)
  tool_files=("${REPO_ROOT}/tools/"*.py)
  tool_shell_files=("${REPO_ROOT}/tools/"*.sh)
  config_files=("${REPO_ROOT}/config/"*.json)
  shopt -u nullglob

  [[ "${#stage_files[@]}" -gt 0 ]] || die "업로드할 stages/*.py 파일이 없습니다: ${REPO_ROOT}/stages"
  [[ "${#lib_files[@]}" -gt 0 ]] || die "업로드할 lib/*.py 파일이 없습니다: ${REPO_ROOT}/lib"
  [[ "${#tool_files[@]}" -gt 0 ]] || die "업로드할 tools/*.py 파일이 없습니다: ${REPO_ROOT}/tools"

  echo "[ev3-session] 업로드 대상: ${ROBOT_HOST}:${REMOTE_BASE}"
  run_or_print ssh "${ROBOT_HOST}" "mkdir -p ${REMOTE_BASE}/stages ${REMOTE_BASE}/old_stages ${REMOTE_BASE}/lib ${REMOTE_BASE}/tools ${REMOTE_BASE}/config"
  run_or_print scp "${stage_files[@]}" "${ROBOT_HOST}:${REMOTE_BASE}/stages/"
  if [[ "${#old_stage_files[@]}" -gt 0 ]]; then
    run_or_print scp "${old_stage_files[@]}" "${ROBOT_HOST}:${REMOTE_BASE}/old_stages/"
  fi
  run_or_print scp "${lib_files[@]}" "${ROBOT_HOST}:${REMOTE_BASE}/lib/"
  if [[ "${#tool_shell_files[@]}" -gt 0 ]]; then
    run_or_print scp "${tool_files[@]}" "${tool_shell_files[@]}" "${ROBOT_HOST}:${REMOTE_BASE}/tools/"
  else
    run_or_print scp "${tool_files[@]}" "${ROBOT_HOST}:${REMOTE_BASE}/tools/"
  fi
  if [[ "${#config_files[@]}" -gt 0 ]]; then
    run_or_print scp "${config_files[@]}" "${ROBOT_HOST}:${REMOTE_BASE}/config/"
  fi
}

launch_terminal() {
  local title="$1"
  local runner="$2"

  if [[ "${DRY_RUN}" == "1" ]]; then
    echo "+ terminal(${SELECTED_TERMINAL}) ${title}: ${runner}"
    return
  fi

  case "${SELECTED_TERMINAL}" in
    gnome-terminal)
      gnome-terminal --title="${title}" -- "${runner}" &
      ;;
    konsole)
      konsole --new-tab --workdir "${REPO_ROOT}" -p "tabtitle=${title}" -e "${runner}" &
      ;;
    xfce4-terminal)
      xfce4-terminal --title="${title}" --execute "${runner}" &
      ;;
    mate-terminal)
      mate-terminal --title="${title}" -- "${runner}" &
      ;;
    kitty)
      kitty --title "${title}" "${runner}" &
      ;;
    alacritty)
      alacritty --title "${title}" -e "${runner}" &
      ;;
    xterm)
      xterm -T "${title}" -e "${runner}" &
      ;;
    tmux)
      launch_tmux_window "${title}" "${runner}"
      ;;
    none)
      echo "[ev3-session] GUI 터미널/tmux 를 찾지 못했습니다. 직접 실행하세요: ${runner}"
      ;;
    *)
      die "지원하지 않는 터미널입니다: ${SELECTED_TERMINAL}"
      ;;
  esac
}

launch_tmux_window() {
  local title="$1"
  local runner="$2"

  if ! tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
    tmux new-session -d -s "${TMUX_SESSION}" -n "${title}" "${runner}"
  else
    tmux new-window -t "${TMUX_SESSION}" -n "${title}" "${runner}"
  fi
}

stage_body="ssh -t $(q "${ROBOT_HOST}") $(q "${REMOTE_STAGE_CMD}")"
tunnel_body="ssh -N -o ExitOnForwardFailure=yes -L $(q "${TUNNEL_SPEC}") $(q "${ROBOT_HOST}")"
watcher_body="sleep 2; python3 tools/telemetry_watcher.py --host 127.0.0.1 --port $(q "${LOCAL_PORT}") --stage $(q "${STAGE_NAME}")"
case "${DASHBOARD_KIND}" in
  dashboard)
    dashboard_titles=("EV3 dashboard")
    dashboard_bodies=("sleep 3; python3 tools/dashboard.py --host 127.0.0.1 --port $(q "${LOCAL_PORT}")")
    ;;
  aplus-pad)
    dashboard_titles=("EV3 aplus pad")
    dashboard_bodies=("sleep 3; python3 tools/aplus_pad.py --host 127.0.0.1 --port $(q "${LOCAL_PORT}") --hz $(q "${PAD_HZ}")")
    ;;
  both)
    dashboard_titles=("EV3 aplus pad" "EV3 dashboard")
    dashboard_bodies=(
      "sleep 3; python3 tools/aplus_pad.py --host 127.0.0.1 --port $(q "${LOCAL_PORT}") --hz $(q "${PAD_HZ}")"
      "sleep 3; python3 tools/dashboard.py --host 127.0.0.1 --port $(q "${LOCAL_PORT}")"
    )
    ;;
esac
robotctl_body=$(cat <<EOF
echo "자주 쓰는 명령:"
echo "  python3 tools/robotctl.py describe"
echo "  python3 tools/robotctl.py latest"
echo "  python3 tools/robotctl.py do bench_toggle"
echo "  python3 tools/robotctl.py do read_reflect"
echo "  python3 tools/robotctl.py do read_color"
echo "  python3 tools/robotctl.py stop"
echo
exec bash
EOF
)

if [[ "${UPLOAD}" == "1" ]]; then
  upload_files
else
  echo "[ev3-session] --no-upload: scp 업로드를 건너뜁니다"
fi

echo "[ev3-session] stage=${STAGE_NAME}, host=${ROBOT_HOST}, terminal=${SELECTED_TERMINAL}, port=${LOCAL_PORT}->${REMOTE_PORT}, dashboard=${DASHBOARD_KIND}"

make_runner "EV3 ${STAGE_NAME}" "${stage_body}"
stage_runner="${RUNNER_RESULT}"
make_runner "EV3 tunnel ${LOCAL_PORT}" "${tunnel_body}"
tunnel_runner="${RUNNER_RESULT}"
make_runner "EV3 watcher" "${watcher_body}"
watcher_runner="${RUNNER_RESULT}"
dashboard_runners=()
for i in "${!dashboard_titles[@]}"; do
  make_runner "${dashboard_titles[$i]}" "${dashboard_bodies[$i]}"
  dashboard_runners+=("${RUNNER_RESULT}")
done

launch_terminal "EV3 ${STAGE_NAME}" "${stage_runner}"
launch_terminal "EV3 tunnel ${LOCAL_PORT}" "${tunnel_runner}"
launch_terminal "EV3 watcher" "${watcher_runner}"
for i in "${!dashboard_titles[@]}"; do
  launch_terminal "${dashboard_titles[$i]}" "${dashboard_runners[$i]}"
done

if [[ "${OPEN_ROBOTCTL}" == "1" ]]; then
  make_runner "EV3 robotctl" "${robotctl_body}"
  robotctl_runner="${RUNNER_RESULT}"
  launch_terminal "EV3 robotctl" "${robotctl_runner}"
fi

if [[ "${SELECTED_TERMINAL}" == "tmux" ]]; then
  echo "[ev3-session] tmux 세션을 열었습니다: ${TMUX_SESSION}"
  echo "[ev3-session] 붙기: tmux attach -t ${TMUX_SESSION}"
fi

if [[ -n "${TMP_DIR}" ]]; then
  echo "[ev3-session] 터미널 runner: ${TMP_DIR}"
fi
