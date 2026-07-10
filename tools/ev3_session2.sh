#!/usr/bin/env bash
# aplus 계열용 EV3 실기 세션 런처.
#
# ev3_session.sh 와 같은 업로드/터널/옵션 흐름을 쓰되 기본 stage 는 aplus,
# 조종 UI 는 tools/aplus_pad.py 로 연다.
#
# 예:
#   tools/ev3_session2.sh
#   tools/ev3_session2.sh -n
#   tools/ev3_session2.sh --host ev3 --terminal tmux
#   tools/ev3_session2.sh aplus_experiment --pad-hz 20

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

SESSION_SCRIPT="${REPO_ROOT}/tools/ev3_session.sh"
if [[ ! -x "${SESSION_SCRIPT}" && -x "${SCRIPT_DIR}/ev3_session.sh" ]]; then
  SESSION_SCRIPT="${SCRIPT_DIR}/ev3_session.sh"
fi

if [[ ! -x "${SESSION_SCRIPT}" ]]; then
  echo "ev3_session2: 실행할 ev3_session.sh 를 찾을 수 없습니다." >&2
  echo "ev3_session2: repo=${REPO_ROOT}" >&2
  echo "ev3_session2: script_dir=${SCRIPT_DIR}" >&2
  exit 1
fi

usage() {
  cat <<'EOF'
Usage: ev3sess2 [options] [stage_name_or_path]

aplus 계열 EV3 실기 세션을 여러 터미널로 한 번에 띄운다.
업로드/터널/watcher/robotctl 옵션은 ev3sess 와 같고, 조종 UI 는
tools/aplus_pad.py 를 기본으로 사용한다.

Examples:
  ev3sess2
  ev3sess2 -n
  ev3sess2 --host ev3 --terminal tmux
  ev3sess2 aplus_experiment --pad-hz 20

Defaults:
  stage: aplus
  dashboard: both (aplus_pad.py + dashboard.py)
  pad telemetry: 10Hz

Useful options:
  -s, --stage PATH_OR_NAME   실행할 stage 파일. 기본: stages/aplus.py
  -h, --host HOST            SSH/scp 대상. 기본: ev3
  -p, --port PORT            로컬/브릭 튜닝 포트. 기본: 8765
  -t, --terminal NAME        auto, GUI 터미널, tmux, none
  -n, --no-upload            scp 업로드를 건너뛴다
      --pad-hz HZ            aplus_pad.py 텔레메트리 폴링 Hz. 기본: 10
      --no-robotctl          robotctl 대기 터미널을 열지 않는다
      --dry-run              실행할 업로드/터미널 명령만 출력한다
      --help                 도움말 출력
EOF
}

for arg in "$@"; do
  case "${arg}" in
    --help)
      usage
      exit 0
      ;;
  esac
done

stage_given="0"
expect_value="0"
for arg in "$@"; do
  if [[ "${expect_value}" == "1" ]]; then
    expect_value="0"
    continue
  fi
  case "${arg}" in
    -s|--stage)
      stage_given="1"
      expect_value="1"
      ;;
    -h|--host|-r|--remote-dir|-p|--port|--local-port|--remote-port|-t|--terminal|--tmux-session|--dashboard|--pad-hz)
      expect_value="1"
      ;;
    --)
      stage_given="1"
      break
      ;;
    --*)
      ;;
    -*)
      ;;
    *)
      stage_given="1"
      ;;
  esac
done

args=(--dashboard both)
if [[ "${stage_given}" == "0" ]]; then
  args+=(--stage aplus)
fi

exec "${SESSION_SCRIPT}" "${args[@]}" "$@"
