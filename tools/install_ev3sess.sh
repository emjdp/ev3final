#!/usr/bin/env bash
# ev3sess 전역 런처를 설치한다.
#
# 목적:
#   - 현재 브랜치에 tools/ev3_session.sh 가 있으면 그 최신 파일을 실행
#   - 다른 브랜치로 전환해 파일이 없어도 캐시본으로 계속 실행
#
# 설치 후 사용:
#   ev3sess run_maze
#   ev3sess stage4v2_color_follow

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
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CACHE_DIR="${XDG_DATA_HOME:-${HOME}/.local/share}/ev3test"
BIN_DIR="${HOME}/.local/bin"
CACHE_SCRIPT="${CACHE_DIR}/ev3_session.sh"
BIN_SCRIPT="${BIN_DIR}/ev3sess"

mkdir -p "${CACHE_DIR}" "${BIN_DIR}"
install -m 0755 "${REPO_ROOT}/tools/ev3_session.sh" "${CACHE_SCRIPT}"

cat >"${BIN_SCRIPT}" <<EOF
#!/usr/bin/env bash
set -euo pipefail

DEFAULT_REPO=$(printf '%q' "${REPO_ROOT}")
EV3TEST_REPO="\${EV3TEST_REPO:-\${DEFAULT_REPO}}"
REPO_SCRIPT="\${EV3TEST_REPO}/tools/ev3_session.sh"
CACHE_SCRIPT=$(printf '%q' "${CACHE_SCRIPT}")

if [[ -x "\${REPO_SCRIPT}" ]]; then
  exec "\${REPO_SCRIPT}" --terminal tmux "\$@"
fi

if [[ -x "\${CACHE_SCRIPT}" ]]; then
  export EV3TEST_REPO
  exec "\${CACHE_SCRIPT}" --terminal tmux "\$@"
fi

echo "ev3sess: 실행할 ev3_session.sh 를 찾을 수 없습니다." >&2
echo "ev3sess: repo=\${EV3TEST_REPO}" >&2
echo "ev3sess: cache=\${CACHE_SCRIPT}" >&2
exit 1
EOF
chmod 0755 "${BIN_SCRIPT}"

echo "설치 완료: ${BIN_SCRIPT}"
echo "캐시본: ${CACHE_SCRIPT}"
case ":${PATH}:" in
  *":${BIN_DIR}:"*) ;;
  *)
    echo "주의: ${BIN_DIR} 이 PATH 에 없습니다. ~/.bashrc 등에 아래 줄을 추가하세요."
    echo "export PATH=\"${BIN_DIR}:\$PATH\""
    ;;
esac
