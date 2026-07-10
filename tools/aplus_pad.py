#!/usr/bin/env python3
"""aplus 전용 원격 조종 패드 — 텔레메트리 전속 고속 폴링 + 즉시 키 전송.

기존 tools/dashboard.py 와 달리:
  - 상태 파일(runs/current/latest_state.json)을 거치지 않는다 — 튜닝 서버에
    영속 TCP 로 직접 get_latest 를 고속 폴링한다(기본 10Hz, --hz).
  - 이벤트 로그를 표시하지 않는다 — 대역폭/화면 전부 텔레메트리에 할당.
  - 키 전송이 UI 를 블로킹하지 않는다 — 송신 전용 스레드(별도 영속 연결)가
    큐를 비우므로 연타/연속 입력이 밀리지 않는다.
  - 결정 대기(await_cmd) 알림은 소리가 아니라 배경색 — 대기 동안 화면
    전체가 노란 바탕으로 바뀐다(로봇 쪽도 tone 없음).

키(로봇 stages/aplus.py 의 액션과 1:1):
  [w] 계속 직진(라인추종 재개)   [u] 180도 유턴
  [a] 좌회전 90                  [d] 우회전 90
  [q] 왼쪽 대각선 약간 진행      [e] 오른쪽 대각선 약간 진행
  [s] 약간 후진(위치 교정)       [p] 그리퍼 강제 닫기  [o] 그리퍼 열기
  [7] 도착 처리(초록 마커 미인식 폴백: 전진→내려놓기→후진→180도 회전)
  [8] 복귀 도착 처리(노랑 마커 미인식 폴백: BACK 시간→그립 해제→완주)
  [x] 명령 큐 비우기             [t] GO(출발)
  [n] 캘리브레이션  [f] 반사광 판독  [h] 컬러 판독
  [1]~[6] "red N" 음성 재생(로봇 쪽 비동기 큐 — 연타 시 순서대로)
  [Space]/[g] 대기 전환(분기/커브처럼 정지 후 명령 대기 — 회전/직진
              실행 중간에도 즉시 끊고 들어간다)
  [r] reset(확인)  [S] STOP(확인)  [Esc] 종료

실행: python3 tools/aplus_pad.py --host <브릭IP>
스모크: python3 tools/aplus_pad.py --once   (연결 1회 시도 후 stdout 렌더)
"""

from __future__ import annotations

import argparse
import curses
import json
import socket
import threading
import time
from typing import Any

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_TIMEOUT = 0.8
DEFAULT_POLL_HZ = 10.0
FRAME_STALE_S = 2.0         # 이보다 오래된 프레임은 믿지 않는다(배너/pause 판단)

# 키 → do 액션. 로봇(stages/aplus.py ACTIONS)과 1:1 — 여기만 고치면 안 되고
# 로봇 쪽 manifest 도 같이 맞춰야 한다.
KEY_ACTIONS = {
    "w": "fwd",
    "u": "uturn",
    "a": "left",
    "d": "right",
    "q": "diag_left",
    "e": "diag_right",
    # 약간 후진(위치 교정, 실행 후 그 자리 대기) — 예전 s(유턴)는 u 로 이동.
    "s": "back",
    "p": "grip_close",
    "o": "grip_open",
    # 도착 폴백: 초록 미인식 시 도착 절차(전진→내려놓기→후진→유턴) 수동 시행.
    "7": "goal_drop",
    # 복귀 도착 폴백: 노랑 미인식 시 완주 절차(BACK 시간→그립 해제) 수동 시행.
    "8": "home_drop",
    # 대기 전환 — 분기/커브에서처럼 정지 후 명령 대기(Space 도 같은 액션).
    "g": "hold",
    "x": "clear",
    "t": "go",
    "n": "calibrate",
    "f": "read_reflect",
    "h": "read_color",
    # 수동 음성: 로봇이 "red N" 을 비동기 큐로 재생(연타 시 순서대로).
    "1": "say_red_1",
    "2": "say_red_2",
    "3": "say_red_3",
    "4": "say_red_4",
    "5": "say_red_5",
    "6": "say_red_6",
}
CONFIRM_KEYS = {"r": "reset", "S": "stop"}      # 파괴적 — y/n 확인을 거친다
MOVE_ACTIONS = ("fwd", "uturn", "left", "right", "diag_left", "diag_right",
                "back")


class Link:
    """영속 newline-JSON 연결(스레드당 1개) — 끊기면 다음 요청에서 재접속."""

    def __init__(self, host: str, port: int, timeout: float):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._file = None

    def request(self, obj: dict[str, Any]) -> dict[str, Any]:
        try:
            if self._sock is None:
                self._sock = socket.create_connection(
                    (self.host, self.port), timeout=self.timeout)
                self._sock.settimeout(self.timeout)
                self._file = self._sock.makefile("rwb")
            data = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
            self._file.write(data)
            self._file.flush()
            line = self._file.readline()
            if not line:
                raise OSError("connection closed")
            resp = json.loads(line.decode("utf-8"))
            if isinstance(resp, dict):
                return resp
            return {"ok": False, "error": "response root is not an object"}
        except (OSError, ValueError) as exc:
            self.close()
            return {"ok": False, "error": f"link: {exc}"}

    def close(self) -> None:
        for closer in (self._file, self._sock):
            try:
                if closer is not None:
                    closer.close()
            except OSError:
                pass
        self._file = None
        self._sock = None


class Poller(threading.Thread):
    """get_latest 고속 폴링 — 최신 프레임/rtt/오류를 공유 상태에 적재."""

    def __init__(self, host: str, port: int, timeout: float, hz: float):
        super().__init__(name="poller", daemon=True)
        self.link = Link(host, port, timeout)
        self.interval = 1.0 / max(hz, 0.5)
        self.lock = threading.Lock()
        self.frame: dict[str, Any] = {}
        self.frame_t: float | None = None
        self.rtt_ms: float | None = None
        self.error = ""
        self._stop = threading.Event()

    def run(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            resp = self.link.request({"cmd": "get_latest"})
            rtt = (time.monotonic() - t0) * 1000.0
            with self.lock:
                if resp.get("ok") and isinstance(resp.get("latest"), dict):
                    self.frame = resp["latest"]
                    self.frame_t = time.monotonic()
                    self.rtt_ms = rtt
                    self.error = ""
                else:
                    self.error = str(resp.get("error", "no frame"))
            self._stop.wait(max(0.0, self.interval - (time.monotonic() - t0)))

    def snapshot(self) -> tuple[dict[str, Any], float | None, float | None, str]:
        with self.lock:
            age = (None if self.frame_t is None
                   else time.monotonic() - self.frame_t)
            return dict(self.frame), age, self.rtt_ms, self.error

    def stop(self) -> None:
        self._stop.set()
        self.link.close()


class Sender(threading.Thread):
    """명령 송신 전용 스레드 — 키 입력은 큐에만 넣고 즉시 리턴(UI 무정지)."""

    def __init__(self, host: str, port: int, timeout: float):
        super().__init__(name="sender", daemon=True)
        self.link = Link(host, port, timeout)
        self.lock = threading.Lock()
        self.queue: list[dict[str, Any]] = []
        self.status = ""
        self._wake = threading.Event()
        self._stop = threading.Event()

    def submit(self, request: dict[str, Any], label: str) -> None:
        with self.lock:
            self.queue.append({"request": request, "label": label})
        self._wake.set()

    def run(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(0.2)
            self._wake.clear()
            while True:
                with self.lock:
                    if not self.queue:
                        break
                    item = self.queue.pop(0)
                resp = self.link.request(item["request"])
                with self.lock:
                    self.status = f"{item['label']}: {_compact(resp)}"

    def last_status(self) -> str:
        with self.lock:
            return self.status

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        self.link.close()


def _compact(resp: dict[str, Any]) -> str:
    if resp.get("ok") is False:
        return "ERR " + str(resp.get("error", resp))
    if "queued_move" in resp:
        queue = resp.get("queue") or []
        return "queued {} [{}]".format(resp["queued_move"], ",".join(queue))
    if "rejected" in resp:
        return f"REJECTED {resp['rejected']} (queue full — [x] to clear)"
    if "cleared" in resp:
        cleared = resp.get("cleared") or []
        return "cleared [{}]".format(",".join(cleared)) if cleared else "queue empty"
    if "queued" in resp:
        return f"queued {resp['queued']}"
    if "paused" in resp:
        return "paused" if resp.get("paused") else "resumed"
    if "stopped" in resp:
        return "STOP sent"
    return json.dumps(resp, ensure_ascii=False, separators=(",", ":"))[:80]


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on", "paused"}
    return False


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.4g}"
    return "-" if value is None else str(value)


def render_lines(frame: dict[str, Any], age: float | None, rtt_ms: float | None,
                 poll_error: str, send_status: str, pending_confirm: str,
                 width: int = 100) -> list[str]:
    """패드 화면(텍스트) — 순수 함수라 --once 스모크/테스트에 재사용."""
    width = max(50, width)
    fresh = age is not None and age <= FRAME_STALE_S
    mode = str(frame.get("mode", "-")) if fresh else "-"
    lines: list[str] = []
    conn = (f"conn OK rtt {rtt_ms:.0f}ms" if rtt_ms is not None and not poll_error
            else f"conn LOST ({poll_error or 'no frame yet'})")
    age_txt = "-" if age is None else f"{age:.1f}s"
    lines.append(f" APLUS PAD — {conn}  frame_age {age_txt} ".center(width, "="))

    if fresh and mode.startswith("await_cmd"):
        kind = frame.get("await_kind", "?")
        exits = "".join(label for label, key in
                        (("L", "has_left"), ("S", "has_straight"),
                         ("R", "has_right")) if _as_bool(frame.get(key))) or "-"
        color = frame.get("color", "-")
        lines.append(f">>> AWAITING COMMAND ({kind})  exits={exits}  "
                     f"color={color} <<<".center(width))
    elif fresh:
        lines.append(f"mode: {mode}".center(width))
    else:
        lines.append("(no fresh telemetry)".center(width))

    queue = frame.get("pending_cmd", "-") if fresh else "-"
    paused = "YES" if fresh and _as_bool(frame.get("paused")) else "no"
    lines.append(f"queue: {queue}   paused: {paused}   "
                 f"session {_fmt(frame.get('session'))}  "
                 f"visits {_fmt(frame.get('visits'))}  "
                 f"grabbed {'YES' if _as_bool(frame.get('grabbed')) else 'no'}  "
                 f"goal {'YES' if _as_bool(frame.get('goal_seen')) else 'no'}")
    lines.append(f"OUT {_fmt(frame.get('out_s'))}s   BACK {_fmt(frame.get('back_s'))}s   "
                 f"t {_fmt(frame.get('t_ms'))}ms")
    lines.append(f"reflect L/R {_fmt(frame.get('reflect_l'))}/{_fmt(frame.get('reflect_r'))}  "
                 f"bits {_fmt(frame.get('bits'))}  color {_fmt(frame.get('color'))}  "
                 f"err {_fmt(frame.get('error'))}  turn {_fmt(frame.get('turn'))}  "
                 f"base {_fmt(frame.get('base'))}")
    lines.append(f"last: {_fmt(frame.get('last_reason'))}")
    lines.append("-" * width)
    lines.append("[w]fwd [a]left90 [d]right90 [u]uturn180 [s]back [q]diagL "
                 "[e]diagR [p]GRIP [o]open [7]goal [8]home")
    lines.append("[t]GO [x]clear [n]calibrate [f]reflect [h]color [1-6]red N "
                 "[Space/g]hold [r]reset [S]STOP [Esc]quit")
    if pending_confirm:
        lines.append(f"confirm {pending_confirm.upper()}? press y to run, n/Esc to cancel")
    else:
        lines.append(f"status: {send_status or 'ready'}")
    return [line[:width] for line in lines]


def handle_key(key: int, sender: Sender,
               pending_confirm: str) -> tuple[bool, str]:
    """키 1개 처리 — (quit 여부, 새 pending_confirm) 반환."""
    if pending_confirm:
        if key in (ord("y"), ord("Y")):
            if pending_confirm == "stop":
                sender.submit({"cmd": "stop", "source": "aplus_pad"}, "STOP")
            else:
                sender.submit({"cmd": "do", "action": "reset",
                               "args": {"source": "aplus_pad"}}, "reset")
            return False, ""
        if key in (27, ord("n"), ord("N")):
            return False, ""
        return False, pending_confirm       # 그 외 키는 확인 대기 유지

    if key == 27:                           # Esc
        return True, ""
    if key == ord(" "):
        # 일시정지 토글이 아니라 대기 전환 — 로봇이 분기/커브에서처럼
        # 정지 후 명령 대기(await_cmd)로 들어간다([g] 와 동일 액션).
        sender.submit({"cmd": "do", "action": "hold", "args": {}}, "do hold")
        return False, ""
    if 0 <= key < 256:
        ch = chr(key)
        if ch in CONFIRM_KEYS:
            return False, CONFIRM_KEYS[ch]
        action = KEY_ACTIONS.get(ch)
        if action:
            sender.submit({"cmd": "do", "action": action, "args": {}},
                          f"do {action}")
    return False, ""


def _curses_main(stdscr: Any, args: argparse.Namespace) -> int:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.timeout(30)          # 30ms — 키 반응성과 CPU 의 균형

    # 결정 대기 알림은 소리가 아니라 배경색 — await_cmd 동안 화면 전체를
    # 노란 바탕으로 물들인다(색 미지원 단말은 배너 반전 강조만).
    use_color = False
    try:
        curses.start_color()
        if curses.has_colors():
            curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_YELLOW)
            use_color = True
    except curses.error:
        pass

    poller = Poller(args.host, args.port, args.timeout, args.hz)
    sender = Sender(args.host, args.port, args.timeout)
    poller.start()
    sender.start()

    pending_confirm = ""
    try:
        while True:
            frame, age, rtt, poll_error = poller.snapshot()
            fresh = age is not None and age <= FRAME_STALE_S
            awaiting = fresh and str(frame.get("mode", "")).startswith("await_cmd")

            height, width = stdscr.getmaxyx()
            if use_color:
                stdscr.bkgd(" ", curses.color_pair(1) if awaiting
                            else curses.A_NORMAL)
            stdscr.erase()
            lines = render_lines(frame, age, rtt, poll_error,
                                 sender.last_status(), pending_confirm,
                                 width - 1)
            for row, line in enumerate(lines[:height]):
                attr = curses.A_NORMAL
                if row == 1 and awaiting:
                    attr = curses.A_REVERSE | curses.A_BOLD
                try:
                    stdscr.addnstr(row, 0, line, max(0, width - 1), attr)
                except curses.error:
                    pass
            stdscr.refresh()

            key = stdscr.getch()
            if key == -1:
                continue
            quit_now, pending_confirm = handle_key(
                key, sender, pending_confirm)
            if quit_now:
                return 0
    finally:
        poller.stop()
        sender.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="aplus remote-control pad")
    parser.add_argument("--host", default=DEFAULT_HOST, help="tuning server host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--hz", type=float, default=DEFAULT_POLL_HZ,
                        help="telemetry poll rate (get_latest)")
    parser.add_argument("--once", action="store_true",
                        help="poll once and render to stdout (smoke test)")
    parser.add_argument("--width", type=int, default=100)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.once:
        link = Link(args.host, args.port, args.timeout)
        t0 = time.monotonic()
        resp = link.request({"cmd": "get_latest"})
        rtt = (time.monotonic() - t0) * 1000.0
        link.close()
        if resp.get("ok") and isinstance(resp.get("latest"), dict):
            print("\n".join(render_lines(resp["latest"], 0.0, rtt, "",
                                         "", "", args.width)))
            return 0
        print("\n".join(render_lines({}, None, None,
                                     str(resp.get("error", "no frame")),
                                     "", "", args.width)))
        return 1
    return curses.wrapper(_curses_main, args)


if __name__ == "__main__":
    raise SystemExit(main())
