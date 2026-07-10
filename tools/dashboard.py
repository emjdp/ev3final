#!/usr/bin/env python3
"""runs/current/latest_state.json 기반 curses 대시보드."""

from __future__ import annotations

import argparse
import curses
import json
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_STATE_PATH = Path("runs/current/latest_state.json")
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_TIMEOUT = 1.5
PARAM_VALUE_STALE_SECONDS = 10.0
ACTION_KEYS = list("1234567890") + list("dfhjklmnprtuuvwxyz")
# 대시보드 자체 기능에 물린 키 — 액션 명시 핫키(key 필드)로 쓸 수 없다.
RESERVED_KEYS = {"q", "s", "r", "a", "c", "g", "S", "R", "G", "Q",
                 " ", ".", "+", "-", "="}
META_KEYS = {
    "stage",
    "latest",
    "telemetry",
    "frame",
    "params",
    "param_values",
    "param_limits",
    "limits",
    "max_step",
    "steps",
    "events",
    "recent_events",
    "last_events",
    "actions",
}


@dataclass
class ParamRow:
    name: str
    value: Any
    limit: Any = None
    step: Any = None
    max_step: Any = None
    unit: str = ""


@dataclass
class ActionBinding:
    key: str
    name: str
    label: str


@dataclass
class DashboardSession:
    selected: int = 0
    status: str = ""
    pending_confirm: str = ""
    auto_rerun: bool = False
    coarse_step: bool = False
    last_action: str = ""


@dataclass
class DashboardModel:
    stage: str
    frame: dict[str, Any]
    params: list[ParamRow]
    actions: list[ActionBinding]
    events: list[dict[str, Any]]
    selected: int = 0
    status: str = ""
    state_error: str = ""
    describe_error: str = ""
    state_age_s: float | None = None
    pending_confirm: str = ""
    auto_rerun: bool = False
    coarse_step: bool = False
    last_action: str = ""


def load_latest_state(path: Path) -> tuple[dict[str, Any], str, float | None]:
    try:
        raw = path.read_text(encoding="utf-8")
        state = json.loads(raw)
    except FileNotFoundError:
        return {}, f"state file not found: {path}", None
    except json.JSONDecodeError as exc:
        return {}, f"state JSON error: {exc}", None
    except OSError as exc:
        return {}, f"state read error: {exc}", None

    if not isinstance(state, dict):
        return {}, "state root must be a JSON object", None

    try:
        age = max(0.0, time.time() - path.stat().st_mtime)
    except OSError:
        age = None
    return state, "", age


def send_command(
    request: dict[str, Any],
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_TIMEOUT,
) -> dict[str, Any]:
    data = (json.dumps(request, separators=(",", ":")) + "\n").encode("utf-8")
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(data)
        with sock.makefile("rb") as reader:
            line = reader.readline()
    if not line:
        return {"ok": False, "error": "empty response"}
    try:
        response = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        return {"ok": False, "error": f"bad response: {exc}"}
    if isinstance(response, dict):
        return response
    return {"ok": False, "error": "response root is not an object", "raw": response}


def load_describe(host: str, port: int, timeout: float = DEFAULT_TIMEOUT) -> tuple[dict[str, Any], str]:
    try:
        response = send_command({"cmd": "describe"}, host, port, timeout)
    except OSError as exc:
        return {}, f"describe connect error: {exc}"
    if response.get("ok") is False:
        return {}, f"describe error: {response.get('error', response)}"
    if not isinstance(response.get("params", []), list):
        return {}, "describe params must be a list"
    if not isinstance(response.get("actions", []), list):
        return {}, "describe actions must be a list"
    return response, ""


def build_model(
    state: dict[str, Any],
    describe: dict[str, Any] | None = None,
    session: DashboardSession | None = None,
    state_error: str = "",
    describe_error: str = "",
    state_age_s: float | None = None,
) -> DashboardModel:
    if session is None:
        session = DashboardSession()
    frame = _extract_frame(state)
    params_state = state
    params_frame = frame
    if state_error or (state_age_s is not None and state_age_s > PARAM_VALUE_STALE_SECONDS):
        params_state = {}
        params_frame = {}
    params = _extract_params(params_state, params_frame, describe or {})
    events = _extract_events(state, frame)
    actions = _extract_actions(describe or {})
    selected = session.selected
    if params:
        selected = max(0, min(selected, len(params) - 1))
    else:
        selected = 0

    stage = str((describe or {}).get("stage") or state.get("stage") or frame.get("stage") or "-")
    return DashboardModel(
        stage=stage,
        frame=frame,
        params=params,
        actions=actions,
        events=events,
        selected=selected,
        status=session.status,
        state_error=state_error,
        describe_error=describe_error,
        state_age_s=state_age_s,
        pending_confirm=session.pending_confirm,
        auto_rerun=session.auto_rerun,
        coarse_step=session.coarse_step,
        last_action=session.last_action,
    )


def render_lines(model: DashboardModel, width: int = 100, height: int = 32) -> list[str]:
    width = max(40, width)
    height = max(8, height)
    sep = "-" * width
    frame = model.frame

    running = _truth_label(frame.get("running"))
    t_ms = _as_number(frame.get("t_ms"))
    dt_ms = _as_number(frame.get("dt_ms"))
    rev = frame.get("param_rev", frame.get("rev", "-"))
    age = "-" if model.state_age_s is None else f"{model.state_age_s:.1f}s"
    title = f" ev3 dashboard ({model.stage}) "

    lines: list[str] = [title.center(width, "-")]
    mode = "auto={}".format("ON" if model.auto_rerun else "OFF")
    step_mode = "step={}".format("coarse" if model.coarse_step else "fine")
    last = model.last_action or "-"
    paused = "paused={}".format("YES" if _as_bool(frame.get("paused")) else "no")
    lines.append(
        _fit(
            f"{running:<8} t={_format_seconds(t_ms)}  dt={_format_ms(dt_ms)}  "
            f"rev={rev}  state_age={age}  {paused}  {mode}  {step_mode}  last={last}  [Space pause] [r reset] [s STOP]",
            width,
        )
    )
    lines.append(_fit(_telemetry_summary(frame), width))
    banner = _command_banner(frame)
    if banner:
        lines.append(_fit(banner, width))
    lines.append(sep)
    lines.append(_fit("params           value        limit          step      max_step  unit", width))

    if model.params:
        param_room = max(3, min(len(model.params), height // 3))
        start = _window_start(model.selected, len(model.params), param_room)
        for idx in range(start, min(len(model.params), start + param_room)):
            row = model.params[idx]
            marker = ">" if idx == model.selected else " "
            lines.append(
                _fit(
                    f"{marker} {row.name:<15} {_format_value(row.value):<12} "
                    f"{_format_limit(row.limit):<14} {_format_step(row.step, row.value):<9} "
                    f"{_format_optional_value(row.max_step):<8} {row.unit}",
                    width,
                )
            )
    else:
        lines.append(_fit("  (latest_state.json has no params)", width))

    lines.append(sep)
    lines.append(_fit(_format_actions(model.actions), width))
    lines.append(_fit("keys: [Space] pause/resume  [r] reset  [.] repeat  [a] auto-rerun  [c] coarse/fine  [g] refresh  [S] save  [R] rollback  [q] quit", width))
    if model.pending_confirm:
        lines.append(_fit(f"confirm {model.pending_confirm}: press y to run, n/Esc to cancel", width))
    elif model.status:
        lines.append(_fit(f"status: {model.status}", width))
    elif model.describe_error:
        lines.append(_fit(f"describe: {model.describe_error}", width))
    elif model.state_error:
        lines.append(_fit(f"state: {model.state_error}", width))
    else:
        lines.append(_fit("status: ready", width))

    lines.append(sep)
    lines.append(_fit("events (recent):", width))
    event_room = max(1, height - len(lines))
    for event in model.events[-event_room:]:
        lines.append(_fit("  " + _format_event(event), width))
    while len(lines) < height:
        lines.append("")
    return [line[:width] for line in lines[:height]]


def handle_key(
    key: int,
    model: DashboardModel,
    session: DashboardSession,
    host: str,
    port: int,
    timeout: float = DEFAULT_TIMEOUT,
) -> tuple[bool, bool]:
    should_quit = False
    refresh_describe = False

    if session.pending_confirm:
        if key in (ord("y"), ord("Y")):
            if session.pending_confirm == "reset":
                # reset 은 plain cmd 가 아니라 스테이지 액션(do)이다.
                request = {"cmd": "do", "action": "reset", "args": {"source": "dashboard"}}
            else:
                request = {"cmd": session.pending_confirm}
            session.status = _send_and_describe(request, host, port, timeout)
            session.pending_confirm = ""
        elif key in (27, ord("n"), ord("N")):
            session.status = f"{session.pending_confirm} canceled"
            session.pending_confirm = ""
        return should_quit, refresh_describe

    if key in (ord("q"), ord("Q")):
        should_quit = True
    elif key in (curses.KEY_UP,):
        session.selected = max(0, model.selected - 1)
    elif key in (curses.KEY_DOWN, 9):
        session.selected = (model.selected + 1) % len(model.params) if model.params else 0
    elif key in (curses.KEY_LEFT, ord("-")):
        session.status = _adjust_selected(model, -1, host, port, timeout)
    elif key in (curses.KEY_RIGHT, ord("+"), ord("=")):
        session.status = _adjust_selected(model, 1, host, port, timeout)
    elif key == ord("s"):
        session.status = _send_and_describe({"cmd": "stop", "source": "dashboard"}, host, port, timeout)
    elif key == ord(" "):
        paused = _as_bool(model.frame.get("paused"))
        session.status = _send_and_describe(
            {"cmd": "pause", "paused": not paused, "source": "dashboard"},
            host,
            port,
            timeout,
        )
    elif key == ord("."):
        session.status = _repeat_last_action(session, host, port, timeout)
    elif key in (ord("a"), ord("A")):
        session.auto_rerun = not session.auto_rerun
        session.status = "auto-rerun {}".format("ON" if session.auto_rerun else "OFF")
    elif key in (ord("c"), ord("C")):
        session.coarse_step = not session.coarse_step
        session.status = "step mode {}".format("coarse" if session.coarse_step else "fine")
    elif key == ord("r"):
        # 세션 리셋(출발 대기 복귀) — 탐색 상태를 통째로 버리므로 confirm 을 거친다.
        session.pending_confirm = "reset"
        session.status = "confirm reset"
    elif key == ord("S"):
        session.pending_confirm = "save"
        session.status = "confirm save"
    elif key == ord("R"):
        session.pending_confirm = "rollback"
        session.status = "confirm rollback"
    elif key in (ord("g"), ord("G")):
        refresh_describe = True
        session.status = "describe refreshed"
    elif 0 <= key < 256:
        action = _action_for_key(chr(key), model.actions)
        if action:
            session.status = _run_action(action, session, host, port, timeout)

    return should_quit, refresh_describe


def run_curses(args: argparse.Namespace) -> int:
    return curses.wrapper(_curses_main, args)


def _curses_main(stdscr: Any, args: argparse.Namespace) -> int:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.timeout(200)

    session = DashboardSession()
    describe, describe_error = load_describe(args.host, args.port, args.timeout)
    if describe_error:
        session.status = describe_error

    while True:
        state, state_error, age = load_latest_state(args.state)
        model = build_model(state, describe, session, state_error, describe_error, age)
        session.selected = model.selected
        height, width = stdscr.getmaxyx()
        stdscr.erase()
        for row, line in enumerate(render_lines(model, width, height)):
            try:
                stdscr.addnstr(row, 0, line, max(0, width - 1))
            except curses.error:
                pass
        stdscr.refresh()

        key = stdscr.getch()
        if key == -1:
            continue
        should_quit, refresh_describe = handle_key(key, model, session, args.host, args.port, args.timeout)
        if refresh_describe:
            describe, describe_error = load_describe(args.host, args.port, args.timeout)
            if describe_error:
                session.status = describe_error
        if should_quit:
            return 0


def _extract_frame(state: dict[str, Any]) -> dict[str, Any]:
    for key in ("latest", "telemetry", "frame"):
        value = state.get(key)
        if isinstance(value, dict):
            return dict(value)
    frame = {key: value for key, value in state.items() if key not in META_KEYS}
    return frame if frame else {}


def _extract_params(state: dict[str, Any], frame: dict[str, Any], describe: dict[str, Any]) -> list[ParamRow]:
    described = describe.get("params")
    if isinstance(described, list):
        values = _state_param_values(state, frame)
        rows = []
        for item in described:
            if not isinstance(item, dict) or "name" not in item:
                continue
            name = str(item["name"])
            value = values.get(name, item.get("value"))
            rows.append(
                ParamRow(
                    name=name,
                    value=value,
                    limit={"min": item.get("min"), "max": item.get("max")},
                    step=item.get("step"),
                    max_step=item.get("max_step"),
                    unit=str(item.get("unit") or ""),
                )
            )
        return rows

    source = state.get("params")
    if source is None:
        source = frame.get("params")

    limits = _dict_or_empty(state.get("param_limits") or state.get("limits"))
    steps = _dict_or_empty(state.get("max_step") or state.get("steps"))

    if isinstance(source, dict):
        limits.update(_dict_or_empty(source.get("limits") or source.get("param_limits")))
        steps.update(_dict_or_empty(source.get("max_step") or source.get("steps")))
        values = source.get("values") or source.get("current") or source.get("params")
        if not isinstance(values, dict):
            values = {k: v for k, v in source.items() if k not in {"limits", "param_limits", "max_step", "steps"}}
        rows = []
        for name in values:
            value = values[name]
            if isinstance(value, dict) and "value" in value:
                rows.append(
                    ParamRow(
                        name=name,
                        value=value.get("value"),
                        limit=value.get("limit") or value.get("limits") or limits.get(name),
                        step=value.get("step") or value.get("max_step") or steps.get(name),
                    )
                )
            else:
                rows.append(ParamRow(name=name, value=value, limit=limits.get(name), step=steps.get(name)))
        return rows

    values = state.get("param_values")
    if isinstance(values, dict):
        return [ParamRow(name=name, value=values[name], limit=limits.get(name), step=steps.get(name)) for name in values]
    return []


def _state_param_values(state: dict[str, Any], frame: dict[str, Any]) -> dict[str, Any]:
    source = state.get("params")
    if source is None:
        source = frame.get("params")
    if isinstance(source, dict):
        values = source.get("values") or source.get("current") or source.get("params")
        if isinstance(values, dict):
            return dict(values)
        return {k: v for k, v in source.items() if k not in {"limits", "param_limits", "max_step", "steps"}}
    values = state.get("param_values")
    if isinstance(values, dict):
        return dict(values)
    return {}


def _extract_actions(describe: dict[str, Any]) -> list[ActionBinding]:
    raw_actions = describe.get("actions")
    if not isinstance(raw_actions, list):
        return []
    bindings = []
    used = set()
    for item in raw_actions:
        if not isinstance(item, dict):
            continue
        name = item.get("name")
        if not name:
            continue
        # 스테이지가 지정한 핫키(key 필드) 우선 — 대시보드 예약키/중복이면
        # 종전처럼 순서 기반 배정으로 폴백한다.
        explicit = str(item.get("key") or "")[:1]
        if explicit and explicit not in used and explicit not in RESERVED_KEYS:
            key = explicit
        else:
            key = _choose_action_key(item, used, len(bindings))
        used.add(key)
        bindings.append(ActionBinding(key=key, name=str(name), label=str(item.get("label") or name)))
    return bindings


def _choose_action_key(action: dict[str, Any], used: set[str], index: int) -> str:
    for key in ACTION_KEYS[index:index + 1] + ACTION_KEYS:
        if key not in used:
            return key
    name = str(action.get("name") or "")
    for char in name:
        if char.isalnum() and char not in used:
            return char
    return "?"


def _extract_events(state: dict[str, Any], frame: dict[str, Any]) -> list[dict[str, Any]]:
    value = state.get("recent_events") or state.get("events") or state.get("last_events")
    if value is None:
        value = frame.get("recent_events") or frame.get("events") or frame.get("last_events")
    if not isinstance(value, list):
        return []
    events = []
    for item in value:
        if isinstance(item, dict):
            events.append(item)
        else:
            events.append({"event": str(item)})
    return events


def _adjust_selected(model: DashboardModel, direction: int, host: str, port: int, timeout: float) -> str:
    if not model.params:
        return "no params to adjust"
    row = model.params[model.selected]
    current = _as_number(row.value)
    if current is None:
        return f"{row.name}: non-numeric value"
    step = _as_number(row.step)
    if step is None or step == 0:
        step = _infer_step(current)
    if model.coarse_step:
        step = step * 5
    new_value = _normalize_number(current + direction * step, row.value)
    response, status = _send_and_status({"cmd": "set", "name": row.name, "value": new_value}, host, port, timeout)
    if response.get("ok") and model.auto_rerun and model.last_action:
        action_status = _send_and_describe(
            {"cmd": "do", "action": model.last_action, "args": {}},
            host,
            port,
            timeout,
        )
        return status + " | auto " + action_status
    return status


def _send_and_describe(request: dict[str, Any], host: str, port: int, timeout: float = DEFAULT_TIMEOUT) -> str:
    return _send_and_status(request, host, port, timeout)[1]


def _send_and_status(request: dict[str, Any], host: str, port: int, timeout: float) -> tuple[dict[str, Any], str]:
    try:
        response = send_command(request, host, port, timeout)
    except OSError as exc:
        response = {"ok": False, "error": "connect error: {}".format(exc)}
    return response, f"{request.get('cmd')}: {_compact_response(response)}"


def _action_for_key(key: str, actions: list[ActionBinding]) -> str:
    for action in actions:
        if action.key == key:
            return action.name
    return ""


def _run_action(action: str, session: DashboardSession, host: str, port: int, timeout: float) -> str:
    response, status = _send_and_status({"cmd": "do", "action": action, "args": {}}, host, port, timeout)
    if response.get("ok"):
        session.last_action = action
    return status


def _repeat_last_action(session: DashboardSession, host: str, port: int, timeout: float) -> str:
    if not session.last_action:
        return "no last action"
    return _run_action(session.last_action, session, host, port, timeout)


def _compact_response(response: dict[str, Any]) -> str:
    if response.get("ok") is False:
        return "error: " + str(response.get("error", response))
    if "value" in response:
        return f"ok value={_format_value(response.get('value'))} rev={response.get('rev', '-')}"
    if "queued" in response:
        return f"ok queued={response.get('queued')}"
    if "paused" in response:
        return "ok {}".format("paused" if response.get("paused") else "resumed")
    if "saved" in response:
        # 저장은 브릭(튜닝 서버)에서 일어난다. 경로도 브릭 파일시스템 기준이므로
        # 노트북에서 찾지 않게 'robot:' 를 붙여 어느 쪽 경로인지 분명히 한다.
        return f"ok saved on robot: {response.get('saved')}"
    if "latest" in response:
        latest = response.get("latest")
        if isinstance(latest, dict):
            return "ok latest " + _telemetry_summary(latest)
    return json.dumps(response, ensure_ascii=False, separators=(",", ":"))[:160]


def _command_banner(frame: dict[str, Any]) -> str:
    """gg8 수동 조종 배너 — 결정 대기 중이거나 명령이 큐에 있으면 크게 알린다."""
    mode = str(frame.get("mode") or "")
    pending = frame.get("pending_cmd")
    pending_txt = "" if pending in (None, "-", "") else f"  queued={pending}"
    if mode.startswith("await_cmd"):
        kind = frame.get("await_kind", "?")
        exits = "".join(
            label
            for label, key in (("L", "has_left"), ("S", "has_straight"), ("R", "has_right"))
            if _as_bool(frame.get(key))
        ) or "-"
        color = frame.get("color")
        color_txt = "" if color in (None, "-", "") else f" color={color}"
        return (
            f">>> AWAITING COMMAND ({kind}{color_txt}) exits={exits}"
            f"{pending_txt}  -- press [j]L [k]S [l]R [u]U <<<"
        )
    if pending_txt:
        return f"cmd queue:{pending_txt} (consumed at next stop: curve/junction/marker, [x] to clear)"
    return ""


def _telemetry_summary(frame: dict[str, Any]) -> str:
    # Stage 3 친화적 우선순위(있는 키만 표시 → Stage 1/2 표시는 그대로 유지).
    # Stage 3: bits/mode/reflect_l·c·r/node flags/error·line_error3/turn/속도가 앞에 온다.
    # Stage 1: reflect/error/turn/left_speed/right_speed (Stage 3 키는 없어 건너뜀).
    priority = [
        "pending_cmd", "await_kind",
        "bits", "mode",
        "reflect_l", "reflect_c", "reflect_r",
        "node_candidate", "node_confirmed",
        "reflect", "error", "line_error3",
        "turn", "left_speed", "right_speed",
        "last_reason",
    ]
    parts = []
    for key in priority:
        if key in frame:
            parts.append(f"{key}={_format_value(frame[key])}")
    if parts:
        return "  ".join(parts)
    items = [(k, v) for k, v in frame.items() if k not in META_KEYS and not isinstance(v, (dict, list))]
    return "  ".join(f"{k}={_format_value(v)}" for k, v in items[:6]) or "(no telemetry frame)"


def _format_actions(actions: list[ActionBinding]) -> str:
    if not actions:
        return "actions: (none from describe)"
    parts = ["[{}] {}".format(action.key, action.label) for action in actions]
    return "actions: " + "   ".join(parts)


def _format_event(event: dict[str, Any]) -> str:
    t_ms = _as_number(event.get("t_ms"))
    prefix = ""
    if t_ms is not None:
        prefix = f"{t_ms / 1000.0:6.2f} "
    name = str(event.get("event") or event.get("reason_code") or "-")
    reason = event.get("reason")
    skip = {"t_ms", "event", "reason_code", "reason"}
    detail = " ".join(f"{k}={_format_value(v)}" for k, v in event.items() if k not in skip)
    if reason and detail:
        return f"{prefix}{name} {reason} {detail}"
    if reason:
        return f"{prefix}{name} {reason}"
    if detail:
        return f"{prefix}{name} {detail}"
    return f"{prefix}{name}"


def _format_seconds(t_ms: float | None) -> str:
    if t_ms is None:
        return "-"
    return f"{t_ms / 1000.0:.1f}s"


def _format_ms(dt_ms: float | None) -> str:
    if dt_ms is None:
        return "-"
    return f"{dt_ms:.0f}ms"


def _format_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def _format_limit(limit: Any) -> str:
    if limit is None:
        return "-"
    if isinstance(limit, dict):
        lo = limit.get("min", limit.get("lo"))
        hi = limit.get("max", limit.get("hi"))
        if lo is not None and hi is not None:
            return f"{lo}..{hi}"
    if isinstance(limit, (list, tuple)) and len(limit) >= 2:
        return f"{limit[0]}..{limit[1]}"
    return str(limit)


def _format_step(step: Any, value: Any) -> str:
    if step is None:
        number = _as_number(value)
        return _format_value(_infer_step(number)) if number is not None else "-"
    return _format_value(step)


def _format_optional_value(value: Any) -> str:
    if value is None:
        return "-"
    return _format_value(value)


def _truth_label(value: Any) -> str:
    if value is True:
        return "RUNNING"
    if value is False:
        return "STOPPED"
    return "UNKNOWN"


def _window_start(selected: int, total: int, room: int) -> int:
    if total <= room:
        return 0
    return min(max(0, selected - room // 2), total - room)


def _dict_or_empty(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}


def _as_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _infer_step(value: float) -> float:
    magnitude = abs(value)
    if magnitude >= 10:
        return 1.0
    if magnitude >= 1:
        return 0.1
    return 0.01


def _normalize_number(number: float, old_value: Any) -> int | float:
    if isinstance(old_value, int) and not isinstance(old_value, bool):
        return int(round(number))
    return round(number, 6)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on", "paused"}
    return False


def _fit(text: str, width: int) -> str:
    if len(text) > width:
        return text[: max(0, width - 1)]
    return text


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render EV3 live-tuning dashboard")
    parser.add_argument("--state", type=Path, default=DEFAULT_STATE_PATH, help="latest_state.json path")
    parser.add_argument("--host", default=DEFAULT_HOST, help="tuning server host for key commands")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="tuning server port for key commands")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="tuning server command timeout")
    parser.add_argument("--once", action="store_true", help="render once to stdout and exit")
    parser.add_argument("--no-curses", action="store_true", help="alias for --once smoke rendering")
    parser.add_argument("--width", type=int, default=100, help="stdout render width")
    parser.add_argument("--height", type=int, default=32, help="stdout render height")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.once or args.no_curses:
        describe, describe_error = load_describe(args.host, args.port, args.timeout)
        state, state_error, age = load_latest_state(args.state)
        session = DashboardSession(status=describe_error)
        model = build_model(state, describe, session, state_error, describe_error, age)
        print("\n".join(render_lines(model, args.width, args.height)))
        return 1 if state_error and describe_error else 0
    return run_curses(args)


if __name__ == "__main__":
    raise SystemExit(main())
