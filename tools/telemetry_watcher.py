#!/usr/bin/env python3
"""EV3 최신 telemetry 를 polling 해서 노트북 runs/ 에 기록한다."""

from __future__ import annotations

import argparse
import json
import socket
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable


RequestFn = Callable[[str, int, dict[str, Any], float], dict[str, Any]]


def request(host: str, port: int, payload: dict[str, Any], timeout: float = 2.0) -> dict[str, Any]:
    """튜닝 서버에 newline-JSON 요청 1개를 보내고 응답 1개를 받는다."""

    data = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(data)
        chunks: list[bytes] = []
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break

    raw = b"".join(chunks).splitlines()
    if not raw:
        raise RuntimeError("empty response")
    return json.loads(raw[0].decode("utf-8"))


def safe_timestamp(now: datetime | None = None) -> str:
    """파일명에 안전한 로컬 타임스탬프."""

    return (now or datetime.now()).strftime("%Y-%m-%dT%H-%M-%S")


def atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(
        json.dumps(data, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    tmp.replace(path)


def append_jsonl(handle, data: dict[str, Any]) -> None:
    handle.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + "\n")
    handle.flush()


def response_params(resp: dict[str, Any]) -> dict[str, Any]:
    """get 응답에서 params dict 를 가능한 형태별로 꺼낸다."""

    for key in ("params", "values", "value"):
        value = resp.get(key)
        if isinstance(value, dict):
            return dict(value)
    return {}


def latest_frame(resp: dict[str, Any]) -> dict[str, Any]:
    latest = resp.get("latest", resp)
    if not isinstance(latest, dict):
        raise RuntimeError("latest is not an object")
    return dict(latest)


def event_records_from_frame(frame: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """frame 의 event/events 를 events.jsonl 용 레코드로 분리한다."""

    sample = dict(frame)
    raw_events: list[Any] = []
    if "event" in sample:
        raw_events.append(sample.pop("event"))
    if "events" in sample:
        events_value = sample.pop("events")
        if isinstance(events_value, list):
            raw_events.extend(events_value)
        else:
            raw_events.append(events_value)

    base = {
        key: frame[key]
        for key in ("t_ms", "dt_ms", "param_rev", "running", "last_reason")
        if key in frame
    }
    records: list[dict[str, Any]] = []
    for raw in raw_events:
        if raw is None:
            continue
        if isinstance(raw, dict):
            event = dict(base)
            event.update(raw)
            records.append(event)
        else:
            event = dict(base)
            event["event"] = str(raw)
            reason = frame.get("reason") or frame.get("last_reason")
            if reason is not None:
                event["reason"] = reason
            records.append(event)
    return sample, records


@dataclass
class WatchRun:
    runs_dir: Path
    stage: str
    started: str = field(default_factory=safe_timestamp)

    def __post_init__(self) -> None:
        self.run_dir = self._unique_run_dir(self.runs_dir / self.started)
        self.current_dir = self.runs_dir / "current"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.current_dir.mkdir(parents=True, exist_ok=True)
        self.samples_path = self.run_dir / "samples.jsonl"
        self.events_path = self.run_dir / "events.jsonl"
        self.params_path = self.run_dir / "params.json"
        self.latest_state_path = self.current_dir / "latest_state.json"
        self.samples = self.samples_path.open("a", encoding="utf-8")
        self.events = self.events_path.open("a", encoding="utf-8")
        self.params_doc: dict[str, Any] = {
            "started": self.started,
            "stage": self.stage,
            "initial": {},
            "changes": [],
        }
        self._current_params: dict[str, Any] = {}
        self._last_param_rev: Any = None
        self._seen_event_keys: set[str] = set()
        self._last_event_keys: list[str] = []
        self._recent_events: list[dict[str, Any]] = []

    @staticmethod
    def _unique_run_dir(base: Path) -> Path:
        if not base.exists():
            return base
        for index in range(1, 100):
            candidate = base.with_name("{}-{:02d}".format(base.name, index))
            if not candidate.exists():
                return candidate
        raise RuntimeError("cannot allocate run directory near {}".format(base))

    def close(self) -> None:
        self.write_params()
        self.samples.close()
        self.events.close()

    def __enter__(self) -> "WatchRun":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def write_params(self) -> None:
        atomic_write_json(self.params_path, self.params_doc)

    def set_initial_params(self, params: dict[str, Any]) -> None:
        self.params_doc["initial"] = dict(params)
        self._current_params = dict(params)
        self.write_params()

    def note_params(self, params: dict[str, Any], *, t_ms: Any = None, rev: Any = None) -> None:
        changes = []
        all_names = sorted(set(self._current_params) | set(params))
        for name in all_names:
            old = self._current_params.get(name)
            new = params.get(name)
            if old != new:
                changes.append(
                    {
                        "t_ms": t_ms,
                        "rev": rev,
                        "name": name,
                        "old": old,
                        "new": new,
                        "source": "watcher:get",
                    }
                )
        if changes:
            self.params_doc["changes"].extend(changes)
            self._current_params = dict(params)
            self.write_params()

    def record_frame(self, frame: dict[str, Any]) -> None:
        sample, events = event_records_from_frame(frame)
        append_jsonl(self.samples, sample)

        new_event_keys: list[str] = []
        for event in events:
            key = json.dumps(event, ensure_ascii=False, sort_keys=True)
            new_event_keys.append(key)
            # 같은 latest frame 을 여러 번 polling 할 때 같은 이벤트가 반복 기록되는 것을 피한다.
            if key in self._seen_event_keys:
                continue
            append_jsonl(self.events, event)
            self._seen_event_keys.add(key)
            self._recent_events.append(event)
            self._recent_events = self._recent_events[-20:]
        self._last_event_keys = new_event_keys

    def update_latest_state(
        self,
        frame: dict[str, Any],
        *,
        ok: bool = True,
        error: str | None = None,
        poll_time: float | None = None,
    ) -> None:
        latest = {
            "ok": ok,
            "stage": self.stage,
            "run_dir": str(self.run_dir),
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "poll_time": poll_time if poll_time is not None else time.time(),
            "frame": frame,
            "params": {"values": self._current_params},
            "recent_events": list(self._recent_events),
        }
        if error:
            latest["error"] = error
        atomic_write_json(self.latest_state_path, latest)

    def should_refresh_params(self, frame: dict[str, Any]) -> bool:
        rev = frame.get("param_rev")
        if rev is None:
            return False
        if self._last_param_rev is None:
            self._last_param_rev = rev
            return False
        if rev != self._last_param_rev:
            self._last_param_rev = rev
            return True
        return False


class Watcher:
    def __init__(
        self,
        host: str,
        port: int,
        run: WatchRun,
        *,
        rate_hz: float = 4.0,
        timeout: float = 2.0,
        request_fn: RequestFn = request,
    ) -> None:
        if not 3.0 <= rate_hz <= 5.0:
            raise ValueError("rate_hz must be between 3 and 5")
        self.host = host
        self.port = port
        self.run = run
        self.rate_hz = rate_hz
        self.timeout = timeout
        self.request_fn = request_fn

    def get(self) -> dict[str, Any]:
        resp = self.request_fn(self.host, self.port, {"cmd": "get"}, self.timeout)
        if resp.get("ok") is False:
            raise RuntimeError(resp.get("error", "get failed"))
        return response_params(resp)

    def get_latest(self) -> dict[str, Any]:
        resp = self.request_fn(self.host, self.port, {"cmd": "get_latest"}, self.timeout)
        if resp.get("ok") is False:
            raise RuntimeError(resp.get("error", "get_latest failed"))
        return latest_frame(resp)

    def poll_once(self) -> dict[str, Any]:
        frame = self.get_latest()
        self.run.record_frame(frame)
        if self.run.should_refresh_params(frame):
            try:
                params = self.get()
            except Exception:
                params = {}
            if params:
                self.run.note_params(params, t_ms=frame.get("t_ms"), rev=frame.get("param_rev"))
        self.run.update_latest_state(frame)
        return frame

    def run_forever(self, *, duration: float | None = None, once: bool = False) -> None:
        try:
            initial = self.get()
        except Exception:
            initial = {}
        self.run.set_initial_params(initial)

        period = 1.0 / self.rate_hz
        deadline = time.monotonic() + duration if duration is not None else None
        while True:
            started = time.monotonic()
            try:
                self.poll_once()
            except Exception as exc:
                self.run.update_latest_state({}, ok=False, error=str(exc))
            if once:
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, period - elapsed))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="EV3 telemetry watcher")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--stage", default="unknown")
    parser.add_argument("--runs-dir", type=Path, default=Path("runs"))
    parser.add_argument("--rate", type=float, default=4.0, help="polling Hz (3..5, default 4)")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=None, help="seconds; omit for Ctrl-C")
    parser.add_argument("--once", action="store_true", help="poll exactly once")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        with WatchRun(args.runs_dir, args.stage) as run:
            watcher = Watcher(
                args.host,
                args.port,
                run,
                rate_hz=args.rate,
                timeout=args.timeout,
            )
            print("recording {}".format(run.run_dir))
            watcher.run_forever(duration=args.duration, once=args.once)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
