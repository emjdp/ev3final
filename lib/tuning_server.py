"""newline JSON 기반 라이브 튜닝 서버."""

import sys
import json
import socket
import threading
import time

try:
    from shared_params import SharedParams
    from telemetry import Telemetry
except ImportError:
    from lib.shared_params import SharedParams
    from lib.telemetry import Telemetry


class TuningServer(object):
    def __init__(self, params, telemetry, host="127.0.0.1", port=8765,
                 do_handler=None, stop_handler=None, pause_handler=None,
                 actions=None, stage=""):
        self.params = params
        self.telemetry = telemetry
        self.host = host
        self.port = port
        self.do_handler = do_handler
        self.stop_handler = stop_handler
        self.pause_handler = pause_handler
        self.actions = self._normalize_actions(actions or [])
        self.stage = stage or ""
        self._sock = None
        self._accept_thread = None
        self._stopping = threading.Event()

    def start(self):
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(8)
        self._sock = sock
        self.port = sock.getsockname()[1]
        thread = threading.Thread(target=self._accept_loop)
        thread.daemon = True
        thread.start()
        self._accept_thread = thread

    def stop(self):
        self._stopping.set()
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def _accept_loop(self):
        while not self._stopping.is_set():
            try:
                conn, _addr = self._sock.accept()
            except socket.error:
                if self._stopping.is_set():
                    break
                continue
            thread = threading.Thread(target=self._handle, args=(conn,))
            thread.daemon = True
            thread.start()

    def _handle(self, conn):
        try:
            infile = conn.makefile("rb")
            outfile = conn.makefile("wb")
            try:
                for raw in infile:
                    resp = self._handle_line(raw)
                    data = json.dumps(resp, sort_keys=True) + "\n"
                    outfile.write(data.encode("utf-8"))
                    outfile.flush()
            finally:
                try:
                    infile.close()
                except Exception:
                    pass
                try:
                    outfile.close()
                except Exception:
                    pass
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def _handle_line(self, raw):
        try:
            if isinstance(raw, bytes):
                raw = raw.decode("utf-8")
            req = json.loads(raw)
            return self._dispatch(req)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def _dispatch(self, req):
        if not isinstance(req, dict):
            return {"ok": False, "error": "request must be an object"}
        cmd = req.get("cmd")
        if cmd == "get":
            return self._cmd_get(req)
        if cmd == "set":
            return self._cmd_set(req)
        if cmd == "stop":
            return self._cmd_stop(req)
        if cmd == "pause":
            return self._cmd_pause(req)
        if cmd == "do":
            return self._cmd_do(req)
        if cmd == "save":
            return self._cmd_save(req)
        if cmd == "rollback":
            return self._cmd_rollback(req)
        if cmd == "get_latest":
            return {"ok": True, "latest": self.telemetry.latest()}
        if cmd == "describe":
            return self._cmd_describe(req)
        return {"ok": False, "error": "unknown cmd: {}".format(cmd)}

    def _cmd_get(self, req):
        name = req.get("name")
        if name is None:
            return {"ok": True, "params": self.params.snapshot(), "rev": self.params.rev()}
        return {"ok": True, "value": self.params.get(name), "rev": self.params.rev()}

    def _cmd_set(self, req):
        if "name" not in req or "value" not in req:
            return {"ok": False, "error": "set requires name and value"}
        ok, msg = self.params.set(req["name"], req["value"])
        if not ok:
            return {"ok": False, "error": msg, "rev": self.params.rev()}
        return {"ok": True, "value": self.params.get(req["name"]), "rev": self.params.rev()}

    def _cmd_stop(self, req):
        source = req.get("source", "network")
        if self.stop_handler is not None:
            self.stop_handler(source)
        return {"ok": True, "stopped": True, "source": source}

    def _cmd_pause(self, req):
        source = req.get("source", "network")
        paused_raw = req.get("paused", req.get("value", True))
        try:
            paused = self._coerce_bool(paused_raw)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        if self.pause_handler is None:
            return {"ok": False, "error": "pause handler is not configured"}
        result = self.pause_handler(paused, source)
        if result is None:
            result = {}
        if not isinstance(result, dict):
            result = {"result": result}
        resp = {"ok": True, "paused": paused, "source": source}
        for key in result:
            resp[key] = result[key]
        return resp

    def _cmd_do(self, req):
        action = req.get("action")
        if not action:
            return {"ok": False, "error": "do requires action"}
        args = req.get("args", {})
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return {"ok": False, "error": "args must be an object"}
        if self.actions and action not in self._action_names():
            return {"ok": False, "error": "unknown action: {}".format(action)}
        if self.do_handler is None:
            return {"ok": False, "error": "do handler is not configured"}
        result = self.do_handler(action, args)
        if result is None:
            result = {}
        if not isinstance(result, dict):
            result = {"result": result}
        resp = {"ok": True}
        for key in result:
            resp[key] = result[key]
        return resp

    def _cmd_save(self, req):
        ok, msg = self.params.save()
        if ok:
            return {"ok": True, "saved": msg, "rev": self.params.rev()}
        return {"ok": False, "error": msg, "rev": self.params.rev()}

    def _cmd_rollback(self, req):
        ok, msg = self.params.rollback()
        if ok:
            return {"ok": True, "message": msg, "params": self.params.snapshot(), "rev": self.params.rev()}
        return {"ok": False, "error": msg, "rev": self.params.rev()}

    def _cmd_describe(self, req):
        if hasattr(self.params, "describe"):
            params = self.params.describe()
        else:
            params = []
        return {
            "ok": True,
            "stage": self.stage,
            "params": params,
            "actions": list(self.actions),
            "supports_pause": self.pause_handler is not None,
        }

    def _normalize_actions(self, actions):
        normalized = []
        for item in actions:
            if not isinstance(item, dict):
                raise ValueError("action manifest entries must be objects")
            name = item.get("name")
            if not name:
                raise ValueError("action manifest entry requires name")
            label = item.get("label") or name
            normalized.append({"name": str(name), "label": str(label)})
        return normalized

    def _action_names(self):
        return set([item["name"] for item in self.actions])

    def _coerce_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            text = value.strip().lower()
            if text in ("1", "true", "yes", "on", "pause", "paused"):
                return True
            if text in ("0", "false", "no", "off", "resume", "resumed"):
                return False
        raise ValueError("paused must be a boolean")


def _request(host, port, item):
    sock = socket.create_connection((host, port), timeout=2.0)
    try:
        fp = sock.makefile("rwb")
        fp.write((json.dumps(item) + "\n").encode("utf-8"))
        fp.flush()
        line = fp.readline()
        return json.loads(line.decode("utf-8"))
    finally:
        sock.close()


def _self_test():
    import os
    import tempfile

    path = os.path.join(tempfile.mkdtemp(), "stage1.json")
    params = SharedParams(
        {"kp": 0.5},
        {"kp": (0.0, 3.0)},
        {"kp": 0.5},
        path,
        {"kp": 0.05},
        {"kp": ""},
    )
    tele = Telemetry()
    tele.publish({"reflect": 34})
    seen = {"stop": None, "pause": None, "do": None}

    def do_handler(action, args):
        seen["do"] = (action, args)
        return {"queued": action}

    def stop_handler(source):
        seen["stop"] = source

    def pause_handler(paused, source):
        seen["pause"] = (paused, source)

    server = TuningServer(
        params,
        tele,
        port=0,
        do_handler=do_handler,
        stop_handler=stop_handler,
        pause_handler=pause_handler,
        actions=[{"name": "nudge", "label": "nudge"}],
        stage="test",
    )
    server.start()
    time.sleep(0.05)
    host = server.host
    port = server.port
    assert _request(host, port, {"cmd": "get", "name": "kp"})["value"] == 0.5
    desc = _request(host, port, {"cmd": "describe"})
    assert desc["stage"] == "test"
    assert desc["params"][0]["name"] == "kp"
    assert desc["params"][0]["step"] == 0.05
    assert desc["actions"][0]["name"] == "nudge"
    assert desc["supports_pause"] is True
    assert _request(host, port, {"cmd": "set", "name": "kp", "value": 0.7})["ok"] is True
    assert _request(host, port, {"cmd": "get_latest"})["latest"]["reflect"] == 34
    assert _request(host, port, {"cmd": "do", "action": "nudge", "args": {"ms": 120}})["queued"] == "nudge"
    assert seen["do"] == ("nudge", {"ms": 120})
    assert _request(host, port, {"cmd": "pause", "paused": True, "source": "test"})["paused"] is True
    assert seen["pause"] == (True, "test")
    assert _request(host, port, {"cmd": "pause", "paused": False, "source": "test"})["paused"] is False
    assert seen["pause"] == (False, "test")
    assert _request(host, port, {"cmd": "stop", "source": "test"})["ok"] is True
    assert seen["stop"] == "test"

    sock = socket.create_connection((host, port), timeout=2.0)
    try:
        fp = sock.makefile("rwb")
        fp.write(b"{broken json\n")
        fp.flush()
        bad = json.loads(fp.readline().decode("utf-8"))
        assert bad["ok"] is False
        fp.write((json.dumps({"cmd": "get", "name": "kp"}) + "\n").encode("utf-8"))
        fp.flush()
        good = json.loads(fp.readline().decode("utf-8"))
        assert good["ok"] is True
    finally:
        sock.close()
        server.stop()
    print("tuning_server self-test ok")


def _demo():
    params = SharedParams(
        {"kp": 0.75, "ki": 0.0, "kd": 0.06, "base_speed": 22, "turn_limit": 35, "target_reflect": 35},
        {
            "kp": (0.0, 3.0),
            "ki": (0.0, 0.5),
            "kd": (0.0, 1.0),
            "base_speed": (5, 45),
            "turn_limit": (5, 60),
            "target_reflect": (0, 100),
        },
        {"kp": 0.1, "ki": 0.02, "kd": 0.05, "base_speed": 5, "turn_limit": 10, "target_reflect": 5},
        "config/demo.json",
        {"kp": 0.05, "ki": 0.01, "kd": 0.01, "base_speed": 1, "turn_limit": 5, "target_reflect": 1},
        {"base_speed": "%", "turn_limit": "%", "target_reflect": "%"},
        ["kp", "ki", "kd", "base_speed", "turn_limit", "target_reflect"],
    )
    telemetry = Telemetry()
    state = {"running": False, "last_reason": None, "last_action": None}
    started = time.time()
    last_publish = [started]

    def publish_demo_frame():
        now = time.time()
        frame = {
            "t_ms": int((now - started) * 1000),
            "dt_ms": int((now - last_publish[0]) * 1000),
            "param_rev": params.rev(),
            "running": state["running"],
        }
        if state["last_reason"] is not None:
            frame["last_reason"] = state["last_reason"]
        if state["last_action"] is not None:
            frame["last_action"] = state["last_action"]
        telemetry.publish(frame)
        last_publish[0] = now

    publish_demo_frame()

    def do_handler(action, args):
        state["last_action"] = action
        state["last_reason"] = "DO_" + action.upper()
        publish_demo_frame()
        return {"queued": action, "args": args}

    def stop_handler(source):
        state["running"] = False
        state["last_reason"] = "EMERGENCY_STOP"
        publish_demo_frame()

    server = TuningServer(
        params,
        telemetry,
        do_handler=do_handler,
        stop_handler=stop_handler,
        actions=[
            {"name": "follow_once", "label": "Follow Once"},
            {"name": "nudge", "label": "Nudge"},
            {"name": "turn_left", "label": "Turn Left"},
            {"name": "turn_right", "label": "Turn Right"},
            {"name": "uturn", "label": "U-Turn"},
        ],
        stage="demo",
    )
    server.start()
    print("demo tuning server listening on {}:{} (Ctrl-C to stop)".format(server.host, server.port))
    try:
        while True:
            publish_demo_frame()
            time.sleep(1.0)
    except KeyboardInterrupt:
        server.stop()
        print("stopped")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--demo":
        _demo()
    else:
        _self_test()
