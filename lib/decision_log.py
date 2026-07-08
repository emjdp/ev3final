"""판단/행동 reason_code 이벤트 기록."""

import threading
import time


class DecisionLog(object):
    def __init__(self, telemetry=None, sink=None):
        self.telemetry = telemetry
        self.sink = sink
        self._start = time.time()
        self._last_reason = None
        self._lock = threading.Lock()
        self._pending = []

    def log(self, event, reason, **detail):
        item = {
            "t_ms": self._now_ms(),
            "event": event,
            "reason": reason,
        }
        for key in detail:
            item[key] = detail[key]
        self._last_reason = event
        with self._lock:
            self._pending.append(dict(item))
        if self.sink is not None:
            self.sink(dict(item))
        return item

    def last_reason(self):
        return self._last_reason

    def drain_events(self):
        """마지막 drain 이후 쌓인 이벤트를 꺼낸다(Runner.publish 에서 frame["events"] 로 실어보냄)."""
        with self._lock:
            pending, self._pending = self._pending, []
        return pending

    def _now_ms(self):
        return int((time.time() - self._start) * 1000)


def _self_test():
    from telemetry import Telemetry

    events = []
    tele = Telemetry()
    log = DecisionLog(telemetry=tele, sink=events.append)
    item = log.log("LINE_FOLLOW", "PID", reflect=33, error=-2)
    assert item["event"] == "LINE_FOLLOW"
    assert item["reason"] == "PID"
    assert "t_ms" in item
    assert events[0]["reflect"] == 33
    assert log.last_reason() == "LINE_FOLLOW"
    drained = log.drain_events()
    assert len(drained) == 1 and drained[0]["event"] == "LINE_FOLLOW"
    assert log.drain_events() == []
    print("decision_log self-test ok")


if __name__ == "__main__":
    _self_test()
