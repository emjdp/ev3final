"""판단/행동 reason_code 이벤트 기록."""

import time


class DecisionLog(object):
    def __init__(self, telemetry=None, sink=None):
        self.telemetry = telemetry
        self.sink = sink
        self._start = time.time()
        self._last_reason = None

    def log(self, event, reason, **detail):
        item = {
            "t_ms": self._now_ms(),
            "event": event,
            "reason": reason,
        }
        for key in detail:
            item[key] = detail[key]
        self._last_reason = event
        self._publish_last_reason(event, item["t_ms"])
        if self.sink is not None:
            self.sink(dict(item))
        return item

    def last_reason(self):
        return self._last_reason

    def _now_ms(self):
        return int((time.time() - self._start) * 1000)

    def _publish_last_reason(self, event, t_ms):
        if self.telemetry is None:
            return
        latest = self.telemetry.latest()
        latest["last_reason"] = event
        if "t_ms" not in latest:
            latest["t_ms"] = t_ms
        self.telemetry.publish(latest)


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
    assert tele.latest()["last_reason"] == "LINE_FOLLOW"
    print("decision_log self-test ok")


if __name__ == "__main__":
    _self_test()
