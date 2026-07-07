"""최신 telemetry 프레임 1개를 보관한다."""

import threading


class Telemetry(object):
    def __init__(self):
        self._lock = threading.RLock()
        self._latest = {}

    def publish(self, frame):
        if frame is None:
            frame = {}
        with self._lock:
            self._latest = dict(frame)

    def latest(self):
        with self._lock:
            return dict(self._latest)


def _self_test():
    telemetry = Telemetry()
    frame = {"reflect": 34}
    telemetry.publish(frame)
    frame["reflect"] = 99
    latest = telemetry.latest()
    assert latest["reflect"] == 34
    latest["reflect"] = 0
    assert telemetry.latest()["reflect"] == 34
    print("telemetry self-test ok")


if __name__ == "__main__":
    _self_test()
