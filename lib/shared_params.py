"""스레드 안전 라이브 파라미터 저장소.

브릭 Python 3.5 호환을 위해 최신 문법을 쓰지 않는다.
"""

import json
import numbers
import os
import threading


class SharedParams(object):
    def __init__(self, defaults, limits, max_step, save_path, ui_step=None, units=None, param_order=None):
        self._lock = threading.RLock()
        self._defaults = dict(defaults)
        self._limits = dict(limits)
        self._max_step = dict(max_step or {})
        self._ui_step = dict(ui_step or {})
        self._units = dict(units or {})
        self._save_path = save_path
        self._rev = 0
        self._order = list(param_order or self._defaults.keys())

        missing_limits = []
        for name in self._defaults:
            if name not in self._limits:
                missing_limits.append(name)
        if missing_limits:
            raise ValueError("missing limits for: {}".format(", ".join(sorted(missing_limits))))

        missing_defaults = []
        for name in self._limits:
            if name not in self._defaults:
                missing_defaults.append(name)
        if missing_defaults:
            raise ValueError("missing defaults for: {}".format(", ".join(sorted(missing_defaults))))

        self._values = dict(self._defaults)
        ok, msg = self._validate_all(self._values)
        if not ok:
            raise ValueError(msg)
        ok, msg = self._validate_metadata()
        if not ok:
            raise ValueError(msg)

    def snapshot(self):
        with self._lock:
            return dict(self._values)

    def get(self, name):
        with self._lock:
            return self._values.get(name)

    def rev(self):
        with self._lock:
            return self._rev

    def describe(self):
        with self._lock:
            rows = []
            names = list(self._order)
            for name in self._values:
                if name not in names:
                    names.append(name)
            for name in names:
                if name not in self._values:
                    continue
                lo, hi = self._limits[name]
                max_step = self._max_step.get(name)
                rows.append({
                    "name": name,
                    "value": self._values[name],
                    "min": lo,
                    "max": hi,
                    "step": self._describe_step(name, lo, hi, max_step),
                    "max_step": max_step,
                    "unit": self._units.get(name, ""),
                })
            return rows

    def set(self, name, value):
        with self._lock:
            if name not in self._limits:
                return False, "unknown param: {}".format(name)
            ok, msg = self._validate_value(name, value)
            if not ok:
                return False, msg
            if name in self._max_step:
                old = self._values[name]
                step = self._max_step[name]
                # 부동소수 여유(예: 1.05-1.0=0.05000000000000004 > 0.05). 정확히 한
                # MAX_STEP 만큼의 변경은 허용해야 한다(문서화된 보정 스텝). 1e-9 는
                # 어떤 param 단위에서도 무의미한 크기라 실제 과도 스텝은 그대로 거부된다.
                if abs(value - old) > step + 1e-9:
                    return False, "step too big for {} (max {}, old {}, new {})".format(
                        name, step, old, value
                    )
            self._values[name] = value
            self._rev += 1
            return True, "ok"

    def save(self):
        with self._lock:
            values = dict(self._values)
            save_path = self._save_path
        try:
            directory = os.path.dirname(save_path)
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            tmp_path = save_path + ".tmp"
            with open(tmp_path, "w") as fp:
                json.dump(values, fp, sort_keys=True, indent=2)
                fp.write("\n")
            os.rename(tmp_path, save_path)
            return True, save_path
        except Exception as exc:
            return False, str(exc)

    def rollback(self):
        if os.path.exists(self._save_path):
            ok, loaded, msg = self._read_saved()
            if not ok:
                return False, msg
        else:
            loaded = dict(self._defaults)
        with self._lock:
            changed = loaded != self._values
            self._values = dict(loaded)
            if changed:
                self._rev += 1
            return True, "ok"

    def load_saved_into_defaults(self):
        ok, loaded, msg = self._read_saved()
        if not ok:
            if os.path.exists(self._save_path):
                return False, msg
            return True, "no saved params"
        with self._lock:
            changed = loaded != self._values
            self._defaults = dict(loaded)
            self._values = dict(loaded)
            if changed:
                self._rev += 1
            return True, "ok"

    def _read_saved(self):
        try:
            with open(self._save_path, "r") as fp:
                loaded = json.load(fp)
        except Exception as exc:
            return False, None, str(exc)
        if not isinstance(loaded, dict):
            return False, None, "saved params must be an object"
        ok, msg = self._validate_all(loaded)
        if not ok:
            return False, None, msg
        return True, dict(loaded), "ok"

    def _validate_all(self, values):
        for name in values:
            if name not in self._limits:
                return False, "unknown param: {}".format(name)
        for name in self._limits:
            if name not in values:
                return False, "missing param: {}".format(name)
            ok, msg = self._validate_value(name, values[name])
            if not ok:
                return False, msg
        return True, "ok"

    def _validate_value(self, name, value):
        if isinstance(value, bool) or not isinstance(value, numbers.Number):
            return False, "param {} must be numeric".format(name)
        lo, hi = self._limits[name]
        if value < lo or value > hi:
            return False, "out of range [{}, {}] for {}: {}".format(lo, hi, name, value)
        return True, "ok"

    def _validate_metadata(self):
        known = set(self._limits.keys())
        for name in self._max_step:
            if name not in known:
                return False, "unknown max_step param: {}".format(name)
            if isinstance(self._max_step[name], bool) or not isinstance(self._max_step[name], numbers.Number):
                return False, "max_step {} must be numeric".format(name)
            if self._max_step[name] <= 0:
                return False, "max_step {} must be positive".format(name)
        for name in self._ui_step:
            if name not in known:
                return False, "unknown ui_step param: {}".format(name)
            if isinstance(self._ui_step[name], bool) or not isinstance(self._ui_step[name], numbers.Number):
                return False, "ui_step {} must be numeric".format(name)
            if self._ui_step[name] <= 0:
                return False, "ui_step {} must be positive".format(name)
        for name in self._units:
            if name not in known:
                return False, "unknown unit param: {}".format(name)
        for name in self._order:
            if name not in known:
                return False, "unknown param_order param: {}".format(name)
        return True, "ok"

    def _describe_step(self, name, lo, hi, max_step):
        if name in self._ui_step:
            return self._ui_step[name]
        if max_step is not None:
            return max_step / 10.0
        return self._infer_step(lo, hi)

    def _infer_step(self, lo, hi):
        span = abs(hi - lo)
        if span >= 100:
            return 1
        if span >= 10:
            return 0.1
        return 0.01


def _self_test():
    import tempfile

    path = os.path.join(tempfile.mkdtemp(), "stage1.json")
    params = SharedParams(
        {"kp": 0.5, "base_speed": 20},
        {"kp": (0.0, 3.0), "base_speed": (5, 45)},
        {"kp": 0.1, "base_speed": 5},
        path,
        {"kp": 0.01, "base_speed": 1},
        {"kp": "", "base_speed": "%"},
        ["kp", "base_speed"],
    )
    assert params.rev() == 0
    described = params.describe()
    assert described[0]["name"] == "kp"
    assert described[0]["step"] == 0.01
    assert described[1]["unit"] == "%"
    assert params.set("missing", 1)[0] is False
    assert params.set("kp", 9)[0] is False
    assert params.set("kp", 0.8)[0] is False
    assert params.set("kp", 0.6)[0] is True
    assert params.rev() == 1
    assert params.save()[0] is True
    assert params.set("kp", 0.7)[0] is True
    assert params.rollback()[0] is True
    assert params.get("kp") == 0.6
    print("shared_params self-test ok")


if __name__ == "__main__":
    _self_test()
