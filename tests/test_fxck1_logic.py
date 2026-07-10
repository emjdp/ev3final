#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fxck1(=gg5 + 의심지점 원격 판단) 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

fxck1 신설 검증(주행/판단층은 gg5 그대로 — 회귀는 test_gg5_logic.py 가 담당):
  - ACTIONS: manual_* 5종이 a/d/w/s/x 키를 선언하고 MANUAL_MOVES 와 정합.
  - Runner.on_do: manual_* 는 의심지점 대기(manual_waiting) 중에만 수리,
    주행 중엔 무시(ignored) — 대시보드 게이트와 이중 안전장치.
  - TuningServer._normalize_actions 가 "key" 필드를 describe 로 보존.
  - 대시보드: describe 의 명시 키를 바인딩에 우선 사용하고,
    manual_wait 모드(신선한 상태 파일)에서만 a/d/w/s/x 를 manual_* 액션으로
    보낸다 — 그 외엔 기존 의미(a=auto-rerun, s=STOP) 유지, stale 이면 비활성.

실행: python3 tests/test_fxck1_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from old_stages.fxck1 import ACTIONS, MANUAL_MOVES, Runner, STAGE_NAME
from lib.tuning_server import TuningServer

sys.path.insert(0, os.path.join(_ROOT, "tools"))
import dashboard


MANUAL_KEYS = {"manual_left": "a", "manual_right": "d", "manual_uturn": "w",
               "manual_back": "s", "manual_straight": "x"}


class _DummyHw(object):
    """Runner 생성용 최소 하드웨어 — 표시 스레드가 부를 수 있는 것만."""

    def __init__(self):
        self.tones = []

    def show_final4_display(self, *args):
        pass

    def stop(self):
        pass

    def tone(self, *args):
        self.tones.append(args)


class _DummyLog(object):

    def log(self, *args, **kwargs):
        pass


def _make_runner():
    return Runner(_DummyHw(), None, None, None)


class TestActions(unittest.TestCase):

    def test_manual_actions_declare_expected_keys(self):
        keys = dict((a["name"], a.get("key")) for a in ACTIONS
                    if a["name"].startswith("manual_"))
        self.assertEqual(keys, MANUAL_KEYS)

    def test_manual_moves_match_actions(self):
        # manual_back 은 이동이 아니라 후진 후 재대기라 MANUAL_MOVES 에 없다.
        names = set(k for k in MANUAL_KEYS) - {"manual_back"}
        self.assertEqual(set(MANUAL_MOVES), names)
        self.assertEqual(MANUAL_MOVES["manual_left"], "L")
        self.assertEqual(MANUAL_MOVES["manual_right"], "R")
        self.assertEqual(MANUAL_MOVES["manual_uturn"], "U")
        self.assertEqual(MANUAL_MOVES["manual_straight"], "S")

    def test_gg5_actions_kept(self):
        names = [a["name"] for a in ACTIONS]
        for name in ("calibrate", "read_color", "read_reflect", "reset"):
            self.assertIn(name, names)


class TestRunnerManualGate(unittest.TestCase):

    def test_manual_ignored_while_driving(self):
        runner = _make_runner()
        resp = runner.on_do("manual_left", {})
        self.assertEqual(resp.get("ignored"), "manual_left")
        self.assertIsNone(runner.manual_cmd)

    def test_manual_queued_while_waiting(self):
        runner = _make_runner()
        runner.manual_waiting = True
        resp = runner.on_do("manual_right", {})
        self.assertEqual(resp.get("queued"), "manual_right")
        self.assertEqual(runner.manual_cmd, "manual_right")
        # 마지막 입력이 이긴다(대기 루프가 하나씩 소비).
        runner.on_do("manual_uturn", {})
        self.assertEqual(runner.manual_cmd, "manual_uturn")

    def test_reset_still_queued_normally(self):
        runner = _make_runner()
        resp = runner.on_do("reset", {"source": "test"})
        self.assertEqual(resp.get("queued"), "reset")
        self.assertTrue(runner.reset_on)


class TestDescribeKeyPassthrough(unittest.TestCase):

    def test_normalize_actions_keeps_key(self):
        server = TuningServer(None, None, port=0, actions=ACTIONS,
                              stage=STAGE_NAME)
        by_name = dict((a["name"], a) for a in server.actions)
        for name, key in MANUAL_KEYS.items():
            self.assertEqual(by_name[name].get("key"), key)
        self.assertNotIn("key", by_name["calibrate"])


def _model(mode="follow", age=0.5, session=None):
    describe = {"stage": STAGE_NAME, "params": [], "actions": ACTIONS}
    state = {"latest": {"running": True, "mode": mode}}
    return dashboard.build_model(state, describe, session,
                                 state_error="", describe_error="",
                                 state_age_s=age)


class TestDashboardBindings(unittest.TestCase):

    def test_explicit_keys_bound(self):
        model = _model()
        binding = dict((a.name, a.key) for a in model.actions)
        for name, key in MANUAL_KEYS.items():
            self.assertEqual(binding[name], key)
        # 기존 4개 액션은 종전대로 1~4.
        self.assertEqual(binding["calibrate"], "1")
        self.assertEqual(binding["reset"], "4")

    def test_manual_wait_active_gating(self):
        self.assertFalse(dashboard._manual_wait_active(_model("follow")))
        self.assertTrue(dashboard._manual_wait_active(_model("manual_wait")))
        # stale 상태 파일이면 과거 프레임을 믿지 않는다.
        self.assertFalse(dashboard._manual_wait_active(
            _model("manual_wait", age=99.0)))
        self.assertFalse(dashboard._manual_wait_active(
            _model("manual_wait", age=None)))


class TestDashboardKeyRouting(unittest.TestCase):

    def setUp(self):
        self.sent = []
        self._orig = dashboard.send_command

        def fake_send(request, host, port, timeout=1.0):
            self.sent.append(request)
            return {"ok": True, "queued": request.get("action")}

        dashboard.send_command = fake_send

    def tearDown(self):
        dashboard.send_command = self._orig

    def _press(self, char, mode, age=0.5):
        session = dashboard.DashboardSession()
        model = _model(mode, age=age, session=session)
        dashboard.handle_key(ord(char), model, session, "h", 1)
        return session

    def test_manual_wait_routes_adwsx_to_actions(self):
        for char, name in (("a", "manual_left"), ("d", "manual_right"),
                           ("w", "manual_uturn"), ("s", "manual_back"),
                           ("x", "manual_straight")):
            self.sent[:] = []
            self._press(char, "manual_wait")
            self.assertEqual(self.sent,
                             [{"cmd": "do", "action": name, "args": {}}],
                             "key " + char)

    def test_follow_keeps_builtin_meanings(self):
        # [s] 는 여전히 STOP.
        self._press("s", "follow")
        self.assertEqual(self.sent[0]["cmd"], "stop")
        # [a] 는 auto-rerun 토글(네트워크 요청 없음).
        self.sent[:] = []
        session = self._press("a", "follow")
        self.assertEqual(self.sent, [])
        self.assertTrue(session.auto_rerun)

    def test_stale_manual_wait_falls_back_to_builtin(self):
        self._press("s", "manual_wait", age=99.0)
        self.assertEqual(self.sent[0]["cmd"], "stop")

    def test_quit_still_works_in_manual_wait(self):
        session = dashboard.DashboardSession()
        model = _model("manual_wait", age=0.5, session=session)
        should_quit, _refresh = dashboard.handle_key(ord("q"), model, session,
                                                     "h", 1)
        self.assertTrue(should_quit)


class TestNoManualTone(unittest.TestCase):

    def test_manual_wait_entry_is_silent(self):
        # 의심지점 대기 진입 시 비프음 없음(사용자 요청으로 제거).
        hw = _DummyHw()
        runner = Runner(hw, None, None, _DummyLog())
        runner.stop_on = True           # 진입 직후 반환하도록
        runner.manual_decide(True, True, False)
        self.assertEqual(hw.tones, [])
        self.assertFalse(hasattr(sys.modules["old_stages.fxck1"], "MANUAL_TONE"))


def _param_model(mode, session, age=0.5):
    """조정 가능한 숫자 파라미터 1개를 가진 모델(화살표 테스트용)."""
    describe = {"stage": STAGE_NAME, "params": [
        {"name": "base_speed", "value": 30, "min": 5, "max": 60, "step": 5},
    ], "actions": ACTIONS}
    state = {"latest": {"running": True, "mode": mode}}
    return dashboard.build_model(state, describe, session,
                                 state_error="", describe_error="",
                                 state_age_s=age)


class TestArrowKeysParamOnly(unittest.TestCase):
    """좌우 화살표 = 파라미터 수정 전용 — 로봇을 움직이는 액션 재발사 금지."""

    def setUp(self):
        self.sent = []
        self._orig = dashboard.send_command

        def fake_send(request, host, port, timeout=1.0):
            self.sent.append(request)
            return {"ok": True}

        dashboard.send_command = fake_send

    def tearDown(self):
        dashboard.send_command = self._orig

    def test_manual_and_reset_not_recorded_as_last_action(self):
        session = dashboard.DashboardSession()
        dashboard._run_action("manual_left", session, "h", 1, 1.0)
        self.assertEqual(session.last_action, "")
        dashboard._run_action("reset", session, "h", 1, 1.0)
        self.assertEqual(session.last_action, "")
        # 튜닝용 액션은 종전대로 기록(auto-rerun/[.] repeat 용).
        dashboard._run_action("read_reflect", session, "h", 1, 1.0)
        self.assertEqual(session.last_action, "read_reflect")

    def test_arrow_no_auto_rerun_in_manual_wait(self):
        session = dashboard.DashboardSession()
        session.auto_rerun = True
        session.last_action = "calibrate"
        model = _param_model("manual_wait", session)
        import curses
        dashboard.handle_key(curses.KEY_RIGHT, model, session, "h", 1)
        self.assertEqual([r["cmd"] for r in self.sent], ["set"])

    def test_arrow_auto_rerun_kept_outside_manual_wait(self):
        # 기존 튜닝 루프(값 변경 → 마지막 read/calibrate 재실행)는 유지.
        session = dashboard.DashboardSession()
        session.auto_rerun = True
        session.last_action = "read_reflect"
        model = _param_model("follow", session)
        import curses
        dashboard.handle_key(curses.KEY_RIGHT, model, session, "h", 1)
        self.assertEqual([r["cmd"] for r in self.sent], ["set", "do"])
        self.assertEqual(self.sent[1]["action"], "read_reflect")


if __name__ == "__main__":
    unittest.main()
