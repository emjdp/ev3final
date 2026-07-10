#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fxck2(=fxck1 + 조종 오버라이드 ijkl 1회 개입 + 의심지점 상대 감속) 단위 테스트.

PC 단독 실행(ev3dev2 불필요). fxck2 신설 검증(주행/판단층 회귀는
test_gg5_logic.py, fxck1 층은 test_fxck1_logic.py 가 담당):
  - ACTIONS: 의심지점 manual_* 5종(a/d/w/s/x)에 더해 오버라이드 4종이
    i/k/j/l 키를 선언하고 OVERRIDE_CMDS/MOVES 와 정합. 오버라이드 키는
    대시보드 내장 키와 겹치지 않는다(일반 액션 폴스루로 아무 때나 전송돼야
    하므로 — manual_wait 게이트를 타는 a/s 와 다르다).
  - Runner.on_do: 오버라이드는 주행 중(override_armed)에만 수리하고 출발/
    완주 대기 중엔 무시. 의심지점 manual_* 게이트(manual_waiting)는 종전대로.
  - manual_override: 이동 1회 후 즉시 반환(잡고 있는 루프 없음 — 통제권
    자동 반환). 의심지점 대기 중 ijkl 은 이동만 하고 계속 대기한다.
  - confirm_speed/slow_speed: base_speed 비례 + 하한 — 기본 base 30% 에선
    fxck1 고정값(7/12%)과 사실상 동일.
  - 대시보드(수정 없음): follow 모드에서 i/k/j/l 이 일반 폴스루로 액션
    전송되고, manual_wait 게이트·내장 키 의미([s] STOP 등)는 종전 그대로.
    오버라이드 액션은 manual_* 접두라 last_action 재발사에서 제외된다.

실행: python3 tests/test_fxck2_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from old_stages.fxck2 import (ACTIONS, CONFIRM_SPEED_MIN, MANUAL_MOVES,
                          OVERRIDE_CMDS, OVERRIDE_MOVES, OVERRIDE_STEP_MM,
                          OVERRIDE_TURN_DEG, Runner, SLOW_SPEED_MIN,
                          STAGE_NAME, confirm_speed, slow_speed)
from lib.tuning_server import TuningServer

sys.path.insert(0, os.path.join(_ROOT, "tools"))
import dashboard


MANUAL_KEYS = {"manual_left": "a", "manual_right": "d", "manual_uturn": "w",
               "manual_back": "s", "manual_straight": "x"}
OVERRIDE_KEYS = {"manual_fwd": "i", "manual_rev": "k",
                 "manual_spin_left": "j", "manual_spin_right": "l",
                 "manual_resume": "o"}
# 대시보드 handle_key 내장 키 — 오버라이드 키가 이들과 겹치면 폴스루로
# 로봇에 도달하지 못한다(manual_wait 게이트 우선권이 없으므로).
DASHBOARD_BUILTIN_KEYS = set("qQsS aAcCrR.gG+-=\t")


class _DummyHw(object):
    """Runner 생성용 최소 하드웨어 — 표시 스레드/정지가 부를 수 있는 것만."""

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

    def last_reason(self):
        return None

    def drain_events(self):
        return []


class _DummyParams(object):

    def rev(self):
        return 0

    def snapshot(self):
        return {}


class _StopOnPublishTele(object):
    """publish 프레임을 기록하고 첫 프레임에서 러너를 세운다(무한 대기 방지)."""

    def __init__(self):
        self.frames = []
        self.runner = None

    def publish(self, frame):
        self.frames.append(frame)
        self.runner.stop_on = True


def _make_runner(log=None):
    return Runner(_DummyHw(), None, None, log)


class TestActions(unittest.TestCase):

    def test_actions_declare_expected_keys(self):
        keys = dict((a["name"], a.get("key")) for a in ACTIONS if "key" in a)
        expected = dict(MANUAL_KEYS)
        expected.update(OVERRIDE_KEYS)
        self.assertEqual(keys, expected)

    def test_override_cmds_and_moves(self):
        self.assertEqual(OVERRIDE_CMDS, frozenset(OVERRIDE_KEYS))
        self.assertEqual(OVERRIDE_MOVES, {"manual_spin_left": "L",
                                          "manual_spin_right": "R"})
        self.assertTrue(OVERRIDE_STEP_MM > 0)
        # j/l 은 90도 격자가 아니라 30도 세분 피벗 — 연타 3회 = 90도.
        self.assertEqual(OVERRIDE_TURN_DEG, 30)

    def test_resume_action_exists(self):
        # [o] = 의심지점 대기 해제(통제권 반환) — 이동 키가 아니므로
        # OVERRIDE_MOVES 에는 없다.
        self.assertIn("manual_resume", [a["name"] for a in ACTIONS])
        self.assertNotIn("manual_resume", OVERRIDE_MOVES)

    def test_override_keys_free_of_dashboard_builtins(self):
        for name, key in OVERRIDE_KEYS.items():
            self.assertNotIn(key, DASHBOARD_BUILTIN_KEYS, name)

    def test_override_names_keep_manual_prefix(self):
        # _run_action 의 last_action 기록 제외([.] repeat/auto-rerun 이
        # 이동을 재발사하지 않는 조건)가 manual_ 접두에 걸려 있다.
        for name in OVERRIDE_CMDS:
            self.assertTrue(name.startswith("manual_"), name)

    def test_fxck1_manual_moves_kept(self):
        names = set(MANUAL_KEYS) - {"manual_back"}
        self.assertEqual(set(MANUAL_MOVES), names)
        self.assertEqual(MANUAL_MOVES["manual_left"], "L")
        self.assertEqual(MANUAL_MOVES["manual_straight"], "S")

    def test_gg5_actions_kept(self):
        names = [a["name"] for a in ACTIONS]
        for name in ("calibrate", "read_color", "read_reflect", "reset"):
            self.assertIn(name, names)


class TestRunnerOverrideGate(unittest.TestCase):

    def test_override_ignored_when_not_driving(self):
        runner = _make_runner()
        resp = runner.on_do("manual_fwd", {})
        self.assertEqual(resp.get("ignored"), "manual_fwd")
        self.assertIsNone(runner.override_cmd)

    def test_override_queued_while_driving(self):
        runner = _make_runner()
        runner.override_armed = True
        resp = runner.on_do("manual_spin_left", {})
        self.assertEqual(resp.get("queued"), "manual_spin_left")
        self.assertEqual(runner.override_cmd, "manual_spin_left")
        # 마지막 입력이 이긴다(한 칸 큐 — 호출부 루프가 하나씩 소비).
        runner.on_do("manual_fwd", {})
        self.assertEqual(runner.override_cmd, "manual_fwd")

    def test_junction_manual_gate_unchanged(self):
        # 주행 중(override_armed)이라도 의심지점 키(a/d/w/s/x)는
        # manual_wait 대기 중에만 수리한다 — 오버라이드 큐로 새지 않는다.
        runner = _make_runner()
        runner.override_armed = True
        resp = runner.on_do("manual_left", {})
        self.assertEqual(resp.get("ignored"), "manual_left")
        self.assertIsNone(runner.manual_cmd)
        self.assertIsNone(runner.override_cmd)

    def test_manual_queued_while_waiting(self):
        runner = _make_runner()
        runner.manual_waiting = True
        resp = runner.on_do("manual_right", {})
        self.assertEqual(resp.get("queued"), "manual_right")
        self.assertEqual(runner.manual_cmd, "manual_right")

    def test_reset_still_queued_normally(self):
        runner = _make_runner()
        resp = runner.on_do("reset", {"source": "test"})
        self.assertEqual(resp.get("queued"), "reset")
        self.assertTrue(runner.reset_on)


class TestOverrideOneShot(unittest.TestCase):

    def test_one_shot_returns_control(self):
        # 잡고 있는 루프가 없다 — 이동 1회 후 즉시 반환(자동 통제권 반환).
        # 구식(hold-until-release) 구현이면 이 테스트는 무한 대기로 실패한다.
        runner = Runner(_DummyHw(), None, None, _DummyLog())
        moves = []
        runner._override_move = lambda cmd: moves.append(cmd)
        runner.manual_override("manual_fwd")
        self.assertEqual(moves, ["manual_fwd"])

    def test_spin_uses_fine_pivot_not_grid_turn(self):
        # j/l 은 30도 세분 피벗으로 간다 — 90도 격자 turn()(heading 갱신 +
        # 라인 재획득)을 부르면 안 된다.
        runner = Runner(_DummyHw(), None, None, _DummyLog())
        pivots = []
        turns = []
        runner._override_pivot = lambda move: pivots.append(move)
        runner.turn = lambda *a, **k: turns.append(a)
        runner._override_move("manual_spin_left")
        runner._override_move("manual_spin_right")
        self.assertEqual(pivots, ["L", "R"])
        self.assertEqual(turns, [])

    def test_seize_is_silent_and_interruptible(self):
        # 진입 비프음 없음(fxck1 의 manual_wait 무음 정책 유지) +
        # stop 플래그면 이동 없이 즉시 반환.
        hw = _DummyHw()
        runner = Runner(hw, None, None, _DummyLog())
        moves = []
        runner._override_move = lambda cmd: moves.append(cmd)
        runner.stop_on = True
        runner.manual_override("manual_fwd")
        self.assertEqual(hw.tones, [])
        self.assertEqual(moves, [])

    def test_manual_wait_o_returns_control(self):
        # [o] — 이동 없이 즉시 대기를 끝내고 통제권을 로봇에 돌려준다.
        runner = Runner(_DummyHw(), None, None, _DummyLog())
        calls = []
        runner.manual_override = lambda cmd: calls.append(cmd)
        with runner._pending_lock:
            runner.override_cmd = "manual_resume"
        runner.manual_decide(True, True, False)     # 즉시 반환해야 한다
        self.assertEqual(calls, [])                 # 이동 아님
        self.assertFalse(runner.manual_waiting)
        self.assertIsNone(runner.override_cmd)

    def test_manual_wait_override_keeps_waiting(self):
        # 의심지점 대기 중 ijkl — 이동만 하고 계속 대기한다(위치 교정,
        # [s] 후진과 같은 성격). 이동 뒤 publish(manual_wait)가 나오는
        # 것이 대기 지속의 증거 — 첫 프레임에서 stop 시켜 루프를 끝낸다.
        tele = _StopOnPublishTele()
        runner = Runner(_DummyHw(), _DummyParams(), tele, _DummyLog())
        tele.runner = runner
        calls = []
        runner.manual_override = lambda cmd: calls.append(cmd)
        with runner._pending_lock:
            runner.override_cmd = "manual_rev"
        runner.manual_decide(True, False, True)
        self.assertEqual(calls, ["manual_rev"])
        self.assertEqual(tele.frames[0]["mode"], "manual_wait")
        self.assertFalse(runner.manual_waiting)
        self.assertIsNone(runner.override_cmd)


class TestRelativeSlowSpeeds(unittest.TestCase):

    def test_matches_fxck1_at_default_base(self):
        # 기본 base 30% — fxck1 고정값(7/12%)과 사실상 동일.
        self.assertAlmostEqual(confirm_speed(30), 7.5)
        self.assertAlmostEqual(slow_speed(30), 12.0)

    def test_scales_with_base(self):
        self.assertAlmostEqual(confirm_speed(60), 15.0)
        self.assertAlmostEqual(slow_speed(60), 24.0)
        self.assertAlmostEqual(confirm_speed(40) / confirm_speed(20), 2.0)
        self.assertAlmostEqual(slow_speed(40) / slow_speed(20), 2.0)

    def test_floors(self):
        self.assertEqual(confirm_speed(5), CONFIRM_SPEED_MIN)
        self.assertEqual(slow_speed(5), SLOW_SPEED_MIN)


class TestDescribeKeyPassthrough(unittest.TestCase):

    def test_normalize_actions_keeps_key(self):
        server = TuningServer(None, None, port=0, actions=ACTIONS,
                              stage=STAGE_NAME)
        by_name = dict((a["name"], a) for a in server.actions)
        for name, key in list(MANUAL_KEYS.items()) + list(OVERRIDE_KEYS.items()):
            self.assertEqual(by_name[name].get("key"), key)
        self.assertNotIn("key", by_name["calibrate"])


def _model(mode="follow", age=0.5, session=None):
    describe = {"stage": STAGE_NAME, "params": [], "actions": ACTIONS}
    state = {"latest": {"running": True, "mode": mode}}
    return dashboard.build_model(state, describe, session,
                                 state_error="", describe_error="",
                                 state_age_s=age)


class TestDashboardOverrideRouting(unittest.TestCase):
    """대시보드 수정 없이 i/k/j/l 이 로봇에 도달하는지 — 일반 액션 폴스루."""

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

    def test_follow_routes_ijkl_to_actions(self):
        for name, char in sorted(OVERRIDE_KEYS.items()):
            self.sent[:] = []
            self._press(char, "follow")
            self.assertEqual(self.sent,
                             [{"cmd": "do", "action": name, "args": {}}],
                             "key " + char)

    def test_manual_wait_routes_ijkl_too(self):
        # 의심지점 대기 중에도 오버라이드 키가 로봇에 도달한다
        # (manual_* 접두라 manual_wait 게이트가 우선 전송).
        for name, char in sorted(OVERRIDE_KEYS.items()):
            self.sent[:] = []
            self._press(char, "manual_wait")
            self.assertEqual(self.sent,
                             [{"cmd": "do", "action": name, "args": {}}],
                             "key " + char)

    def test_builtin_meanings_kept_outside_manual_wait(self):
        # [s] 는 여전히 STOP, [a] 는 auto-rerun 토글(네트워크 요청 없음).
        self._press("s", "follow")
        self.assertEqual(self.sent[0]["cmd"], "stop")
        self.sent[:] = []
        session = self._press("a", "follow")
        self.assertEqual(self.sent, [])
        self.assertTrue(session.auto_rerun)

    def test_override_not_recorded_as_last_action(self):
        session = dashboard.DashboardSession()
        for name in sorted(OVERRIDE_CMDS):
            dashboard._run_action(name, session, "h", 1, 1.0)
            self.assertEqual(session.last_action, "", name)


class TestDashboardBindings(unittest.TestCase):

    def test_explicit_keys_bound(self):
        model = _model()
        binding = dict((a.name, a.key) for a in model.actions)
        for name, key in list(MANUAL_KEYS.items()) + list(OVERRIDE_KEYS.items()):
            self.assertEqual(binding[name], key)
        # 기존 4개 액션은 종전대로 1~4.
        self.assertEqual(binding["calibrate"], "1")
        self.assertEqual(binding["reset"], "4")


if __name__ == "__main__":
    unittest.main()
