#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fxck3(풀 수동 조종 wasd) 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

fxck3 신설 검증:
  - ACTIONS: manual_* 6종이 w/s/a/d/j/l 키를 선언하고 MANUAL_CMDS/TURNS 와 정합.
  - Runner.on_do: wasd 는 주행 세션 중(manual_armed)에만 수리, 대기 중 무시.
  - 대시보드(수정 없음): 로봇이 manual_wait 를 publish 하는 동안 w/s/a/d 가
    manual_* 액션으로 라우팅된다(특히 [s]=STOP, [a]=auto-rerun 내장 의미를
    게이트가 이긴다). manual_wait 가 아니면(출발 대기/paused) 내장 의미 복귀.
  - handle_marker: 자동 판단 없음 — 빨강 red N(가는 길/초록 후 페이즈),
    초록 최초 1회만 배달+OUT 시간, 노랑은 초록 후에만 완주(그립 해제),
    다수결 미달이면 기각.

실행: python3 tests/test_fxck3_logic.py
"""

import os
import sys
import time
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from old_stages.fxck3 import (ACTIONS, COL_GREEN, COL_RED, COL_WHITE, COL_YELLOW,
                          MANUAL_CMDS, MANUAL_TURNS, PARAM_ORDER, Runner,
                          STAGE_NAME)
from lib.tuning_server import TuningServer

sys.path.insert(0, os.path.join(_ROOT, "tools"))
import dashboard


MANUAL_KEYS = {"manual_fwd": "w", "manual_rev": "s",
               "manual_left": "a", "manual_right": "d",
               "manual_left_fine": "j", "manual_right_fine": "l"}


class _DummyHw(object):
    """Runner 생성/마커 테스트용 최소 하드웨어."""

    def __init__(self, color=COL_WHITE):
        self.color = color          # read_center_color_now 가 돌려줄 색
        self.tones = []
        self.wavs = []
        self.grip = []              # ("open"|"close", speed)

    def show_final4_display(self, *args):
        pass

    def stop(self):
        pass

    def tone(self, *args):
        self.tones.append(args)

    def play_wav(self, path):
        self.wavs.append(os.path.basename(path))

    def beep_ok(self):
        self.wavs.append("beep")

    def read_center_color_now(self):
        return self.color

    def grip_open(self, speed, sec):
        self.grip.append(("open", speed))

    def grip_close(self, speed, sec):
        self.grip.append(("close", speed))


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
        return {"fwd_speed": 20, "step_mm": 50, "turn_deg": 30,
                "turn_speed": 8, "turn_ramp_ms": 250, "turn_90_factor": 0.65,
                "goal_advance_mm": 100, "grab_dist_cm": 6.0, "grip_speed": 50}


def _make_runner(hw=None):
    runner = Runner(hw or _DummyHw(), _DummyParams(), None, _DummyLog())
    runner.straight = lambda *a, **k: 0.0   # 배달 전·후진 무동작(모터 없음)
    return runner


class TestActions(unittest.TestCase):

    def test_manual_actions_declare_wasd(self):
        keys = dict((a["name"], a.get("key")) for a in ACTIONS if "key" in a)
        self.assertEqual(keys, MANUAL_KEYS)

    def test_cmds_and_turns_match(self):
        self.assertEqual(MANUAL_CMDS, frozenset(MANUAL_KEYS))
        self.assertEqual(MANUAL_TURNS, {"manual_left": "L",
                                        "manual_right": "R",
                                        "manual_left_fine": "L",
                                        "manual_right_fine": "R"})

    def test_manual_prefix_kept(self):
        # _run_action 의 last_action 기록 제외([.] repeat/auto-rerun 이
        # 이동을 재발사하지 않는 조건)가 manual_ 접두에 걸려 있다.
        for name in MANUAL_CMDS:
            self.assertTrue(name.startswith("manual_"), name)

    def test_driving_params_exist(self):
        for name in ("fwd_speed", "step_mm", "turn_deg"):
            self.assertIn(name, PARAM_ORDER)


class TestRunnerGate(unittest.TestCase):

    def test_wasd_ignored_when_not_driving(self):
        runner = _make_runner()
        resp = runner.on_do("manual_fwd", {})
        self.assertEqual(resp.get("ignored"), "manual_fwd")
        self.assertIsNone(runner.manual_cmd)

    def test_wasd_queued_while_driving(self):
        runner = _make_runner()
        runner.manual_armed = True
        resp = runner.on_do("manual_left", {})
        self.assertEqual(resp.get("queued"), "manual_left")
        self.assertEqual(runner.manual_cmd, "manual_left")
        # 마지막 입력이 이긴다(한 칸 큐 — 루프가 하나씩 소비).
        runner.on_do("manual_fwd", {})
        self.assertEqual(runner.manual_cmd, "manual_fwd")

    def test_reset_still_queued_normally(self):
        runner = _make_runner()
        resp = runner.on_do("reset", {"source": "test"})
        self.assertEqual(resp.get("queued"), "reset")
        self.assertTrue(runner.reset_on)


class TestMarkerEvents(unittest.TestCase):
    """자동 판단 없음 — UI/그리퍼 이벤트만. straight 는 무동작 스텁."""

    def _marker(self, runner, color):
        runner.hw.color = color             # 확정 재판독도 같은 색
        runner.last_marker_t = -1e9         # 디바운스 통과
        return runner.handle_marker(color, "test")

    def test_red_phases(self):
        runner = _make_runner()
        self.assertTrue(self._marker(runner, COL_RED))
        self.assertEqual(runner.visits, 1)
        self.assertEqual(runner.out_red_spoken, 1)
        runner.goal_seen = True             # 초록 후에는 다시 1부터
        self.assertTrue(self._marker(runner, COL_RED))
        self.assertEqual(runner.return_red_spoken, 1)
        self.assertIn("red.wav", runner.hw.wavs)

    def test_green_delivers_once(self):
        runner = _make_runner()
        runner.timer_start = time.monotonic()
        self.assertTrue(self._marker(runner, COL_GREEN))
        self.assertTrue(runner.goal_seen)
        self.assertIsNotNone(runner.out_elapsed)
        self.assertIn(("open", 50), runner.hw.grip)     # 배달(그립 오픈)
        self.assertIn("good_job.wav", runner.hw.wavs)
        # 두 번째 초록은 무시(배달/시간 갱신 없음).
        out1 = runner.out_elapsed
        grips = len(runner.hw.grip)
        self.assertTrue(self._marker(runner, COL_GREEN))
        self.assertEqual(runner.out_elapsed, out1)
        self.assertEqual(len(runner.hw.grip), grips)

    def test_yellow_only_after_green(self):
        runner = _make_runner()
        runner.timer_start = time.monotonic()
        self.assertTrue(self._marker(runner, COL_YELLOW))
        self.assertFalse(runner.done)       # 초록 전 노랑은 무시
        runner.goal_seen = True
        runner.grabbed = True
        self.assertTrue(self._marker(runner, COL_YELLOW))
        self.assertTrue(runner.done)        # 완주
        self.assertIsNotNone(runner.return_elapsed)
        self.assertFalse(runner.grabbed)    # 그립 해제
        self.assertIn(("open", 50), runner.hw.grip)

    def test_unconfirmed_marker_rejected(self):
        # 첫 프레임은 초록이었지만 정지 재판독이 전부 흰색 — 다수결 미달 기각.
        runner = _make_runner()
        runner.hw.color = COL_WHITE
        runner.last_marker_t = -1e9
        self.assertFalse(runner.handle_marker(COL_GREEN, "test"))
        self.assertFalse(runner.goal_seen)

    def test_non_marker_color_false(self):
        runner = _make_runner()
        self.assertFalse(runner.handle_marker(COL_WHITE, "test"))


class TestDescribeKeyPassthrough(unittest.TestCase):

    def test_normalize_actions_keeps_key(self):
        server = TuningServer(None, None, port=0, actions=ACTIONS,
                              stage=STAGE_NAME)
        by_name = dict((a["name"], a) for a in server.actions)
        for name, key in MANUAL_KEYS.items():
            self.assertEqual(by_name[name].get("key"), key)
        self.assertNotIn("key", by_name["read_color"])


def _model(mode="manual_wait", age=0.5, session=None):
    describe = {"stage": STAGE_NAME, "params": [], "actions": ACTIONS}
    state = {"latest": {"running": True, "mode": mode}}
    return dashboard.build_model(state, describe, session,
                                 state_error="", describe_error="",
                                 state_age_s=age)


class TestDashboardWasdRouting(unittest.TestCase):
    """대시보드 수정 없이 wasd 가 로봇에 도달하는지 — manual_wait 게이트."""

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

    def test_manual_wait_routes_wasd_to_actions(self):
        # 특히 [s]=STOP, [a]=auto-rerun 내장 의미를 게이트가 이긴다.
        for name, char in sorted(MANUAL_KEYS.items()):
            self.sent[:] = []
            self._press(char, "manual_wait")
            self.assertEqual(self.sent,
                             [{"cmd": "do", "action": name, "args": {}}],
                             "key " + char)

    def test_builtins_restored_outside_manual_wait(self):
        # 출발 대기(waiting_start)/paused 에선 [s]=STOP 이 되살아난다 —
        # fxck3 의 비상 탈출구(pause 후 stop).
        for mode in ("waiting_start", "paused", "finished"):
            self.sent[:] = []
            self._press("s", mode)
            self.assertEqual(self.sent[0]["cmd"], "stop", mode)

    def test_stale_manual_wait_falls_back_to_builtin(self):
        self._press("s", "manual_wait", age=99.0)
        self.assertEqual(self.sent[0]["cmd"], "stop")

    def test_wasd_not_recorded_as_last_action(self):
        session = dashboard.DashboardSession()
        for name in sorted(MANUAL_CMDS):
            dashboard._run_action(name, session, "h", 1, 1.0)
            self.assertEqual(session.last_action, "", name)


if __name__ == "__main__":
    unittest.main()
