#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aplus(원격 조종 결정 모드) 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

aplus 는 자율 판단이 없다 — 남은 순수 로직을 검증한다:
  - CommandQueue: FIFO 다칸 큐(연속 입력) / 가득 차면 거부 / clear / peek 비소비.
  - on_do 디스패치: 이동 명령은 큐, GO/clear/reset 은 즉시, 모터 액션은 FIFO.
  - MOVE_ACTIONS/ACTIONS 정합: w/s/a/d/q/e/p 키 규약(사용자 지정 키맵).
  - confirm/slow 속도: base_speed 비례 + 하한(fxck2 승계) — base 15 기준.
  - adaptive_base / PidSteer 속도 비례 조향 / node_bits / on_line(gg5 승계).
  - 조종 패드(tools/aplus_pad.py) 키맵이 로봇 manifest 와 1:1 인지.

실행: python3 tests/test_aplus_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.aplus import (CommandQueue, PidSteer, Runner, adaptive_base,
                          confirm_speed, slow_speed, node_bits, on_line,
                          corner_min_speed, accel_start_speed,
                          INITIAL_PARAMS, MOVE_ACTIONS, DIAG_MOVES, ACTIONS,
                          SPEED_REF, STEER_SCALE_MAX, PID_TURN_LIMIT,
                          COL_BLACK, COL_GREEN, COL_YELLOW, COL_RED,
                          COL_WHITE, COL_BROWN)
from lib.decision_log import DecisionLog
from lib.telemetry import Telemetry


class CommandQueueCase(unittest.TestCase):
    """FIFO 다칸 큐 — 연속 입력이 순서대로 소비된다."""

    def setUp(self):
        self.q = CommandQueue()

    def test_empty_take(self):
        move, waited = self.q.take()
        self.assertIsNone(move)
        self.assertIsNone(waited)

    def test_fifo_order(self):
        for token in ("DL", "DL", "S"):
            accepted, _snap = self.q.push(token)
            self.assertTrue(accepted)
        self.assertEqual(self.q.take()[0], "DL")
        self.assertEqual(self.q.take()[0], "DL")
        self.assertEqual(self.q.take()[0], "S")
        self.assertIsNone(self.q.take()[0])

    def test_push_returns_snapshot(self):
        self.q.push("L")
        accepted, snap = self.q.push("U")
        self.assertTrue(accepted)
        self.assertEqual(snap, ["L", "U"])

    def test_full_rejects_new(self):
        for _i in range(CommandQueue.MAXLEN):
            accepted, _snap = self.q.push("S")
            self.assertTrue(accepted)
        accepted, snap = self.q.push("L")
        self.assertFalse(accepted)
        self.assertEqual(len(snap), CommandQueue.MAXLEN)
        self.assertNotIn("L", snap)

    def test_peek_all_does_not_consume(self):
        self.q.push("R")
        self.q.push("S")
        self.assertEqual(self.q.peek_all(), ["R", "S"])
        self.assertEqual(self.q.peek_all(), ["R", "S"])
        self.assertEqual(self.q.take()[0], "R")

    def test_clear_returns_all(self):
        self.q.push("S")
        self.q.push("U")
        self.assertEqual(self.q.clear(), ["S", "U"])
        self.assertEqual(self.q.peek_all(), [])
        self.assertIsNone(self.q.take()[0])

    def test_take_reports_wait_time(self):
        self.q.push("S")
        _move, waited = self.q.take()
        self.assertGreaterEqual(waited, 0.0)


class OnDoDispatchCase(unittest.TestCase):
    """네트워크 스레드 경로(on_do) — 이동 명령은 큐, GO/clear/reset 은 즉시,
    모터 액션은 제어 루프 FIFO. hw 를 건드리지 않아 PC 에서 검증 가능."""

    def setUp(self):
        tele = Telemetry()
        self.runner = Runner(None, None, tele, DecisionLog(telemetry=tele))

    def test_move_action_queues_fifo(self):
        self.assertEqual(self.runner.on_do("fwd", {})["queued_move"], "S")
        self.assertEqual(self.runner.on_do("left", {})["queued_move"], "L")
        self.assertEqual(self.runner.cmd.peek_all(), ["S", "L"])

    def test_diag_actions_queue_tokens(self):
        self.runner.on_do("diag_left", {})
        self.runner.on_do("diag_right", {})
        self.assertEqual(self.runner.cmd.peek_all(), ["DL", "DR"])

    def test_queue_full_rejected(self):
        for _i in range(CommandQueue.MAXLEN):
            self.runner.on_do("fwd", {})
        resp = self.runner.on_do("uturn", {})
        self.assertEqual(resp.get("rejected"), "U")
        self.assertEqual(resp.get("reason"), "queue_full")

    def test_clear_action(self):
        self.runner.on_do("right", {})
        self.runner.on_do("uturn", {})
        resp = self.runner.on_do("clear", {})
        self.assertEqual(resp["cleared"], ["R", "U"])
        self.assertEqual(self.runner.cmd.peek_all(), [])

    def test_go_action_sets_flag(self):
        self.assertFalse(self.runner.go_on)
        self.runner.on_do("go", {})
        self.assertTrue(self.runner.go_on)

    def test_reset_action_sets_flag(self):
        self.runner.on_do("reset", {"source": "test"})
        self.assertTrue(self.runner.reset_on)
        self.assertEqual(self.runner.reset_source, "test")

    def test_motor_actions_queue_pending(self):
        """grip/calibrate 는 CommandQueue 가 아니라 제어 루프 FIFO 로 간다."""
        self.runner.on_do("grip_close", {})
        self.runner.on_do("grip_open", {})
        self.runner.on_do("calibrate", {})
        self.assertEqual(self.runner._pending,
                         ["grip_close", "grip_open", "calibrate"])
        self.assertEqual(self.runner.cmd.peek_all(), [])


class ActionManifestCase(unittest.TestCase):
    """사용자 지정 키맵 규약 — w/s/a/d/q/e/p, 전부 유일."""

    def test_user_specified_keys(self):
        keys = dict((a["name"], a.get("key")) for a in ACTIONS)
        self.assertEqual(keys["fwd"], "w")
        self.assertEqual(keys["uturn"], "s")
        self.assertEqual(keys["left"], "a")
        self.assertEqual(keys["right"], "d")
        self.assertEqual(keys["diag_left"], "q")
        self.assertEqual(keys["diag_right"], "e")
        self.assertEqual(keys["grip_close"], "p")

    def test_all_actions_have_unique_keys(self):
        keys = [a.get("key") for a in ACTIONS]
        self.assertTrue(all(keys))
        self.assertEqual(len(keys), len(set(keys)))

    def test_move_actions_map(self):
        self.assertEqual(MOVE_ACTIONS,
                         {"fwd": "S", "left": "L", "right": "R", "uturn": "U",
                          "diag_left": "DL", "diag_right": "DR"})
        self.assertEqual(DIAG_MOVES, ("DL", "DR"))

    def test_move_actions_all_in_manifest(self):
        names = set(a["name"] for a in ACTIONS)
        for action in MOVE_ACTIONS:
            self.assertIn(action, names)


class PadKeymapCase(unittest.TestCase):
    """조종 패드(tools/aplus_pad.py) 키맵이 로봇 manifest 키와 1:1."""

    def test_pad_keys_match_robot_manifest(self):
        from tools.aplus_pad import KEY_ACTIONS
        manifest = dict((a["name"], a["key"]) for a in ACTIONS
                        if a["name"] != "reset")    # reset 은 패드 [r](확인)
        self.assertEqual(dict((v, k) for k, v in KEY_ACTIONS.items()),
                         manifest)


class SpeedHelpersCase(unittest.TestCase):
    """confirm/slow 는 base_speed 비례 + 하한(fxck2 승계)."""

    def test_default_base_is_15(self):
        self.assertEqual(INITIAL_PARAMS["base_speed"], 15)

    def test_curve_auto_default_on(self):
        # 커브는 기본 자동 통과(fxck2) — 0 으로 내리면 gg8 식 정지 대기.
        self.assertEqual(INITIAL_PARAMS["curve_auto"], 1)

    def test_confirm_floor_at_base_15(self):
        self.assertEqual(confirm_speed(15), 5.0)    # 15×0.25=3.75 → 하한 5

    def test_slow_floor_at_base_15(self):
        self.assertEqual(slow_speed(15), 8.0)       # 15×0.40=6.0 → 하한 8

    def test_proportional_above_floor(self):
        self.assertEqual(confirm_speed(40), 10.0)
        self.assertEqual(slow_speed(40), 16.0)


class AdaptiveBaseCase(unittest.TestCase):
    """커브 자동 감속 + 출발 가속 램프 — 하한/시작이 base 비례라 base 15
    에서도 체감된다(gg5 절대값 12 는 base 15 에서 여유 3%뿐이었다)."""

    def test_straight_full_speed_after_ramp(self):
        self.assertEqual(adaptive_base(15, 0.5, 0.0, 700, 700), 15.0)

    def test_corner_floor_proportional_at_base_15(self):
        # 15×0.6=9 — base 15 에서도 커브 감속 폭(15→9)이 살아 있다.
        self.assertEqual(corner_min_speed(15), 9.0)
        self.assertEqual(adaptive_base(15, 0.5, 100.0, 700, 9999), 9.0)

    def test_corner_floor_reproduces_gg5_at_ref_speed(self):
        self.assertEqual(corner_min_speed(20), 12.0)    # gg5 절대값 12 재현

    def test_corner_floor_has_absolute_min(self):
        self.assertEqual(corner_min_speed(10), 8.0)     # 10×0.6=6 → 하한 8
        self.assertEqual(corner_min_speed(7), 7.0)      # 하한이 target 초과 금지

    def test_ramp_starts_proportional(self):
        # 재출발도 base 비례로 천천히 — 15 는 9 부터, 30 은 18 부터.
        self.assertEqual(accel_start_speed(15), 9.0)
        self.assertEqual(adaptive_base(15, 0.5, 0.0, 700, 0), 9.0)
        self.assertEqual(adaptive_base(30, 0.5, 0.0, 700, 0), 18.0)

    def test_ramp_disabled_when_zero(self):
        self.assertEqual(adaptive_base(30, 0.5, 0.0, 0, 0), 30.0)


class SpeedScaledSteerCase(unittest.TestCase):
    """gg5 승계 — 속도 비례 조향."""

    def setUp(self):
        self.snap = dict(INITIAL_PARAMS)

    def _turn_at(self, base):
        pid = PidSteer()
        _l, _r, _e, turn, _t = pid.step(20.0, 80.0, self.snap, base)
        return turn

    def test_ref_speed_unscaled(self):
        expected = self.snap["kp"] * (60.0 - self.snap["deadband"])
        self.assertAlmostEqual(self._turn_at(SPEED_REF), expected)

    def test_turn_scales_with_speed(self):
        self.assertAlmostEqual(self._turn_at(30), self._turn_at(20) * 1.5)

    def test_scale_clamped_at_max(self):
        self.assertAlmostEqual(self._turn_at(60),
                               self._turn_at(20) * STEER_SCALE_MAX)

    def test_turn_limit_scales(self):
        snap = dict(self.snap)
        snap["kp"] = 3.0
        pid = PidSteer()
        _l, _r, _e, turn, _t = pid.step(0.0, 100.0, snap, 30)
        self.assertAlmostEqual(abs(turn), PID_TURN_LIMIT * 1.5)


class OnLineCase(unittest.TestCase):
    """중앙 '라인 위' 판정 — 흰색만 아니면 라인(gg3 실측 승계)."""

    def test_black_is_line(self):
        self.assertTrue(on_line(COL_BLACK))

    def test_white_is_off(self):
        self.assertFalse(on_line(COL_WHITE))

    def test_boundary_misreads_are_line(self):
        self.assertTrue(on_line(COL_BROWN))

    def test_markers_are_line(self):
        for c in (COL_RED, COL_GREEN, COL_YELLOW):
            self.assertTrue(on_line(c))


class NodeBitsCase(unittest.TestCase):
    """bits(좌,중,우) — 좌/우는 원시 반사광 임계, 중앙은 on_line."""

    def setUp(self):
        self.snap = dict(INITIAL_PARAMS)

    def test_follow(self):
        self.assertEqual(node_bits(70, COL_BLACK, 70, self.snap), (0, 1, 0))

    def test_cross(self):
        self.assertEqual(node_bits(10, COL_BLACK, 8, self.snap), (1, 1, 1))

    def test_t_without_straight(self):
        self.assertEqual(node_bits(10, COL_WHITE, 8, self.snap), (1, 0, 1))

    def test_all_white_is_lost(self):
        self.assertEqual(node_bits(80, COL_WHITE, 80, self.snap), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
