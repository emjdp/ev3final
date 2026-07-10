#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gg8(수동 결정 모드) 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

gg8 은 자율 판단(Explorer)이 없다 — 남은 순수 로직을 검증한다:
  - CommandBox: 1칸 큐(마지막 입력 승리) / take 소비 / clear / peek 비소비.
  - adaptive_base: 커브 자동 감속 + 출발 가속 램프(gg5 승계).
  - PidSteer 속도 비례 조향(gg5 승계): SPEED_REF 에선 배율 1.0.
  - node_bits / on_line 스모크(gg3 승계).
  - MOVE_ACTIONS/ACTIONS 정합: 이동 명령 4종이 액션 목록 맨 앞(핫키 1~4).

실행: python3 tests/test_gg8_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.gg8 import (CommandBox, PidSteer, Runner, adaptive_base,
                        node_bits, on_line, INITIAL_PARAMS, MOVE_ACTIONS,
                        ACTIONS, ACCEL_START_SPEED, CORNER_MIN_SPEED,
                        SPEED_REF, STEER_SCALE_MAX, PID_TURN_LIMIT,
                        COL_BLACK, COL_GREEN, COL_YELLOW, COL_RED, COL_WHITE,
                        COL_BROWN)
from lib.decision_log import DecisionLog
from lib.telemetry import Telemetry


class CommandBoxCase(unittest.TestCase):
    """대시보드 이동 명령 1칸 큐 — 마지막 입력 승리, take 는 소비."""

    def setUp(self):
        self.box = CommandBox()

    def test_empty_take(self):
        move, waited = self.box.take()
        self.assertIsNone(move)
        self.assertIsNone(waited)

    def test_set_take(self):
        self.box.set("L")
        move, waited = self.box.take()
        self.assertEqual(move, "L")
        self.assertGreaterEqual(waited, 0.0)
        self.assertIsNone(self.box.take()[0])   # 소비됐다

    def test_last_wins(self):
        """오입력 정정: 새 명령이 이전 명령을 덮는다."""
        self.assertIsNone(self.box.set("L"))
        self.assertEqual(self.box.set("U"), "L")    # 이전 값 반환
        self.assertEqual(self.box.take()[0], "U")

    def test_peek_does_not_consume(self):
        self.box.set("R")
        self.assertEqual(self.box.peek(), "R")
        self.assertEqual(self.box.peek(), "R")
        self.assertEqual(self.box.take()[0], "R")

    def test_clear(self):
        self.box.set("S")
        self.assertEqual(self.box.clear(), "S")
        self.assertIsNone(self.box.peek())
        self.assertIsNone(self.box.take()[0])


class OnDoDispatchCase(unittest.TestCase):
    """네트워크 스레드 경로(on_do) — 이동 명령/GO/clear 는 즉시 반영,
    모터 액션은 제어 루프 FIFO 로. hw 를 건드리지 않아 PC 에서 검증 가능."""

    def setUp(self):
        tele = Telemetry()
        self.runner = Runner(None, None, tele, DecisionLog(telemetry=tele))

    def test_move_action_sets_command_box(self):
        resp = self.runner.on_do("left", {})
        self.assertEqual(resp["queued_move"], "L")
        self.assertEqual(self.runner.cmd.peek(), "L")

    def test_move_action_last_wins(self):
        self.runner.on_do("left", {})
        resp = self.runner.on_do("uturn", {})
        self.assertEqual(resp["replaced"], "L")
        self.assertEqual(self.runner.cmd.peek(), "U")

    def test_clear_action(self):
        self.runner.on_do("right", {})
        resp = self.runner.on_do("clear", {})
        self.assertEqual(resp["cleared"], "R")
        self.assertIsNone(self.runner.cmd.peek())

    def test_go_action_sets_flag(self):
        self.assertFalse(self.runner.go_on)
        self.runner.on_do("go", {})
        self.assertTrue(self.runner.go_on)

    def test_reset_action_sets_flag(self):
        self.runner.on_do("reset", {"source": "test"})
        self.assertTrue(self.runner.reset_on)
        self.assertEqual(self.runner.reset_source, "test")

    def test_motor_actions_queue_fifo(self):
        """grip/nudge 는 CommandBox 가 아니라 제어 루프 FIFO 로 간다."""
        self.runner.on_do("grip_open", {})
        self.runner.on_do("nudge_fwd", {})
        self.assertEqual(self.runner._pending, ["grip_open", "nudge_fwd"])
        self.assertIsNone(self.runner.cmd.peek())


class ActionManifestCase(unittest.TestCase):
    """이동 명령 4종이 액션 목록 맨 앞 = 대시보드 핫키 [1]~[4] 보장."""

    def test_move_actions_first(self):
        names = [a["name"] for a in ACTIONS]
        self.assertEqual(names[:4], ["left", "straight", "right", "uturn"])
        self.assertEqual(names[4], "clear")

    def test_move_actions_map(self):
        self.assertEqual(MOVE_ACTIONS,
                         {"left": "L", "straight": "S",
                          "right": "R", "uturn": "U"})


class AdaptiveBaseCase(unittest.TestCase):
    """gg5 승계 — 커브 자동 감속 + 출발 가속 램프."""

    def test_straight_full_speed_after_ramp(self):
        self.assertEqual(adaptive_base(30, 0.5, 0.0, 700, 700), 30.0)

    def test_corner_slowdown_proportional(self):
        self.assertEqual(adaptive_base(30, 0.5, 20.0, 700, 9999), 20.0)

    def test_corner_slowdown_floor(self):
        self.assertEqual(adaptive_base(30, 0.5, 100.0, 700, 9999),
                         float(CORNER_MIN_SPEED))

    def test_ramp_starts_at_accel_start(self):
        self.assertEqual(adaptive_base(30, 0.5, 0.0, 700, 0),
                         float(ACCEL_START_SPEED))

    def test_ramp_midpoint_linear(self):
        self.assertAlmostEqual(adaptive_base(30, 0.5, 0.0, 700, 350),
                               (ACCEL_START_SPEED + 30) / 2.0)

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
