#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manual_run1(교차로 수동 조향) 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

검증 항목:
  - CommandBox: 단일 슬롯 — 최신 명령이 이전 것을 덮고, take 는 꺼내며 비운다.
  - on_do 네트워크 핸들러: 방향 액션 4개가 올바른 move 로 큐잉되고,
    go/reset 플래그가 서고, 그 외 액션은 pending 으로 넘어간다.
    (Runner 생성은 hw 없이 가능 — hw 는 주행 메서드에서만 쓴다.)
  - ACTIONS 매니페스트: 방향 4개가 맨 앞(대시보드 키 [1]~[4] 고정).
  - node_bits / on_line / normalize 스모크(final_run8 동일 로직 회귀).

실행: python3 tests/test_manual_run1_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.manual_run1 import (ACTIONS, CommandBox, INITIAL_PARAMS,
                                MOVE_ACTIONS, MOVE_BY_ACTION, Runner,
                                COL_WHITE, node_bits, normalize, on_line)
from lib.shared_params import SharedParams
from lib.telemetry import Telemetry
from lib.decision_log import DecisionLog
from stages.manual_run1 import PARAM_LIMITS, MAX_STEP


COL_BLACK = 1
COL_RED = 5
COL_BROWN = 7


def make_runner():
    params = SharedParams(dict(INITIAL_PARAMS), dict(PARAM_LIMITS),
                          dict(MAX_STEP), os.devnull)
    tele = Telemetry()
    log = DecisionLog(telemetry=tele)
    return Runner(None, params, tele, log)   # hw 는 주행 메서드에서만 사용


class TestCommandBox(unittest.TestCase):

    def test_take_empties_slot(self):
        box = CommandBox()
        self.assertIsNone(box.take())
        box.put("L")
        self.assertEqual(box.peek(), "L")
        self.assertEqual(box.take(), "L")
        self.assertIsNone(box.take())

    def test_latest_command_overwrites(self):
        box = CommandBox()
        self.assertIsNone(box.put("L"))
        self.assertEqual(box.put("R"), "L")   # 이전 명령을 돌려주며 덮는다
        self.assertEqual(box.take(), "R")

    def test_clear(self):
        box = CommandBox()
        box.put("U")
        box.clear()
        self.assertIsNone(box.peek())


class TestOnDo(unittest.TestCase):

    def test_move_actions_queue_moves(self):
        runner = make_runner()
        for action, move in MOVE_ACTIONS:
            resp = runner.on_do(action, {})
            self.assertEqual(resp["move"], move)
            self.assertEqual(runner.cmd.take(), move)

    def test_latest_move_wins(self):
        runner = make_runner()
        runner.on_do("left", {})
        runner.on_do("uturn", {})
        self.assertEqual(runner.cmd.take(), "U")
        self.assertIsNone(runner.cmd.take())

    def test_go_and_reset_set_flags(self):
        runner = make_runner()
        self.assertFalse(runner.go_on)
        runner.on_do("go", {})
        self.assertTrue(runner.go_on)
        self.assertFalse(runner.reset_on)
        runner.on_do("reset", {"source": "test"})
        self.assertTrue(runner.reset_on)
        self.assertEqual(runner.reset_source, "test")

    def test_other_action_goes_pending(self):
        runner = make_runner()
        runner.on_do("calibrate", {})
        self.assertEqual(runner._pending, "calibrate")
        self.assertIsNone(runner.cmd.peek())


class TestActionManifest(unittest.TestCase):

    def test_move_actions_first_for_dashboard_keys(self):
        # 대시보드는 describe 순서대로 [1]~ 키를 배정한다 — 방향 4개 고정.
        names = [a["name"] for a in ACTIONS]
        self.assertEqual(names[:4], ["left", "straight", "right", "uturn"])
        for action, _move in MOVE_ACTIONS:
            self.assertIn(action, names)

    def test_move_mapping_complete(self):
        self.assertEqual(sorted(MOVE_BY_ACTION.values()), ["L", "R", "S", "U"])


class TestPureHelpers(unittest.TestCase):

    def test_node_bits_thresholds(self):
        snap = dict(INITIAL_PARAMS)           # left 35 / right 30
        self.assertEqual(node_bits(10, COL_BLACK, 10, snap), (1, 1, 1))
        self.assertEqual(node_bits(70, COL_BLACK, 70, snap), (0, 1, 0))
        self.assertEqual(node_bits(70, COL_WHITE, 70, snap), (0, 0, 0))
        self.assertEqual(node_bits(10, COL_WHITE, 70, snap), (1, 0, 0))

    def test_on_line_white_only_off(self):
        self.assertFalse(on_line(COL_WHITE))
        self.assertTrue(on_line(COL_BLACK))
        self.assertTrue(on_line(COL_BROWN))   # 경계 오분류도 라인(§E)
        self.assertTrue(on_line(COL_RED))     # 마커 스티커도 라인 위

    def test_normalize_identity_when_uncalibrated(self):
        self.assertEqual(normalize(37, 0, 100), 37.0)
        # span < CAL_MIN_SPAN 이면 원시값 유지(폭주 방지)
        self.assertEqual(normalize(37, 40, 50), 37.0)
        # 정상 캘리브레이션이면 선형 매핑 + 클램프
        self.assertEqual(normalize(30, 20, 80), 100.0 * 10 / 60)
        self.assertEqual(normalize(10, 20, 80), 0.0)
        self.assertEqual(normalize(90, 20, 80), 100.0)


if __name__ == "__main__":
    unittest.main()
