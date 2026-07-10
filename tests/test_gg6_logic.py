#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gg6 판단층 단위 테스트 — PC 단독 실행(ev3dev2 불필요)."""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.gg6 import INITIAL_PARAMS, should_escalate


class NodeGuardEscalationCase(unittest.TestCase):
    """NODE_GUARD deep-drop 승격 판단 회귀 테스트."""

    def setUp(self):
        self.snap = dict(INITIAL_PARAMS)

    def test_right_deep_drop_during_guard_escalates(self):
        self.assertTrue(should_escalate((0, 0, 1), 72, 10, True, self.snap))

    def test_left_deep_drop_during_guard_escalates(self):
        self.assertTrue(should_escalate((1, 0, 0), 9, 61, True, self.snap))

    def test_deep_drop_without_guard_does_not_escalate(self):
        self.assertFalse(should_escalate((0, 0, 1), 72, 10, False, self.snap))

    def test_margin_only_does_not_escalate(self):
        self.assertFalse(should_escalate((0, 1, 0), 41, 61, True, self.snap))

    def test_lost_bits_does_not_escalate(self):
        self.assertFalse(should_escalate((0, 0, 0), 73, 36, True, self.snap))


if __name__ == "__main__":
    unittest.main()
