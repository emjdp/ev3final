#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_run8 중앙 '라인 위' 판정 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

§변경 E: 중앙 컬러센서가 '검정이어야' 라인이 아니라 '흰색만 아니면' 라인이다.

근거(실측 follow 3919 샘플): 중앙색은 BLACK 92.4% / WHITE 4.6% / BROWN 2.0% /
BLUE 0.9% / NONE 0.03%. BROWN·BLUE·NONE 은 EV3 컬러센서가 검정↔흰색 경계나
포화 검정에서 내는 오분류이지 흰 바닥이 아니다. 그런데 bits 000(=LOST_BITS)로
읽힌 프레임 221개 중 90개(41%)가 이 BROWN/BLUE 라서, 라인 위인데 유실로
판정해 후진하고 다른 길을 찾아버렸다.

실행: python3 tests/test_final_run8_line.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stages.final_run8 import (node_bits, on_line, LOST_BITS, NODE_CANDIDATES,
                               INITIAL_PARAMS, COL_NONE, COL_BLACK, COL_GREEN,
                               COL_YELLOW, COL_RED, COL_WHITE, COL_BROWN)

WHITE_REFLECT = 77      # 좌/우 반사광 일반주행 실측 중앙값
BLACK_REFLECT = 9       # 가로선 위 실측(8~10)


class OnLineCase(unittest.TestCase):
    """on_line(color) = '흰 바닥이 아니다'."""

    def test_white_is_off_line(self):
        self.assertFalse(on_line(COL_WHITE))

    def test_black_is_on_line(self):
        self.assertTrue(on_line(COL_BLACK))

    def test_boundary_misclassifications_are_on_line(self):
        # 경계/포화에서 나오는 오분류 — 이걸 유실로 보면 후진해 버린다.
        for color in (COL_BROWN, COL_NONE):
            self.assertTrue(on_line(color), "color=%d 가 라인 밖으로 읽힘" % color)

    def test_markers_are_on_line(self):
        # 스티커는 경로 위에 있다. 마커 디바운스로 handle_marker 가 넘긴
        # 프레임이 유실로 빠지면 안 된다.
        for color in (COL_RED, COL_YELLOW, COL_GREEN):
            self.assertTrue(on_line(color))


class NodeBitsCenterCase(unittest.TestCase):
    """중앙 bit 가 on_line() 을 따르는지 + 유실/노드 판정이 살아 있는지."""

    def bits(self, center_color, left=WHITE_REFLECT, right=WHITE_REFLECT):
        return node_bits(left, center_color, right, dict(INITIAL_PARAMS))

    def test_boundary_color_no_longer_reads_lost(self):
        # 좌/우 흰색 + 중앙이 경계 오분류 = 정상 주행이지 유실이 아니다.
        for color in (COL_BROWN, COL_NONE):
            self.assertNotEqual(self.bits(color), LOST_BITS,
                                "color=%d 가 유실로 읽힘" % color)

    def test_real_loss_still_detected(self):
        # 좌/우 흰색 + 중앙 흰색 = 진짜 유실. 이건 반드시 잡혀야 한다.
        self.assertEqual(self.bits(COL_WHITE), LOST_BITS)

    def test_normal_following_is_not_a_node(self):
        for color in (COL_BLACK, COL_BROWN, COL_NONE):
            self.assertNotIn(self.bits(color), NODE_CANDIDATES)

    def test_branches_still_detected(self):
        B, W = BLACK_REFLECT, WHITE_REFLECT
        cases = ((B, COL_BLACK, W),      # 좌분기 110
                 (W, COL_BLACK, B),      # 우분기 011
                 (B, COL_BLACK, B),      # 십자   111
                 (B, COL_WHITE, B))      # T(직진 없음) 101
        for left, color, right in cases:
            self.assertIn(node_bits(left, color, right, dict(INITIAL_PARAMS)),
                          NODE_CANDIDATES)

    def test_t_junction_needs_white_center(self):
        # 101(직진 없음)은 중앙이 '흰 바닥'일 때만 — 경계 오분류면 십자(111)다.
        B = BLACK_REFLECT
        snap = dict(INITIAL_PARAMS)
        self.assertEqual(node_bits(B, COL_WHITE, B, snap), (1, 0, 1))
        self.assertEqual(node_bits(B, COL_BROWN, B, snap), (1, 1, 1))


class HasStraightCase(unittest.TestCase):
    """handle_node 의 has_straight = on_line(color) — 직진로 유무."""

    def test_straight_present_on_line_colors(self):
        for color in (COL_BLACK, COL_BROWN, COL_NONE):
            self.assertTrue(on_line(color))

    def test_no_straight_on_white(self):
        self.assertFalse(on_line(COL_WHITE))


if __name__ == "__main__":
    unittest.main(verbosity=2)
