#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gg1 판단층 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

픽스처 1: 실제 미로(return_revisit_all_minimal_route.html 의 NODES/EDGES/MARK
그대로, 화면좌표 → 절대방향 변환). 커브(차수 2 노드)가 많은 것이 특징 —
복귀 실행이 실주행 heading 으로 rel_move 하면 굽은 복도에서 깨진다.
gg1 의 노드-로컬 라벨 방식(on_arrive_home + _arrival_facing)을 검증한다.

검증 항목:
  - out 탐색(좌>우>직)이 빨강 6개를 모두 방문한 '후' 초록에 도착한다.
  - 복귀는 빨강 6개를 모두 재방문하고, 초록은 재방문하지 않으며,
    폴백 없이 노랑(출발지)에서 끝난다.
  - 복귀 스텝 수 = 2×간선 − 트렁크 간선(최소거리 공식).

픽스처 2: 직선 코스 — 로컬 라벨 방식이 직선 복도에서도 동일하게 동작하는지
(회귀 방지) + 폴백이 절대 멈추지 않는지.

추가: node_bits / on_line / PidSteer 순수 로직 스모크 테스트.

실행: python3 tests/test_gg1_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.gg1 import (Explorer, PidSteer, node_bits, on_line, shortest_path,
                        turn_heading, INITIAL_PARAMS,
                        COL_BLACK, COL_GREEN, COL_YELLOW, COL_RED, COL_WHITE,
                        COL_BROWN)

# --- 픽스처 1: HTML 미로(화면좌표 y아래 → S=아래) ---------------------------

MAZE_ADJ = {
    "S":   {"W": "J14"},
    "J14": {"E": "S", "N": "J11"},
    "J11": {"S": "J14", "W": "J13", "N": "J9"},
    "J13": {"E": "J11", "S": "R6"},
    "R6":  {"N": "J13"},
    "J9":  {"S": "J11", "W": "J8", "E": "J6"},
    "J8":  {"E": "J9", "N": "R4"},
    "R4":  {"S": "J8"},
    "J6":  {"W": "J9", "N": "J4", "E": "J7", "S": "J12"},
    "J7":  {"W": "J6", "N": "R3"},
    "R3":  {"S": "J7"},
    "J12": {"N": "J6", "W": "R5"},
    "R5":  {"E": "J12"},
    "J4":  {"S": "J6", "W": "J3", "N": "J5"},
    "J3":  {"E": "J4", "N": "J2"},
    "J2":  {"S": "J3", "W": "J1"},
    "J1":  {"E": "J2", "W": "G", "S": "R1"},
    "G":   {"E": "J1"},
    "R1":  {"N": "J1"},
    "J5":  {"S": "J4", "E": "R2"},
    "R2":  {"W": "J5"},
}
MAZE_MARK = {"R1": "red", "R2": "red", "R3": "red", "R4": "red",
             "R5": "red", "R6": "red", "G": "green"}
RED_COUNT = 6

# --- 픽스처 2: 직선 코스 -----------------------------------------------------

COURSE_ADJ = {
    "home": {"N": 0},
    0: {"S": "home", "W": "r1", "N": 1},
    1: {"S": 0, "W": "r2", "E": "r3", "N": 2},
    2: {"S": 1, "W": "r4", "E": "r5", "N": 3},
    3: {"S": 2, "W": "r6", "E": "green"},
    "r1": {"E": 0},
    "r2": {"E": 1},
    "r3": {"W": 1},
    "r4": {"E": 2},
    "r5": {"W": 2},
    "r6": {"E": 3},
    "green": {"W": 3},
}
COURSE_MARK = {"r1": "red", "r2": "red", "r3": "red", "r4": "red",
               "r5": "red", "r6": "red", "green": "green"}


def _curve_exit(adj, pos, heading):
    """차수 2 노드(커브)의 유일한 출구 상대방향("L"/"R"/"S") 또는 None."""
    exits = [d for d in adj[pos] if d != turn_heading(heading, "U")]
    if len(exits) != 1:
        return None
    for m in ("L", "R", "S"):
        if turn_heading(heading, m) == exits[0]:
            return m
    return None


def simulate_out(adj, mark, start, heading, max_steps=500):
    """out 탐색 재현 — Runner 와 동일 순서. 커브는 Explorer 미개입 강제 이동.
    반환: (ex, heading, 마커 방문 순서)."""
    ex = Explorer()
    pos = start
    order = []
    for _ in range(max_steps):
        pos = adj[pos][heading]
        kind = mark.get(pos)
        if kind == "green":
            order.append(pos)
            heading = turn_heading(heading, "U")
            ex.apply_move("U")
            ex.start_home()
            return ex, heading, order
        if kind is not None or len(adj[pos]) == 1:
            order.append(pos)
            heading = turn_heading(heading, "U")
            ex.apply_move("U")
            ex.on_probe_end(kind or "dead_end")
            continue
        rel = _curve_exit(adj, pos, heading)
        if rel is not None:
            ex.apply_move(rel)
            heading = turn_heading(heading, rel)
            continue
        has_l = turn_heading(heading, "L") in adj[pos]
        has_r = turn_heading(heading, "R") in adj[pos]
        has_s = heading in adj[pos]
        move, _ev = ex.on_junction(has_l, has_r, has_s)
        ex.apply_move(move)
        heading = turn_heading(heading, move)
        if ex.mode == "HOME":
            return ex, heading, order
    raise AssertionError("out simulation did not finish")


def simulate_return(ex, adj, mark, pos, heading, home_pos, max_steps=500):
    """복귀 실행층 재현 — Runner 와 동일 순서(마커/막다른길/분기 도착마다
    on_arrive_home, 커브는 스텝 소비 없음). 반환: (빨강 재방문 리스트,
    초록 재방문 수, 최종 pos, 걸은 노드 리스트)."""
    reds = []
    green_revisits = 0
    walked = [pos]
    for _ in range(max_steps):
        pos = adj[pos][heading]
        walked.append(pos)
        if pos == home_pos:
            return reds, green_revisits, pos, walked
        kind = mark.get(pos, "junction")
        if kind == "red":
            reds.append(pos)
        if kind == "green":
            green_revisits += 1
        if kind == "junction":
            rel = _curve_exit(adj, pos, heading)
            if rel is not None:
                ex.apply_move(rel)
                heading = turn_heading(heading, rel)
                continue
            has_l = turn_heading(heading, "L") in adj[pos]
            has_r = turn_heading(heading, "R") in adj[pos]
            has_s = heading in adj[pos]
            move, _ev = ex.on_arrive_home("junction", has_l, has_r, has_s)
        else:
            move, _ev = ex.on_arrive_home(kind)
        ex.apply_move(move)
        heading = turn_heading(heading, move)
    raise AssertionError("return simulation did not finish")


class HtmlMazeCase(unittest.TestCase):
    """실제 미로: 좌>우>직 탐색 + 최소거리 전노드 재방문 복귀(HTML 시뮬 동치)."""

    @classmethod
    def setUpClass(cls):
        cls.ex, cls.heading, cls.order = simulate_out(
            MAZE_ADJ, MAZE_MARK, "S", "W")

    def test_all_reds_before_green(self):
        """모든 빨강(경유지) 방문 후 초록(도착) 도달."""
        self.assertEqual(self.order[-1], "G")
        self.assertEqual(sorted(self.order[:-1]),
                         ["R1", "R2", "R3", "R4", "R5", "R6"])
        self.assertEqual(self.ex.mode, "HOME")
        self.assertEqual(self.ex.home_red_total, RED_COUNT)
        self.assertFalse(self.ex.home_fallback)

    def test_return_revisits_all_reds_no_green(self):
        """복귀: 빨강 6개 전부 재방문, 초록 재방문 없음, 노랑에서 폴백 없이 종료."""
        ex, heading, _order = simulate_out(MAZE_ADJ, MAZE_MARK, "S", "W")
        reds, green_revisits, end, _walked = simulate_return(
            ex, MAZE_ADJ, MAZE_MARK, "G", heading, "S")
        self.assertEqual(end, "S")
        self.assertEqual(sorted(reds), ["R1", "R2", "R3", "R4", "R5", "R6"])
        self.assertEqual(green_revisits, 0)
        self.assertFalse(ex.home_fallback)
        self.assertEqual(ex.route_left(), 1)    # 남은 스텝 = home 진입뿐

    def test_route_step_count_minimal(self):
        """복귀 스텝 = 2×간선 − 트렁크 간선(최소거리 공식)."""
        adj = self.ex.route_adj
        edges = sum(len(adj[n]) for n in adj) // 2
        greens = [n for n in self.ex.route_mark
                  if self.ex.route_mark[n] == "green"]
        trunk = shortest_path(adj, greens[0], "home")
        self.assertEqual(len(self.ex.route), 2 * edges - (len(trunk) - 1))


class StraightCourseRegressionCase(unittest.TestCase):
    """직선 코스: 로컬 라벨 방식이 직선 복도에서 기존과 동일하게 동작."""

    def test_return_full(self):
        ex, heading, order = simulate_out(COURSE_ADJ, COURSE_MARK, "home", "N")
        self.assertEqual(order[-1], "green")
        reds, green_revisits, end, _walked = simulate_return(
            ex, COURSE_ADJ, COURSE_MARK, "green", heading, "home")
        self.assertEqual(end, "home")
        self.assertEqual(len(reds), RED_COUNT)
        self.assertEqual(green_revisits, 0)
        self.assertFalse(ex.home_fallback)
        self.assertEqual(ex.route_left(), 1)

    def test_fallback_never_stops(self):
        ex, _heading, _order = simulate_out(COURSE_ADJ, COURSE_MARK,
                                            "home", "N")
        move, events = ex.on_arrive_home("dead_end", True, False, True)
        self.assertTrue(ex.home_fallback)
        self.assertIn(move, ("L", "R", "S", "U"))
        self.assertIn("RETURN_FALLBACK", [e[0] for e in events])
        move2, _ev2 = ex.on_arrive_home("junction", False, False, False)
        self.assertEqual(move2, "U")


class OnLineCase(unittest.TestCase):
    """중앙 '라인 위' 판정 — 흰색만 아니면 라인(경계 오분류 흡수)."""

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


class PidSteerCase(unittest.TestCase):
    """조향 PID — error = 정규화 우반사광 - 정규화 좌반사광."""

    def setUp(self):
        self.snap = dict(INITIAL_PARAMS)
        self.pid = PidSteer()

    def test_centered_no_turn(self):
        left, right, error, turn, _trim = self.pid.step(50.0, 50.0,
                                                        self.snap, 20)
        self.assertEqual(error, 0.0)
        self.assertEqual(turn, 0.0)
        self.assertEqual(left, right)

    def test_offset_steers_back(self):
        # 우측이 밝다(라인이 왼쪽) → error > 0 → 좌측 감속/우측 가속.
        left, right, error, turn, _trim = self.pid.step(20.0, 80.0,
                                                        self.snap, 20)
        self.assertGreater(error, 0.0)
        self.assertGreater(turn, 0.0)
        self.assertLess(left, right)

    def test_deadband_suppresses_small_error(self):
        _l, _r, error, turn, _trim = self.pid.step(50.0, 52.0, self.snap, 20)
        self.assertGreater(error, 0.0)
        self.assertEqual(turn, 0.0)


if __name__ == "__main__":
    unittest.main()
