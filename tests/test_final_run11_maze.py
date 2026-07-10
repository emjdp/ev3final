#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_run11 판단층 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

픽스처 1: 실제 미로(return_revisit_all_minimal_route.html 의 NODES/EDGES/MARK
그대로, 화면좌표 → 절대방향 변환). 커브(차수 2 노드)가 많은 것이 특징 —
final_run8 의 복귀 실행(rel_move(실주행 heading, 계획 라벨))이 굽은 복도에서
깨지는 것을 이 픽스처가 재현했고, run9 의 노드-로컬 라벨 방식(§변경 C)이
고친 것을 검증한다.

검증 항목(미션 명세):
  - out 탐색(좌>우>직)이 빨강 6개를 모두 방문한 '후' 초록에 도착한다(명세 2).
  - 복귀는 빨강 6개를 모두 재방문하고, 초록은 재방문하지 않으며(명세 2 —
    도착 지점 재방문 불가), 폴백 없이 노랑(출발지)에서 끝난다.
  - 복귀 스텝 수 = 2×간선 − 트렁크 간선(최소거리 공식).

픽스처 2: 기존 직선 코스(test_run_maze_v4_logic.py) — 로컬 라벨 방식이
직선 복도에서 기존 결과와 동일함을 확인(회귀 방지).

추가: classify_rgb / PidSteer 엣지 팔로잉 / edge_exit_dir(§run10 부호 규약)의
순수 로직 테스트.

실행: python3 tests/test_final_run11_maze.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.final_run11 import (Explorer, PidSteer, classify_rgb, node_bits,
                                edge_exit_dir, turn_heading, INITIAL_PARAMS,
                                COL_BLACK, COL_GREEN, COL_YELLOW, COL_RED,
                                COL_WHITE, COL_NONE)

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

# --- 픽스처 2: 기존 직선 코스(test_run_maze_v4_logic.py 와 동일) ------------

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
    """실제 미로: 좌>우>직 탐색 + 최소거리 전빨강 재방문 복귀(HTML 시뮬 동치)."""

    @classmethod
    def setUpClass(cls):
        cls.ex, cls.heading, cls.order = simulate_out(
            MAZE_ADJ, MAZE_MARK, "S", "W")

    def test_all_reds_before_green(self):
        """명세 2: 모든 빨강(경유지) 방문 후 초록(도착) 도달."""
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
        from stages.final_run11 import shortest_path
        greens = [n for n in self.ex.route_mark
                  if self.ex.route_mark[n] == "green"]
        trunk = shortest_path(adj, greens[0], "home")
        self.assertEqual(len(self.ex.route), 2 * edges - (len(trunk) - 1))


class StraightCourseRegressionCase(unittest.TestCase):
    """직선 코스(기존 픽스처): 로컬 라벨 방식이 기존과 동일하게 동작."""

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


class ClassifyRgbCase(unittest.TestCase):
    """classify_rgb: 대표 RGB 값이 의도한 색으로 판정되는지(기본 파라미터)."""

    def cls(self, r, g, b):
        color, _bright = classify_rgb(r, g, b, INITIAL_PARAMS)
        return color

    def test_black_line(self):
        self.assertEqual(self.cls(40, 45, 35), COL_BLACK)

    def test_white_floor(self):
        # 흰 바닥은 b 채널이 약간 낮아도(전형 r/b 1.5~1.8) 노랑이 아니어야 한다.
        self.assertEqual(self.cls(250, 280, 160), COL_WHITE)

    def test_red_sticker(self):
        self.assertEqual(self.cls(260, 60, 45), COL_RED)

    def test_green_sticker(self):
        self.assertEqual(self.cls(70, 180, 90), COL_GREEN)

    def test_yellow_sticker(self):
        self.assertEqual(self.cls(300, 240, 60), COL_YELLOW)

    def test_gray_boundary_is_none(self):
        self.assertEqual(self.cls(90, 100, 80), COL_NONE)


class PidEdgeFollowCase(unittest.TestCase):
    """중앙 센서 엣지 팔로잉: error = steer_sign*(norm_c - edge_target)."""

    def error_of(self, nc, snap=None):
        pid = PidSteer()
        _l, _r, error, _turn, _trim = pid.step(nc, snap or INITIAL_PARAMS, 15)
        return error

    def test_on_edge_zero(self):
        # 중앙이 경계값(edge_target=50) 위 = 에러 0(조향 없음).
        self.assertAlmostEqual(self.error_of(50), 0.0)

    def test_on_white_positive(self):
        # 흰 바닥 쪽(밝음)으로 벗어남 → error > 0(steer_sign=+1 기본).
        self.assertGreater(self.error_of(85), 0.0)

    def test_on_black_negative(self):
        # 검은 라인 쪽(어두움)으로 벗어남 → error < 0.
        self.assertLess(self.error_of(10), 0.0)

    def test_steer_sign_flips(self):
        snap = dict(INITIAL_PARAMS)
        snap["steer_sign"] = -1
        self.assertLess(self.error_of(85, snap), 0.0)
        self.assertGreater(self.error_of(10, snap), 0.0)

    def test_uses_only_center(self):
        # 좌/우 값과 무관하게 중앙만으로 결정됨을 시그니처로 보장(인자 1개).
        self.assertAlmostEqual(self.error_of(50), 0.0)


class EdgeExitDirCase(unittest.TestCase):
    """edge_exit_dir(§run10): steer_sign ↔ 검정에서 나갈 피벗 방향의 부호 규약.

    PidSteer 와 acquire_edge 가 같은 엣지를 가리켜야 회전 후에도 조향이
    수렴한다 — 규약이 어긋나면 획득 직후 반대 엣지에서 폭주한다.
    """

    def test_plus_rides_right_edge(self):
        # +1: 밝으면 좌로 조향(라인이 왼쪽) = 오른쪽 엣지 → 검정에선 R 로 탈출.
        self.assertEqual(edge_exit_dir(1), "R")

    def test_minus_rides_left_edge(self):
        self.assertEqual(edge_exit_dir(-1), "L")


class NodeBitsCase(unittest.TestCase):
    def test_center_analog_threshold(self):
        snap = dict(INITIAL_PARAMS)
        self.assertEqual(node_bits(10, 10.0, 10, snap), (1, 1, 1))
        self.assertEqual(node_bits(80, 80.0, 80, snap), (0, 0, 0))
        self.assertEqual(node_bits(10, 80.0, 10, snap), (1, 0, 1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
