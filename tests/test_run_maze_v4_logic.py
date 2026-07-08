#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_run3 복귀(home) 판단층 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

코스 픽스처(빨강 6 + 초록 1). 절대방향 그리드 트리:

    home --N-- n0 --N-- n1 --N-- n2 --N-- n3 --E-- green
               |W       |W  |E   |W  |E   |W
               r1        r2  r3   r4  r5   r6

분기 정리형(좌>우>직, 보류 LIFO) out 탐색을 그대로 시뮬레이션하면
r1→r2→(n2 발견·보류)→r3→r4→(n3 발견·보류)→r5→r6→green 순서가 되어
초록 도달 시점에 빨강 6개가 모두 지도에 있다(초록 직후 탐색 중단 사양).

실행: python3 tests/test_run_maze_v4_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.final_run3 import (Explorer, build_return_route, shortest_path,
                               explorer_to_graph, turn_heading)

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
RED_COUNT = 6


def simulate_out(adj_true, mark_true, max_steps=500):
    """out 탐색을 Runner 없이 재현(순수). Explorer 판단대로 코스 그래프를
    걷다가 초록에서 배달 유턴까지 적용해 start_home() 호출, 또는 초록이
    없으면 EXPLORE_DONE 으로 HOME 전환될 때까지 진행.

    Runner 와 동일한 순서: 마커/막다른길 = turn("U")(apply_move) 후
    on_probe_end / 분기점 = on_junction 이 준 move 를 apply_move.
    반환: (ex, pos, heading, events) — pos/heading 은 HOME 전환 시점 값.
    """
    ex = Explorer()
    pos = "home"
    heading = "N"
    all_events = []
    for _ in range(max_steps):
        pos = adj_true[pos][heading]            # 한 간선 전진
        kind = mark_true.get(pos)
        if kind == "green":
            heading = turn_heading(heading, "U")    # 배달 시퀀스의 유턴
            ex.apply_move("U")
            all_events += ex.start_home()
            return ex, pos, heading, all_events
        if kind is not None or len(adj_true[pos]) == 1:
            heading = turn_heading(heading, "U")    # 마커/막다른길 유턴
            ex.apply_move("U")
            all_events += ex.on_probe_end(kind or "dead_end")
            continue
        has_l = turn_heading(heading, "L") in adj_true[pos]
        has_r = turn_heading(heading, "R") in adj_true[pos]
        has_s = heading in adj_true[pos]
        move, events = ex.on_junction(has_l, has_r, has_s)
        all_events += events
        ex.apply_move(move)
        heading = turn_heading(heading, move)
        if ex.mode == "HOME":                   # EXPLORE_DONE(초록 없는 코스)
            return ex, pos, heading, all_events
    raise AssertionError("out simulation did not finish")


def walk_route(adj, start, route):
    """route 를 그래프 위에서 걸어 (최종 위치, 방문 노드 리스트, 간선별
    통과 횟수 dict)를 반환. 스텝이 실제 간선과 안 맞으면 AssertionError."""
    pos = start
    visited = [pos]
    edge_count = {}
    for step_dir, dest in route:
        if step_dir not in adj[pos]:
            raise AssertionError(
                "no edge {} from {}".format(step_dir, pos))
        nxt = adj[pos][step_dir]
        if nxt != dest:
            raise AssertionError(
                "step ({}, {}) from {} lands on {}".format(
                    step_dir, dest, pos, nxt))
        key = tuple(sorted((str(pos), str(dest))))
        edge_count[key] = edge_count.get(key, 0) + 1
        pos = nxt
        visited.append(pos)
    return pos, visited, edge_count


def edge_key(a, b):
    return tuple(sorted((str(a), str(b))))


def count_edges(adj):
    total = 0
    for n in adj:
        total += len(adj[n])
    return total // 2


def simulate_return(ex, adj, mark, pos, heading, max_steps=500):
    """복귀 실행층을 재현: 간선 전진 → 도착 종류별 on_arrive_home →
    move 적용. 노랑(home) 도달 시 (재방문 빨강 수, 최종 pos) 반환."""
    reds = 0
    for _ in range(max_steps):
        pos = adj[pos][heading]
        if pos == "home":
            return reds, pos
        kind = mark.get(pos, "junction")
        if kind == "red":
            reds += 1
        if kind == "junction":
            has_l = turn_heading(heading, "L") in adj[pos]
            has_r = turn_heading(heading, "R") in adj[pos]
            has_s = heading in adj[pos]
            move, _ev = ex.on_arrive_home("junction", has_l, has_r, has_s)
        else:
            move, _ev = ex.on_arrive_home(kind)
        ex.apply_move(move)
        heading = turn_heading(heading, move)
    raise AssertionError("return simulation did not finish")


class GreenCourseCase(unittest.TestCase):
    """빨강 6 + 초록 1 코스: 초록 직후 start_home 이 세운 계획 검증."""

    @classmethod
    def setUpClass(cls):
        cls.ex, cls.pos, cls.heading, cls.events = simulate_out(
            COURSE_ADJ, COURSE_MARK)
        cls.adj, cls.mark = explorer_to_graph(cls.ex.nodes, cls.ex.arm_ends)
        greens = [n for n in cls.mark if cls.mark[n] == "green"]
        assert len(greens) == 1
        cls.start = greens[0]
        cls.route = cls.ex.route
        cls.trunk = shortest_path(cls.adj, cls.start, "home")

    def red_leaves(self):
        return set(n for n in self.mark if self.mark[n] == "red")

    def test_map_complete_at_green(self):
        """초록 도달 시점에 빨강 6개가 전부 지도에 있어야 한다(코스 전제)."""
        self.assertEqual(self.ex.mode, "HOME")
        self.assertEqual(len(self.red_leaves()), RED_COUNT)
        self.assertEqual(self.ex.home_red_total, RED_COUNT)

    def test_a_route_revisits_all_reds(self):
        end, visited, _counts = walk_route(self.adj, self.start, self.route)
        self.assertEqual(end, "home")
        self.assertTrue(self.red_leaves().issubset(set(visited)))

    def test_b_trunk_once_branches_twice(self):
        _end, _visited, counts = walk_route(self.adj, self.start, self.route)
        trunk_edges = set(edge_key(self.trunk[i], self.trunk[i + 1])
                          for i in range(len(self.trunk) - 1))
        all_edges = set()
        for n in self.adj:
            for d in self.adj[n]:
                all_edges.add(edge_key(n, self.adj[n][d]))
        for edge in all_edges:
            expected = 1 if edge in trunk_edges else 2
            self.assertEqual(counts.get(edge, 0), expected,
                             "edge {} traversed {} times, expected {}".format(
                                 edge, counts.get(edge, 0), expected))

    def test_c_priority_swap_same_coverage(self):
        route_rls = build_return_route(self.adj, self.mark, self.start,
                                       "home", self.heading,
                                       priority=("R", "L", "S"))
        end, visited, counts = walk_route(self.adj, self.start, route_rls)
        self.assertEqual(end, "home")
        self.assertTrue(self.red_leaves().issubset(set(visited)))
        _e, _v, counts_lrs = walk_route(self.adj, self.start, self.route)
        self.assertEqual(counts, counts_lrs)        # (b) 동일: 간선별 횟수
        self.assertEqual(len(route_rls), len(self.route))
        self.assertNotEqual(route_rls, self.route)  # 방문 순서만 다르다

    def test_d_step_count_formula(self):
        edges = count_edges(self.adj)
        trunk_edges = len(self.trunk) - 1
        self.assertEqual(len(self.route), 2 * edges - trunk_edges)

    def test_shortest_path_endpoints(self):
        self.assertEqual(self.trunk[0], self.start)
        self.assertEqual(self.trunk[-1], "home")
        for i in range(len(self.trunk) - 1):
            self.assertIn(self.trunk[i + 1],
                          list(self.adj[self.trunk[i]].values()))

    def test_first_step_matches_uturned_heading(self):
        """유턴 직후 heading == route[0] 방향 — 추가 회전 없이 바로 주행."""
        self.assertEqual(self.route[0][0], self.heading)

    def test_execute_return_full(self):
        """실행층 재현: 계획대로 걸으면 폴백 없이 home 도달 + 빨강 6 재방문."""
        ex2, pos, heading, _ev = simulate_out(COURSE_ADJ, COURSE_MARK)
        adj, mark = explorer_to_graph(ex2.nodes, ex2.arm_ends)
        start = [n for n in mark if mark[n] == "green"][0]
        reds, end = simulate_return(ex2, adj, mark, start, heading)
        self.assertEqual(end, "home")
        self.assertEqual(reds, RED_COUNT)
        self.assertFalse(ex2.home_fallback)
        self.assertEqual(ex2.route_left(), 1)   # 남은 스텝 = home 진입뿐

    def test_fallback_never_stops(self):
        """도착 종류 불일치 → RETURN_FALLBACK 후에도 항상 move 를 낸다."""
        ex2, _pos, _heading, _ev = simulate_out(COURSE_ADJ, COURSE_MARK)
        move, events = ex2.on_arrive_home("dead_end", True, False, True)
        self.assertTrue(ex2.home_fallback)
        self.assertIn(move, ("L", "R", "S", "U"))
        self.assertIn("RETURN_FALLBACK", [e[0] for e in events])
        move2, _ev2 = ex2.on_arrive_home("junction", False, False, False)
        self.assertEqual(move2, "U")            # 즉석 탐색도 멈추지 않는다


class GreenlessCourseCase(unittest.TestCase):
    """초록이 없는 코스: EXPLORE_DONE 시 루트에서 같은 계획 함수 사용."""

    @classmethod
    def setUpClass(cls):
        adj = dict(COURSE_ADJ)
        adj[3] = {"S": 2, "W": "r6", "E": "d1"}     # green → 막다른길
        adj["d1"] = {"W": 3}
        del adj["green"]
        mark = dict(COURSE_MARK)
        del mark["green"]
        cls.ex, cls.pos, cls.heading, cls.events = simulate_out(adj, mark)
        cls.adj, cls.mark = explorer_to_graph(cls.ex.nodes, cls.ex.arm_ends)

    def test_route_from_root_covers_all(self):
        self.assertEqual(self.ex.mode, "HOME")
        self.assertEqual(self.ex.home_red_total, RED_COUNT)
        # start = 루트(노드 0), 트렁크 = 루트→home 1간선.
        end, visited, _counts = walk_route(self.adj, 0, self.ex.route)
        self.assertEqual(end, "home")
        reds = set(n for n in self.mark if self.mark[n] == "red")
        self.assertTrue(reds.issubset(set(visited)))
        self.assertEqual(len(self.ex.route), 2 * count_edges(self.adj) - 1)


class NoMapGreenCase(unittest.TestCase):
    """첫 분기 전 초록: 지도 없음 — 빈 계획으로 HOME 전환(온 길 = 노랑)."""

    def test_empty_route_home(self):
        ex = Explorer()
        ex.apply_move("U")      # 배달 유턴
        events = ex.start_home()
        self.assertEqual(ex.mode, "HOME")
        self.assertEqual(ex.route, [])
        self.assertFalse(ex.home_fallback)
        self.assertEqual(ex.home_red_total, 0)
        self.assertIn("RETURN_PLAN", [e[0] for e in events])


if __name__ == "__main__":
    unittest.main(verbosity=2)
