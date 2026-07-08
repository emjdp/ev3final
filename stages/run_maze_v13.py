#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_maze_v13 — v12 + 000(전백) 오판 → 가짜 데드엔드 유턴 방지판.

v12 문제(실기): 라인트레이싱 중 로봇이 살짝 틀어지면 중앙 컬러센서만 라인을
벗어나 bits=000 이 되고, 이것이 노드 후보로 확정 → handle_lost → 복구 후 4초
안에 재유실 → 후진 없이 즉시 유턴으로 이어졌다(run_maze_v13_notes.md).

v13 변경 4가지:
  1) 000 지속시간 필터 — 000 은 lost_persist_ms(기본 100ms) 연속 유지될 때만
     유실 의심으로 처리. 컬러센서 깜빡임/데드밴드 스침을 거른다.
  2) 000 은 노드 후보에서 제외 — confirm 저속 '전진' 없이 즉시 정지 후
     정지 상태 복수 샘플(3회) 재판정. 한 샘플이라도 라인이 보이면 취소.
  3) 복구 연속 한도(lost_max_recover, 기본 2) — 유턴 확정 전에 후진 복구를
     최대 N회 허용. 진짜 막다른길은 복구해도 곧 다시 유실돼 한도에 걸린다.
  4) 복구 후 재정렬 — 후진으로 라인을 찾으면 중앙 컬러가 black 이 되도록
     좌/우 소각 스캔(REALIGN)으로 자세를 교정한 뒤 재출발한다.

v13.1 추가(T/십자 분기 미인식 → 가짜 데드엔드 수정):
  5) confirm 취소 디바운스 분리 — 취소 후 재감지 대기를 900ms → 150ms 로
     줄여 블라인드 주행 중 분기점을 타넘는 것을 막는다.
  6) passed-over 확정 — 좌/우 팔이 있던 후보(111/101)가 재판정에서 전백(000)이
     되면 '가로선을 지나침'으로 보고 처음 bits 로 확정한다(취소 아님).
  7) 회전 후 라인 재획득 — 피벗 직후 중앙이 라인 밖(흰/없음)이면 REALIGN
     스캔으로 올라탄 뒤 출발한다(마커 색 위에서는 생략).
  8) lost_streak 은 회전 시 리셋 — 이전 구간의 복구 카운트가 이월돼
     회전 직후 첫 유실에서 즉시 유턴하던 버그 수정.

나머지 동작은 v12 와 동일(run_maze_v12_notes.md):
  - 라인트레이싱은 좌/우 반사광 차이(R-L)만 쓰는 PD(KD 0.05 고정, D clamp+EMA).
    중앙 컬러센서는 노드/마커 색 판단 전용.
  - 노드/분기 의심 bits 가 뜨면 PD 를 즉시 끄고 저속 직진(node_confirm_ms) 후
    정지 상태에서 재판정한다. 재판정을 통과해야 노드로 처리한다.
    확정까지 전진한 거리는 node_advance_mm 에서 빼서 총 전진이 40mm 를 넘지 않게 한다.
  - 빨강/노랑/초록 마커는 색센서 인식 즉시 정지 + 짧은 부저 2번.
    노랑@복귀 = 완주(정지), 초록@탐색 = 배달(전진→그리퍼 열기→후진), 나머지 = 유턴.
  - 소리 구분: 색 마커 = 부저 2번 / 커브 = 단일 tone / 분기점 = 2음 tone /
    유턴 = 낮은 tone 2번.
  - 세션 루프: 출발 대기(노랑) → 탐색 → 복귀 → 완주 후 대기. 대시보드 reset 액션
    ([r] 키 또는 액션 버튼)은 언제든 상태를 전부 버리고 출발 대기로 되돌린다.
    stop 만 프로그램을 끝낸다.
  - 물체: 탐색 중 초음파 거리 < grab_dist_cm 이면 그리퍼로 잡고, 초록에서 놓는다.

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      BACK 버튼 미사용 — 정지는 네트워크 stop 또는 Ctrl-C, 재시작은 네트워크 reset.

실행(브릭):   python3 stages/run_maze_v13.py
문법 점검(PC): python3 -m py_compile stages/run_maze_v13.py lib/*.py
"""

import os
import sys
import threading
import time

# stages/ 에서 단독 실행해도 lib/ 를 import 하도록 저장소 루트를 경로에 넣는다.
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from lib.shared_params import SharedParams                          # noqa: E402
from lib.telemetry import Telemetry                                 # noqa: E402
from lib.decision_log import DecisionLog                            # noqa: E402
from lib.tuning_server import TuningServer                          # noqa: E402
# lib.hardware(ev3dev2)는 run() 안에서 import 한다.


# ---------------------------------------------------------------------
# 상수 — 색/노드 패턴/기하/고정 주행값 (정리.md, run_maze_v13_notes.md)
# ---------------------------------------------------------------------

COL_BLACK = 1
COL_GREEN = 3
COL_YELLOW = 4
COL_RED = 5

MARKER_COLORS = (COL_RED, COL_YELLOW, COL_GREEN)
MARKER_NAMES = {COL_RED: "red", COL_YELLOW: "yellow", COL_GREEN: "green"}

# 노드 후보 bits(좌,중,우): 좌회전 / 우회전 / 십자 / T(직진 없음).
# v13: 000(전백)은 노드 후보가 아니라 '유실 의심'으로 따로 다룬다(§변경 1,2).
NODE_CANDIDATES = ((1, 1, 0), (0, 1, 1), (1, 1, 1), (1, 0, 1))
LOST_BITS = (0, 0, 0)
SLOW_ON = ((1, 1, 1), (1, 0, 1))    # 이 패턴이 스치면 확정 전이라도 감속

# 기하(정리.md 엔코더/기하값)
MM_PER_DEG = 3.14159265 * 56.0 / 360.0      # 바퀴 지름 56mm
BASE_PIVOT_DEG_90 = 193.0
BASE_PIVOT_DEG_180 = 386.0
POST_TURN_SETTLE_S = 0.12

# 고정 주행값(라이브 튜닝 대상 아님)
STRAIGHT_SPEED = 15         # 출발 이탈 / 배달 전·후진
CONFIRM_SPEED = 7           # 의심지점 저속 직진 + 노드 전진
SLOW_SPEED = 12             # SLOW_ON 패턴/유실 의심 감속 주행
BACKUP_SPEED = 10           # 선 유실 후진
START_EXIT_MM = 50          # 출발 노랑에서 벗어나는 거리
LOST_BACKUP_MM = 100        # 선 유실 시 최대 후진 거리
LOST_RETRY_WINDOW_MS = 4000  # 이 시간 지나면 복구 연속 카운트(lost_streak) 리셋
LOST_GUARD_TURN = 8.0       # |turn| 이 크면 000 은 위빙으로 보고 무시
LOST_CONFIRM_SAMPLES = 3    # 유실 확정 전 정지 재판정 샘플 수(전부 000 이어야 확정)
LOST_CONFIRM_GAP_S = 0.02   # 재판정 샘플 간격
REALIGN_SPEED = 6           # 복구 후 재정렬 소각 스캔 속도(%)
REALIGN_MAX_DEG = 70        # 재정렬 스캔 한쪽 최대 enc deg(로봇 약 50도)
NODE_DEBOUNCE_MS = 900      # 노드 '확정' 후 재감지 최소 간격
NODE_CANCEL_DEBOUNCE_MS = 150  # v13.1: confirm '취소' 후 간격 — 900ms 블라인드로
                               # 진짜 분기점(T/십자)을 지나치는 것을 막는다
MARKER_DEBOUNCE_MS = 1500   # 같은 마커 재감지 방지
MARKER_PAUSE_S = 0.08       # 마커 정지 후 잠깐 멈춤
CONFIRM_SETTLE_S = 0.08     # 재판정 전 정지 안정화
GRIP_SEC = 0.8              # 그리퍼 열기/닫기 구동 시간
LOOP_DELAY_S = 0.015
FOLLOW_LOG_S = 0.25         # LINE_FOLLOW 로그 최소 간격
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)

# PD — KD 는 notes 대로 고정, derivative 는 clamp + EMA 로 완화
PD_KD = 0.05
PD_TURN_LIMIT = 16
PD_DERIV_LIMIT = 220.0
PD_D_EMA_ALPHA = 0.35

# 소리 구분(notes) — (주파수Hz, 길이ms) 나열. 색 마커는 beep 2번.
TONE_CURVE = ((700, 120),)
TONE_BRANCH = ((600, 100), (900, 100))
TONE_UTURN = ((300, 150), (300, 150))


# ---------------------------------------------------------------------
# 라이브 파라미터 — 대시보드/robotctl 로 실기에서 튜닝한다.
# ---------------------------------------------------------------------

# 파라미터당 한 줄: (이름, 초기값, min, max, max_step, ui_step, 단위)
PARAM_TABLE = (
    ("base_speed",      16,    5,   45,   5,    1,    "%"),
    ("kp",              0.17,  0.0, 3.0,  0.1,  0.01, ""),
    ("turn_speed",      6,     5,   40,   5,    1,    "%"),
    ("node_confirm_ms", 40,    0,   1000, 60,   10,   "ms"),
    ("left_th_steer",   66,    0,   100,  3,    1,    "%"),   # 유실 복구 검정 판정
    ("right_th_steer",  63,    0,   100,  3,    1,    "%"),
    ("left_th_node",    18,    0,   100,  3,    1,    "%"),   # 노드 bits 판정
    ("right_th_node",   14,    0,   100,  3,    1,    "%"),
    ("node_advance_mm", 40,    0,   120,  10,   10,   "mm"),  # 의심지점 기준 총 전진
    ("goal_advance_mm", 20,    0,   200,  10,   10,   "mm"),  # 배달 전·후진 거리
    ("turn_90_factor",  0.66,  0.3, 2.0,  0.05, 0.01, "x"),
    ("turn_180_factor", 0.71,  0.3, 2.0,  0.05, 0.01, "x"),
    ("grab_dist_cm",    6.0,   1.0, 20.0, 1.0,  0.5,  "cm"),
    ("grip_speed",      50,    5,   80,   5,    1,    "%"),
    ("lost_persist_ms", 100,   0,   500,  100,  10,   "ms"),  # v13: 000 지속 필터
    ("lost_max_recover", 2,    1,   3,    1,    1,    ""),    # v13: 유턴 전 복구 허용 횟수
)

INITIAL_PARAMS = dict((r[0], r[1]) for r in PARAM_TABLE)
PARAM_LIMITS = dict((r[0], (r[2], r[3])) for r in PARAM_TABLE)
MAX_STEP = dict((r[0], r[4]) for r in PARAM_TABLE)
UI_STEP = dict((r[0], r[5]) for r in PARAM_TABLE)
UNITS = dict((r[0], r[6]) for r in PARAM_TABLE)
PARAM_ORDER = [r[0] for r in PARAM_TABLE]

SAVE_PATH = os.path.join(_ROOT, "config", "run_maze_v13.json")
STAGE_NAME = "run_maze_v13"

ACTIONS = [
    {"name": "read_color", "label": "Read Center Color"},
    {"name": "read_reflect", "label": "Read L/R Reflect"},
    {"name": "reset", "label": "Reset to Start (wait YELLOW)"},
]


# ---------------------------------------------------------------------
# 순수 헬퍼
# ---------------------------------------------------------------------

def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def node_bits(reflect_l, center_color, reflect_r, snap):
    """노드 판정 bits(좌,중,우) — 좌/우는 반사광, 중앙은 컬러 black 여부."""
    return (1 if reflect_l < snap["left_th_node"] else 0,
            1 if center_color == COL_BLACK else 0,
            1 if reflect_r < snap["right_th_node"] else 0)


def bits_str(bits):
    return "{}{}{}".format(bits[0], bits[1], bits[2])


DIRS = ("N", "E", "S", "W")
MOVES = ("S", "R", "U", "L")    # index = 시계방향 90도 회전 수


def turn_heading(heading, move):
    return DIRS[(DIRS.index(heading) + MOVES.index(move)) % 4]


def rel_move(heading, target_dir):
    return MOVES[(DIRS.index(target_dir) - DIRS.index(heading)) % 4]


# ---------------------------------------------------------------------
# 탐색 상태머신(순수) — 전역 위치인식 없이 분기 트리를 관리한다.
# ---------------------------------------------------------------------

class Explorer(object):
    """전 분기 방문(우선순위 L>R>S, 보류는 LIFO) 후 부모 체인으로 복귀.

    mode: TO_FIRST(첫 분기 전) / PROBE(미탐색 팔 진입 중) /
          RETURN_TO_WORK(팔 끝에서 유턴 복귀 중) / GOTO_PENDING(보류 자식으로 이동) /
          BACKTRACK(부모로 이동) / HOME(복귀 계획 소비 중)
    판단 결과는 (move, events) — move 는 "L"/"R"/"S"/"U", events 는 로그용.
    """

    PRIORITY = ("L", "R", "S")

    def __init__(self):
        # id -> {"parent": id|None, "parent_dir": 들어온 방위,
        #        "arms": {방위: "OPEN"|"DONE"|자식id}, "pending": [방위]}
        self.nodes = {}
        self.heading = "N"
        self.mode = "TO_FIRST"
        self.work = None
        self.probe_arm = None
        self.goto_arm = None
        self.plan = []          # HOME: [(node_id, exit_dir), ...]

    def apply_move(self, move):
        self.heading = turn_heading(self.heading, move)

    def _add_node(self, has_left, has_right, has_straight, parent):
        nid = len(self.nodes)
        arms = {}
        if has_left:
            arms[turn_heading(self.heading, "L")] = "OPEN"
        if has_right:
            arms[turn_heading(self.heading, "R")] = "OPEN"
        if has_straight:
            arms[self.heading] = "OPEN"
        self.nodes[nid] = {"parent": parent,
                           "parent_dir": turn_heading(self.heading, "U"),
                           "arms": arms, "pending": []}
        return nid

    def _open_dirs(self, nid):
        arms = self.nodes[nid]["arms"]
        return [d for d in arms if arms[d] == "OPEN"]

    def on_junction(self, has_left, has_right, has_straight):
        """탐색 중 분기점 도착 처리."""
        events = []
        if self.mode == "TO_FIRST":
            self.work = self._add_node(has_left, has_right, has_straight, None)
            events.append(("NODE_NEW", "FIRST", {"id": self.work}))
        elif self.mode == "PROBE":
            nid = self._add_node(has_left, has_right, has_straight, self.work)
            self.nodes[self.work]["arms"][self.probe_arm] = nid
            events.append(("NODE_NEW", "PROBE",
                           {"id": nid, "parent": self.work, "via": self.probe_arm}))
            if self._open_dirs(self.work):
                # work 에 미탐색 팔이 남았으면 새 노드는 보류하고 유턴 복귀.
                self.nodes[self.work]["pending"].append(self.probe_arm)
                events.append(("PENDING_SAVED", "WORK_HAS_OPEN_ARMS",
                               {"work": self.work, "via": self.probe_arm}))
                self.probe_arm = None
                self.mode = "RETURN_TO_WORK"
                return "U", events
            self.work = nid
            self.probe_arm = None
        elif self.mode == "RETURN_TO_WORK":
            events.append(("BACK_TO_WORK", "PROBE_END", {"work": self.work}))
        elif self.mode == "GOTO_PENDING":
            self.work = self.nodes[self.work]["arms"][self.goto_arm]
            self.goto_arm = None
            events.append(("BACK_TO_WORK", "PENDING_ARRIVED", {"work": self.work}))
        elif self.mode == "BACKTRACK":
            self.work = self.nodes[self.work]["parent"]
            events.append(("BACK_TO_WORK", "BACKTRACK_ARRIVED", {"work": self.work}))
        move, more = self._select_next()
        return move, events + more

    def _select_next(self):
        """work 노드에서 다음 행선지: 미탐색 팔 > 보류 자식 > 부모 > 복귀 시작."""
        events = []
        node = self.nodes[self.work]
        open_dirs = self._open_dirs(self.work)
        for rel in self.PRIORITY:
            d = turn_heading(self.heading, rel)
            if d in open_dirs:
                self.probe_arm = d
                self.mode = "PROBE"
                events.append(("PROBE", "PRIORITY_L_R_S",
                               {"work": self.work, "arm": d, "move": rel}))
                return rel, events
        if node["pending"]:
            self.goto_arm = node["pending"].pop()
            self.mode = "GOTO_PENDING"
            move = rel_move(self.heading, self.goto_arm)
            events.append(("GOTO_PENDING", "LIFO",
                           {"work": self.work, "arm": self.goto_arm, "move": move,
                            "pending_left": len(node["pending"])}))
            return move, events
        if node["parent"] is not None:
            self.mode = "BACKTRACK"
            move = rel_move(self.heading, node["parent_dir"])
            events.append(("BACKTRACK", "WORK_DONE",
                           {"to": node["parent"], "move": move}))
            return move, events
        self.plan = self._path_home()
        self.mode = "HOME"
        events.append(("EXPLORE_DONE", "MAP_COMPLETE",
                       {"nodes": len(self.nodes), "steps": len(self.plan)}))
        move, more = self.on_junction_home(True, True, True)
        return move, events + more

    def on_probe_end(self, kind):
        """PROBE 팔이 분기점 없이 끝났다(마커/막다른길). 팔을 DONE 처리."""
        if self.mode != "PROBE":
            return [("PROBE_END", "IGNORED_MODE_" + self.mode, {"kind": kind})]
        self.nodes[self.work]["arms"][self.probe_arm] = "DONE"
        events = [("PROBE_END", "ARM_DONE",
                   {"work": self.work, "arm": self.probe_arm, "kind": kind})]
        self.probe_arm = None
        self.mode = "RETURN_TO_WORK"
        return events

    def _path_home(self):
        chain = []
        nid = self.work
        while nid is not None:
            chain.append((nid, self.nodes[nid]["parent_dir"]))
            nid = self.nodes[nid]["parent"]
        return chain

    def on_junction_home(self, has_left, has_right, has_straight):
        """복귀 중 분기점: 계획된 출구로. 계획이 어긋나면 우선순위 fallback."""
        events = []
        avail = {"L": has_left, "R": has_right, "S": has_straight, "U": True}
        if self.plan:
            nid, exit_dir = self.plan.pop(0)
            move = rel_move(self.heading, exit_dir)
            if avail[move]:
                events.append(("RETURN_STEP", "PLAN",
                               {"node": nid, "move": move,
                                "plan_left": len(self.plan)}))
                return move, events
            events.append(("RETURN_FALLBACK", "PATH_MISMATCH",
                           {"node": nid, "exit": exit_dir, "move": move}))
        else:
            events.append(("RETURN_FALLBACK", "PLAN_EMPTY", {}))
        for rel in self.PRIORITY:
            if avail[rel]:
                return rel, events
        return "U", events


# ---------------------------------------------------------------------
# PD 조향(순수)
# ---------------------------------------------------------------------

class PdSteer(object):
    """error = 우반사광 - 좌반사광. KD 고정, derivative 는 clamp + EMA."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.prev_t = None
        self.deriv = 0.0

    def step(self, reflect_l, reflect_r, kp, base_speed):
        error = float(reflect_r - reflect_l)
        t = time.monotonic()
        if self.prev_t is not None:
            dt = max(t - self.prev_t, 0.001)
            raw = clamp((error - self.prev_error) / dt,
                        -PD_DERIV_LIMIT, PD_DERIV_LIMIT)
            self.deriv = (PD_D_EMA_ALPHA * raw +
                          (1.0 - PD_D_EMA_ALPHA) * self.deriv)
        turn = clamp(kp * error + PD_KD * self.deriv,
                     -PD_TURN_LIMIT, PD_TURN_LIMIT)
        self.prev_error = error
        self.prev_t = t
        return base_speed - turn, base_speed + turn, error, turn


# ---------------------------------------------------------------------
# 구동 러너 — hw 를 몰고 Explorer 판단을 실행한다. run() 에서만 생성.
# ---------------------------------------------------------------------

class Runner(object):

    def __init__(self, hw, params, tele, log):
        self.hw = hw
        self.params = params
        self.tele = tele
        self.log = log
        self.started = time.monotonic()

        # 네트워크 스레드가 세팅하고 제어 루프가 안전한 시점에 소비하는 플래그.
        self.stop_on = False
        self.stop_source = None
        self.reset_on = False
        self.reset_source = None
        self.paused = False
        self._pending = None
        self._pending_lock = threading.Lock()

        self.session = 0
        self.ex = Explorer()
        self.pd = PdSteer()
        self._init_session_state()

    def _init_session_state(self):
        self.visits = 0
        self.goal_seen = False
        self.grabbed = False
        self.done = False
        self.last_turn = 0.0
        self.last_marker_t = -1e9
        self.last_node_t = -1e9
        self.last_recover_t = -1e9
        self.lost_since = None      # v13: 000 연속 시작 시각(지속 필터)
        self.lost_streak = 0        # v13: 윈도 내 연속 복구 횟수
        self.node_debounce_ms = 0   # v13.1: 직전 confirm 결과에 따른 재감지 간격

    # ---- 네트워크 핸들러(TuningServer 스레드에서 호출) ----

    def on_stop(self, source):
        self.stop_on = True
        self.stop_source = source

    def on_pause(self, paused, source):
        self.paused = bool(paused)
        self.log.log("PAUSE" if paused else "RESUME", "NETWORK", source=source)
        return {"mode": "paused" if paused else "run"}

    def on_do(self, action, args):
        if action == "reset":
            self.reset_on = True
            self.reset_source = (args or {}).get("source", "dashboard")
            return {"queued": "reset"}
        with self._pending_lock:
            self._pending = action
        return {"queued": action}

    # ---- 공용 ----

    def interrupted(self):
        return self.stop_on or self.reset_on

    def publish(self, mode, **extra):
        frame = {
            "t_ms": int((time.monotonic() - self.started) * 1000),
            "param_rev": self.params.rev(),
            "running": True,
            "mode": mode,
            "paused": self.paused,
            "session": self.session,
            "ex_mode": self.ex.mode,
            "visits": self.visits,
            "grabbed": self.grabbed,
        }
        frame.update(extra)
        self.tele.publish(frame)

    def log_events(self, events):
        for event, rule, detail in events:
            self.log.log(event, rule, **detail)

    def handle_pending(self):
        """대시보드 read_color / read_reflect 액션 처리(없으면 no-op)."""
        with self._pending_lock:
            action = self._pending
            self._pending = None
        if action == "read_color":
            color = self.hw.read_center_color_now()
            self.log.log("COLOR_READ", "DO_TRIGGER", color=color)
            self.publish("read_color", color=color)
        elif action == "read_reflect":
            rl = self.hw.read_left_reflect()
            rr = self.hw.read_right_reflect()
            self.log.log("REFLECT_READ", "DO_TRIGGER", reflect_l=rl, reflect_r=rr)
            self.publish("read_reflect", reflect_l=rl, reflect_r=rr)

    def play(self, tones):
        for freq, ms in tones:
            self.hw.tone(freq, ms)

    def reset_steer(self):
        self.pd.reset()
        self.last_turn = 0.0
        self.lost_since = None      # 모션 프리미티브 후 stale 유실 타이머 방지(v13)

    def _hold_while_paused(self, mode):
        """모션 중 pause: 모터를 세우고 해제/중단까지 대기."""
        self.hw.stop()
        while self.paused and not self.interrupted():
            self.publish(mode + "_paused")
            time.sleep(0.05)

    def read_bits(self, snap):
        color = self.hw.read_center_color_now()
        rl = self.hw.read_left_reflect()
        rr = self.hw.read_right_reflect()
        return node_bits(rl, color, rr, snap), color, rl, rr

    # ---- 모션 프리미티브 ----

    def straight(self, dist_mm, speed, mode="advancing"):
        """엔코더 기준 직진(speed<0 후진). 중단 시점까지의 mm 반환."""
        self.hw.reset_encoders()
        if dist_mm <= 0:
            return 0.0
        target_deg = dist_mm / MM_PER_DEG
        try:
            self.hw.drive(speed, speed)
            while self.hw.enc_avg() < target_deg:
                if self.interrupted():
                    break
                if self.paused:
                    self._hold_while_paused(mode)
                    if self.interrupted():
                        break
                    self.hw.drive(speed, speed)
                self.publish(mode, dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
        return self.hw.enc_avg() * MM_PER_DEG

    def backup_to_line(self, max_mm, snap):
        """선을 다시 찾을 때까지 후진(최대 max_mm). (찾음 여부, 후진 mm)."""
        self.hw.reset_encoders()
        target_deg = max_mm / MM_PER_DEG
        found = False
        try:
            self.hw.drive(-BACKUP_SPEED, -BACKUP_SPEED)
            while self.hw.enc_avg() < target_deg:
                if self.interrupted():
                    break
                if self.paused:
                    self._hold_while_paused("lost_backup")
                    if self.interrupted():
                        break
                    self.hw.drive(-BACKUP_SPEED, -BACKUP_SPEED)
                color = self.hw.read_center_color_now()
                rl = self.hw.read_left_reflect()
                rr = self.hw.read_right_reflect()
                if (color == COL_BLACK or rl < snap["left_th_steer"] or
                        rr < snap["right_th_steer"]):
                    found = True
                    break
                self.publish("lost_backup",
                             dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
        return found, self.hw.enc_avg() * MM_PER_DEG

    def turn(self, move):
        """제자리 회전(L/R/U) + heading 갱신 + PD 리셋. 유턴은 낮은 tone 2번 선행.
        회전각/속도를 바꾸려면 여기(와 params 의 factor)만 보면 된다."""
        # v13.1: 노드/마커 처리를 마치면 새 구간 — 직전 구간의 복구 카운트를
        # 이월하면 회전 직후 첫 유실에서 후진 없이 즉시 유턴해 버린다.
        self.lost_streak = 0
        if move == "S":
            return
        snap = self.params.snapshot()
        if move == "U":
            self.play(TONE_UTURN)
            target = BASE_PIVOT_DEG_180 * snap["turn_180_factor"]
        else:
            target = BASE_PIVOT_DEG_90 * snap["turn_90_factor"]
        left_dir, right_dir = (-1, 1) if move == "L" else (1, -1)
        speed = snap["turn_speed"]
        self.hw.reset_encoders()
        try:
            self.hw.drive_raw(left_dir * speed, right_dir * speed)
            while self.hw.enc_avg() < target:
                if self.interrupted():
                    break
                if self.paused:
                    self._hold_while_paused("turning")
                    if self.interrupted():
                        break
                    self.hw.drive_raw(left_dir * speed, right_dir * speed)
                self.publish("turning", target_deg=round(target, 1),
                             enc_avg=round(self.hw.enc_avg(), 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
        actual = self.hw.enc_avg()
        time.sleep(POST_TURN_SETTLE_S)
        self.log.log("TURN", {"L": "TURN_LEFT", "R": "TURN_RIGHT", "U": "UTURN"}[move],
                     target_deg=round(target, 1), enc_avg=round(actual, 1),
                     error_deg=round(actual - target, 1),
                     stopped_early=self.interrupted())
        self.ex.apply_move(move)
        self.reset_steer()
        # v13.1: 회전 직후 라인 재획득 — 피벗 오차로 중앙이 라인 밖(흰/없음)이면
        # 000 유실 체인으로 빠지기 전에 소각 스캔으로 라인에 올라탄다.
        # 마커 색 위(유턴 직후 등)에서는 스캔하지 않고, 실패해도 계속 진행한다.
        if not self.interrupted():
            color = self.hw.read_center_color_now()
            if color != COL_BLACK and color not in MARKER_COLORS:
                self.log.log("TURN_ACQUIRE", "CENTER_OFF_LINE_AFTER_TURN",
                             move=move, color=color)
                self.realign_to_line(self.params.snapshot())

    # ---- 마커 / 노드 처리 ----

    def handle_marker(self, color, context):
        """빨강/노랑/초록 마커 — 즉시 정지 + 짧은 부저 2번(최우선).
        노랑@복귀 = 완주 / 초록@탐색 = 배달 후 유턴 / 그 외 = 유턴. 처리했으면 True."""
        if color not in MARKER_COLORS:
            return False
        if (time.monotonic() - self.last_marker_t) * 1000 < MARKER_DEBOUNCE_MS:
            return False

        self.hw.stop()
        time.sleep(MARKER_PAUSE_S)
        self.hw.beep_ok()
        self.hw.beep_ok()
        name = MARKER_NAMES[color]
        self.log.log("MARKER", "COLOR_{}_IMMEDIATE".format(name.upper()),
                     color=color, context=context, ex_mode=self.ex.mode,
                     session=self.session)

        if color == COL_YELLOW and self.ex.mode == "HOME":
            self.done = True
            self.log.log("HOME_REACHED", "COLOR_YELLOW", plan_left=len(self.ex.plan))
            return True

        if color == COL_RED:
            self.visits += 1
        elif color == COL_GREEN and self.ex.mode != "HOME":
            self.deliver()
            if self.interrupted():
                return True
        self.turn("U")
        if self.ex.mode == "PROBE":
            self.log_events(self.ex.on_probe_end(name))
        self.last_marker_t = time.monotonic()
        return True

    def deliver(self):
        """초록(goal): goal_advance_mm 전진 → 그리퍼 열어 물체 놓기 → 같은 거리 후진."""
        snap = self.params.snapshot()
        self.goal_seen = True
        self.log.log("GOAL_DELIVER", "COLOR_GREEN",
                     goal_advance_mm=snap["goal_advance_mm"])
        self.straight(snap["goal_advance_mm"], STRAIGHT_SPEED)
        if self.interrupted():
            return
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
        self.straight(snap["goal_advance_mm"], -STRAIGHT_SPEED)

    def confirm_node(self, first_bits, snap):
        """의심지점: PD off → 저속 직진(node_confirm_ms) → 정지 후 재판정.
        (확정 bits 또는 None, 그동안 전진한 mm) 반환. 마커를 만나면 처리 후 None.
        v13: 000 은 여기로 들어오지 않는다 — creep/재판정 중 000 이 보이면 취소."""
        self.reset_steer()
        self.hw.reset_encoders()
        self.log.log("NODE_CANDIDATE", "PD_OFF_SLOW_STRAIGHT",
                     bits=bits_str(first_bits),
                     confirm_ms=snap["node_confirm_ms"], speed=CONFIRM_SPEED)
        end = time.monotonic() + snap["node_confirm_ms"] / 1000.0
        self.hw.drive(CONFIRM_SPEED, CONFIRM_SPEED)
        while time.monotonic() < end:
            if self.interrupted():
                self.hw.stop()
                return None, self.hw.enc_avg() * MM_PER_DEG
            if self.paused:
                self._hold_while_paused("node_confirm")
                if self.interrupted():
                    return None, self.hw.enc_avg() * MM_PER_DEG
                self.hw.drive(CONFIRM_SPEED, CONFIRM_SPEED)
            bits, color, rl, rr = self.read_bits(snap)
            if self.handle_marker(color, "node_confirm"):
                return None, 0.0
            if bits not in NODE_CANDIDATES:
                self.hw.stop()
                creep_mm = self.hw.enc_avg() * MM_PER_DEG
                if bits == LOST_BITS and first_bits[0] == 1 and first_bits[2] == 1:
                    # v13.1: 좌/우 팔이 있던 후보(111/101)가 전백이 됐다 =
                    # T/십자의 가로선을 지나쳤다는 확실한 증거. 취소하면
                    # 블라인드 주행 → 000 유실 체인 → 가짜 데드엔드가 되므로
                    # 처음 본 bits 로 확정한다(직진 유무는 handle_node 가 색으로 재판정).
                    self.log.log("NODE_CONFIRMED", "PASSED_OVER_DURING_CREEP",
                                 first_bits=bits_str(first_bits),
                                 reflect_l=rl, reflect_r=rr, color=color,
                                 creep_mm=round(creep_mm, 1))
                    return first_bits, creep_mm
                self.log.log("NODE_CANDIDATE", "CANCELLED_DURING_CREEP",
                             first_bits=bits_str(first_bits), bits=bits_str(bits),
                             reflect_l=rl, reflect_r=rr, color=color)
                return None, creep_mm
            self.publish("node_confirm", bits=bits_str(bits),
                         reflect_l=rl, reflect_r=rr, color=color)
            time.sleep(LOOP_DELAY_S)

        self.hw.stop()
        time.sleep(CONFIRM_SETTLE_S)
        creep_mm = self.hw.enc_avg() * MM_PER_DEG
        bits, color, rl, rr = self.read_bits(snap)
        if self.handle_marker(color, "node_confirm_stop"):
            return None, 0.0
        if bits in NODE_CANDIDATES:
            self.log.log("NODE_CONFIRMED", "SLOW_STRAIGHT_STOP",
                         first_bits=bits_str(first_bits), bits=bits_str(bits),
                         reflect_l=rl, reflect_r=rr, color=color,
                         creep_mm=round(creep_mm, 1))
            return bits, creep_mm
        if bits == LOST_BITS and first_bits[0] == 1 and first_bits[2] == 1:
            # v13.1: 정지 재판정에서 전백 — 위 creep 케이스와 같은 passed-over.
            self.log.log("NODE_CONFIRMED", "PASSED_OVER_AT_STOP",
                         first_bits=bits_str(first_bits),
                         reflect_l=rl, reflect_r=rr, color=color,
                         creep_mm=round(creep_mm, 1))
            return first_bits, creep_mm
        self.log.log("NODE_CANDIDATE", "CANCELLED_AT_STOP",
                     first_bits=bits_str(first_bits), bits=bits_str(bits),
                     reflect_l=rl, reflect_r=rr, color=color)
        return None, creep_mm

    def handle_node(self, bits, creep_mm):
        """확정 노드 처리. creep_mm(확정 중 전진분)을 빼서 총 전진을 40mm 로 맞춘다.
        v13: bits 는 NODE_CANDIDATES 멤버만 온다(000 은 유실 경로로 분리)."""
        snap = self.params.snapshot()
        self.straight(max(0.0, snap["node_advance_mm"] - creep_mm), CONFIRM_SPEED)
        if self.interrupted():
            return

        color = self.hw.read_center_color_now()
        if self.handle_marker(color, "after_node_advance"):
            return

        has_left = bits[0] == 1
        has_right = bits[2] == 1
        has_straight = color == COL_BLACK
        n_exits = int(has_left) + int(has_right) + int(has_straight)

        if n_exits == 0:
            # NODE_CANDIDATES 는 좌/우 팔이 있어 정상적으론 안 온다(방어용).
            self.log.log("DEAD_END", "NO_EXIT_AFTER_ADVANCE",
                         bits=bits_str(bits), color=color)
            self._dead_end_uturn()
            return

        if n_exits == 1:
            move = "L" if has_left else ("R" if has_right else "S")
            self.log.log("CURVE", "FORCED_" + {"L": "LEFT", "R": "RIGHT",
                                               "S": "STRAIGHT"}[move],
                         bits=bits_str(bits), color=color)
            self.play(TONE_CURVE)
            self.turn(move)
            return

        if self.ex.mode == "HOME":
            move, events = self.ex.on_junction_home(has_left, has_right, has_straight)
        else:
            move, events = self.ex.on_junction(has_left, has_right, has_straight)
        self.log_events(events)
        self.play(TONE_BRANCH)
        self.turn(move)

    # ---- 유실(000) 처리 — v13 ----

    def _dead_end_uturn(self):
        """막다른길 확정 공통 처리: 유턴 + PROBE 팔이면 DONE."""
        self.turn("U")
        if self.ex.mode == "PROBE":
            self.log_events(self.ex.on_probe_end("dead_end"))

    def lost_check(self, snap):
        """000 지속 확정 전 재판정: 즉시 정지 → 정지 상태 복수 샘플.
        v12 의 '저속 전진 confirm' 은 유실 상황에선 라인에서 더 멀어져 오판을
        굳히므로 전진 없이 판정한다. 한 샘플이라도 라인이 보이면 취소.
        반환: "lost" / "line" / "marker" / "interrupted"."""
        self.hw.stop()
        time.sleep(CONFIRM_SETTLE_S)
        self.log.log("LOST_SUSPECT", "PERSIST_STOP_RECHECK",
                     persist_ms=snap["lost_persist_ms"],
                     samples=LOST_CONFIRM_SAMPLES)
        for i in range(LOST_CONFIRM_SAMPLES):
            if self.interrupted():
                return "interrupted"
            bits, color, rl, rr = self.read_bits(snap)
            if self.handle_marker(color, "lost_check"):
                return "marker"
            if bits != LOST_BITS:
                self.log.log("LOST_SUSPECT", "CANCELLED_LINE_SEEN",
                             bits=bits_str(bits), reflect_l=rl, reflect_r=rr,
                             color=color, sample=i)
                return "line"
            time.sleep(LOST_CONFIRM_GAP_S)
        self.log.log("LINE_LOST", "CONFIRMED_ALL_WHITE",
                     samples=LOST_CONFIRM_SAMPLES)
        return "lost"

    def _pivot_scan(self, direction, max_deg):
        """중앙 컬러가 black 이 될 때까지 소각 피벗(L/R). (found, 진행 enc deg)."""
        if max_deg <= 0:
            return False, 0.0
        left_dir, right_dir = (-1, 1) if direction == "L" else (1, -1)
        self.hw.reset_encoders()
        found = False
        try:
            self.hw.drive_raw(left_dir * REALIGN_SPEED, right_dir * REALIGN_SPEED)
            while self.hw.enc_avg() < max_deg:
                if self.interrupted():
                    break
                if self.hw.read_center_color_now() == COL_BLACK:
                    found = True
                    break
                time.sleep(0.005)
        finally:
            self.hw.stop()
        return found, self.hw.enc_avg()

    def realign_to_line(self, snap):
        """복구 직후 재정렬 — 중앙 컬러가 라인(black) 위에 오도록 좌/우 소각 스캔.

        v12 는 라인을 '찾기만' 하고 틀어진 자세 그대로 재출발해 몇 초 안에 다시
        유실되곤 했다. 어두운 쪽부터 스캔하고, 양쪽 다 실패하면 원래 방향으로
        복원 후 False(호출부가 막다른길 처리). 소각이라 heading 격자는 안 바꾼다."""
        if self.hw.read_center_color_now() == COL_BLACK:
            self.log.log("REALIGN", "ALREADY_ON_LINE")
            return True
        rl = self.hw.read_left_reflect()
        rr = self.hw.read_right_reflect()
        first = "L" if rl <= rr else "R"
        other = "R" if first == "L" else "L"
        found, d1 = self._pivot_scan(first, REALIGN_MAX_DEG)
        if found:
            time.sleep(POST_TURN_SETTLE_S)
            self.reset_steer()
            self.log.log("REALIGN", "SCAN_" + first, deg=round(d1, 1))
            return True
        if self.interrupted():
            return False
        found, d2 = self._pivot_scan(other, d1 + REALIGN_MAX_DEG)
        if found:
            time.sleep(POST_TURN_SETTLE_S)
            self.reset_steer()
            self.log.log("REALIGN", "SCAN_" + other, deg=round(d2 - d1, 1))
            return True
        if self.interrupted():
            return False
        # 양쪽 다 실패 — 원래 방향으로 복원(복원 중 black 을 만나면 성공 처리).
        offset = d1 - d2                      # first 방향 기준 잔여 각
        back_dir = other if offset > 0 else first
        found, _d3 = self._pivot_scan(back_dir, abs(offset))
        time.sleep(POST_TURN_SETTLE_S)
        self.reset_steer()
        if found:
            self.log.log("REALIGN", "FOUND_ON_RESTORE")
            return True
        self.log.log("REALIGN", "NOT_FOUND", scan_deg=round(d1 + d2, 1))
        return False

    def handle_lost(self):
        """확정 유실: 후진→재정렬 복구. 연속 복구 한도 초과/후진·재정렬 실패면 유턴.

        진짜 막다른길은 복구해도 곧 다시 유실돼 lost_streak 이 한도에 걸리고,
        단순 틀어짐은 재정렬 후 정상 주행으로 돌아가 윈도(4s)가 지나면 리셋된다."""
        snap = self.params.snapshot()
        if (time.monotonic() - self.last_recover_t) * 1000 >= LOST_RETRY_WINDOW_MS:
            self.lost_streak = 0
        if self.lost_streak >= snap["lost_max_recover"]:
            self.log.log("DEAD_END", "LOST_STREAK_LIMIT",
                         streak=self.lost_streak)
            self._dead_end_uturn()
            return
        self.log.log("LINE_LOST", "ALL_WHITE_BACKUP",
                     backup_mm=LOST_BACKUP_MM, streak=self.lost_streak)
        found, dist = self.backup_to_line(LOST_BACKUP_MM, snap)
        if self.interrupted():
            return
        if not found:
            self.log.log("DEAD_END", "BACKUP_NO_LINE", dist_mm=round(dist, 1))
            self._dead_end_uturn()
            return
        if not self.realign_to_line(snap):
            if self.interrupted():
                return
            self.log.log("DEAD_END", "REALIGN_NO_LINE", dist_mm=round(dist, 1))
            self._dead_end_uturn()
            return
        self.lost_streak += 1
        self.last_recover_t = time.monotonic()
        self.log.log("LINE_RECOVER", "BACKUP_REALIGN_OK",
                     dist_mm=round(dist, 1), streak=self.lost_streak)

    # ---- 세션 단계 ----

    def wait_for_start(self):
        """노랑 위에 놓일 때까지 대기 → beep → 출발선 이탈 전진. status 반환."""
        snap = self.params.snapshot()
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
        while self.hw.read_center_color_now() != COL_YELLOW:
            if self.stop_on:
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            self.publish("waiting_start")
            time.sleep(0.05)
        self.hw.beep_ok()
        self.log.log("START", "COLOR_YELLOW")
        self.straight(START_EXIT_MM, STRAIGHT_SPEED, mode="start_exit")
        self.last_marker_t = time.monotonic()   # 출발 노랑 재감지 방지
        return "go"

    def explore(self):
        """탐색+복귀 메인 루프. status(stop/reset/done) 반환."""
        last_follow_log = -1e9
        while not self.done:
            if self.stop_on:
                self.hw.stop()
                self.log.log("EMERGENCY_STOP", "NETWORK", source=self.stop_source)
                return "stop"
            if self.reset_on:
                self.hw.stop()
                return "reset"
            if self.paused:
                self.hw.stop()
                self.reset_steer()
                self.publish("paused")
                time.sleep(LOOP_DELAY_S)
                continue
            self.handle_pending()

            snap = self.params.snapshot()
            color = self.hw.read_center_color_now()
            if self.handle_marker(color, "follow"):
                continue

            if (self.ex.mode != "HOME" and not self.grabbed and
                    self.hw.read_distance_cm() < snap["grab_dist_cm"]):
                self.hw.stop()
                self.hw.grip_close(snap["grip_speed"], GRIP_SEC)
                self.grabbed = True
                self.hw.beep_ok()
                self.log.log("GRAB", "ULTRASONIC_NEAR",
                             grab_dist_cm=snap["grab_dist_cm"])
                self.reset_steer()

            rl = self.hw.read_left_reflect()
            rr = self.hw.read_right_reflect()
            bits = node_bits(rl, color, rr, snap)
            now = time.monotonic()

            if bits == LOST_BITS:
                # v13: 000 은 노드 후보가 아니라 유실 의심 — 지속시간 필터를 거친다.
                if abs(self.last_turn) > LOST_GUARD_TURN:
                    # 코너 위빙 중 잠깐 선을 놓친 것 — 의심을 누적하지 않는다.
                    self.lost_since = None
                elif self.lost_since is None:
                    self.lost_since = now
                elif (now - self.lost_since) * 1000 >= snap["lost_persist_ms"]:
                    self.lost_since = None
                    verdict = self.lost_check(snap)
                    self.reset_steer()
                    if verdict == "lost":
                        self.handle_lost()
                    continue
            else:
                self.lost_since = None
                if (bits in NODE_CANDIDATES and
                        (now - self.last_node_t) * 1000 >= self.node_debounce_ms):
                    confirmed, creep_mm = self.confirm_node(bits, snap)
                    self.last_node_t = time.monotonic()
                    # v13.1: 취소는 150ms 만 쉰다 — 취소가 900ms 감지 블라인드를
                    # 만들면 그 사이 분기점을 타넘어 가짜 데드엔드가 된다.
                    self.node_debounce_ms = (NODE_DEBOUNCE_MS
                                             if confirmed is not None
                                             else NODE_CANCEL_DEBOUNCE_MS)
                    self.reset_steer()
                    if confirmed is not None:
                        self.handle_node(confirmed, creep_mm)
                    continue

            # 유실 의심 누적 중에도 감속해 라인 이탈 관성을 줄인다.
            slow = bits in SLOW_ON or self.lost_since is not None
            base = SLOW_SPEED if slow else snap["base_speed"]
            left, right, error, turn = self.pd.step(rl, rr, snap["kp"], base)
            self.hw.drive(left, right)
            self.last_turn = turn

            if now - last_follow_log >= FOLLOW_LOG_S:
                self.log.log("LINE_FOLLOW", "PD", reflect_l=rl, reflect_r=rr,
                             bits=bits_str(bits), error=error,
                             turn=round(turn, 2))
                last_follow_log = now
            self.publish("follow", reflect_l=rl, reflect_r=rr, color=color,
                         bits=bits_str(bits), error=error, turn=round(turn, 2),
                         left_speed=round(left, 1), right_speed=round(right, 1),
                         arrived=self.goal_seen, nodes=len(self.ex.nodes),
                         plan_left=len(self.ex.plan),
                         lost_streak=self.lost_streak)
            time.sleep(LOOP_DELAY_S)
        return "done"

    def idle_after_done(self):
        """완주 후 대기 — 그 자리 노랑에서 무한 재출발하지 않도록 reset 을 기다린다."""
        self.hw.stop()
        while True:
            if self.stop_on:
                self.log.log("EMERGENCY_STOP", "NETWORK", source=self.stop_source)
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            self.publish("finished", nodes=len(self.ex.nodes))
            time.sleep(0.05)

    # ---- 세션 루프 ----

    def new_session(self):
        """탐색 상태를 전부 버리고 새 세션 준비(시작/reset 공용)."""
        self.ex = Explorer()
        self._init_session_state()
        self.reset_steer()
        self.reset_on = False
        self.session += 1
        if self.session == 1:
            self.log.log("SESSION_READY", "STARTUP", session=self.session)
        else:
            self.log.log("SESSION_RESET", "DASHBOARD", source=self.reset_source,
                         session=self.session)

    def run_sessions(self):
        """세션 = 출발 대기 → 탐색/복귀 → 완주 후 대기. reset 은 새 세션, stop 은 종료."""
        while not self.stop_on:
            self.new_session()
            status = self.wait_for_start()
            if status == "go":
                status = self.explore()
            if status == "done":
                status = self.idle_after_done()
            if status == "stop":
                break
            # status == "reset" → 루프 상단에서 new_session.


def run():
    from lib.hardware import Ev3Hardware   # ev3dev2 — 브릭에서만 import 가능

    params = SharedParams(INITIAL_PARAMS, PARAM_LIMITS, MAX_STEP, SAVE_PATH,
                          ui_step=UI_STEP, units=UNITS, param_order=PARAM_ORDER)
    params.load_saved_into_defaults()

    tele = Telemetry()
    log = DecisionLog(telemetry=tele)
    hw = Ev3Hardware()
    hw.read_center_color(COLOR_MODE_SETTLE_S, 1)    # 중앙센서를 컬러 모드로 진입
    runner = Runner(hw, params, tele, log)

    server = TuningServer(params, tele, do_handler=runner.on_do,
                          stop_handler=runner.on_stop, pause_handler=runner.on_pause,
                          actions=ACTIONS, stage=STAGE_NAME)
    server.start()

    print("run_maze_v13 ready. dashboard 'reset' ([r] key) returns to YELLOW start "
          "any time. (Ctrl-C or robotctl stop to quit)")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        try:
            hw.stop()
        finally:
            server.stop()
    print("run_maze_v13 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
