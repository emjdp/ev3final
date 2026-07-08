#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_run1 — run_maze_v13.1 기반, 라인트레이싱 '한쪽 치우침' 구조 수정판.

문제(실기): 라인을 따라가긴 하는데 정상 주행 중에도 항상 한쪽으로 기울어
달린다. v13 까지의 조향은 error = 우반사광 - 좌반사광 '원시값' PD 인데,
이 구조엔 치우침의 원인이 두 개 내장돼 있다.

  원인 A — 센서 개체차: 좌/우 컬러센서는 같은 흰 바닥/같은 검정 위에서도
    원시값이 다르다(장착 높이·LED·개체 편차). 그래서 로봇이 기하학적으로
    라인 중앙에 있어도 error != 0 이고, PD 가 수렴하는 'error = 0' 지점은
    중앙에서 한쪽으로 밀린 지점이 된다. (지금 임계값이 좌 66/우 63,
    좌 18/우 14 로 비대칭 튜닝된 것 자체가 두 센서가 다르다는 증거.)
  원인 B — 정상상태 오차: 좌/우 모터 출력이 조금이라도 다르면 직진에
    0 이 아닌 turn 이 상시 필요하다. P(D) 제어는 error 가 있어야만 turn 을
    내므로, 그 turn 을 만들 만큼의 error 를 '항상 유지한 채' 달린다 =
    라인 중앙에서 일정량 비켜서 추종한다.

final_run1 변경 3가지(하드웨어 동일: 좌/우 반사광 + 중앙 컬러 + 초음파):
  1) 센서별 캘리브레이션 + 정규화(원인 A 제거) — 대시보드 calibrate 액션이
     라인 위에서 좌→우 소각 스윕하며 좌/우 센서 각각의 black/white 실측
     min/max 를 기록한다(cal_* 라이브 파라미터, 성공 시 자동 save). 조향
     에러는 정규화값 norm = 100*(raw-black)/(white-black) 의 차이(R-L)로
     계산해 'error 0 = 실제 라인 중앙'이 되게 한다. 미보정 기본값(0/100)
     이면 정규화가 항등이라 v13 과 동일하게 동작한다(안전한 기본).
  2) PD → PID(원인 B 제거) — 작은 적분항 ki 가 모터 개체차 같은 계통
     편향을 학습해 정상상태 치우침을 0 으로 만든다. windup 가드 3중:
     |error| > INTEG_BAND(커브/노드 진입)에선 적분 동결, 기여는
     ±INTEG_TURN_LIMIT 로 클램프, 회전/복구 후 reset 때는 절반만 남겨
     (모터 편향은 회전해도 그대로이므로) 재수렴을 빠르게 한다.
  3) P soft-deadband — 중앙 근처 미세 에러(|e| <= deadband)는 P 항을 0
     으로 두고(경계 불연속이 없도록 밴드 폭만큼 빼는 연속형), 직진 유지는
     적분이 학습한 트림이 담당한다. 센서 노이즈로 인한 미세 헌팅 제거.

사용법: 라인 위(직선 구간, 센서가 라인 근처)에 로봇을 세우고 대시보드
calibrate 액션 실행 → 소각 스윕 후 beep 2번(성공) / 낮은 tone(실패, 기존값
유지). 이후 kp/ki/deadband 를 라이브 튜닝. ki=0 이면 적분 완전 off.

나머지 동작(노드/마커/유실/탐색/세션)은 run_maze_v13.1 과 동일:
  - 노드/분기 의심 bits 는 PD off → 저속 직진 confirm → 정지 재판정.
    000 은 노드 후보가 아니라 지속 필터를 거치는 유실 의심.
  - 빨강/노랑/초록 마커 즉시 정지 + 부저 2번. 노랑@복귀 = 완주,
    초록@탐색 = 배달, 나머지 = 유턴.
  - 유실 확정 시 후진 → 재정렬(REALIGN) 복구, 연속 한도 초과면 유턴.
  - 세션 루프: 출발 대기(노랑) → 탐색 → 복귀 → 완주 후 대기. reset 은
    언제든 출발 대기로.
  * 노드 bits / 유실 복구 임계값(left_th_* 등)은 v13 그대로 '원시값' 기준
    이다 — 이미 실기 튜닝된 값이라 캘리브레이션과 분리해 유지한다.

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      BACK 버튼 미사용 — 정지는 네트워크 stop 또는 Ctrl-C, 재시작은 네트워크 reset.

실행(브릭):   python3 stages/final_run1.py
문법 점검(PC): python3 -m py_compile stages/final_run1.py lib/*.py
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
# 000(전백)은 노드 후보가 아니라 '유실 의심'으로 따로 다룬다(v13).
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
NODE_CANCEL_DEBOUNCE_MS = 150  # confirm '취소' 후 간격(v13.1)
MARKER_DEBOUNCE_MS = 1500   # 같은 마커 재감지 방지
MARKER_PAUSE_S = 0.08       # 마커 정지 후 잠깐 멈춤
CONFIRM_SETTLE_S = 0.08     # 재판정 전 정지 안정화
GRIP_SEC = 0.8              # 그리퍼 열기/닫기 구동 시간
LOOP_DELAY_S = 0.015
FOLLOW_LOG_S = 0.25         # LINE_FOLLOW 로그 최소 간격
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)

# PID — KD 는 notes 대로 고정, derivative 는 clamp + EMA 로 완화.
PID_KD = 0.05
PID_TURN_LIMIT = 16
PID_DERIV_LIMIT = 220.0
PID_D_EMA_ALPHA = 0.35
# 적분(I) windup 가드(§변경 2). BAND 밖(커브/노드 진입)에선 적분 동결,
# I 항 기여는 ±TURN_LIMIT 의 절반로 제한해 P 를 이기지 못하게 한다.
INTEG_BAND = 25.0           # |error(정규화)| 가 이 이하일 때만 적분
INTEG_TURN_LIMIT = 8.0      # I 항이 turn 에 기여할 수 있는 최대치
INTEG_RESET_KEEP = 0.5      # reset_steer 때 적분 잔존 비율(모터 편향은 회전해도 유지)

# 캘리브레이션 스윕(§변경 1). REALIGN 과 같은 소각 피벗으로 좌/우 센서를
# 라인 위로 쓸어 지나가며 센서별 min(black)/max(white)를 실측한다.
CAL_SPEED = 6               # 스윕 피벗 속도(%)
CAL_HALF_DEG = 60           # 한쪽 스윕 enc deg(로봇 약 40~45도)
CAL_MIN_SPAN = 20           # white-black 이 이보다 좁으면 실패(라인을 못 봤다)

# 소리 구분(notes) — (주파수Hz, 길이ms) 나열. 색 마커는 beep 2번.
TONE_CURVE = ((700, 120),)
TONE_BRANCH = ((600, 100), (900, 100))
TONE_UTURN = ((300, 150), (300, 150))
TONE_CAL_FAIL = ((250, 300),)


# ---------------------------------------------------------------------
# 라이브 파라미터 — 대시보드/robotctl 로 실기에서 튜닝한다.
# ---------------------------------------------------------------------

# 파라미터당 한 줄: (이름, 초기값, min, max, max_step, ui_step, 단위)
# cal_* 4개는 calibrate 액션이 실측치를 직접 기록하므로 max_step=100(제한 없음).
# 기본값 black 0 / white 100 = 정규화 항등(원시값 그대로) → 미보정 시 v13 동작.
PARAM_TABLE = (
    ("base_speed",      10,    5,   45,   5,    1,    "%"),
    ("kp",              0.17,  0.0, 3.0,  0.1,  0.01, ""),
    ("ki",              0.06,  0.0, 0.5,  0.05, 0.01, ""),
    ("deadband",        3.0,   0.0, 20.0, 2.0,  0.5,  ""),
    ("turn_speed",      10,    5,   40,   5,    1,    "%"),
    ("node_confirm_ms", 40,    0,   1000, 60,   10,   "ms"),
    ("left_th_steer",   66,    0,   100,  3,    1,    "%"),   # 유실 복구 검정 판정(원시값)
    ("right_th_steer",  63,    0,   100,  3,    1,    "%"),
    ("left_th_node",    18,    0,   100,  3,    1,    "%"),   # 노드 bits 판정(원시값)
    ("right_th_node",   14,    0,   100,  3,    1,    "%"),
    ("cal_l_black",     0,     0,   100,  100,  1,    "%"),   # calibrate 가 기록
    ("cal_l_white",     100,   0,   100,  100,  1,    "%"),
    ("cal_r_black",     0,     0,   100,  100,  1,    "%"),
    ("cal_r_white",     100,   0,   100,  100,  1,    "%"),
    ("node_advance_mm", 60,    0,   120,  10,   10,   "mm"),  # 의심지점 기준 총 전진
    ("goal_advance_mm", 100,   0,   200,  10,   10,   "mm"),  # 배달 전·후진 거리
    ("turn_90_factor",  0.66,  0.3, 2.0,  0.05, 0.01, "x"),
    ("turn_180_factor", 0.71,  0.3, 2.0,  0.05, 0.01, "x"),
    ("grab_dist_cm",    6.0,   1.0, 20.0, 1.0,  0.5,  "cm"),
    ("grip_speed",      50,    5,   80,   5,    1,    "%"),
    ("lost_persist_ms", 200,   0,   500,  100,  10,   "ms"),  # 000 지속 필터(v13)
    ("lost_max_recover", 3,    1,   3,    1,    1,    ""),    # 유턴 전 복구 허용 횟수(v13)
)

INITIAL_PARAMS = dict((r[0], r[1]) for r in PARAM_TABLE)
PARAM_LIMITS = dict((r[0], (r[2], r[3])) for r in PARAM_TABLE)
MAX_STEP = dict((r[0], r[4]) for r in PARAM_TABLE)
UI_STEP = dict((r[0], r[5]) for r in PARAM_TABLE)
UNITS = dict((r[0], r[6]) for r in PARAM_TABLE)
PARAM_ORDER = [r[0] for r in PARAM_TABLE]

SAVE_PATH = os.path.join(_ROOT, "config", "final_run1.json")
STAGE_NAME = "final_run1"

ACTIONS = [
    {"name": "calibrate", "label": "Calibrate L/R on line (sweep)"},
    {"name": "read_color", "label": "Read Center Color"},
    {"name": "read_reflect", "label": "Read L/R Reflect (raw+norm)"},
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


def normalize(raw, black, white):
    """센서별 캘리브레이션 정규화: black→0, white→100 선형 매핑 후 클램프.

    span 이 CAL_MIN_SPAN 미만(미보정/불량 캘리브레이션)이면 원시값을 그대로
    반환한다 — 잘못된 스케일로 조향이 폭주하는 것보다 v13 동작이 안전하다.
    """
    span = float(white) - float(black)
    if span < CAL_MIN_SPAN:
        return clamp(float(raw), 0.0, 100.0)
    return clamp(100.0 * (float(raw) - float(black)) / span, 0.0, 100.0)


def node_bits(reflect_l, center_color, reflect_r, snap):
    """노드 판정 bits(좌,중,우) — 좌/우는 반사광 '원시값', 중앙은 컬러 black 여부.

    임계값(left/right_th_node)이 이미 원시값 기준으로 실기 튜닝돼 있으므로
    조향 정규화와 분리해 원시값을 유지한다(헤더 주석 참조).
    """
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
# PID 조향(순수) — §변경 2, 3
# ---------------------------------------------------------------------

class PidSteer(object):
    """error = 정규화 우반사광 - 정규화 좌반사광.

    P: soft-deadband — |error| <= deadband 면 0, 넘으면 밴드 폭만큼 뺀 값
       (경계에서 불연속 점프가 없어 D 항을 자극하지 않는다).
    I: |error| <= INTEG_BAND 일 때만 누적(커브/노드 진입 windup 방지),
       기여는 ±INTEG_TURN_LIMIT 클램프. 계통 편향(모터 개체차)을 학습해
       정상상태 치우침을 없애는 것이 목적이라 시정수는 느려도 된다.
    D: v13 그대로 KD 고정 + raw derivative clamp + EMA.
    """

    def __init__(self):
        self.full_reset()

    def full_reset(self):
        """세션 시작/캘리브레이션 후 — 학습된 트림까지 전부 버린다."""
        self.prev_error = 0.0
        self.prev_t = None
        self.deriv = 0.0
        self.integ = 0.0

    def reset(self):
        """회전/노드/복구 후 — P/D 이력은 버리되 적분은 절반 유지.

        적분이 학습한 것은 좌/우 모터 편향(회전해도 그대로인 성질)이므로
        전부 버리면 매 구간 재학습으로 초반 치우침이 되살아난다. 절반만
        남기면 잘못 누적된 값(커브 잔재)도 두 번 리셋이면 거의 사라진다.
        """
        self.prev_error = 0.0
        self.prev_t = None
        self.deriv = 0.0
        self.integ *= INTEG_RESET_KEEP

    def step(self, norm_l, norm_r, snap, base_speed):
        error = float(norm_r - norm_l)

        deadband = snap["deadband"]
        if error > deadband:
            p_error = error - deadband
        elif error < -deadband:
            p_error = error + deadband
        else:
            p_error = 0.0

        t = time.monotonic()
        ki = snap["ki"]
        if self.prev_t is not None:
            dt = max(t - self.prev_t, 0.001)
            raw = clamp((error - self.prev_error) / dt,
                        -PID_DERIV_LIMIT, PID_DERIV_LIMIT)
            self.deriv = (PID_D_EMA_ALPHA * raw +
                          (1.0 - PID_D_EMA_ALPHA) * self.deriv)
            if ki > 0 and abs(error) <= INTEG_BAND:
                limit = INTEG_TURN_LIMIT / ki
                self.integ = clamp(self.integ + error * dt, -limit, limit)

        i_term = ki * self.integ if ki > 0 else 0.0
        turn = clamp(snap["kp"] * p_error + i_term + PID_KD * self.deriv,
                     -PID_TURN_LIMIT, PID_TURN_LIMIT)
        self.prev_error = error
        self.prev_t = t
        return base_speed - turn, base_speed + turn, error, turn, i_term


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
        self.pid = PidSteer()
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
        self.lost_since = None      # 000 연속 시작 시각(지속 필터, v13)
        self.lost_streak = 0        # 윈도 내 연속 복구 횟수(v13)
        self.node_debounce_ms = 0   # 직전 confirm 결과에 따른 재감지 간격(v13.1)

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
        last_reason = self.log.last_reason()
        if last_reason is not None:
            frame["last_reason"] = last_reason
        events = self.log.drain_events()
        if events:
            frame["events"] = events
        self.tele.publish(frame)

    def log_events(self, events):
        for event, rule, detail in events:
            self.log.log(event, rule, **detail)

    def handle_pending(self):
        """대시보드 calibrate / read_color / read_reflect 액션 처리(없으면 no-op)."""
        with self._pending_lock:
            action = self._pending
            self._pending = None
        if action == "calibrate":
            self.calibrate_line()
        elif action == "read_color":
            color = self.hw.read_center_color_now()
            self.log.log("COLOR_READ", "DO_TRIGGER", color=color)
            self.publish("read_color", color=color)
        elif action == "read_reflect":
            snap = self.params.snapshot()
            rl = self.hw.read_left_reflect()
            rr = self.hw.read_right_reflect()
            nl = normalize(rl, snap["cal_l_black"], snap["cal_l_white"])
            nr = normalize(rr, snap["cal_r_black"], snap["cal_r_white"])
            self.log.log("REFLECT_READ", "DO_TRIGGER", reflect_l=rl, reflect_r=rr,
                         norm_l=round(nl, 1), norm_r=round(nr, 1))
            self.publish("read_reflect", reflect_l=rl, reflect_r=rr,
                         norm_l=round(nl, 1), norm_r=round(nr, 1))

    def play(self, tones):
        for freq, ms in tones:
            self.hw.tone(freq, ms)

    def reset_steer(self):
        self.pid.reset()
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

    # ---- 캘리브레이션(§변경 1) ----

    def _cal_sweep(self, direction, max_deg, stats):
        """소각 피벗(L/R)하며 좌/우 반사광 min/max 수집. 진행 enc deg 반환."""
        if max_deg <= 0:
            return 0.0
        left_dir, right_dir = (-1, 1) if direction == "L" else (1, -1)
        self.hw.reset_encoders()
        try:
            self.hw.drive_raw(left_dir * CAL_SPEED, right_dir * CAL_SPEED)
            while self.hw.enc_avg() < max_deg:
                if self.interrupted():
                    break
                rl, rr = self.hw.read_side_reflect()
                if rl < stats["l_min"]:
                    stats["l_min"] = rl
                if rl > stats["l_max"]:
                    stats["l_max"] = rl
                if rr < stats["r_min"]:
                    stats["r_min"] = rr
                if rr > stats["r_max"]:
                    stats["r_max"] = rr
                time.sleep(0.005)
        finally:
            self.hw.stop()
        return self.hw.enc_avg()

    def calibrate_line(self):
        """로봇을 라인(직선 구간) 위에 세워두고 실행 — 좌/우 센서가 라인을
        가로지르도록 좌→우→복귀 소각 스윕하며 센서별 black/white 실측치를
        cal_* 파라미터에 기록한다. 성공: beep 2번 + params.save(현재 라이브
        값 전체 저장) / 실패(어느 쪽이든 span < CAL_MIN_SPAN): 낮은 tone,
        기존 캘리브레이션 유지. 소각 왕복이라 자세는 대략 원위치."""
        self.hw.stop()
        self.log.log("CAL_START", "DO_TRIGGER",
                     half_deg=CAL_HALF_DEG, speed=CAL_SPEED)
        self.publish("calibrating")
        stats = {"l_min": 100.0, "l_max": 0.0, "r_min": 100.0, "r_max": 0.0}
        d1 = self._cal_sweep("L", CAL_HALF_DEG, stats)
        d2 = self._cal_sweep("R", d1 + CAL_HALF_DEG, stats)
        self._cal_sweep("L", max(d2 - d1, 0.0), stats)   # 대략 원위치 복귀
        time.sleep(POST_TURN_SETTLE_S)
        self.reset_steer()
        if self.interrupted():
            return
        l_span = stats["l_max"] - stats["l_min"]
        r_span = stats["r_max"] - stats["r_min"]
        if l_span < CAL_MIN_SPAN or r_span < CAL_MIN_SPAN:
            self.log.log("CAL_FAIL", "SPAN_TOO_NARROW",
                         l_min=stats["l_min"], l_max=stats["l_max"],
                         r_min=stats["r_min"], r_max=stats["r_max"],
                         min_span=CAL_MIN_SPAN)
            self.play(TONE_CAL_FAIL)
            return
        self.params.set("cal_l_black", int(round(stats["l_min"])))
        self.params.set("cal_l_white", int(round(stats["l_max"])))
        self.params.set("cal_r_black", int(round(stats["r_min"])))
        self.params.set("cal_r_white", int(round(stats["r_max"])))
        saved, save_msg = self.params.save()
        self.pid.full_reset()   # 에러 스케일이 바뀌었으므로 학습된 트림도 무효
        self.log.log("CAL_OK", "SWEEP_MIN_MAX",
                     l_black=int(round(stats["l_min"])),
                     l_white=int(round(stats["l_max"])),
                     r_black=int(round(stats["r_min"])),
                     r_white=int(round(stats["r_max"])),
                     saved=saved, save_msg=save_msg)
        self.hw.beep_ok()
        self.hw.beep_ok()

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
        """제자리 회전(L/R/U) + heading 갱신 + PID 리셋. 유턴은 낮은 tone 2번 선행.
        회전각/속도를 바꾸려면 여기(와 params 의 factor)만 보면 된다."""
        # 노드/마커 처리를 마치면 새 구간 — 복구 카운트 이월 방지(v13.1).
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
        # 회전 직후 라인 재획득(v13.1) — 피벗 오차로 중앙이 라인 밖이면
        # 000 유실 체인으로 빠지기 전에 소각 스캔으로 라인에 올라탄다.
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
        """의심지점: PID off → 저속 직진(node_confirm_ms) → 정지 후 재판정.
        (확정 bits 또는 None, 그동안 전진한 mm) 반환. 마커를 만나면 처리 후 None.
        000 은 여기로 들어오지 않는다 — creep/재판정 중 000 이 보이면 취소(v13)."""
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
                    # 좌/우 팔이 있던 후보(111/101)가 전백이 됐다 = T/십자의
                    # 가로선을 지나쳤다는 확실한 증거(v13.1 passed-over).
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
            # 정지 재판정에서 전백 — 위 creep 케이스와 같은 passed-over(v13.1).
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
        bits 는 NODE_CANDIDATES 멤버만 온다(000 은 유실 경로로 분리, v13)."""
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
        유실 상황의 저속 전진 confirm 은 라인에서 더 멀어져 오판을 굳히므로
        전진 없이 판정한다. 한 샘플이라도 라인이 보이면 취소.
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

        어두운 쪽부터 스캔하고, 양쪽 다 실패하면 원래 방향으로 복원 후
        False(호출부가 막다른길 처리). 소각이라 heading 격자는 안 바꾼다."""
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
        """노랑 위에 놓일 때까지 대기 → beep → 출발선 이탈 전진. status 반환.

        대기 중에도 handle_pending 이 돌므로 이 상태에서 calibrate 를 실행하면
        된다(로봇을 라인 위로 옮겨 놓고 액션 → beep 2번 후 노랑 위로 복귀)."""
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
                # 000 은 노드 후보가 아니라 유실 의심 — 지속시간 필터(v13).
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
                    # 취소는 150ms 만 쉰다 — 900ms 감지 블라인드가 분기점을
                    # 타넘는 가짜 데드엔드를 만들기 때문(v13.1).
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
            # §변경 1: 조향 에러는 캘리브레이션 정규화값으로 계산한다.
            norm_l = normalize(rl, snap["cal_l_black"], snap["cal_l_white"])
            norm_r = normalize(rr, snap["cal_r_black"], snap["cal_r_white"])
            left, right, error, turn, trim = self.pid.step(norm_l, norm_r,
                                                           snap, base)
            self.hw.drive(left, right)
            self.last_turn = turn

            if now - last_follow_log >= FOLLOW_LOG_S:
                self.log.log("LINE_FOLLOW", "PID", reflect_l=rl, reflect_r=rr,
                             norm_l=round(norm_l, 1), norm_r=round(norm_r, 1),
                             bits=bits_str(bits), error=round(error, 2),
                             turn=round(turn, 2), trim=round(trim, 2))
                last_follow_log = now
            self.publish("follow", reflect_l=rl, reflect_r=rr, color=color,
                         norm_l=round(norm_l, 1), norm_r=round(norm_r, 1),
                         bits=bits_str(bits), error=round(error, 2),
                         turn=round(turn, 2), trim=round(trim, 2),
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
        self.pid.full_reset()   # 이전 세션의 학습 트림까지 버리고 새로 학습
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

    print("final_run1 ready. put robot ON the line and run dashboard 'calibrate' "
          "first, then place on YELLOW to start. dashboard 'reset' ([r] key) "
          "returns to start any time. (Ctrl-C or robotctl stop to quit)")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        try:
            hw.stop()
        finally:
            server.stop()
    print("final_run1 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
