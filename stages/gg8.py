#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gg8 — 최후의 코드: gg5 주행층 + 대시보드 실시간 조종(자율 판단 전부 제거).

계획 변경(2026-07-10): 자율 탐색(Explorer/복귀계획)은 실기 오류가 누적돼
회생 불가 판정. gg8 은 검증된 주행층만 남기고 '판단'을 전부 사람에게 넘긴다.
로봇은 라인을 타고, 분기를 감지해 멈췄다 미끄러지며 확정하고, 그 자리에서
대시보드가 준 이동 명령(left/straight/right/uturn)을 실행한다.

역할 분담:
  로봇(자동) — gg5 실기 검증분 그대로:
    - PID 라인트레이싱(캘리브레이션 정규화 + 커브 자동 감속 + 재출발 가속
      램프 + 속도 비례 조향) / 교차로 접근 가드
    - 노드 의심점: 정지 → 저속 직진(미끄러지며) confirm → 정지 재판정
    - 유실(000): 지속 필터 → 정지 재판정 → 후진+소각 재정렬 복구
    - 물체 자동 파지(초음파 grab_dist 이내, 그립 비었을 때)
  사람(대시보드) — 모든 '결정':
    - 결정 지점 = 커브(출구 1개)/분기(출구 2+)/마커(빨강·초록·노랑 확정)/
      막다른길 — 노드로 확정된 곳은 전부 멈추고 명령을 기다린다(자동
      통과 없음). 커브도 로봇이 판단하지 않는다 — 텔레메트리 exits 를
      보고 사람이 방향 키를 누른다.
    - 정지는 감속 정지(soft stop): 순항 속도에서 급브레이크 대신 SLOW 로
      한 템포 줄인 뒤 선다(앞쏠림/미끄러짐으로 인한 마커 이탈 방지).
    - 이동 명령은 언제든 미리 넣어둘 수 있다(1칸 큐, 마지막 입력 승리).
      주행 중/미끄러지는 confirm 중에 눌러두면 결정 지점 도착 즉시 소비.
    - 큐가 비어 있으면 로봇은 그 자리에 정지해 tone 을 울리고 명령을
      기다린다(await_cmd 모드 — 텔레메트리로 상황(bits/색/출구) 방송).
    - 그리퍼 열기/닫기, 전/후 nudge(배달 위치 잡기)도 대시보드 액션.

대시보드 키(액션 등록 순서 = 핫키 순서):
  [1] left   [2] straight   [3] right   [4] uturn   [5] clear cmd
  [6] grip open  [7] grip close  [8] nudge fwd  [9] nudge back  [0] GO(출발)
  [d] calibrate  [f] read reflect  [h] read color  (reset 은 [r] 공용 키)
  이동 명령/GO 는 네트워크 스레드에서 즉시 반영(블로킹 없음) — 나머지는
  제어 루프가 안전한 시점에 처리한다.

명령 큐 규약(CommandBox):
  - 1칸, 마지막 입력이 이전 입력을 덮는다(잘못 눌렀으면 다시 누르면 됨).
  - clear 로 비울 수 있다. 현재 큐는 매 텔레메트리 프레임 pending_cmd 로
    방송된다(대시보드 상단에 표시).
  - 소비 시점: 커브/분기 확정 후 / 마커 색 확정 후 / 막다른길 확정 후 —
    모든 정지 지점이 명령 하나를 소비한다(유실 복구만 소비하지 않는다).
  - straight 명령은 회전 없이 그대로 직진 재개(램프부터).
  - 명령은 출구 유효성 검사를 하지 않는다 — 조종자가 보스다(유령 팔
    오인식을 사람이 눈으로 덮어쓰는 것이 gg8 의 존재 이유). 가능한 출구와
    다르면 로그에 흔적만 남긴다.

출발/종료:
  - 출발: 브릭 가운데 버튼 또는 대시보드 [0] GO. 출발 직진(노랑 이탈) 후
    라인트레이싱 시작.
  - 마커에서도 결정 지점으로 멈추므로, 노랑 복귀(완주)면 그 자리에서
    [7] grip open 으로 물체를 내려놓고 [s] stop 또는 [r] reset.
  - 완주 자동 판정 없음 — 세션은 stop/reset 까지 계속된다.

파라미터 승계: config/gg8.json 이 없으면 gg5.json → gg3.json 순으로 실기
튜닝값을 승계한다(gg3 에서 가져올 땐 속도 계열 제외). 트랙이 바뀌었으면
대시보드 calibrate 1회.

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      정지는 네트워크 stop 또는 Ctrl-C, 재시작은 가운데 버튼 또는 reset.

실행(브릭):   python3 stages/gg8.py
문법 점검(PC): python3 -m py_compile stages/gg8.py lib/*.py
단위 테스트:  python3 tests/test_gg8_logic.py (ev3dev2 불필요)
"""

import json
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
# 상수 — gg5 주행층 그대로(자율 판단용 상수만 제거)
# ---------------------------------------------------------------------

COL_NONE = 0
COL_BLACK = 1
COL_GREEN = 3
COL_YELLOW = 4
COL_RED = 5
COL_WHITE = 6
COL_BROWN = 7

MARKER_COLORS = (COL_RED, COL_YELLOW, COL_GREEN)
MARKER_NAMES = {COL_RED: "red", COL_YELLOW: "yellow", COL_GREEN: "green"}

# 노드 후보 bits(좌,중,우): 좌회전 / 우회전 / 십자 / T(직진 없음).
NODE_CANDIDATES = ((1, 1, 0), (0, 1, 1), (1, 1, 1), (1, 0, 1))
LOST_BITS = (0, 0, 0)
SLOW_ON = ((1, 1, 1), (1, 0, 1))    # 이 패턴이 스치면 확정 전이라도 감속

# 기하(정리.md 엔코더/기하값)
MM_PER_DEG = 3.14159265 * 56.0 / 360.0      # 바퀴 지름 56mm
BASE_PIVOT_DEG_90 = 193.0
BASE_PIVOT_DEG_180 = 386.0
POST_TURN_SETTLE_S = 0.12

# 고정 주행값(라이브 튜닝 대상 아님)
STRAIGHT_SPEED = 20         # 출발 이탈 / nudge 전·후진(hw 램프로 출발 완화)
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
NODE_CANCEL_DEBOUNCE_MS = 150  # confirm '취소' 후 간격
NODE_GUARD_MARGIN = 12      # 반사광 < 노드임계+마진 이면 가로선 접근으로 본다
NODE_GUARD_MAX_MS = 700     # 직진 유지 상한 — 초과 지속이면 가로선이 아니라 정렬 불량
MARKER_DEBOUNCE_MS = 1500   # 같은 마커 재감지 방지
MARKER_PAUSE_S = 0.08       # 마커 정지 후 잠깐 멈춤
CONFIRM_SETTLE_S = 0.08     # 재판정 전 정지 안정화
GRIP_SEC = 0.8              # 그리퍼 열기/닫기 구동 시간
LOOP_DELAY_S = 0.010        # gg5: 30% 순항의 프레임당 이동거리 보상
FOLLOW_LOG_S = 0.25         # LINE_FOLLOW 로그 최소 간격
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)
AWAIT_POLL_S = 0.05         # 결정 대기 중 명령 폴링 간격
SOFT_STOP_S = 0.12          # 감속 정지: SLOW_SPEED 로 줄여 이만큼 굴린 뒤 brake

# 마커 확정 — 색은 1프레임이 아니라 정지 후 재판독 다수결로 확정한다.
MARKER_CONFIRM_SAMPLES = 4  # 정지 후 재판독 횟수
MARKER_CONFIRM_MIN = 3      # 같은 마커 색이 이만큼 이상이어야 확정
MARKER_CONFIRM_GAP_S = 0.015

# PID — KD 는 notes 대로 고정, derivative 는 clamp + EMA 로 완화.
PID_KD = 0.05
PID_TURN_LIMIT = 16
PID_DERIV_LIMIT = 220.0
PID_D_EMA_ALPHA = 0.35
INTEG_BAND = 25.0           # |error(정규화)| 가 이 이하일 때만 적분
INTEG_TURN_LIMIT = 8.0      # I 항이 turn 에 기여할 수 있는 최대치
INTEG_RESET_KEEP = 0.5      # reset_steer 때 적분 잔존 비율

# 고속 적응(gg5) — 커브 감속/가속 램프/속도 비례 조향 상수.
SPEED_REF = 20.0            # 조향(kp/ki/deadband) 실기 튜닝 기준 속도(%)
STEER_SCALE_MIN = 0.75      # 속도 비례 조향 배율 하한
STEER_SCALE_MAX = 2.0       # 배율 상한
ACCEL_START_SPEED = 12      # 출발 가속 램프 시작 속도(%)
CORNER_MIN_SPEED = 12       # 커브 자동 감속 하한(%)
ERR_EMA_ALPHA = 0.30        # 커브 감속용 |error| EMA 계수
STRAIGHT_RAMP_MS = 250      # 직진 프리미티브 하드웨어 가속 램프(ms)

# 캘리브레이션 스윕(gg5 동일).
CAL_SPEED = 6
CAL_HALF_DEG = 60
CAL_MIN_SPAN = 20

# 소리 신호(gg8) — 조종자 알림용 tone(비동기 큐, 주행 비블로킹).
GRAB_TONE = (880, 150)      # 물체 파지
AWAIT_TONE = (660, 250)     # 결정 대기 진입(명령 주세요)
CMD_TONE = (990, 80)        # 명령 소비(실행 시작)


# ---------------------------------------------------------------------
# 라이브 파라미터 — gg5 표 그대로(전부 대시보드에서 실시간 조절).
# ---------------------------------------------------------------------

PARAM_TABLE = (
    ("base_speed",      30,    5,   60,   5,    1,    "%"),
    ("kp",              0.17,  0.0, 3.0,  0.1,  0.01, ""),
    ("ki",              0.06,  0.0, 0.5,  0.05, 0.01, ""),
    ("deadband",        3.0,   0.0, 20.0, 2.0,  0.5,  ""),
    ("turn_speed",      8,     5,   40,   5,    1,    "%"),
    ("accel_ramp_ms",   700,   0,   2000, 200,  50,   "ms"),
    ("corner_gain",     0.5,   0.0, 2.0,  0.25, 0.05, ""),
    ("turn_ramp_ms",    250,   0,   600,  100,  50,   "ms"),
    ("node_confirm_ms", 40,    0,   1000, 60,   10,   "ms"),
    ("left_th_steer",   66,    0,   100,  3,    1,    "%"),
    ("right_th_steer",  63,    0,   100,  3,    1,    "%"),
    ("left_th_node",    35,    0,   100,  3,    1,    "%"),
    ("right_th_node",   30,    0,   100,  3,    1,    "%"),
    ("cal_l_black",     0,     0,   100,  100,  1,    "%"),
    ("cal_l_white",     100,   0,   100,  100,  1,    "%"),
    ("cal_r_black",     0,     0,   100,  100,  1,    "%"),
    ("cal_r_white",     100,   0,   100,  100,  1,    "%"),
    ("node_advance_mm", 60,    0,   120,  10,   10,   "mm"),
    ("goal_advance_mm", 100,   0,   200,  10,   10,   "mm"),  # nudge 전·후진 거리
    ("turn_90_factor",  0.65,  0.3, 2.0,  0.05, 0.01, "x"),
    ("turn_180_factor", 0.71,  0.3, 2.0,  0.05, 0.01, "x"),
    ("grab_dist_cm",    6.0,   1.0, 20.0, 1.0,  0.5,  "cm"),
    ("grip_speed",      50,    5,   80,   5,    1,    "%"),
    ("lost_persist_ms", 200,   0,   500,  100,  10,   "ms"),
    ("lost_max_recover", 3,    1,   3,    1,    1,    ""),
)

INITIAL_PARAMS = dict((r[0], r[1]) for r in PARAM_TABLE)
PARAM_LIMITS = dict((r[0], (r[2], r[3])) for r in PARAM_TABLE)
MAX_STEP = dict((r[0], r[4]) for r in PARAM_TABLE)
UI_STEP = dict((r[0], r[5]) for r in PARAM_TABLE)
UNITS = dict((r[0], r[6]) for r in PARAM_TABLE)
PARAM_ORDER = [r[0] for r in PARAM_TABLE]

SAVE_PATH = os.path.join(_ROOT, "config", "gg8.json")
STAGE_NAME = "gg8"

# 파라미터 승계 순서: gg8.json 없으면 gg5.json → gg3.json.
# gg3 에서 가져올 땐 속도 계열은 gg8 기본값(고속) 유지.
SEED_SOURCES = (
    (os.path.join(_ROOT, "config", "gg5.json"), ()),
    (os.path.join(_ROOT, "config", "gg3.json"), ("base_speed", "turn_speed")),
)

# 이동 명령 액션 → move 문자. 등록 순서가 대시보드 핫키(1..0,d,f,h) 순서다.
MOVE_ACTIONS = {"left": "L", "straight": "S", "right": "R", "uturn": "U"}

ACTIONS = [
    {"name": "left", "label": "CMD Left"},
    {"name": "straight", "label": "CMD Straight"},
    {"name": "right", "label": "CMD Right"},
    {"name": "uturn", "label": "CMD U-Turn"},
    {"name": "clear", "label": "CMD Clear"},
    {"name": "grip_open", "label": "Grip Open"},
    {"name": "grip_close", "label": "Grip Close"},
    {"name": "nudge_fwd", "label": "Nudge Fwd (goal mm)"},
    {"name": "nudge_back", "label": "Nudge Back (goal mm)"},
    {"name": "go", "label": "GO (start)"},
    {"name": "calibrate", "label": "Calibrate L/R on line (sweep)"},
    {"name": "read_reflect", "label": "Read L/R Reflect (raw+norm)"},
    {"name": "read_color", "label": "Read Center Color"},
    {"name": "reset", "label": "Reset to Start"},
]


# ---------------------------------------------------------------------
# 순수 헬퍼(gg5 동일)
# ---------------------------------------------------------------------

def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def normalize(raw, black, white):
    """센서별 캘리브레이션 정규화: black→0, white→100 선형 매핑 후 클램프."""
    span = float(white) - float(black)
    if span < CAL_MIN_SPAN:
        return clamp(float(raw), 0.0, 100.0)
    return clamp(100.0 * (float(raw) - float(black)) / span, 0.0, 100.0)


def on_line(center_color):
    """중앙 컬러센서가 '라인 위'인가 — 흰 바닥이 아니면 전부 라인(gg3 실측)."""
    return center_color != COL_WHITE


def node_bits(reflect_l, center_color, reflect_r, snap):
    """노드 판정 bits(좌,중,우) — 좌/우는 반사광 원시값, 중앙은 라인 위 여부."""
    return (1 if reflect_l < snap["left_th_node"] else 0,
            1 if on_line(center_color) else 0,
            1 if reflect_r < snap["right_th_node"] else 0)


def bits_str(bits):
    return "{}{}{}".format(bits[0], bits[1], bits[2])


def adaptive_base(target, corner_gain, err_ema, ramp_ms, elapsed_ms):
    """gg5 순항속도(순수): 커브 자동 감속 + 출발 가속 램프."""
    lo = min(float(CORNER_MIN_SPEED), float(target))
    base = clamp(float(target) - float(corner_gain) * float(err_ema),
                 lo, float(target))
    if ramp_ms > 0 and elapsed_ms < ramp_ms:
        cap = (ACCEL_START_SPEED +
               (base - ACCEL_START_SPEED) * float(elapsed_ms) / float(ramp_ms))
        base = min(base, cap)
    return base


# ---------------------------------------------------------------------
# CommandBox — 대시보드 이동 명령 1칸 큐(마지막 입력 승리, 스레드 안전)
# ---------------------------------------------------------------------

class CommandBox(object):
    """네트워크 스레드가 set(), 제어 루프가 결정 지점에서 take().

    1칸 큐: 새 명령이 이전 명령을 덮는다(오입력은 다시 누르면 정정).
    peek() 는 텔레메트리 방송용(소비하지 않음), clear() 는 [5] 키.
    take() 는 (move, 대기했던 초) — 없으면 (None, None).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._move = None
        self._set_t = None

    def set(self, move):
        with self._lock:
            prev = self._move
            self._move = move
            self._set_t = time.monotonic()
            return prev

    def clear(self):
        with self._lock:
            prev = self._move
            self._move = None
            self._set_t = None
            return prev

    def peek(self):
        with self._lock:
            return self._move

    def take(self):
        with self._lock:
            move = self._move
            set_t = self._set_t
            self._move = None
            self._set_t = None
        if move is None:
            return None, None
        return move, time.monotonic() - set_t


# ---------------------------------------------------------------------
# PID 조향(gg5 동일 — 속도 비례 조향 포함)
# ---------------------------------------------------------------------

class PidSteer(object):
    """error = 정규화 우반사광 - 정규화 좌반사광. gg5 속도 비례 조향 포함."""

    def __init__(self):
        self.full_reset()

    def full_reset(self):
        self.prev_error = 0.0
        self.prev_t = None
        self.deriv = 0.0
        self.integ = 0.0

    def reset(self):
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
        scale = clamp(base_speed / SPEED_REF, STEER_SCALE_MIN, STEER_SCALE_MAX)
        turn = clamp(scale * (snap["kp"] * p_error + i_term
                              + PID_KD * self.deriv),
                     -PID_TURN_LIMIT * scale, PID_TURN_LIMIT * scale)
        self.prev_error = error
        self.prev_t = t
        return base_speed - turn, base_speed + turn, error, turn, i_term


# ---------------------------------------------------------------------
# 구동 러너 — 주행은 자동, 결정은 대시보드. run() 에서만 생성.
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
        self._pending = []              # 제어 루프에서 실행할 액션 FIFO
        self._pending_lock = threading.Lock()

        self.cmd = CommandBox()         # 이동 명령(즉시 반영, 결정 지점 소비)
        self.session = 0
        self.pid = PidSteer()
        self._init_session_state()

    def _init_session_state(self):
        self.visits = 0             # 빨강 확정 횟수(참고 카운트)
        self.grabbed = False
        self.go_on = False          # 대시보드 GO 출발 플래그
        self.last_turn = 0.0
        self.last_marker_t = -1e9
        self.last_node_t = -1e9
        self.last_recover_t = -1e9
        self.lost_since = None
        self.guard_since = None
        self.lost_streak = 0
        self.node_debounce_ms = 0
        self.err_ema = 0.0
        self.ramp_t0 = -1e9

    # ---- 네트워크 핸들러(TuningServer 스레드에서 호출) ----

    def on_stop(self, source):
        self.stop_on = True
        self.stop_source = source

    def on_pause(self, paused, source):
        self.paused = bool(paused)
        self.log.log("PAUSE" if paused else "RESUME", "NETWORK", source=source)
        return {"mode": "paused" if paused else "run"}

    def on_do(self, action, args):
        """대시보드 액션 분기 — 이동 명령/GO/clear 는 여기(네트워크 스레드)서
        즉시 반영한다(제어 루프 개입 불필요, '미끄러지는 동안 입력' 보장).
        모터를 움직이는 액션(grip/nudge/calibrate)은 제어 루프 FIFO 로 넘긴다."""
        if action in MOVE_ACTIONS:
            move = MOVE_ACTIONS[action]
            prev = self.cmd.set(move)
            self.log.log("CMD_SET", "DASHBOARD", move=move,
                         replaced=prev if prev else "-")
            return {"queued_move": move, "replaced": prev}
        if action == "clear":
            prev = self.cmd.clear()
            self.log.log("CMD_CLEAR", "DASHBOARD",
                         cleared=prev if prev else "-")
            return {"cleared": prev}
        if action == "go":
            self.go_on = True
            self.log.log("GO", "DASHBOARD")
            return {"queued": "go"}
        if action == "reset":
            self.reset_on = True
            self.reset_source = (args or {}).get("source", "dashboard")
            return {"queued": "reset"}
        with self._pending_lock:
            self._pending.append(action)
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
            "visits": self.visits,
            "grabbed": self.grabbed,
            "pending_cmd": self.cmd.peek() or "-",
        }
        frame.update(extra)
        last_reason = self.log.last_reason()
        if last_reason is not None:
            frame["last_reason"] = last_reason
        events = self.log.drain_events()
        if events:
            frame["events"] = events
        self.tele.publish(frame)

    def handle_pending(self):
        """제어 루프 시점 액션 처리(그립/nudge/캘리브레이션/판독)."""
        while True:
            with self._pending_lock:
                if not self._pending:
                    return
                action = self._pending.pop(0)
            snap = self.params.snapshot()
            if action == "calibrate":
                self.calibrate_line()
            elif action == "grip_open":
                self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
                self.grabbed = False
                self.log.log("GRIP", "OPEN_DASHBOARD")
            elif action == "grip_close":
                self.hw.grip_close(snap["grip_speed"], GRIP_SEC)
                self.grabbed = True
                self.log.log("GRIP", "CLOSE_DASHBOARD")
            elif action == "nudge_fwd":
                self.log.log("NUDGE", "FWD", mm=snap["goal_advance_mm"])
                self.straight(snap["goal_advance_mm"], STRAIGHT_SPEED,
                              mode="nudge")
                self.reset_steer()
            elif action == "nudge_back":
                self.log.log("NUDGE", "BACK", mm=snap["goal_advance_mm"])
                self.straight(snap["goal_advance_mm"], -STRAIGHT_SPEED,
                              mode="nudge")
                self.reset_steer()
            elif action == "read_color":
                color = self.hw.read_center_color_now()
                self.log.log("COLOR_READ", "DO_TRIGGER", color=color)
                self.publish("read_color", color=color)
            elif action == "read_reflect":
                rl = self.hw.read_left_reflect()
                rr = self.hw.read_right_reflect()
                nl = normalize(rl, snap["cal_l_black"], snap["cal_l_white"])
                nr = normalize(rr, snap["cal_r_black"], snap["cal_r_white"])
                self.log.log("REFLECT_READ", "DO_TRIGGER",
                             reflect_l=rl, reflect_r=rr,
                             norm_l=round(nl, 1), norm_r=round(nr, 1))
                self.publish("read_reflect", reflect_l=rl, reflect_r=rr,
                             norm_l=round(nl, 1), norm_r=round(nr, 1))

    def soft_stop(self):
        """감속 정지 — 순항 속도에서 급브레이크 대신 SLOW 로 한 템포 줄여
        관성을 빼고 선다(앞쏠림/미끄러짐으로 마커를 지나치는 것 방지).
        이미 저속(creep 등)이면 사실상 즉시 정지와 같다."""
        self.hw.drive(SLOW_SPEED, SLOW_SPEED)
        time.sleep(SOFT_STOP_S)
        self.hw.stop()

    def reset_steer(self):
        self.pid.reset()
        self.last_turn = 0.0
        self.lost_since = None
        self.guard_since = None
        self.err_ema = 0.0
        self.ramp_t0 = time.monotonic()     # 재출발 가속 램프 시작(gg5)

    def _hold_while_paused(self, mode):
        self.hw.stop()
        while self.paused and not self.interrupted():
            self.publish(mode + "_paused")
            time.sleep(0.05)

    def read_bits(self, snap):
        color = self.hw.read_center_color_now()
        rl = self.hw.read_left_reflect()
        rr = self.hw.read_right_reflect()
        return node_bits(rl, color, rr, snap), color, rl, rr

    # ---- 캘리브레이션(gg5 동일) ----

    def _cal_sweep(self, direction, max_deg, stats):
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
        self.hw.stop()
        self.log.log("CAL_START", "DO_TRIGGER",
                     half_deg=CAL_HALF_DEG, speed=CAL_SPEED)
        self.publish("calibrating")
        stats = {"l_min": 100.0, "l_max": 0.0, "r_min": 100.0, "r_max": 0.0}
        d1 = self._cal_sweep("L", CAL_HALF_DEG, stats)
        d2 = self._cal_sweep("R", d1 + CAL_HALF_DEG, stats)
        self._cal_sweep("L", max(d2 - d1, 0.0), stats)
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
            return
        self.params.set("cal_l_black", int(round(stats["l_min"])))
        self.params.set("cal_l_white", int(round(stats["l_max"])))
        self.params.set("cal_r_black", int(round(stats["r_min"])))
        self.params.set("cal_r_white", int(round(stats["r_max"])))
        saved, save_msg = self.params.save()
        self.pid.full_reset()
        self.log.log("CAL_OK", "SWEEP_MIN_MAX",
                     l_black=int(round(stats["l_min"])),
                     l_white=int(round(stats["l_max"])),
                     r_black=int(round(stats["r_min"])),
                     r_white=int(round(stats["r_max"])),
                     saved=saved, save_msg=save_msg)

    # ---- 모션 프리미티브(gg5 동일, heading 추적만 제거) ----

    def straight(self, dist_mm, speed, mode="advancing"):
        """엔코더 기준 직진(speed<0 후진). hw 가속 램프로 부드럽게 출발."""
        self.hw.reset_encoders()
        if dist_mm <= 0:
            return 0.0
        target_deg = dist_mm / MM_PER_DEG
        self.hw.set_ramp(STRAIGHT_RAMP_MS)
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
            self.hw.set_ramp(0)
        return self.hw.enc_avg() * MM_PER_DEG

    def backup_to_line(self, max_mm, snap):
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
                if (on_line(color) or rl < snap["left_th_steer"] or
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
        """제자리 회전(L/R/U) + PID 리셋 + 회전 직후 라인 재획득(gg5 동일)."""
        self.lost_streak = 0
        if move == "S":
            return
        snap = self.params.snapshot()
        if move == "U":
            target = BASE_PIVOT_DEG_180 * snap["turn_180_factor"]
        else:
            target = BASE_PIVOT_DEG_90 * snap["turn_90_factor"]
        left_dir, right_dir = (-1, 1) if move == "L" else (1, -1)
        speed = snap["turn_speed"]
        self.hw.coast()
        self.hw.reset_encoders()
        self.hw.set_ramp(snap["turn_ramp_ms"])
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
            self.hw.set_ramp(0)
        actual = self.hw.enc_avg()
        time.sleep(POST_TURN_SETTLE_S)
        self.log.log("TURN", {"L": "TURN_LEFT", "R": "TURN_RIGHT",
                              "U": "UTURN"}[move],
                     target_deg=round(target, 1), enc_avg=round(actual, 1),
                     error_deg=round(actual - target, 1),
                     stopped_early=self.interrupted())
        self.reset_steer()
        if not self.interrupted():
            color = self.hw.read_center_color_now()
            if not on_line(color):
                self.log.log("TURN_ACQUIRE", "CENTER_OFF_LINE_AFTER_TURN",
                             move=move, color=color)
                self.realign_to_line(self.params.snapshot())

    # ---- 결정 지점(gg8 핵심) ----

    def decide_and_move(self, kind, has_left, has_right, has_straight,
                        color=None):
        """분기/마커/막다른길 도착 — 대시보드 명령을 소비해 실행한다.

        큐에 명령이 있으면(주행/미끄러지는 중 미리 입력) 즉시 실행, 없으면
        정지 상태로 tone 을 울리고 기다린다. 명령은 출구 유효성 검사 없이
        그대로 실행한다(조종자가 보스) — 다만 가능한 출구와 다르면 로그에
        흔적을 남긴다. 중단(stop/reset)이면 아무 것도 하지 않는다."""
        avail = {"L": has_left, "R": has_right, "S": has_straight, "U": True}
        move, waited = self.cmd.take()
        source = "QUEUED"
        if move is None:
            move = self.await_command(kind, has_left, has_right, has_straight,
                                      color)
            source = "AWAITED"
            if move is None:
                return              # stop/reset 중단
        self.hw.tone(CMD_TONE[0], CMD_TONE[1])
        self.log.log("DECISION", kind.upper(), move=move, source=source,
                     queued_s=(round(waited, 2) if waited is not None else None),
                     has_left=has_left, has_right=has_right,
                     has_straight=has_straight,
                     off_menu=(not avail.get(move, True)))
        self.turn(move)             # "S" 는 no-op — 그대로 직진 재개
        if move == "S":
            self.reset_steer()      # 직진 재개도 가속 램프부터

    def await_command(self, kind, has_left, has_right, has_straight, color):
        """정지 대기 — 이동 명령이 올 때까지 상황을 방송하며 기다린다.

        대기 중에도 handle_pending 이 돌므로 grip/nudge/calibrate/판독을
        먼저 실행해 놓고 이동 명령을 마지막에 넣으면 된다(초록 배달 절차).
        반환: move 또는 None(stop/reset)."""
        self.hw.stop()
        self.hw.tone(AWAIT_TONE[0], AWAIT_TONE[1])
        self.log.log("AWAIT_CMD", kind.upper(),
                     has_left=has_left, has_right=has_right,
                     has_straight=has_straight,
                     color=(color if color is not None else "-"))
        while True:
            if self.interrupted():
                return None
            if self.paused:
                self._hold_while_paused("await_cmd")
                continue
            self.handle_pending()
            move, waited = self.cmd.take()
            if move is not None:
                return move
            self.publish("await_cmd", await_kind=kind,
                         has_left=has_left, has_right=has_right,
                         has_straight=has_straight,
                         color=(color if color is not None else "-"))
            time.sleep(AWAIT_POLL_S)

    # ---- 마커 / 노드 처리 ----

    def _confirm_marker_color(self, first):
        votes = {}
        for _i in range(MARKER_CONFIRM_SAMPLES):
            time.sleep(MARKER_CONFIRM_GAP_S)
            c = self.hw.read_center_color_now()
            votes[c] = votes.get(c, 0) + 1
        best = None
        for c in votes:
            if best is None or votes[c] > votes[best]:
                best = c
        if best in MARKER_COLORS and votes[best] >= MARKER_CONFIRM_MIN:
            return best
        return None

    def handle_marker(self, color, context):
        """빨강/노랑/초록 마커: 정지 → 색 확정 → 결정 지점. 처리했으면 True.

        미션 자동 분기는 없다 — 확정 색을 방송하고 조종자의 명령을 따른다.
        (빨강 = 보통 uturn / 초록 = nudge+grip open 후 uturn / 노랑 복귀 =
        grip open 후 stop — 전부 대시보드에서 사람이 실행.)"""
        if color not in MARKER_COLORS:
            return False
        if (time.monotonic() - self.last_marker_t) * 1000 < MARKER_DEBOUNCE_MS:
            return False

        self.soft_stop()            # 순항 중 마커 — 감속 정지(급브레이크 X)
        time.sleep(MARKER_PAUSE_S)
        color = self._confirm_marker_color(color)
        if color is None:
            self.ramp_t0 = time.monotonic()     # 재개는 가속 램프부터
            self.log.log("MARKER_REJECT", "UNCONFIRMED", context=context)
            return False
        name = MARKER_NAMES[color]
        if color == COL_RED:
            self.visits += 1
        self.log.log("MARKER", "COLOR_{}_CONFIRMED".format(name.upper()),
                     color=color, context=context, visits=self.visits,
                     session=self.session)
        self.decide_and_move(name, False, False, False, color=color)
        self.last_marker_t = time.monotonic()
        return True

    def confirm_node(self, first_bits, snap):
        """의심지점: PID off → 저속 직진(미끄러지며) → 정지 후 재판정(gg5 동일).

        이 creep 동안에도 네트워크 스레드가 CommandBox 를 채울 수 있다 —
        '미끄러지는 동안 입력'이 여기서 성립한다."""
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
        """확정 노드: 총 전진을 node_advance_mm 로 맞춘 뒤 결정 지점으로.

        커브(출구 1개)든 분기(출구 2+)든 자동 통과 없이 전부 멈춰서 명령을
        소비/대기한다 — 로봇은 출구 모양(exits)만 방송하고 방향은 사람이
        정한다. 커브의 뻔한 출구는 로그에 suggest 로만 남긴다."""
        snap = self.params.snapshot()
        self.straight(max(0.0, snap["node_advance_mm"] - creep_mm), CONFIRM_SPEED)
        if self.interrupted():
            return

        color = self.hw.read_center_color_now()
        if self.handle_marker(color, "after_node_advance"):
            return

        has_left = bits[0] == 1
        has_right = bits[2] == 1
        has_straight = on_line(color)
        n_exits = int(has_left) + int(has_right) + int(has_straight)

        if n_exits == 0:
            self.log.log("DEAD_END", "NO_EXIT_AFTER_ADVANCE",
                         bits=bits_str(bits), color=color)
            self.decide_and_move("dead_end", False, False, False)
            return

        if n_exits == 1:
            suggest = "L" if has_left else ("R" if has_right else "S")
            self.log.log("CURVE", "STOP_FOR_CMD",
                         bits=bits_str(bits), color=color, suggest=suggest)
            self.decide_and_move("curve", has_left, has_right, has_straight)
            return

        self.decide_and_move("junction", has_left, has_right, has_straight)

    # ---- 유실(000) 처리(gg5 동일 — 막다른길만 결정 지점으로) ----

    def lost_check(self, snap):
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
                if on_line(self.hw.read_center_color_now()):
                    found = True
                    break
                time.sleep(0.005)
        finally:
            self.hw.stop()
        return found, self.hw.enc_avg()

    def realign_to_line(self, snap):
        if on_line(self.hw.read_center_color_now()):
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
        offset = d1 - d2
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
        """확정 유실: 후진→재정렬 복구(자동). 복구 불가면 결정 지점(사람 호출)."""
        snap = self.params.snapshot()
        if (time.monotonic() - self.last_recover_t) * 1000 >= LOST_RETRY_WINDOW_MS:
            self.lost_streak = 0
        if self.lost_streak >= snap["lost_max_recover"]:
            self.log.log("DEAD_END", "LOST_STREAK_LIMIT",
                         streak=self.lost_streak)
            self.decide_and_move("dead_end", False, False, False)
            return
        self.log.log("LINE_LOST", "ALL_WHITE_BACKUP",
                     backup_mm=LOST_BACKUP_MM, streak=self.lost_streak)
        found, dist = self.backup_to_line(LOST_BACKUP_MM, snap)
        if self.interrupted():
            return
        if not found:
            self.log.log("DEAD_END", "BACKUP_NO_LINE", dist_mm=round(dist, 1))
            self.decide_and_move("dead_end", False, False, False)
            return
        if not self.realign_to_line(snap):
            if self.interrupted():
                return
            self.log.log("DEAD_END", "REALIGN_NO_LINE", dist_mm=round(dist, 1))
            self.decide_and_move("dead_end", False, False, False)
            return
        self.lost_streak += 1
        self.last_recover_t = time.monotonic()
        self.log.log("LINE_RECOVER", "BACKUP_REALIGN_OK",
                     dist_mm=round(dist, 1), streak=self.lost_streak)

    # ---- 세션 단계 ----

    def wait_for_start(self):
        """가운데 버튼 press→release 또는 대시보드 [0] GO 로 출발.

        대기 중에도 handle_pending 이 돌므로 calibrate/grip 을 먼저 실행할
        수 있고, 이동 명령을 미리 넣어두면 첫 결정 지점에서 바로 소비된다."""
        snap = self.params.snapshot()
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)   # 물체 받을 준비
        while True:
            if self.stop_on:
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            if self.go_on:
                self.go_on = False
                self.log.log("START", "DASHBOARD_GO")
                break
            self.publish("waiting_start")
            pressed = self.hw.wait_center_button(
                stop_cb=lambda: self.stop_on or self.reset_on or self.go_on,
                timeout=0.2)
            if pressed:
                self.log.log("START", "CENTER_BUTTON")
                break
        self.straight(START_EXIT_MM, STRAIGHT_SPEED, mode="start_exit")
        self.last_marker_t = time.monotonic()   # 출발 노랑 재감지 방지
        self.ramp_t0 = time.monotonic()         # 첫 라인추종도 가속 램프부터
        return "go"

    def drive_loop(self):
        """라인트레이싱 메인 루프 — 완주 판정 없음, stop/reset 까지 계속."""
        last_follow_log = -1e9
        while True:
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

            # 물체를 만나면(초음파 grab_dist 이내) 자동 파지(그립 비었을 때).
            if (not self.grabbed and
                    self.hw.read_distance_cm() < snap["grab_dist_cm"]):
                self.hw.stop()
                self.hw.grip_close(snap["grip_speed"], GRIP_SEC)
                self.grabbed = True
                self.hw.tone(GRAB_TONE[0], GRAB_TONE[1])
                self.log.log("GRAB", "ULTRASONIC_NEAR",
                             grab_dist_cm=snap["grab_dist_cm"])
                self.reset_steer()

            rl = self.hw.read_left_reflect()
            rr = self.hw.read_right_reflect()
            bits = node_bits(rl, color, rr, snap)
            now = time.monotonic()

            if bits == LOST_BITS:
                if abs(self.last_turn) > LOST_GUARD_TURN:
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
                    self.node_debounce_ms = (NODE_DEBOUNCE_MS
                                             if confirmed is not None
                                             else NODE_CANCEL_DEBOUNCE_MS)
                    self.reset_steer()
                    if confirmed is not None:
                        self.handle_node(confirmed, creep_mm)
                    continue

            # 교차로 접근 가드(gg5 동일) — 가로선 스침 구간은 감속 직진.
            near_node = (rl < snap["left_th_node"] + NODE_GUARD_MARGIN or
                         rr < snap["right_th_node"] + NODE_GUARD_MARGIN)
            if near_node:
                if self.guard_since is None:
                    self.guard_since = now
                    self.pid.prev_t = None
                    self.log.log("NODE_GUARD", "STRAIGHT_HOLD",
                                 reflect_l=rl, reflect_r=rr,
                                 bits=bits_str(bits))
                if (now - self.guard_since) * 1000 < NODE_GUARD_MAX_MS:
                    self.hw.drive(SLOW_SPEED, SLOW_SPEED)
                    self.last_turn = 0.0
                    self.publish("node_guard", reflect_l=rl, reflect_r=rr,
                                 color=color, bits=bits_str(bits),
                                 lost_streak=self.lost_streak)
                    time.sleep(LOOP_DELAY_S)
                    continue
            else:
                if self.guard_since is not None:
                    self.ramp_t0 = now      # 가드→순항 복귀도 램프로
                self.guard_since = None

            slow = bits in SLOW_ON or self.lost_since is not None
            if slow:
                base = SLOW_SPEED
            else:
                base = adaptive_base(snap["base_speed"], snap["corner_gain"],
                                     self.err_ema, snap["accel_ramp_ms"],
                                     (now - self.ramp_t0) * 1000.0)
            norm_l = normalize(rl, snap["cal_l_black"], snap["cal_l_white"])
            norm_r = normalize(rr, snap["cal_r_black"], snap["cal_r_white"])
            left, right, error, turn, trim = self.pid.step(norm_l, norm_r,
                                                           snap, base)
            self.hw.drive(left, right)
            self.last_turn = turn
            self.err_ema = (ERR_EMA_ALPHA * abs(error) +
                            (1.0 - ERR_EMA_ALPHA) * self.err_ema)

            if now - last_follow_log >= FOLLOW_LOG_S:
                self.log.log("LINE_FOLLOW", "PID", reflect_l=rl, reflect_r=rr,
                             norm_l=round(norm_l, 1), norm_r=round(norm_r, 1),
                             bits=bits_str(bits), error=round(error, 2),
                             turn=round(turn, 2), trim=round(trim, 2),
                             base=round(base, 1),
                             err_ema=round(self.err_ema, 1))
                last_follow_log = now
            self.publish("follow", reflect_l=rl, reflect_r=rr, color=color,
                         norm_l=round(norm_l, 1), norm_r=round(norm_r, 1),
                         bits=bits_str(bits), error=round(error, 2),
                         turn=round(turn, 2), trim=round(trim, 2),
                         base=round(base, 1),
                         left_speed=round(left, 1), right_speed=round(right, 1),
                         lost_streak=self.lost_streak)
            time.sleep(LOOP_DELAY_S)

    # ---- 세션 루프 ----

    def new_session(self):
        self._init_session_state()
        self.cmd.clear()            # 이전 세션 잔여 명령 폐기
        self.pid.full_reset()
        self.reset_steer()
        self.reset_on = False
        self.session += 1
        if self.session == 1:
            self.log.log("SESSION_READY", "STARTUP", session=self.session)
        else:
            self.log.log("SESSION_RESET", "DASHBOARD", source=self.reset_source,
                         session=self.session)

    def run_sessions(self):
        while not self.stop_on:
            self.new_session()
            status = self.wait_for_start()
            if status == "go":
                status = self.drive_loop()
            if status == "stop":
                break
            # status == "reset" → 루프 상단에서 new_session.


def seed_initial(initial):
    """gg8 첫 실행 승계: gg8.json 이 없으면 SEED_SOURCES 순서(gg5→gg3)로
    실기 튜닝값을 initial 에 병합해 (승계 이름 리스트, 출처 경로)를 반환."""
    if os.path.exists(SAVE_PATH):
        return [], None
    for path, skip in SEED_SOURCES:
        if not os.path.exists(path):
            continue
        try:
            with open(path, "r") as fp:
                saved = json.load(fp)
        except Exception:
            continue
        if not isinstance(saved, dict):
            continue
        seeded = []
        for name in sorted(saved):
            if name in skip or name not in initial:
                continue
            lo, hi = PARAM_LIMITS[name]
            try:
                value = clamp(float(saved[name]), lo, hi)
            except (TypeError, ValueError):
                continue
            if isinstance(initial[name], int) and isinstance(saved[name], int):
                value = int(round(value))
            if value != initial[name]:
                initial[name] = value
                seeded.append(name)
        return seeded, path
    return [], None


def run():
    from lib.hardware import Ev3Hardware   # ev3dev2 — 브릭에서만 import 가능

    initial = dict(INITIAL_PARAMS)
    seeded, seed_path = seed_initial(initial)
    params = SharedParams(initial, PARAM_LIMITS, MAX_STEP, SAVE_PATH,
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

    if seeded:
        log.log("PARAM_SEED", "CARRYOVER", source=seed_path,
                names=",".join(seeded))
    print("gg8 ready — MANUAL DECISION MODE. dashboard keys: [1]L [2]S [3]R "
          "[4]U [5]clear [6]grip-open [7]grip-close [8]fwd [9]back [0]GO. "
          "robot soft-stops at EVERY curve/junction/marker/dead-end and "
          "waits; queue a move anytime to run it on arrival. start: CENTER "
          "button or [0]. (Ctrl-C or robotctl stop to quit)")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        try:
            hw.stop()
        finally:
            server.stop()
    print("gg8 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
