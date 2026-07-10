#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manual_run1 — final_run8 라인트레이싱 + 교차로 수동 조향(대시보드 원격 제어).

목적: 회전 판단을 로봇이 아니라 **사람이 대시보드에서** 내린다.
  - 라인추종은 final_run8 검증본 그대로: 캘리브레이션 정규화 PID(kp/kd/ki
    라이브 튜닝), 교차로 접근 가드(§F), 000 유실 후진·재정렬 복구.
  - 교차로 판별도 final_run8 그대로: 노드 후보 bits → 감속 → 거리 결정형
    confirm(node_confirm_mm) → 정지 재판정 → node_advance_mm 전진.
  - 여기서부터가 다르다: 출구가 2개 이상인 분기(진짜 교차로)면 **정지하고
    대시보드 방향 명령(left/straight/right/uturn)을 기다린다.** 명령이
    미리 눌려 있으면(단일 슬롯, 최신 명령이 덮음) 기다리지 않고 즉시 회전.
  - 출구가 1개뿐인 커브는 선택의 여지가 없으므로 종전처럼 자동 회전.
  - 막다른길(유실 확정 체인)도 자동 유턴하지 않고 정지 후 명령 대기.

제거한 것(운반 미션 전부): Explorer 지도/탐색, 복귀 계획, 마커(빨/노/초)
미션 분기, 그리퍼, 초음파 파지, 노랑 출발 대기. 바닥의 색 스티커는
on_line()에서 '라인 위'로만 취급된다(흰색만 아니면 라인).

대시보드 조작(tools/dashboard.py 수정 불필요 — manifest key 필드로 핫키 명시,
gg8 과 같은 배치라 조작감 동일):
  [j] left  [k] straight  [l] right  [u] uturn   ← 교차로 방향 명령
  [x] clear cmd  [t] go(주행 시작)  [d] calibrate  [h] read_color
  [e] read_reflect  [z] reset([r] 공용 키도 됨)  [Space] pause  [s] STOP
robotctl 로도 동일: python3 tools/robotctl.py do left 등.
대기 프레임은 gg8 과 같은 규약(mode=await_cmd, pending_cmd/await_kind/
has_left·straight·right)으로 publish 해 대시보드 AWAITING COMMAND 배너를
그대로 쓴다.

방향 명령 규칙:
  - 단일 슬롯 큐 — 주행 중 언제 눌러도 저장되고, 최신 명령이 이전 것을 덮는다.
  - 교차로 도착 시 슬롯에 명령이 있으면 즉시 소비(무정차 대기 없이 회전),
    없으면 정지한 채 명령이 올 때까지 대기(await_turn 프레임 publish).
  - 명령은 무조건 따른다(수동 제어가 우선). 다만 그 방향 출구가 bits 상
    없으면 EXIT_MISMATCH_OBEYED 로 경고 로그를 남긴다.

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      정지는 네트워크 stop 또는 Ctrl-C, 재시작은 네트워크 reset.

실행(브릭):   python3 stages/manual_run1.py
문법 점검(PC): python3 -m py_compile stages/manual_run1.py lib/*.py
단위 테스트:  python3 tests/test_manual_run1_logic.py (ev3dev2 불필요)
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
# 상수 — final_run8 검증값 그대로(라인추종/노드 판별에 필요한 부분만).
# ---------------------------------------------------------------------

COL_WHITE = 6

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

# 고정 주행값(라이브 튜닝 대상 아님) — final_run8 과 동일.
CONFIRM_SPEED = 7           # 의심지점 저속 직진 + 노드 전진
SLOW_SPEED = 12             # SLOW_ON 패턴/유실 의심 감속 주행
BACKUP_SPEED = 10           # 선 유실 후진
LOST_BACKUP_MM = 100        # 선 유실 시 최대 후진 거리
LOST_RETRY_WINDOW_MS = 4000  # 이 시간 지나면 복구 연속 카운트(lost_streak) 리셋
LOST_GUARD_TURN = 8.0       # |turn| 이 크면 000 은 위빙으로 보고 무시
LOST_CONFIRM_SAMPLES = 3    # 유실 확정 전 정지 재판정 샘플 수(전부 000 이어야 확정)
LOST_CONFIRM_GAP_S = 0.02   # 재판정 샘플 간격
REALIGN_SPEED = 6           # 복구 후 재정렬 소각 스캔 속도(%)
REALIGN_MAX_DEG = 70        # 재정렬 스캔 한쪽 최대 enc deg(로봇 약 50도)
NODE_GUARD_MARGIN = 12      # 반사광 < 노드임계+마진 이면 가로선 접근으로 본다(§F)
NODE_GUARD_MAX_MS = 700     # 직진 유지 상한 — 초과 지속이면 가로선이 아니라 정렬 불량
NODE_DEBOUNCE_MS = 900      # 노드 '확정' 후 재감지 최소 간격
NODE_CANCEL_DEBOUNCE_MS = 150  # confirm '취소' 후 간격(v13.1)
CONFIRM_SETTLE_S = 0.08     # 재판정 전 정지 안정화
CONFIRM_TIMEOUT_S = 1.5     # 거리 기반 confirm 안전 상한(스톨/걸림 대비)
LOOP_DELAY_S = 0.015
AWAIT_POLL_S = 0.05         # 교차로 명령 대기 폴링 간격
FOLLOW_LOG_S = 0.25         # LINE_FOLLOW 로그 최소 간격
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)

# PID — final_run8 과 동일(KD 라이브 파라미터, derivative clamp + EMA).
PID_TURN_LIMIT = 16
PID_DERIV_LIMIT = 220.0
PID_D_EMA_ALPHA = 0.35
INTEG_BAND = 25.0           # |error(정규화)| 가 이 이하일 때만 적분
INTEG_TURN_LIMIT = 8.0      # I 항이 turn 에 기여할 수 있는 최대치
INTEG_RESET_KEEP = 0.5      # reset_steer 때 적분 잔존 비율

# 캘리브레이션 스윕 — final_run8 과 동일.
CAL_SPEED = 6               # 스윕 피벗 속도(%)
CAL_HALF_DEG = 60           # 한쪽 스윕 enc deg(로봇 약 40~45도)
CAL_MIN_SPAN = 20           # white-black 이 이보다 좁으면 실패(라인을 못 봤다)


# ---------------------------------------------------------------------
# 라이브 파라미터 — final_run8 에서 미션(그리퍼/초음파/배달) 값만 뺀 것.
# ---------------------------------------------------------------------

PARAM_TABLE = (
    ("base_speed",      20,    5,   45,   5,    1,    "%"),
    ("kp",              0.17,  0.0, 3.0,  0.1,  0.01, ""),
    ("kd",              0.05,  0.0, 1.0,  0.05, 0.005, ""),
    ("ki",              0.0,   0.0, 0.5,  0.05, 0.01, ""),
    ("turn_speed",      5,     5,   40,   5,    1,    "%"),
    ("turn_ramp_ms",    250,   0,   600,  100,  50,   "ms"),
    ("node_confirm_mm", 8.0,   0.0, 25.0, 5.0,  1.0,  "mm"),
    ("left_th_steer",   66,    0,   100,  3,    1,    "%"),
    ("right_th_steer",  63,    0,   100,  3,    1,    "%"),
    ("left_th_node",    35,    0,   100,  3,    1,    "%"),
    ("right_th_node",   30,    0,   100,  3,    1,    "%"),
    ("cal_l_black",     0,     0,   100,  100,  1,    "%"),   # calibrate 가 기록
    ("cal_l_white",     100,   0,   100,  100,  1,    "%"),
    ("cal_r_black",     0,     0,   100,  100,  1,    "%"),
    ("cal_r_white",     100,   0,   100,  100,  1,    "%"),
    ("node_advance_mm", 60,    0,   120,  10,   10,   "mm"),
    ("turn_90_factor",  0.65,  0.3, 2.0,  0.05, 0.01, "x"),
    ("turn_180_factor", 0.71,  0.3, 2.0,  0.05, 0.01, "x"),
    ("lost_persist_ms", 200,   0,   500,  100,  10,   "ms"),
    ("lost_max_recover", 3,    1,   3,    1,    1,    ""),
)

INITIAL_PARAMS = dict((r[0], r[1]) for r in PARAM_TABLE)
PARAM_LIMITS = dict((r[0], (r[2], r[3])) for r in PARAM_TABLE)
MAX_STEP = dict((r[0], r[4]) for r in PARAM_TABLE)
UI_STEP = dict((r[0], r[5]) for r in PARAM_TABLE)
UNITS = dict((r[0], r[6]) for r in PARAM_TABLE)
PARAM_ORDER = [r[0] for r in PARAM_TABLE]

SAVE_PATH = os.path.join(_ROOT, "config", "manual_run1.json")
STAGE_NAME = "manual_run1"

# 방향 명령 액션 → 회전 move. 액션 순서가 대시보드 키 [1]~[4] 가 되므로
# 방향 4개를 맨 앞에 둔다.
MOVE_ACTIONS = (("left", "L"), ("straight", "S"), ("right", "R"), ("uturn", "U"))
MOVE_BY_ACTION = dict(MOVE_ACTIONS)

# key 필드 = 대시보드 명시 핫키(gg8 과 동일 배치 — 조작감 통일).
ACTIONS = [
    {"name": "left", "label": "Turn LEFT", "key": "j"},
    {"name": "straight", "label": "Go STRAIGHT", "key": "k"},
    {"name": "right", "label": "Turn RIGHT", "key": "l"},
    {"name": "uturn", "label": "U-TURN", "key": "u"},
    {"name": "clear", "label": "Clear queued cmd", "key": "x"},
    {"name": "go", "label": "GO (start driving)", "key": "t"},
    {"name": "calibrate", "label": "Calibrate L/R on line (sweep)", "key": "d"},
    {"name": "read_color", "label": "Read Center Color", "key": "h"},
    {"name": "read_reflect", "label": "Read L/R Reflect (raw+norm)", "key": "e"},
    {"name": "reset", "label": "Reset (back to idle)", "key": "z"},
]


# ---------------------------------------------------------------------
# 순수 헬퍼 — final_run8 그대로.
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
    반환한다 — 잘못된 스케일로 조향이 폭주하는 것보다 원시값이 안전하다.
    """
    span = float(white) - float(black)
    if span < CAL_MIN_SPAN:
        return clamp(float(raw), 0.0, 100.0)
    return clamp(100.0 * (float(raw) - float(black)) / span, 0.0, 100.0)


def on_line(center_color):
    """중앙 컬러센서가 '라인 위'인가 — 흰 바닥이 아니면 전부 라인으로 본다.

    근거는 final_run8 §변경 E(실측 3919샘플): BROWN/BLUE/NONE 은 검정↔흰색
    경계·포화 검정의 오분류다. 마커 색 스티커도 라인 위다.
    """
    return center_color != COL_WHITE


def node_bits(reflect_l, center_color, reflect_r, snap):
    """노드 판정 bits(좌,중,우) — 좌/우는 반사광 '원시값', 중앙은 라인 위 여부."""
    return (1 if reflect_l < snap["left_th_node"] else 0,
            1 if on_line(center_color) else 0,
            1 if reflect_r < snap["right_th_node"] else 0)


def bits_str(bits):
    return "{}{}{}".format(bits[0], bits[1], bits[2])


class CommandBox(object):
    """대시보드 방향 명령 단일 슬롯(스레드 안전) — 최신 명령이 이전 것을 덮는다.

    네트워크 스레드가 put, 제어 루프가 take(꺼내며 비움)/peek(관찰만) 한다.
    큐가 아니라 슬롯인 이유: 교차로 하나에 명령 하나가 자연스럽고, 잘못 누른
    버튼을 다음 버튼으로 즉시 덮어쓸 수 있어야 한다.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._move = None

    def put(self, move):
        with self._lock:
            prev = self._move
            self._move = move
        return prev

    def take(self):
        with self._lock:
            move = self._move
            self._move = None
        return move

    def peek(self):
        with self._lock:
            return self._move

    def clear(self):
        with self._lock:
            self._move = None


# ---------------------------------------------------------------------
# PID 조향(순수) — final_run8 그대로.
# ---------------------------------------------------------------------

class PidSteer(object):
    """error = 정규화 우반사광 - 정규화 좌반사광. (final_run8 검증본)"""

    def __init__(self):
        self.full_reset()

    def full_reset(self):
        """세션 시작/캘리브레이션 후 — 학습된 트림까지 전부 버린다."""
        self.prev_error = 0.0
        self.prev_t = None
        self.deriv = 0.0
        self.integ = 0.0

    def reset(self):
        """회전/노드/복구 후 — P/D 이력은 버리되 적분은 절반 유지."""
        self.prev_error = 0.0
        self.prev_t = None
        self.deriv = 0.0
        self.integ *= INTEG_RESET_KEEP

    def step(self, norm_l, norm_r, snap, base_speed):
        error = float(norm_r - norm_l)

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
        turn = clamp(snap["kp"] * error + i_term + snap["kd"] * self.deriv,
                     -PID_TURN_LIMIT, PID_TURN_LIMIT)
        self.prev_error = error
        self.prev_t = t
        return base_speed - turn, base_speed + turn, error, turn, i_term


# ---------------------------------------------------------------------
# 구동 러너 — run() 에서만 생성(ev3dev2).
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
        self.go_on = False
        self._pending = None
        self._pending_lock = threading.Lock()
        self.cmd = CommandBox()

        self.session = 0
        self.pid = PidSteer()
        self._init_session_state()

    def _init_session_state(self):
        self.junctions = 0          # 수동 명령으로 처리한 교차로 수(텔레메트리)
        self.last_turn = 0.0
        self.last_node_t = -1e9
        self.last_recover_t = -1e9
        self.lost_since = None      # 000 연속 시작 시각(지속 필터, v13)
        self.lost_streak = 0        # 윈도 내 연속 복구 횟수(v13)
        self.node_debounce_ms = 0   # 직전 confirm 결과에 따른 재감지 간격(v13.1)
        self.guard_since = None     # 가로선 접근 가드 시작 시각(§F)

    # ---- 네트워크 핸들러(TuningServer 스레드에서 호출) ----

    def on_stop(self, source):
        self.stop_on = True
        self.stop_source = source

    def on_pause(self, paused, source):
        self.paused = bool(paused)
        self.log.log("PAUSE" if paused else "RESUME", "NETWORK", source=source)
        return {"mode": "paused" if paused else "run"}

    def on_do(self, action, args):
        if action in MOVE_BY_ACTION:
            move = MOVE_BY_ACTION[action]
            prev = self.cmd.put(move)
            self.log.log("COMMAND", "QUEUED", move=move,
                         replaced=prev if prev is not None else "-")
            return {"queued": action, "move": move}
        if action == "clear":
            self.cmd.clear()
            self.log.log("COMMAND", "CLEARED", source="dashboard")
            return {"queued": "clear"}
        if action == "go":
            self.go_on = True
            return {"queued": "go"}
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
            "junctions": self.junctions,
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

    def reset_steer(self):
        self.pid.reset()
        self.last_turn = 0.0
        self.lost_since = None
        self.guard_since = None

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

    # ---- 캘리브레이션 — final_run8 그대로 ----

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
        """로봇을 라인(직선 구간) 위에 세워두고 실행 — 좌→우→복귀 소각 스윕으로
        센서별 black/white 실측치를 cal_* 파라미터에 기록. 성공 시 자동 save."""
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

    # ---- 모션 프리미티브 — final_run8 그대로(Explorer heading 갱신만 제거) ----

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
        """제자리 회전(L/R/U) + PID 리셋 — final_run8 스무스 턴 그대로.
        S 는 회전 없음. 회전 후 중앙이 흰 바닥이면 소각 스캔으로 재획득."""
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
        self.hw.coast()                 # 직전 brake-hold 해제 → 리셋 킥 방지
        self.hw.reset_encoders()
        self.hw.set_ramp(snap["turn_ramp_ms"])   # 가속만 램프(감속=0)
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
            self.hw.set_ramp(0)         # 라인추종 조향엔 램프 미적용
        actual = self.hw.enc_avg()
        time.sleep(POST_TURN_SETTLE_S)
        self.log.log("TURN", {"L": "TURN_LEFT", "R": "TURN_RIGHT", "U": "UTURN"}[move],
                     target_deg=round(target, 1), enc_avg=round(actual, 1),
                     error_deg=round(actual - target, 1),
                     stopped_early=self.interrupted())
        self.reset_steer()
        # 회전 직후 라인 재획득(v13.1) — 피벗 오차로 중앙이 라인 밖이면
        # 000 유실 체인으로 빠지기 전에 소각 스캔으로 라인에 올라탄다.
        if not self.interrupted():
            color = self.hw.read_center_color_now()
            if not on_line(color):
                self.log.log("TURN_ACQUIRE", "CENTER_OFF_LINE_AFTER_TURN",
                             move=move, color=color)
                self.realign_to_line(self.params.snapshot())

    # ---- 교차로 수동 조향(이 스테이지의 핵심) ----

    def await_turn(self, has_left, has_right, has_straight, context):
        """교차로/막다른길에서 대시보드 방향 명령을 기다렸다 회전한다.

        슬롯에 명령이 이미 있으면(접근 중 미리 누름) 무정차로 즉시 소비.
        없으면 정지한 채 await_turn 프레임을 publish 하며 대기 — 제어 루프는
        네트워크를 기다리지 않는다는 원칙(§AGENTS)은 '주행 판단'에 대한
        것이고, 여기는 사람이 결정권자라 정지 대기가 사양이다. stop/reset
        은 즉시 빠져나간다.
        """
        exits = "{}{}{}".format(1 if has_left else 0, 1 if has_straight else 0,
                                1 if has_right else 0)
        move = self.cmd.take()
        waited = move is None
        if waited:
            self.hw.stop()
            self.log.log("AWAIT_TURN", "WAIT_DASHBOARD", context=context,
                         exits_lsr=exits)
            while move is None:
                if self.interrupted():
                    return
                if self.paused:
                    self._hold_while_paused("await_cmd")
                    continue
                self.handle_pending()
                # gg8 과 같은 프레임 규약 — 대시보드 AWAITING COMMAND 배너용.
                self.publish("await_cmd", await_kind=context,
                             has_left=has_left, has_straight=has_straight,
                             has_right=has_right, exits_lsr=exits)
                move = self.cmd.take()
                if move is None:
                    time.sleep(AWAIT_POLL_S)
        avail = {"L": has_left, "R": has_right, "S": has_straight, "U": True}
        if not avail.get(move, False):
            # 수동 제어가 우선 — 따르되 bits 상 없는 출구임을 경고로 남긴다.
            self.log.log("MANUAL_TURN", "EXIT_MISMATCH_OBEYED", move=move,
                         exits_lsr=exits, context=context, waited=waited)
        else:
            self.log.log("MANUAL_TURN", "DASHBOARD", move=move,
                         exits_lsr=exits, context=context, waited=waited)
        self.junctions += 1
        self.turn(move)

    # ---- 노드 처리 — 판별은 final_run8, 회전 결정만 수동 ----

    def confirm_node(self, first_bits, snap):
        """의심지점: PID off → 저속 직진(감지점+node_confirm_mm) → 정지 후 재판정.
        final_run8 거리 결정형 그대로, 마커 미션 분기만 없다."""
        self.reset_steer()
        self.hw.reset_encoders()
        self.log.log("NODE_CANDIDATE", "PD_OFF_SLOW_STRAIGHT",
                     bits=bits_str(first_bits),
                     confirm_mm=snap["node_confirm_mm"], speed=CONFIRM_SPEED)
        target_mm = snap["node_confirm_mm"]
        deadline = time.monotonic() + CONFIRM_TIMEOUT_S
        timeout = False
        self.hw.drive(CONFIRM_SPEED, CONFIRM_SPEED)
        while self.hw.enc_avg() * MM_PER_DEG < target_mm:
            if time.monotonic() >= deadline:
                timeout = True
                break
            if self.interrupted():
                self.hw.stop()
                return None, self.hw.enc_avg() * MM_PER_DEG
            if self.paused:
                self._hold_while_paused("node_confirm")
                if self.interrupted():
                    return None, self.hw.enc_avg() * MM_PER_DEG
                self.hw.drive(CONFIRM_SPEED, CONFIRM_SPEED)
            bits, color, rl, rr = self.read_bits(snap)
            if bits not in NODE_CANDIDATES:
                self.hw.stop()
                creep_mm = self.hw.enc_avg() * MM_PER_DEG
                if bits == LOST_BITS and first_bits[0] == 1 and first_bits[2] == 1:
                    # 좌/우 팔이 있던 후보(111/101)가 전백 = 가로선을 지나쳤다.
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
        if bits in NODE_CANDIDATES:
            self.log.log("NODE_CONFIRMED", "SLOW_STRAIGHT_STOP",
                         first_bits=bits_str(first_bits), bits=bits_str(bits),
                         reflect_l=rl, reflect_r=rr, color=color,
                         creep_mm=round(creep_mm, 1), timeout=timeout)
            return bits, creep_mm
        if bits == LOST_BITS and first_bits[0] == 1 and first_bits[2] == 1:
            self.log.log("NODE_CONFIRMED", "PASSED_OVER_AT_STOP",
                         first_bits=bits_str(first_bits),
                         reflect_l=rl, reflect_r=rr, color=color,
                         creep_mm=round(creep_mm, 1), timeout=timeout)
            return first_bits, creep_mm
        self.log.log("NODE_CANDIDATE", "CANCELLED_AT_STOP",
                     first_bits=bits_str(first_bits), bits=bits_str(bits),
                     reflect_l=rl, reflect_r=rr, color=color, timeout=timeout)
        return None, creep_mm

    def handle_node(self, bits, creep_mm):
        """확정 노드 처리. 전진 후 출구를 세서 커브는 자동, 교차로는 수동."""
        snap = self.params.snapshot()
        self.straight(max(0.0, snap["node_advance_mm"] - creep_mm), CONFIRM_SPEED)
        if self.interrupted():
            return

        color = self.hw.read_center_color_now()
        has_left = bits[0] == 1
        has_right = bits[2] == 1
        has_straight = on_line(color)
        n_exits = int(has_left) + int(has_right) + int(has_straight)

        if n_exits == 0:
            # NODE_CANDIDATES 는 좌/우 팔이 있어 정상적으론 안 온다(방어용).
            self.log.log("DEAD_END", "NO_EXIT_AFTER_ADVANCE",
                         bits=bits_str(bits), color=color)
            self.await_turn(False, False, False, "dead_end")
            return

        if n_exits == 1:
            # 커브 — 선택의 여지가 없으므로 종전처럼 자동 회전.
            move = "L" if has_left else ("R" if has_right else "S")
            self.log.log("CURVE", "FORCED_" + {"L": "LEFT", "R": "RIGHT",
                                               "S": "STRAIGHT"}[move],
                         bits=bits_str(bits), color=color)
            self.turn(move)
            return

        self.await_turn(has_left, has_right, has_straight, "junction")

    # ---- 유실(000) 처리 — final_run8, 막다른길 유턴만 수동으로 ----

    def lost_check(self, snap):
        """000 지속 확정 전 재판정: 즉시 정지 → 정지 상태 복수 샘플.
        반환: "lost" / "line" / "interrupted"."""
        self.hw.stop()
        time.sleep(CONFIRM_SETTLE_S)
        self.log.log("LOST_SUSPECT", "PERSIST_STOP_RECHECK",
                     persist_ms=snap["lost_persist_ms"],
                     samples=LOST_CONFIRM_SAMPLES)
        for i in range(LOST_CONFIRM_SAMPLES):
            if self.interrupted():
                return "interrupted"
            bits, color, rl, rr = self.read_bits(snap)
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
        """중앙이 라인 위(흰색 아님)가 될 때까지 소각 피벗(L/R). (found, 진행 enc deg)."""
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
        """복구 직후 재정렬 — 중앙 컬러가 라인 위에 오도록 좌/우 소각 스캔."""
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
        # 양쪽 다 실패 — 원래 방향으로 복원(복원 중 라인을 만나면 성공 처리).
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
        """확정 유실: 후진→재정렬 복구. 한도 초과/복구 실패 = 막다른길로 보고
        자동 유턴 대신 정지 후 대시보드 명령을 기다린다(수동 제어 원칙)."""
        snap = self.params.snapshot()
        if (time.monotonic() - self.last_recover_t) * 1000 >= LOST_RETRY_WINDOW_MS:
            self.lost_streak = 0
        if self.lost_streak >= snap["lost_max_recover"]:
            self.log.log("DEAD_END", "LOST_STREAK_LIMIT",
                         streak=self.lost_streak)
            self.await_turn(False, False, False, "dead_end")
            return
        self.log.log("LINE_LOST", "ALL_WHITE_BACKUP",
                     backup_mm=LOST_BACKUP_MM, streak=self.lost_streak)
        found, dist = self.backup_to_line(LOST_BACKUP_MM, snap)
        if self.interrupted():
            return
        if not found:
            self.log.log("DEAD_END", "BACKUP_NO_LINE", dist_mm=round(dist, 1))
            self.await_turn(False, False, False, "dead_end")
            return
        if not self.realign_to_line(snap):
            if self.interrupted():
                return
            self.log.log("DEAD_END", "REALIGN_NO_LINE", dist_mm=round(dist, 1))
            self.await_turn(False, False, False, "dead_end")
            return
        self.lost_streak += 1
        self.last_recover_t = time.monotonic()
        self.log.log("LINE_RECOVER", "BACKUP_REALIGN_OK",
                     dist_mm=round(dist, 1), streak=self.lost_streak)

    # ---- 세션 단계 ----

    def wait_for_go(self):
        """대시보드 go([5])까지 대기. 대기 중 calibrate/read_* 사용 가능.
        미리 눌러둔 방향 명령은 세션 시작 시 비운다(이월 방지).
        status('go'|'stop'|'reset') 반환."""
        self.go_on = False
        self.cmd.clear()
        while not self.go_on:
            if self.stop_on:
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            self.publish("waiting_go")
            time.sleep(0.05)
        self.log.log("START", "DASHBOARD_GO", session=self.session)
        return "go"

    def drive(self):
        """라인추종 메인 루프 — final_run8 explore() 에서 미션(마커/파지)만 뺀 것.
        status(stop/reset) 반환(수동 주행은 reset/stop 으로만 끝난다)."""
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
            rl = self.hw.read_left_reflect()
            rr = self.hw.read_right_reflect()
            bits = node_bits(rl, color, rr, snap)
            now = time.monotonic()

            if bits == LOST_BITS:
                # 000 은 노드 후보가 아니라 유실 의심 — 지속시간 필터(v13).
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

            # §F — 교차로 접근 가드: 조향 off + 감속 직진으로 자세 유지.
            near_node = (rl < snap["left_th_node"] + NODE_GUARD_MARGIN or
                         rr < snap["right_th_node"] + NODE_GUARD_MARGIN)
            if near_node:
                if self.guard_since is None:
                    self.guard_since = now
                    self.pid.prev_t = None  # 재개 첫 프레임 D/I 계산 무효화
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
                self.guard_since = None

            # 유실 의심 누적 중에도 감속해 라인 이탈 관성을 줄인다.
            slow = bits in SLOW_ON or self.lost_since is not None
            base = SLOW_SPEED if slow else snap["base_speed"]
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
                         lost_streak=self.lost_streak)
            time.sleep(LOOP_DELAY_S)

    # ---- 세션 루프 ----

    def new_session(self):
        """상태를 전부 버리고 새 세션 준비(시작/reset 공용)."""
        self._init_session_state()
        self.pid.full_reset()
        self.reset_steer()
        self.cmd.clear()
        self.reset_on = False
        self.session += 1
        if self.session == 1:
            self.log.log("SESSION_READY", "STARTUP", session=self.session)
        else:
            self.log.log("SESSION_RESET", "DASHBOARD", source=self.reset_source,
                         session=self.session)

    def run_sessions(self):
        """세션 = go 대기 → 수동 주행. reset 은 새 세션, stop 은 종료."""
        while not self.stop_on:
            self.new_session()
            status = self.wait_for_go()
            if status == "go":
                status = self.drive()
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

    print("manual_run1 ready — REMOTE JUNCTION CONTROL. dashboard keys: "
          "[j]left [k]straight [l]right [u]uturn [x]clear [t]go [d]calibrate. "
          "robot line-follows, slows at junctions, and waits for your "
          "direction command. (Ctrl-C or robotctl stop to quit)")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        try:
            hw.stop()
        finally:
            server.stop()
    print("manual_run1 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
