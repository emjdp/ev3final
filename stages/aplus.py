#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aplus — 원격 조종 결정 모드: fxck2 주행 흐름 + fxck3 UI층 + FIFO 연속 입력 큐.

배경: fxck1/2 는 유실 자동 복구(후진+재정렬)가 로직 꼬임의 온상이었다.
aplus 는 라인트레이싱 '주행'과 뻔한 커브는 로봇이 흐르듯 처리하고(fxck2),
분기/유실/마커 같은 '판단'은 전부 사람이 한다. 전용 조종 패드
(tools/aplus_pad.py)가 텔레메트리를 고속 폴링하고 키 입력을 즉시 전송하며,
판단이 넘어오면 소리 대신 화면 배경색으로 알린다(로그 표시 없음).

역할 분담:
  로봇(자동) — gg5/fxck2 실기 검증분 그대로:
    - PID 라인트레이싱(캘리브레이션 정규화 + 커브 자동 감속 + 재출발 가속
      램프 + 속도 비례 조향) / 교차로 접근 가드. 기본 순항 15%.
      감속 하한·램프 시작은 base 비례라 base 15 에서도 부드럽게 체감된다.
    - 커브(출구 1개): 자동 통과(fxck2) — 멈추지 않고 뻔한 출구로 돈다.
      (curve_auto=0 으로 내리면 gg8 처럼 커브도 정지 후 명령 대기.)
    - 노드 의심점: 정지 → 저속 직진(미끄러지며) confirm → 정지 재판정.
    - 마커: 정지 후 복수 재판독 다수결로 색 확정(가짜 마커 방지).
    - 물체 자동 파지(초음파 grab_dist 이내, 그립 비었을 때) + [p] 강제 파지.
    - UI(fxck3 승계, 전부 비동기): 버튼 출발+스톱워치, 발표 숫자(임시 고정 2),
      빨강 "red N"(가는 길 1~6, 초록 후 다시 1부터), 초록 good_job+OUT
      시간+배달(전진→그립 오픈→후진), 노랑 복귀 완주(BACK 시간+그립 해제),
      LCD 유지 스레드(0.3s 재출력).
  사람(조종 패드) — 애매한 '결정' 전부:
    - 결정 지점 = 분기(출구 2+)/막힘/유실 확정/빨강/초록(배달 후) —
      감속 정지(soft stop) 후 명령을 기다린다(알림 tone 없음 — 패드
      배경색이 바뀐다). 자동 복구 없음: 유실도 후진 복구 없이 그 자리에
      서서 사람을 부른다(꼬임 제거 — q/e/s/u 로 사람이 푼다).
    - 명령 키(조종 패드 = 액션 manifest key):
        [w] 계속 직진(라인추종 재개)   [u] 180도 유턴
        [a] 좌회전 90                  [d] 우회전 90
        [q] 왼쪽 대각선 약간 진행      [e] 오른쪽 대각선 약간 진행
        [s] 약간 후진(위치 교정)       [p] 그리퍼 강제 닫기  [o] 그리퍼 열기
        [7] 도착 처리(초록 폴백: 전진→내려놓기→후진→180도 회전)
        [8] 복귀 도착 처리(노랑 폴백: BACK 시간+그립 해제+완주)
        [g] 대기 전환(정지 후 명령 대기 — 패드는 Space. 회전/직진/대각선
            실행 중간에도 즉시 끊고 들어간다)
        [x] 큐 비우기  [t] GO(출발)  [n] 캘리브레이션  [z] 리셋
        [1]~[6] "red N" 수동 재생(비동기 큐 — 연타 시 순서대로, 카운터 무관)
      q/e(아크 전진)/s(직선 후진)는 위치 교정용 — 실행 후 그 자리에서 계속
      대기한다(연타로 미세 조정 후 [w] 로 재개). w/a/d/u 는 이동 후
      라인추종 재개.
    - 연속 입력: 명령은 FIFO 다칸 큐(CommandQueue, 최대 8) — 주행/미끄러짐
      중에 미리 여러 개 눌러두면 결정 지점마다 하나씩 소비된다(q,q,w 같은
      체이닝도 대기 중 즉시 순차 소비). 가득 차면 새 입력은 거부(full).
      현재 큐는 매 프레임 pending_cmd 로 방송된다.
    - 명령은 출구 유효성 검사 없이 그대로 실행한다(조종자가 보스) —
      가능한 출구와 다르면 로그에 흔적만 남긴다.

마커 미션 분기(fxck3 승계 — 이동 판단은 전부 사람):
  빨강: "red N" 재생 후 결정 대기(보통 [u] 유턴).
  초록(최초): good_job + OUT 시간 + 숫자(고정 2) 갱신·재생 + 배달(전진→그립
    오픈→후진) 후 결정 대기 — 유턴은 사람이 한다. 두 번째 초록은 무시.
    초록을 못 읽고 지나쳤으면 패드 [7](goal_drop)로 같은 절차를 그 자리에서
    수동 시행한다 — 이때는 마무리 180도 회전까지 이어서 하고 주행을 재개.
  노랑(초록 후): 완주 — BACK 시간 + 스톱워치 정지 + 그립 해제, 가운데
    버튼(또는 reset)으로 새 세션. 초록 전 노랑은 무시(로그만).
    노랑을 못 읽고 지나쳤으면 패드 [8](home_drop)로 같은 완주 절차를
    그 자리에서 수동 시행한다.

파라미터 승계: config/aplus.json 이 없으면 gg8.json → gg5.json 순으로 실기
튜닝값을 승계한다(base_speed 는 제외 — aplus 기본 15 유지). 트랙이 바뀌면
조종 패드 [n] calibrate 1회.

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      정지는 네트워크 stop 또는 Ctrl-C, 재시작은 가운데 버튼 또는 reset.

실행(브릭):   python3 stages/aplus.py
조종(PC):     python3 tools/aplus_pad.py --host <브릭IP>
문법 점검(PC): python3 -m py_compile stages/aplus.py lib/*.py
단위 테스트:  python3 tests/test_aplus_logic.py (ev3dev2 불필요)
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
# 상수 — gg8 주행층 그대로(자동 유실복구 상수만 제거)
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
STRAIGHT_SPEED = 15         # 출발 이탈 / 배달 전·후진 — aplus 기본 속도 15
# 의심지점 감속(fxck2 승계): confirm/slow 는 base_speed 비례 —
# confirm_speed()/slow_speed() 참조. base 15 에선 하한이 걸려 5/8%.
CONFIRM_SPEED_FRAC = 0.25   # 의심지점 저속 직진 + 노드 전진: base 비율
CONFIRM_SPEED_MIN = 5       # 그 하한(%)
SLOW_SPEED_FRAC = 0.40      # SLOW_ON/유실 의심/노드 가드 감속: base 비율
SLOW_SPEED_MIN = 8          # 그 하한(%)
START_EXIT_MM = 50          # 출발 노랑에서 벗어나는 거리
LOST_GUARD_TURN = 8.0       # |turn| 이 크면 000 은 위빙으로 보고 무시
LOST_CONFIRM_SAMPLES = 3    # 유실 확정 전 정지 재판정 샘플 수(전부 000)
LOST_CONFIRM_GAP_S = 0.02   # 재판정 샘플 간격
REALIGN_SPEED = 6           # 회전 직후 라인 재획득 소각 스캔 속도(%)
REALIGN_MAX_DEG = 70        # 재획득 스캔 한쪽 최대 enc deg(로봇 약 50도)
NODE_DEBOUNCE_MS = 900      # 노드 '확정' 후 재감지 최소 간격
NODE_CANCEL_DEBOUNCE_MS = 150  # confirm '취소' 후 간격
NODE_GUARD_MARGIN = 12      # 반사광 < 노드임계+마진 이면 가로선 접근으로 본다
NODE_GUARD_MAX_MS = 700     # 직진 유지 상한 — 초과 지속이면 정렬 불량
MARKER_DEBOUNCE_MS = 1500   # 같은 마커 재감지 방지
MARKER_PAUSE_S = 0.08       # 마커 정지 후 잠깐 멈춤
CONFIRM_SETTLE_S = 0.08     # 재판정 전 정지 안정화
GRIP_SEC = 0.8              # 그리퍼 열기/닫기 구동 시간
LOOP_DELAY_S = 0.010        # 제어 루프 주기(gg5)
FOLLOW_LOG_S = 0.25         # LINE_FOLLOW 로그 최소 간격
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)
AWAIT_POLL_S = 0.05         # 결정 대기 중 명령 폴링 간격
SOFT_STOP_S = 0.12          # 감속 정지: slow 로 줄여 이만큼 굴린 뒤 brake

# 직진 종점 감속 — soft_stop 과 같은 철학을 엔코더 거리로: 남은 거리가
# STRAIGHT_DECEL_MM 이하면 저속으로 줄여 관성을 뺀 뒤 brake 한다(배달
# 전·후진/노드 전진이 급브레이크로 앞쏠리는 것 방지).
STRAIGHT_DECEL_MM = 25      # 이 거리 남으면 감속 구간 진입
STRAIGHT_DECEL_FRAC = 0.5   # 감속 속도 = |speed|×이 값
STRAIGHT_DECEL_MIN = 6      # 그 하한(%) — 단, 원래 speed 보다 빨라지진 않는다

# 대각선 진행(aplus 신설) — [q]/[e] 위치 교정용 아크 전진.
# 안쪽 바퀴를 바깥쪽의 DIAG_INNER_FRAC 배로 굴려 완만하게 비껴 나간다
# (diag_step_mm 40 기준 헤딩 변화 약 20도). 실행 후에도 계속 대기한다.
DIAG_INNER_FRAC = 0.30

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
# 램프 시작·커브 감속 하한은 base 비례(fxck2 의 상대 속도 철학) — gg5 는
# 절대값 12/12 였는데 base 15 에선 겨우 3% 여유라 '부드러운 느낌'이 죽었다.
# 0.6 비율은 gg5 기준 속도(20%)에서 정확히 예전 12 를 재현한다.
SPEED_REF = 20.0            # 조향(kp/ki/deadband) 실기 튜닝 기준 속도(%)
STEER_SCALE_MIN = 0.75      # 속도 비례 조향 배율 하한
STEER_SCALE_MAX = 2.0       # 배율 상한
ACCEL_START_FRAC = 0.6      # 출발 가속 램프 시작 = base×이 값(하한 아래)
ACCEL_START_MIN = 6         # 램프 시작 속도 하한(%)
CORNER_MIN_FRAC = 0.6       # 커브 자동 감속 하한 = base×이 값(하한 아래)
CORNER_MIN_MIN = 8          # 커브 감속 하한의 하한(%)
ERR_EMA_ALPHA = 0.30        # 커브 감속용 |error| EMA 계수
STRAIGHT_RAMP_MS = 250      # 직진 프리미티브 하드웨어 가속 램프(ms)

# 캘리브레이션 스윕(gg5 동일).
CAL_SPEED = 6
CAL_HALF_DEG = 60
CAL_MIN_SPAN = 20

# 소리 신호 — 그리퍼 이벤트만 남긴다(fxck3 철학). 결정 대기 진입/명령
# 소비 tone 은 없다: 판단이 넘어온 것은 조종 패드가 배경색으로 알린다.
GRAB_TONE = (880, 150)      # 물체 파지

# 오디오/LCD(fxck3 승계) — 저장소 sounds/ wav 를 비동기 큐(aplay)로 재생.
SOUND_ROOT = os.path.join(_ROOT, "sounds")
SOUND_RED = os.path.join(SOUND_ROOT, "red.wav")
SOUND_GOOD_JOB = os.path.join(SOUND_ROOT, "good_job.wav")
NUMBER_SOUNDS = {
    1: os.path.join(SOUND_ROOT, "num_1.wav"),
    2: os.path.join(SOUND_ROOT, "num_2.wav"),
    3: os.path.join(SOUND_ROOT, "num_3.wav"),
    4: os.path.join(SOUND_ROOT, "num_4.wav"),
    5: os.path.join(SOUND_ROOT, "num_5.wav"),
    6: os.path.join(SOUND_ROOT, "num_6.wav"),
}
RED_SAY_MAX = 6             # "red N" 은 1~6 — 초과 방문은 6 으로 클램프
FIXED_NUMBER = 2            # 발표 숫자 임시 고정 — 갈 때/올 때 둘 다 2(랜덤 봉인)
DISPLAY_REFRESH_S = 0.3     # LCD 유지 재출력 주기 — brickman 이 덮지 못하게


# ---------------------------------------------------------------------
# 라이브 파라미터 — 조종 패드/robotctl 로 실기에서 튜닝한다.
# ---------------------------------------------------------------------

# 파라미터당 한 줄: (이름, 초기값, min, max, max_step, ui_step, 단위)
PARAM_TABLE = (
    ("base_speed",      15,    5,   60,   5,    1,    "%"),   # aplus 기본 15
    ("curve_auto",      1,     0,   1,    1,    1,    ""),    # 1=커브 자동 통과(fxck2)
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
    ("goal_advance_mm", 100,   0,   200,  10,   10,   "mm"),  # 배달 전·후진
    ("diag_step_mm",    40,    10,  120,  20,   10,   "mm"),  # [q]/[e] 1회
    ("turn_90_factor",  0.65,  0.3, 2.0,  0.05, 0.01, "x"),
    ("turn_180_factor", 0.71,  0.3, 2.0,  0.05, 0.01, "x"),
    ("grab_dist_cm",    6.0,   1.0, 20.0, 1.0,  0.5,  "cm"),
    ("grip_speed",      50,    5,   80,   5,    1,    "%"),
    ("lost_persist_ms", 200,   0,   500,  100,  10,   "ms"),  # 000 지속 필터
)

INITIAL_PARAMS = dict((r[0], r[1]) for r in PARAM_TABLE)
PARAM_LIMITS = dict((r[0], (r[2], r[3])) for r in PARAM_TABLE)
MAX_STEP = dict((r[0], r[4]) for r in PARAM_TABLE)
UI_STEP = dict((r[0], r[5]) for r in PARAM_TABLE)
UNITS = dict((r[0], r[6]) for r in PARAM_TABLE)
PARAM_ORDER = [r[0] for r in PARAM_TABLE]

SAVE_PATH = os.path.join(_ROOT, "config", "aplus.json")
STAGE_NAME = "aplus"

# 파라미터 승계: aplus.json 없으면 gg8.json → gg5.json. base_speed 는
# 승계하지 않는다 — aplus 기본 15 를 유지한다(사용자 지정).
SEED_SOURCES = (
    (os.path.join(_ROOT, "config", "gg8.json"), ("base_speed",)),
    (os.path.join(_ROOT, "config", "gg5.json"), ("base_speed",)),
)

# 이동 명령 액션 → 큐 토큰. S/L/R/U 는 실행 후 라인추종 재개,
# DL/DR(대각선)/B(약간 후진)는 실행 후에도 그 자리에서 계속 대기(위치 교정).
MOVE_ACTIONS = {"fwd": "S", "left": "L", "right": "R", "uturn": "U",
                "diag_left": "DL", "diag_right": "DR", "back": "B"}
DIAG_MOVES = ("DL", "DR")
STAY_MOVES = DIAG_MOVES + ("B",)

# 조종 패드 키 = 액션 manifest 의 key. 구 대시보드에서 a/s/q 는 예약키라
# 순서 기반 폴백 키로 배정되지만, 전용 패드(aplus_pad)는 이 키를 그대로 쓴다.
ACTIONS = [
    {"name": "fwd", "label": "CMD Forward (resume follow)", "key": "w"},
    {"name": "uturn", "label": "CMD U-Turn 180", "key": "u"},
    {"name": "left", "label": "CMD Left 90", "key": "a"},
    {"name": "right", "label": "CMD Right 90", "key": "d"},
    {"name": "diag_left", "label": "Diag-Left nudge (stay)", "key": "q"},
    {"name": "diag_right", "label": "Diag-Right nudge (stay)", "key": "e"},
    {"name": "back", "label": "Back nudge (stay)", "key": "s"},
    {"name": "grip_close", "label": "Grip CLOSE (force)", "key": "p"},
    {"name": "grip_open", "label": "Grip Open", "key": "o"},
    {"name": "goal_drop", "label": "GOAL drop (green fallback)", "key": "7"},
    {"name": "home_drop", "label": "HOME drop (yellow fallback)", "key": "8"},
    {"name": "hold", "label": "Hold (stop, await cmd)", "key": "g"},
    {"name": "clear", "label": "Clear queued cmds", "key": "x"},
    {"name": "go", "label": "GO (start)", "key": "t"},
    {"name": "calibrate", "label": "Calibrate L/R on line (sweep)", "key": "n"},
    {"name": "read_reflect", "label": "Read L/R Reflect (raw+norm)", "key": "f"},
    {"name": "read_color", "label": "Read Center Color", "key": "h"},
    {"name": "reset", "label": "Reset to Start", "key": "z"},
]
# 수동 음성: [1]~[6] = "red N" 재생(카운터와 무관, 비동기 큐라 연타 시 순서대로).
ACTIONS += [{"name": "say_red_%d" % n, "label": "Say red %d" % n, "key": str(n)}
            for n in range(1, RED_SAY_MAX + 1)]


# ---------------------------------------------------------------------
# 순수 헬퍼(gg5/fxck2 동일)
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


def corner_min_speed(target):
    """커브 자동 감속 하한(순수) — base 비례(×CORNER_MIN_FRAC, 하한 MIN)."""
    return min(max(float(CORNER_MIN_MIN), float(target) * CORNER_MIN_FRAC),
               float(target))


def accel_start_speed(target):
    """출발 가속 램프 시작 속도(순수) — base 비례(×ACCEL_START_FRAC, 하한 MIN)."""
    return min(max(float(ACCEL_START_MIN), float(target) * ACCEL_START_FRAC),
               float(target))


def adaptive_base(target, corner_gain, err_ema, ramp_ms, elapsed_ms):
    """gg5 순항속도(순수): 커브 자동 감속 + 출발 가속 램프.

    하한/시작 속도가 base 비례라 base 15 에서도 커브 감속(15→9)과
    천천히 재출발(9→15)이 체감된다(gg5 절대값 12 는 base 15 에서 무감각)."""
    lo = corner_min_speed(target)
    base = clamp(float(target) - float(corner_gain) * float(err_ema),
                 lo, float(target))
    if ramp_ms > 0 and elapsed_ms < ramp_ms:
        start = accel_start_speed(target)
        cap = start + (base - start) * float(elapsed_ms) / float(ramp_ms)
        base = min(base, cap)
    return base


def confirm_speed(base_speed):
    """의심지점 confirm 저속 직진/노드 전진 속도(순수, fxck2 승계).

    base_speed 비례(×CONFIRM_SPEED_FRAC, 하한 MIN) — base 를 어떻게 튜닝해도
    의심지점에선 베이스 대비 같은 비율로 느리다."""
    return max(float(CONFIRM_SPEED_MIN),
               float(base_speed) * CONFIRM_SPEED_FRAC)


def slow_speed(base_speed):
    """감속 주행(SLOW_ON/유실 의심/노드 가드/soft stop) 속도(순수, fxck2 승계)."""
    return max(float(SLOW_SPEED_MIN), float(base_speed) * SLOW_SPEED_FRAC)


def straight_decel_speed(speed, remaining_mm):
    """직진 종점 감속 속도(순수) — 남은 거리가 감속 구간 밖이면 speed 그대로,
    안이면 저속(비례+하한, 원래 |speed| 초과 금지). 부호는 speed 를 따른다
    (후진 포함)."""
    if remaining_mm > STRAIGHT_DECEL_MM:
        return float(speed)
    mag = abs(float(speed))
    slow = min(mag, max(float(STRAIGHT_DECEL_MIN), mag * STRAIGHT_DECEL_FRAC))
    return slow if speed >= 0 else -slow


# ---------------------------------------------------------------------
# CommandQueue — FIFO 다칸 큐(연속 입력, 스레드 안전)
# ---------------------------------------------------------------------

class CommandQueue(object):
    """네트워크 스레드가 push(), 제어 루프가 결정 지점에서 take().

    gg8 의 1칸 큐와 달리 FIFO 다칸(최대 MAXLEN) — 주행 중 여러 명령을
    미리 눌러두면 결정 지점마다 하나씩 순서대로 소비된다(연속 입력).
    가득 차면 새 입력은 거부한다(잘못 연타한 폭주 방지 — [x] 로 비우고
    다시). peek_all() 은 텔레메트리 방송용(비소비), clear() 는 [x] 키.
    take() 는 (token, 대기했던 초) — 없으면 (None, None).
    """

    MAXLEN = 8

    def __init__(self):
        self._lock = threading.Lock()
        self._items = []        # [(token, push 시각), ...]

    def push(self, token):
        """큐 끝에 추가. (수용 여부, push 후 큐 스냅샷) 반환."""
        with self._lock:
            if len(self._items) >= self.MAXLEN:
                return False, [t for t, _ in self._items]
            self._items.append((token, time.monotonic()))
            return True, [t for t, _ in self._items]

    def clear(self):
        with self._lock:
            cleared = [t for t, _ in self._items]
            self._items = []
            return cleared

    def peek_all(self):
        with self._lock:
            return [t for t, _ in self._items]

    def take(self):
        with self._lock:
            if not self._items:
                return None, None
            token, pushed_t = self._items.pop(0)
        return token, time.monotonic() - pushed_t


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
# 구동 러너 — 주행은 자동, 결정은 조종 패드. run() 에서만 생성.
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

        self.cmd = CommandQueue()       # 이동 명령 FIFO(결정 지점 소비)
        self.session = 0
        self.pid = PidSteer()
        self._init_session_state()

        # LCD 유지 스레드(fxck3 승계) — 표시할 내용이 생기면 주기 재출력.
        self._display_stop = False
        self._display_thread = threading.Thread(target=self._display_loop,
                                                name="display")
        self._display_thread.daemon = True
        self._display_thread.start()

    def _init_session_state(self):
        self.visits = 0             # 빨강 확정 횟수
        self.goal_seen = False      # 초록(배달) 완료 여부 — red N 위상 전환
        self.grabbed = False
        self.done = False           # 노랑 복귀 완주
        self.go_on = False          # 조종 패드 GO 출발 플래그
        self.hold_on = False        # 패드 [Space]/[g] 대기 전환 플래그
        self.last_turn = 0.0
        self.last_marker_t = -1e9
        self.last_node_t = -1e9
        self.lost_since = None
        self.guard_since = None
        self.node_debounce_ms = 0
        self.err_ema = 0.0
        self.ramp_t0 = -1e9
        # UI 상태(fxck3 승계) — 판단/주행에는 쓰이지 않는다.
        self.timer_start = None     # 스톱워치 시작 시각(출발 시)
        self.out_elapsed = None     # 초록 도착 소요시간(LCD 첫 줄 OUT)
        self.return_elapsed = None  # 노랑 복귀 누적 소요시간(LCD 둘째 줄 BACK)
        self.bottom_number = None   # LCD 하단 랜덤 숫자(1~4)
        self.out_red_spoken = 0     # "red N" 카운트 — 가는 길
        self.return_red_spoken = 0  # "red N" 카운트 — 초록 후(다시 1부터)

    # ---- 네트워크 핸들러(TuningServer 스레드에서 호출) ----

    def on_stop(self, source):
        self.stop_on = True
        self.stop_source = source

    def on_pause(self, paused, source):
        self.paused = bool(paused)
        self.log.log("PAUSE" if paused else "RESUME", "NETWORK", source=source)
        return {"mode": "paused" if paused else "run"}

    def on_do(self, action, args):
        """조종 패드 액션 분기 — 이동 명령/GO/clear 는 여기(네트워크 스레드)서
        즉시 반영한다(블로킹 없음, '주행 중 미리 입력' 보장). 모터를 움직이는
        액션(grip/calibrate/판독)은 제어 루프 FIFO 로 넘긴다."""
        if action in MOVE_ACTIONS:
            token = MOVE_ACTIONS[action]
            accepted, queued = self.cmd.push(token)
            if not accepted:
                self.log.log("CMD_FULL", "DASHBOARD", move=token,
                             queue=",".join(queued))
                return {"rejected": token, "reason": "queue_full",
                        "queue": queued}
            self.log.log("CMD_PUSH", "DASHBOARD", move=token,
                         queue=",".join(queued))
            return {"queued_move": token, "queue": queued}
        if action == "clear":
            cleared = self.cmd.clear()
            self.log.log("CMD_CLEAR", "DASHBOARD",
                         cleared=",".join(cleared) if cleared else "-")
            return {"cleared": cleared}
        if action == "go":
            self.go_on = True
            self.log.log("GO", "DASHBOARD")
            return {"queued": "go"}
        if action == "hold":
            # [Space] 대기 전환 — 플래그만 세우고(네트워크 스레드) 제어
            # 루프가 안전한 시점에 분기/커브와 동일한 결정 대기로 들어간다.
            self.hold_on = True
            self.log.log("HOLD", "DASHBOARD")
            return {"queued": "hold"}
        if action == "reset":
            self.reset_on = True
            self.reset_source = (args or {}).get("source", "dashboard")
            return {"queued": "reset"}
        if action.startswith("say_red_"):
            # 수동 음성 — play_wav 는 오디오 큐에 넣기만 하므로(비블로킹)
            # 여기(네트워크 스레드)서 즉시 처리한다. red N 카운터는 건드리지
            # 않는다(마커 방문 음성과 별개).
            try:
                number = int(action[len("say_red_"):])
            except ValueError:
                return {"ok": False, "error": "bad action: " + action}
            number = int(clamp(number, 1, RED_SAY_MAX))
            self.hw.play_wav(SOUND_RED)
            self.play_number(number)
            self.log.log("SAY_RED", "DASHBOARD", number=number)
            return {"queued": "say_red_%d" % number}
        with self._pending_lock:
            self._pending.append(action)
        return {"queued": action}

    # ---- 공용 ----

    def interrupted(self):
        return self.stop_on or self.reset_on

    def motion_break(self):
        """모션 프리미티브(직진/회전/대각선) 중단 조건 — stop/reset 에 더해
        hold(대기 전환)도 실행 중간에 즉시 끊는다. 플래그는 여기서 지우지
        않는다 — 상위 루프(drive_loop/decide/await)가 대기 진입 때 흡수."""
        return self.interrupted() or self.hold_on

    def publish(self, mode, **extra):
        queue = self.cmd.peek_all()
        frame = {
            "t_ms": int((time.monotonic() - self.started) * 1000),
            "param_rev": self.params.rev(),
            "running": True,
            "mode": mode,
            "paused": self.paused,
            "session": self.session,
            "visits": self.visits,
            "grabbed": self.grabbed,
            "goal_seen": self.goal_seen,
            "pending_cmd": ",".join(queue) if queue else "-",
            "queue_len": len(queue),
        }
        if self.out_elapsed is not None:
            frame["out_s"] = round(self.out_elapsed, 1)
        if self.return_elapsed is not None:
            frame["back_s"] = round(self.return_elapsed, 1)
        frame.update(extra)
        last_reason = self.log.last_reason()
        if last_reason is not None:
            frame["last_reason"] = last_reason
        events = self.log.drain_events()
        if events:
            frame["events"] = events
        self.tele.publish(frame)

    def handle_pending(self):
        """제어 루프 시점 액션 처리(그립/캘리브레이션/판독)."""
        while True:
            with self._pending_lock:
                if not self._pending:
                    return
                action = self._pending.pop(0)
            snap = self.params.snapshot()
            if action == "calibrate":
                self.calibrate_line()
            elif action == "goal_drop":
                self.manual_goal()
            elif action == "home_drop":
                self.manual_home()
            elif action == "grip_open":
                self.hw.stop()
                self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
                self.grabbed = False
                self.log.log("GRIP", "OPEN_DASHBOARD")
                self.reset_steer()
            elif action == "grip_close":
                # [p] 강제 파지 — 주행 중이면 세우고 잡은 뒤 램프로 재개.
                self.hw.stop()
                self.hw.grip_close(snap["grip_speed"], GRIP_SEC)
                self.grabbed = True
                self.hw.tone(GRAB_TONE[0], GRAB_TONE[1])
                self.log.log("GRIP", "CLOSE_DASHBOARD")
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

    # ---- 오디오/LCD(fxck3 승계) — 전부 비동기·best-effort ----

    def play_number(self, number):
        """숫자음 재생(비동기 큐). wav 가 없으면 beep 폴백."""
        path = NUMBER_SOUNDS.get(int(number))
        if path is None:
            self.hw.beep_ok()
            return
        self.hw.play_wav(path)

    def choose_random_number(self, phase):
        """발표 숫자를 LCD 하단에 표시(기존 값 대체)하고 숫자음 재생.
        임시 고정(요청): 갈 때(START)/올 때(GREEN) 둘 다 FIXED_NUMBER=2."""
        self.bottom_number = FIXED_NUMBER
        self.refresh_display()
        self.play_number(self.bottom_number)
        self.log.log("RANDOM_NUMBER", phase, number=self.bottom_number)

    def announce_red(self, phase):
        """빨강 방문 음성 "red N" — 가는 길 1~6, 초록 후 다시 1부터 6."""
        if phase == "RETURN":
            self.return_red_spoken += 1
            number = self.return_red_spoken
        else:
            self.out_red_spoken += 1
            number = self.out_red_spoken
        self.hw.play_wav(SOUND_RED)
        self.play_number(min(number, RED_SAY_MAX))
        self.log.log("RED_SPOKEN", phase, number=number)

    def refresh_display(self):
        self.hw.show_final4_display(self.out_elapsed, self.return_elapsed,
                                    self.bottom_number)

    def _display_loop(self):
        """표시할 내용이 생긴 뒤부터 현재 상태를 주기 재출력해 화면을 유지한다."""
        while not self._display_stop:
            if (self.out_elapsed is not None or self.return_elapsed is not None
                    or self.bottom_number is not None):
                self.refresh_display()
            time.sleep(DISPLAY_REFRESH_S)

    # ---- 조향 상태 ----

    def soft_stop(self):
        """감속 정지 — 순항 속도에서 급브레이크 대신 slow 로 한 템포 줄여
        관성을 빼고 선다(앞쏠림/미끄러짐으로 마커를 지나치는 것 방지)."""
        slow = slow_speed(self.params.snapshot()["base_speed"])
        self.hw.drive(slow, slow)
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

    # ---- 모션 프리미티브(gg5 동일) ----

    def straight(self, dist_mm, speed, mode="advancing"):
        """엔코더 기준 직진(speed<0 후진). hw 가속 램프로 부드럽게 출발하고,
        종점 STRAIGHT_DECEL_MM 은 저속으로 굴려 부드럽게 선다(급브레이크 X)."""
        self.hw.reset_encoders()
        if dist_mm <= 0:
            return 0.0
        target_deg = dist_mm / MM_PER_DEG
        self.hw.set_ramp(STRAIGHT_RAMP_MS)
        last_cmd = None
        try:
            while self.hw.enc_avg() < target_deg:
                if self.motion_break():
                    break
                if self.paused:
                    self._hold_while_paused(mode)
                    if self.motion_break():
                        break
                    last_cmd = None     # 재개 시 속도 재명령
                done_mm = self.hw.enc_avg() * MM_PER_DEG
                cmd = straight_decel_speed(speed, dist_mm - done_mm)
                if cmd != last_cmd:     # 속도가 바뀔 때만 명령(순항→감속 2회)
                    self.hw.drive(cmd, cmd)
                    last_cmd = cmd
                self.publish(mode, dist_mm=round(done_mm, 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
            self.hw.set_ramp(0)
        return self.hw.enc_avg() * MM_PER_DEG

    def turn(self, move):
        """제자리 회전(L/R/U) + PID 리셋 + 회전 직후 라인 재획득(gg5 동일).
        [a]/[d]=90도, [u]=180도 유턴. "S" 는 no-op(그대로 직진 재개)."""
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
                if self.motion_break():
                    break
                if self.paused:
                    self._hold_while_paused("turning")
                    if self.motion_break():
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
                     stopped_early=self.motion_break())
        self.reset_steer()
        # 회전 직후 라인 재획득 — 피벗 오차로 중앙이 라인 밖이면 소각 스캔.
        if not self.motion_break():
            color = self.hw.read_center_color_now()
            if not on_line(color):
                self.log.log("TURN_ACQUIRE", "CENTER_OFF_LINE_AFTER_TURN",
                             move=move, color=color)
                self.realign_to_line(self.params.snapshot())

    def diag_nudge(self, token):
        """[q]/[e] 대각선 약간 진행(aplus 신설) — 위치 교정용 아크 전진.

        안쪽 바퀴를 DIAG_INNER_FRAC 배로 굴려 완만하게 비껴 나간다
        (diag_step_mm 평균 이동). 실행 후 호출부가 계속 대기하므로 연타로
        미세 조정한 뒤 [w] 로 재개하면 된다. 조향/heading 상태는 없다."""
        snap = self.params.snapshot()
        outer = float(STRAIGHT_SPEED)
        inner = outer * DIAG_INNER_FRAC
        if token == "DL":
            left, right = inner, outer      # 왼쪽으로 비껴 나간다
        else:
            left, right = outer, inner
        target = snap["diag_step_mm"] / MM_PER_DEG
        self.hw.reset_encoders()
        self.hw.set_ramp(STRAIGHT_RAMP_MS)
        try:
            self.hw.drive(left, right)
            while self.hw.enc_avg() < target:
                if self.motion_break():
                    break
                if self.paused:
                    self._hold_while_paused("diag_nudge")
                    if self.motion_break():
                        break
                    self.hw.drive(left, right)
                self.publish("diag_nudge", moving=token,
                             dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
            self.hw.set_ramp(0)
        self.log.log("DIAG_NUDGE", "LEFT" if token == "DL" else "RIGHT",
                     step_mm=snap["diag_step_mm"],
                     dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1))

    def back_nudge(self):
        """[s] 약간 후진(aplus 신설) — 위치 교정용 직선 후진.

        diag_step_mm 만큼 뒤로 물러난다(종점 감속 포함 — straight 재사용).
        실행 후 호출부가 계속 대기하므로 연타로 물러난 뒤 [w] 로 재개한다."""
        snap = self.params.snapshot()
        moved = self.straight(snap["diag_step_mm"], -STRAIGHT_SPEED,
                              mode="back_nudge")
        self.log.log("BACK_NUDGE", "PAD_KEY", step_mm=snap["diag_step_mm"],
                     dist_mm=round(moved, 1))

    # ---- 결정 지점(gg8 승계 + 대각선/FIFO 확장) ----

    def decide_and_move(self, kind, has_left, has_right, has_straight,
                        color=None):
        """분기/커브/유실/마커 도착 — 큐 명령을 소비해 실행한다.

        큐에 명령이 있으면(주행/미끄러지는 중 미리 입력) 즉시 실행, 없으면
        정지 상태로 기다린다(패드가 배경색으로 알림). 대각선(DL/DR)/약간
        후진(B)은 실행 후 같은 결정 지점에서 계속 명령을 소비/대기한다
        (위치 교정 체이닝) — w/a/d/u 가 오면 이동 후 라인추종으로 복귀한다.
        명령은 출구 유효성 검사 없이 실행한다(조종자가 보스) — 다르면
        로그에 흔적만 남긴다."""
        avail = {"L": has_left, "R": has_right, "S": has_straight, "U": True,
                 "DL": True, "DR": True, "B": True}
        while True:
            if self.interrupted() or self.done:
                return
            if self.hold_on:
                # 대기 전환이 모션을 끊고 여기로 왔다 — 이미 결정 지점이므로
                # 플래그만 흡수한다(아래 take/await 가 대기 그 자체).
                self.hold_on = False
                self.hw.stop()
            move, waited = self.cmd.take()
            source = "QUEUED"
            if move is None:
                move = self.await_command(kind, has_left, has_right,
                                          has_straight, color)
                source = "AWAITED"
                if move is None:
                    return              # stop/reset 중단
            self.log.log("DECISION", kind.upper(), move=move, source=source,
                         queued_s=(round(waited, 2) if waited is not None
                                   else None),
                         has_left=has_left, has_right=has_right,
                         has_straight=has_straight,
                         off_menu=(not avail.get(move, True)))
            if move in STAY_MOVES:
                if move == "B":
                    self.back_nudge()
                else:
                    self.diag_nudge(move)
                continue                # 같은 결정 지점 — 계속 소비/대기
            self.turn(move)             # "S" 는 no-op — 그대로 직진 재개
            self.reset_steer()          # 재개는 가속 램프부터
            return

    def await_command(self, kind, has_left, has_right, has_straight, color):
        """정지 대기 — 이동 명령이 올 때까지 상황을 방송하며 기다린다.

        대기 중에도 handle_pending 이 돌므로 grip/calibrate/판독을 먼저
        실행할 수 있다. 알림 tone 없음 — 조종 패드가 배경색으로 알린다.
        반환: 큐 토큰 또는 None(stop/reset/완주)."""
        self.hw.stop()
        self.log.log("AWAIT_CMD", kind.upper(),
                     has_left=has_left, has_right=has_right,
                     has_straight=has_straight,
                     color=(color if color is not None else "-"))
        while True:
            if self.interrupted() or self.done:
                return None
            self.hold_on = False    # 이미 대기 중 — hold 는 매 사이클 흡수
            if self.paused:
                self._hold_while_paused("await_cmd")
                continue
            self.handle_pending()
            move, _waited = self.cmd.take()
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
        """빨강/노랑/초록 마커: 정지 → 색 확정 → UI/그리퍼 → 결정 대기.

        이동 판단은 전부 사람(fxck3 승계):
        빨강 = "red N" 후 결정 대기 / 초록(최초) = OUT 시간+good_job+랜덤
        숫자+배달 후 결정 대기 / 노랑(초록 후) = 완주(done) / 초록 전 노랑·
        두 번째 초록 = 로그만 남기고 주행 재개. 처리했으면 True."""
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
        self.log.log("MARKER", "COLOR_{}_CONFIRMED".format(name.upper()),
                     color=color, context=context, visits=self.visits,
                     session=self.session)
        self.last_marker_t = time.monotonic()

        if color == COL_RED:
            self.visits += 1
            self.announce_red("RETURN" if self.goal_seen else "OUT")
            self.decide_and_move("red", False, False, False, color=color)
            self.last_marker_t = time.monotonic()
            return True

        if color == COL_GREEN:
            if self.goal_seen:
                self.log.log("MARKER_IGNORED", "GREEN_AGAIN")
                self.ramp_t0 = time.monotonic()
                return True
            # 초록 도착(목적지): good_job + OUT 시간 + 새 랜덤 숫자 — 전부
            # 비동기라 이어지는 배달 동작과 병행된다(fxck3 승계).
            if self.timer_start is not None and self.out_elapsed is None:
                self.out_elapsed = time.monotonic() - self.timer_start
                self.log.log("STOPWATCH_OUT", "COLOR_GREEN",
                             elapsed_s=round(self.out_elapsed, 1))
            self.hw.play_wav(SOUND_GOOD_JOB)
            self.choose_random_number("GREEN")
            self.deliver()      # 물체 내려놓기 — 유턴은 사람이 한다
            if self.interrupted():
                return True
            self.decide_and_move("green", False, False, False, color=color)
            self.last_marker_t = time.monotonic()
            return True

        # COL_YELLOW
        if not self.goal_seen:
            self.log.log("MARKER_IGNORED", "YELLOW_BEFORE_GREEN")
            self.ramp_t0 = time.monotonic()
            return True
        # 완주: BACK 시간 표시 + 스톱워치 정지 + 그립 해제(fxck3 승계).
        if self.timer_start is not None and self.return_elapsed is None:
            self.return_elapsed = time.monotonic() - self.timer_start
            self.refresh_display()
            self.log.log("STOPWATCH_RETURN", "COLOR_YELLOW",
                         elapsed_s=round(self.return_elapsed, 1))
        self._release_at_home()
        self.done = True
        self.log.log("HOME_REACHED", "COLOR_YELLOW", visits=self.visits)
        return True

    def deliver(self):
        """초록점: 운반해 온 물체를 내려놓는다 — 전진→그립 오픈→후진(gg5).
        (그립을 열면 grabbed=False → 이후 물체를 다시 만나면 자동 재파지.)"""
        snap = self.params.snapshot()
        self.goal_seen = True
        self.log.log("GOAL_DROP", "COLOR_GREEN",
                     goal_advance_mm=snap["goal_advance_mm"])
        self.straight(snap["goal_advance_mm"], STRAIGHT_SPEED)
        if self.motion_break():     # stop/reset/hold — 배달 잔여 절차 중단
            return
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
        self.grabbed = False
        self.straight(snap["goal_advance_mm"], -STRAIGHT_SPEED)

    def manual_goal(self):
        """[7] 수동 도착 처리(aplus 신설) — 초록 마커 미인식 대비 폴백.

        초록 확정 직후 절차를 그 자리에서 그대로 시행한다: 감속 정지 →
        OUT 시간 + good_job + 랜덤 숫자 → 배달(전진→그립 오픈→후진) →
        180도 회전(원래 사람이 [s] 로 하던 마무리)까지 이어서 하고
        라인추종으로 복귀한다. 검사 없이 그대로 실행(조종자가 보스) —
        이미 배달했어도 다시 한다(goal_seen 은 로그로 흔적만)."""
        self.soft_stop()
        self.log.log("MANUAL_GOAL", "PAD_KEY", goal_seen=self.goal_seen)
        if self.timer_start is not None and self.out_elapsed is None:
            self.out_elapsed = time.monotonic() - self.timer_start
            self.log.log("STOPWATCH_OUT", "MANUAL_GOAL",
                         elapsed_s=round(self.out_elapsed, 1))
        self.hw.play_wav(SOUND_GOOD_JOB)
        self.choose_random_number("MANUAL_GOAL")
        self.deliver()
        if self.motion_break():     # stop/reset/hold — 마무리 유턴 생략
            return
        self.turn("U")
        self.reset_steer()
        self.last_marker_t = time.monotonic()   # 뒤에 깔린 초록 재감지 방지

    def manual_home(self):
        """[8] 수동 복귀 도착 처리(aplus 신설) — 노랑 마커 미인식 대비 폴백.

        노랑 확정 직후 완주 절차를 그 자리에서 그대로 시행한다: 감속 정지 →
        BACK 시간 표시 + 스톱워치 정지 → 그립 해제 → 완주(done, 가운데 버튼
        또는 reset 으로 새 세션). 검사 없이 그대로 실행(조종자가 보스) —
        goal_seen 여부는 로그로 흔적만 남긴다."""
        self.soft_stop()
        self.log.log("MANUAL_HOME", "PAD_KEY", goal_seen=self.goal_seen)
        if self.timer_start is not None and self.return_elapsed is None:
            self.return_elapsed = time.monotonic() - self.timer_start
            self.refresh_display()
            self.log.log("STOPWATCH_RETURN", "MANUAL_HOME",
                         elapsed_s=round(self.return_elapsed, 1))
        self._release_at_home()
        self.done = True
        self.log.log("HOME_REACHED", "MANUAL_HOME", visits=self.visits)

    def _release_at_home(self):
        """노란점 완주: 물체를 내려놓는다(그립 해제)."""
        snap = self.params.snapshot()
        if self.grabbed:
            self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
            self.grabbed = False
            self.log.log("DROP_HOME", "COLOR_YELLOW")

    def confirm_node(self, first_bits, snap):
        """의심지점: PID off → 저속 직진(미끄러지며) → 정지 후 재판정(gg5).

        이 creep 동안에도 네트워크 스레드가 CommandQueue 를 채울 수 있다 —
        '미끄러지는 동안 미리 입력'이 여기서 성립한다."""
        creep = confirm_speed(snap["base_speed"])
        self.reset_steer()
        self.hw.reset_encoders()
        self.log.log("NODE_CANDIDATE", "PD_OFF_SLOW_STRAIGHT",
                     bits=bits_str(first_bits),
                     confirm_ms=snap["node_confirm_ms"],
                     speed=round(creep, 1))
        end = time.monotonic() + snap["node_confirm_ms"] / 1000.0
        self.hw.drive(creep, creep)
        while time.monotonic() < end:
            if self.interrupted():
                self.hw.stop()
                return None, self.hw.enc_avg() * MM_PER_DEG
            if self.paused:
                self._hold_while_paused("node_confirm")
                if self.interrupted():
                    return None, self.hw.enc_avg() * MM_PER_DEG
                self.hw.drive(creep, creep)
            bits, color, rl, rr = self.read_bits(snap)
            if self.handle_marker(color, "node_confirm"):
                return None, 0.0
            if bits not in NODE_CANDIDATES:
                self.hw.stop()
                creep_mm = self.hw.enc_avg() * MM_PER_DEG
                if bits == LOST_BITS and first_bits[0] == 1 and first_bits[2] == 1:
                    # 좌/우 팔이 있던 후보(111/101)가 전백 — 가로선을
                    # 지나쳤다는 확실한 증거(passed-over).
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
        """확정 노드: 총 전진을 node_advance_mm 로 맞춘 뒤 처리.

        커브(출구 1개)는 자동 통과(fxck2) — 멈추지 않고 뻔한 출구로 돌아
        흐름을 유지한다(curve_auto=0 이면 gg8 처럼 정지 후 명령 대기).
        자동 통과는 명령 큐를 건드리지 않는다 — 큐는 결정 지점(분기/유실/
        마커)에서만 소비된다. 분기(출구 2+)/막힘은 사람이 정한다."""
        snap = self.params.snapshot()
        self.straight(max(0.0, snap["node_advance_mm"] - creep_mm),
                      confirm_speed(snap["base_speed"]))
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
            move = "L" if has_left else ("R" if has_right else "S")
            if snap["curve_auto"] >= 1:
                self.log.log("CURVE", "AUTO_" + {"L": "LEFT", "R": "RIGHT",
                                                 "S": "STRAIGHT"}[move],
                             bits=bits_str(bits), color=color)
                self.turn(move)
                self.reset_steer()      # "S" 포함 — 재개는 가속 램프부터
                return
            self.log.log("CURVE", "STOP_FOR_CMD",
                         bits=bits_str(bits), color=color, suggest=move)
            self.decide_and_move("curve", has_left, has_right, has_straight)
            return

        self.decide_and_move("junction", has_left, has_right, has_straight)

    # ---- 유실(000) — 자동 복구 없음: 확정되면 사람을 부른다 ----

    def lost_check(self, snap):
        """000 지속 확정 전 재판정: 즉시 정지 → 정지 상태 복수 샘플.
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
        """회전 직후 라인 재획득 — 중앙이 라인 위에 오도록 좌/우 소각 스캔.

        (유실 자동 복구가 아니라 사람이 시킨 회전의 마무리 — 피벗 오차
        보정이다. 실패해도 그냥 두면 곧 유실 확정 → 결정 대기로 넘어간다.)"""
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

    # ---- 세션 단계 ----

    def wait_for_start(self):
        """가운데 버튼 press→release 또는 조종 패드 [t] GO 로 출발.

        떼는 순간 스톱워치 시작 + 랜덤 숫자(1~4) LCD 하단 표시·재생(fxck3).
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
        self.hold_on = False        # 출발 대기 중 눌린 hold 잔재는 무시
        self.timer_start = time.monotonic()     # 스톱워치 시작(세션 내 지속)
        self.choose_random_number("START")
        self.straight(START_EXIT_MM, STRAIGHT_SPEED, mode="start_exit")
        self.last_marker_t = time.monotonic()   # 출발 노랑 재감지 방지
        self.ramp_t0 = time.monotonic()         # 첫 라인추종도 가속 램프부터
        return "go"

    def drive_loop(self):
        """라인트레이싱 메인 루프 — status(stop/reset/done) 반환."""
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
            if self.done:
                continue            # [8] 수동 완주 — 모터는 이미 정지 상태
            if self.hold_on:
                # [Space] 대기 전환 — 분기/커브와 동일하게 감속 정지 후
                # 결정 대기(패드 배경색 알림)로 들어간다.
                self.hold_on = False
                self.soft_stop()
                self.decide_and_move("hold", False, False, False)
                continue

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
                    self.lost_since = None      # 위빙 중 000 은 무시
                elif self.lost_since is None:
                    self.lost_since = now
                elif (now - self.lost_since) * 1000 >= snap["lost_persist_ms"]:
                    self.lost_since = None
                    verdict = self.lost_check(snap)
                    self.reset_steer()
                    if verdict == "lost":
                        # 자동 복구 없음(aplus) — 그 자리에서 사람을 부른다.
                        self.decide_and_move("lost", False, False, False)
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
                    guard = slow_speed(snap["base_speed"])
                    self.hw.drive(guard, guard)
                    self.last_turn = 0.0
                    self.publish("node_guard", reflect_l=rl, reflect_r=rr,
                                 color=color, bits=bits_str(bits))
                    time.sleep(LOOP_DELAY_S)
                    continue
            else:
                if self.guard_since is not None:
                    self.ramp_t0 = now      # 가드→순항 복귀도 램프로
                self.guard_since = None

            slow = bits in SLOW_ON or self.lost_since is not None
            if slow:
                base = slow_speed(snap["base_speed"])
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
                         left_speed=round(left, 1), right_speed=round(right, 1))
            time.sleep(LOOP_DELAY_S)
        return "done"

    def idle_after_done(self):
        """완주 후 대기 — LCD 기록을 유지한 채 리셋을 기다린다(fxck3)."""
        self.hw.stop()
        while True:
            if self.stop_on:
                self.log.log("EMERGENCY_STOP", "NETWORK", source=self.stop_source)
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            self.publish("finished")
            pressed = self.hw.wait_center_button(
                stop_cb=lambda: self.stop_on or self.reset_on, timeout=0.2)
            if pressed:
                self.reset_source = "center_button"
                return "reset"

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
        """세션 = 출발 대기 → 조종 주행 → 완주 후 대기. reset 은 새 세션."""
        while not self.stop_on:
            self.new_session()
            status = self.wait_for_start()
            if status == "go":
                status = self.drive_loop()
            if status == "done":
                status = self.idle_after_done()
            if status == "stop":
                break
            # status == "reset" → 루프 상단에서 new_session.


def seed_initial(initial):
    """aplus 첫 실행 승계: aplus.json 이 없으면 SEED_SOURCES 순서(gg8→gg5)로
    실기 튜닝값을 initial 에 병합해 (승계 이름 리스트, 출처 경로)를 반환.
    base_speed 는 skip — aplus 기본 15 유지."""
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
    print("aplus ready — REMOTE DECISION MODE (base 15%). robot line-follows, "
          "flows through curves (curve_auto=1), stops at junction/marker/lost. "
          "pad keys (tools/aplus_pad.py): [w]fwd [a]left90 [d]right90 "
          "[u]uturn180 [q]diag-L [e]diag-R [s]back (stay) [p]grip-close [o]open "
          "[7]goal-drop(green fallback) "
          "[x]clear [t]GO [n]calibrate [z]reset [1-6]say-red-N. "
          "commands queue FIFO (max 8) "
          "and are consumed one per stop. start: CENTER button or [t]. "
          "(Ctrl-C or robotctl stop to quit)")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        runner._display_stop = True
        try:
            hw.stop()
        finally:
            server.stop()
    print("aplus stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
