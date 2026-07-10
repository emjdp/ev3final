#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fxck3 — 풀 수동 조종(wasd). 라인트레이싱/탐색/복귀 없음 — 사람이 다 몬다.

조종층(fxck3 신설):
  - 출발 후 로봇은 세션 내내 mode="manual_wait" 로 publish 하며 대시보드
    키를 기다린다 — fxck1 의 manual_wait 게이트를 상시 켜 두는 것. 대시보드는
    manual_wait 일 때만 manual_* 키를 액션으로 보내므로(그 외엔 s=STOP,
    a=auto-rerun 등 내장 의미) 대시보드 수정 없이 wasd 가 로봇으로 온다.
    이동 중에도 같은 모드로 publish 해 연타가 내장 STOP 으로 새지 않는다
    (비상 정지: Space=pause, robotctl stop, Ctrl-C — 어차피 키 하나당
    step_mm/turn_deg 만 움직이고 멈추므로 폭주가 없다).
      [w] 전진(step_mm)      [s] 180도 유턴
      [a] 좌로 90도          [d] 우로 90도
      [j] 좌로 turn_deg(30)  [l] 우로 turn_deg(30)
    연타하면 이동 중 눌린 키가 한 칸 큐(마지막 입력이 이김)에 남아 이어서
    수행된다. 출발 대기/완주 대기 중엔 wasd 를 무시한다(주행 중에만 무장).
  - 전진 중 중앙 컬러가 마커색이면 즉시 멈추고 fxck1 과 같은 정지 재판독
    다수결로 확정한다. 자동 '판단'(회전/경로)은 없다 — 마커는 UI/그리퍼
    이벤트만 일으킨다:
      빨강: "red N" 재생 — 가는 길 1~6, 초록 후 다시 1부터.
      초록(최초): good_job + OUT 시간 표시 + 랜덤 숫자 갱신·재생 + 배달
        (전진→그립 오픈→후진, gg5 deliver 그대로) — 유턴은 사람이 한다.
      노랑(초록 후): 완주 — BACK 시간 표시 + 스톱워치 정지 + 그립 해제,
        가운데 버튼(또는 reset)으로 새 세션.
      초록 전 노랑 / 두 번째 초록: 로그만 남기고 무시(디바운스 후 재판독).
  - 그리퍼/디스플레이/소리 층은 fxck1/2 그대로 승계(건드리지 않음):
    초음파 grab_dist 이내 자동 파지+삡(대기/전진 중 모두), 가운데 버튼
    출발+스톱워치+랜덤 숫자(1~4), LCD 유지 스레드(0.3s 재출력), wav 큐.
  - 좌/우 반사광 센서와 PID/노드/유실/Explorer/복귀 계획은 아예 없다.
    캘리브레이션도 필요 없다(조향을 안 하므로).

파라미터(라이브 튜닝): fwd_speed(직진 속도) / step_mm([w] 1회 거리) /
  turn_deg([j][l] 1회 각도) / turn_speed·turn_ramp_ms·turn_90_factor(피벗) /
  goal_advance_mm(배달 전·후진) / grab_dist_cm·grip_speed(그리퍼).
  config/fxck3.json 이 없으면 fxck2→fxck1→gg5 저장값에서 겹치는 이름을
  자동 승계한다(피벗/그리퍼 계열 — 실기 튜닝 재사용).

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      정지는 네트워크 stop 또는 Ctrl-C, 재시작은 가운데 버튼 또는 reset.

실행(브릭):   python3 stages/fxck3.py
문법 점검(PC): python3 -m py_compile stages/fxck3.py lib/*.py
단위 테스트:  python3 tests/test_fxck3_logic.py (ev3dev2 불필요)
"""

import json
import os
import random
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
# 상수 — 색/기하/고정 주행값(gg5/fxck1 과 동일 값 유지)
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

# 기하(정리.md 엔코더/기하값)
MM_PER_DEG = 3.14159265 * 56.0 / 360.0      # 바퀴 지름 56mm
BASE_PIVOT_DEG_90 = 193.0
POST_TURN_SETTLE_S = 0.12

# 고정 주행값
START_EXIT_MM = 50          # 출발 노랑에서 벗어나는 거리
STRAIGHT_RAMP_MS = 250      # 직진 하드웨어 가속 램프(ms) — 출발 슬립 방지
GRIP_SEC = 0.8              # 그리퍼 열기/닫기 구동 시간
MANUAL_POLL_S = 0.05        # 키 대기 폴링 주기
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)

# 마커 확정 — 색은 1프레임이 아니라 정지 후 재판독 다수결로 확정한다(fxck1).
MARKER_DEBOUNCE_MS = 1500   # 같은 마커 재감지 방지
MARKER_PAUSE_S = 0.08       # 마커 정지 후 잠깐 멈춤
MARKER_CONFIRM_SAMPLES = 4  # 정지 후 재판독 횟수
MARKER_CONFIRM_MIN = 3      # 같은 마커 색이 이만큼 이상이어야 확정
MARKER_CONFIRM_GAP_S = 0.015

# 오디오/LCD(gg3 그대로) — 저장소 sounds/ wav 를 비동기 큐(aplay)로 재생.
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
GRAB_TONE = (880, 150)      # 물체 파지 삡소리(Hz, ms)
DISPLAY_REFRESH_S = 0.3     # LCD 유지 재출력 주기 — brickman 이 덮지 못하게


# ---------------------------------------------------------------------
# 라이브 파라미터 — 대시보드/robotctl 로 실기에서 튜닝한다.
# ---------------------------------------------------------------------

# 파라미터당 한 줄: (이름, 초기값, min, max, max_step, ui_step, 단위)
PARAM_TABLE = (
    ("fwd_speed",       20,    5,   60,   5,    1,    "%"),   # [w] 직진 속도
    ("step_mm",         50,    10,  200,  50,   10,   "mm"),  # [w] 1회 거리
    ("turn_deg",        30,    5,   90,   30,   5,    "deg"),  # [j]/[l] 1회 각도
    ("turn_speed",      8,     5,   40,   5,    1,    "%"),
    ("turn_ramp_ms",    250,   0,   600,  100,  50,   "ms"),  # 피벗 가속 램프
    ("turn_90_factor",  0.65,  0.3, 2.0,  0.05, 0.01, "x"),   # 90도 피벗 실측 배율
    ("goal_advance_mm", 100,   0,   200,  10,   10,   "mm"),  # 배달 전·후진 거리
    ("grab_dist_cm",    6.0,   1.0, 20.0, 1.0,  0.5,  "cm"),
    ("grip_speed",      50,    5,   80,   5,    1,    "%"),
)

INITIAL_PARAMS = dict((r[0], r[1]) for r in PARAM_TABLE)
PARAM_LIMITS = dict((r[0], (r[2], r[3])) for r in PARAM_TABLE)
MAX_STEP = dict((r[0], r[4]) for r in PARAM_TABLE)
UI_STEP = dict((r[0], r[5]) for r in PARAM_TABLE)
UNITS = dict((r[0], r[6]) for r in PARAM_TABLE)
PARAM_ORDER = [r[0] for r in PARAM_TABLE]

SAVE_PATH = os.path.join(_ROOT, "config", "fxck3.json")
STAGE_NAME = "fxck3"

# 실기 튜닝값 승계(최초 1회) — fxck3.json 이 없으면 fxck2→fxck1→gg5 중
# 처음 존재하는 저장값에서 겹치는 이름만 가져온다(피벗/그리퍼 계열).
SEED_PATHS = (os.path.join(_ROOT, "config", "fxck2.json"),
              os.path.join(_ROOT, "config", "fxck1.json"),
              os.path.join(_ROOT, "config", "gg5.json"))
SEED_SKIP = ()

# wasd — 대시보드는 로봇이 manual_wait 모드일 때만 이 키를 액션으로 보낸다.
# fxck3 는 세션 내내 manual_wait 로 publish 하므로 사실상 상시 조종 키다.
ACTIONS = [
    {"name": "read_color", "label": "Read Center Color"},
    {"name": "reset", "label": "Reset to Start (wait CENTER button)"},
    {"name": "manual_fwd", "label": "Manual FORWARD", "key": "w"},
    {"name": "manual_rev", "label": "Manual U-TURN 180", "key": "s"},
    {"name": "manual_left", "label": "Manual LEFT 90", "key": "a"},
    {"name": "manual_right", "label": "Manual RIGHT 90", "key": "d"},
    {"name": "manual_left_fine", "label": "Manual LEFT 30", "key": "j"},
    {"name": "manual_right_fine", "label": "Manual RIGHT 30", "key": "l"},
]
MANUAL_CMDS = frozenset(("manual_fwd", "manual_rev",
                         "manual_left", "manual_right",
                         "manual_left_fine", "manual_right_fine"))
MANUAL_TURNS = {"manual_left": "L", "manual_right": "R",
                "manual_left_fine": "L", "manual_right_fine": "R"}
MANUAL_FINE = frozenset(("manual_left_fine", "manual_right_fine"))


# ---------------------------------------------------------------------
# 순수 헬퍼
# ---------------------------------------------------------------------

def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


# ---------------------------------------------------------------------
# 구동 러너 — hw 를 몰고 대시보드 키를 실행한다. run() 에서만 생성.
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
        # 수동 조종(fxck3) — 주행 세션 중(manual_armed)에만 wasd 를 받는다.
        self.manual_armed = False
        self.manual_cmd = None

        self.session = 0
        self._init_session_state()

        # LCD 유지 스레드(gg3 그대로) — 표시할 내용이 생기면 주기 재출력.
        self._display_stop = False
        self._display_thread = threading.Thread(target=self._display_loop,
                                                name="display")
        self._display_thread.daemon = True
        self._display_thread.start()

    def _init_session_state(self):
        self.visits = 0
        self.goal_seen = False
        self.grabbed = False
        self.done = False
        self.last_marker_t = -1e9
        # UI 상태(gg3 그대로) — 조종에는 쓰이지 않는다.
        self.timer_start = None     # 스톱워치 시작 시각(버튼 출발)
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
        if action == "reset":
            self.reset_on = True
            self.reset_source = (args or {}).get("source", "dashboard")
            return {"queued": "reset"}
        if action in MANUAL_CMDS:
            # 주행 세션 중에만 수리 — 출발 대기/완주 대기 중 눌린 키가
            # 세션 시작 후 튀어나오지 않게 한다(한 칸 큐, 마지막 입력이 이김).
            with self._pending_lock:
                if not self.manual_armed:
                    return {"ignored": action, "reason": "not_driving"}
                self.manual_cmd = action
            return {"queued": action}
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
            "visits": self.visits,
            "grabbed": self.grabbed,
            "goal_seen": self.goal_seen,
        }
        frame.update(extra)
        last_reason = self.log.last_reason()
        if last_reason is not None:
            frame["last_reason"] = last_reason
        events = self.log.drain_events()
        if events:
            frame["events"] = events
        self.tele.publish(frame)

    # ---- 오디오/LCD(gg3 그대로) — 전부 비동기·best-effort ----

    def play_number(self, number):
        """숫자음 재생(비동기 큐). wav 가 없으면 beep 폴백."""
        path = NUMBER_SOUNDS.get(int(number))
        if path is None:
            self.hw.beep_ok()
            return
        self.hw.play_wav(path)

    def choose_random_number(self, phase):
        """랜덤 숫자(1~4)를 LCD 하단에 표시(기존 값 대체)하고 숫자음 재생."""
        self.bottom_number = random.randint(1, 4)
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

    def handle_pending(self):
        """대시보드 read_color 액션 처리(없으면 no-op)."""
        with self._pending_lock:
            action = self._pending
            self._pending = None
        if action == "read_color":
            color = self.hw.read_center_color_now()
            self.log.log("COLOR_READ", "DO_TRIGGER", color=color)
            self.publish("read_color", color=color)

    def _hold_while_paused(self, mode):
        """모션 중 pause: 모터를 세우고 해제/중단까지 대기.

        paused 모드로 publish 하므로 대시보드 manual_wait 게이트가 꺼져
        내장 키([s] STOP 등)가 되살아난다 — pause 가 비상 탈출구를 겸한다."""
        self.hw.stop()
        while self.paused and not self.interrupted():
            self.publish(mode)
            time.sleep(0.05)

    # ---- 모션 프리미티브 ----

    def straight(self, dist_mm, speed, mode="manual_wait"):
        """엔코더 기준 직진(speed<0 후진). 중단 시점까지의 mm 반환.

        하드웨어 가속 램프(STRAIGHT_RAMP_MS)로 출발 슬립을 없앤다(gg5).
        publish 기본 모드는 manual_wait — 이동 중에도 대시보드 wasd 게이트
        유지(연타가 내장 STOP 으로 새지 않는다)."""
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
                    self._hold_while_paused("paused")
                    if self.interrupted():
                        break
                    self.hw.drive(speed, speed)
                self.publish(mode, dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
            self.hw.set_ramp(0)
        return self.hw.enc_avg() * MM_PER_DEG

    def pivot(self, move, snap, deg=None):
        """피벗(L/R): [a]/[d]=90도, [j]/[l]=turn_deg(기본 30도, 연타 누적),
        [s]=180도 유턴.

        coast→가속 램프 스무스 턴은 gg5 turn() 과 동일. 격자 heading 이
        없으므로(지도 없음) 그냥 돌기만 한다. 회전각은 90도 피벗 실측치
        (BASE_PIVOT_DEG_90 × turn_90_factor)를 deg/90 비례 스케일."""
        if deg is None:
            deg = snap["turn_deg"]
        target = (BASE_PIVOT_DEG_90 * snap["turn_90_factor"]
                  * deg / 90.0)
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
                    self._hold_while_paused("paused")
                    if self.interrupted():
                        break
                    self.hw.drive_raw(left_dir * speed, right_dir * speed)
                self.publish("manual_wait", moving="turn_" + move,
                             target_deg=round(target, 1),
                             enc_avg=round(self.hw.enc_avg(), 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
            self.hw.set_ramp(0)
        time.sleep(POST_TURN_SETTLE_S)
        self.log.log("MANUAL_MOVE", "TURN_" + move, deg=deg,
                     target_deg=round(target, 1),
                     enc_avg=round(self.hw.enc_avg(), 1))

    def manual_forward(self, snap):
        """[w] 전진 step_mm — 이동 중 마커/물체를 감시한다.

        중앙 컬러가 마커색이면(디바운스 통과) 즉시 멈추고 정지 다수결로
        확정 처리(handle_marker), 초음파 grab_dist 이내면 멈추고 메인
        루프가 파지한다(가는 길에 만나면 잡는다 — 그리퍼 층 그대로)."""
        self.hw.reset_encoders()
        target = snap["step_mm"] / MM_PER_DEG
        speed = snap["fwd_speed"]
        hit_color = None
        self.hw.set_ramp(STRAIGHT_RAMP_MS)
        try:
            self.hw.drive(speed, speed)
            while self.hw.enc_avg() < target:
                if self.interrupted():
                    break
                if self.paused:
                    self._hold_while_paused("paused")
                    if self.interrupted():
                        break
                    self.hw.drive(speed, speed)
                if (not self.grabbed and
                        self.hw.read_distance_cm() < snap["grab_dist_cm"]):
                    break   # 정지 후 메인 루프가 파지한다
                color = self.hw.read_center_color_now()
                if (color in MARKER_COLORS and
                        (time.monotonic() - self.last_marker_t) * 1000
                        >= MARKER_DEBOUNCE_MS):
                    hit_color = color
                    break
                self.publish("manual_wait", moving="fwd",
                             dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
            self.hw.set_ramp(0)
        self.log.log("MANUAL_MOVE", "FORWARD",
                     dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1),
                     step_mm=snap["step_mm"])
        if hit_color is not None and not self.interrupted():
            self.handle_marker(hit_color, "manual_fwd")

    # ---- 마커 처리(자동 판단 없음 — UI/그리퍼 이벤트만) ----

    def _confirm_marker_color(self, first):
        """마커 색 확정: 정지 상태에서 여러 번 재판독해 다수결(fxck1 그대로)."""
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
        """빨강/노랑/초록 마커 처리(정지 → 색 확정 → UI/그리퍼). 처리했으면 True.

        자동 회전/경로 판단은 없다 — 이후 이동은 전부 사람이 한다.
        빨강 = red N / 초록(최초) = OUT 시간+good_job+랜덤 숫자+배달 /
        노랑(초록 후) = 완주 / 그 외 = 로그만 남기고 무시."""
        if color not in MARKER_COLORS:
            return False
        if (time.monotonic() - self.last_marker_t) * 1000 < MARKER_DEBOUNCE_MS:
            return False

        self.hw.stop()
        time.sleep(MARKER_PAUSE_S)
        color = self._confirm_marker_color(color)
        if color is None:
            self.log.log("MARKER_REJECT", "UNCONFIRMED", context=context)
            return False
        name = MARKER_NAMES[color]
        self.log.log("MARKER", "COLOR_{}_CONFIRMED".format(name.upper()),
                     color=color, context=context, session=self.session)
        self.last_marker_t = time.monotonic()

        if color == COL_RED:
            self.visits += 1
            self.announce_red("RETURN" if self.goal_seen else "OUT")
            return True

        if color == COL_GREEN:
            if self.goal_seen:
                self.log.log("MARKER_IGNORED", "GREEN_AGAIN")
                return True
            # 초록 도착(목적지): good_job + OUT 시간 + 새 랜덤 숫자 — 전부
            # 비동기라 이어지는 배달 동작과 병행된다(gg3 그대로).
            if self.timer_start is not None and self.out_elapsed is None:
                self.out_elapsed = time.monotonic() - self.timer_start
                self.log.log("STOPWATCH_OUT", "COLOR_GREEN",
                             elapsed_s=round(self.out_elapsed, 1))
            self.hw.play_wav(SOUND_GOOD_JOB)
            self.choose_random_number("GREEN")
            self.deliver()      # 물체 내려놓기 — 유턴은 사람이 한다
            return True

        # COL_YELLOW
        if not self.goal_seen:
            self.log.log("MARKER_IGNORED", "YELLOW_BEFORE_GREEN")
            return True
        # 완주: BACK 시간 표시 + 스톱워치 정지 + 그립 해제(fxck1 그대로).
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
        self.straight(snap["goal_advance_mm"], snap["fwd_speed"])
        if self.interrupted():
            return
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
        self.grabbed = False
        self.straight(snap["goal_advance_mm"], -snap["fwd_speed"])

    def _release_at_home(self):
        """노란점 완주: 물체를 내려놓는다(그립 해제)."""
        snap = self.params.snapshot()
        if self.grabbed:
            self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
            self.grabbed = False
            self.log.log("DROP_HOME", "COLOR_YELLOW")

    # ---- 세션 단계 ----

    def wait_for_start(self):
        """가운데 버튼 press→release 로 출발(gg3 그대로 — 색 대기 없음).

        떼는 순간 스톱워치 시작 + 랜덤 숫자(1~4) LCD 하단 표시·재생.
        물체는 여기서 잡지 않고 주행 중 초음파로 만나면 잡는다."""
        snap = self.params.snapshot()
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)   # 물체 받을 준비
        while True:
            if self.stop_on:
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            self.publish("waiting_start")
            pressed = self.hw.wait_center_button(
                stop_cb=lambda: self.stop_on or self.reset_on, timeout=0.2)
            if pressed:
                break
        self.log.log("START", "CENTER_BUTTON")
        self.timer_start = time.monotonic()     # 스톱워치 시작(세션 내 지속)
        self.choose_random_number("START")
        self.straight(START_EXIT_MM, snap["fwd_speed"], mode="start_exit")
        self.last_marker_t = time.monotonic()   # 출발 노랑 재감지 방지
        return "go"

    def drive_manual(self):
        """풀 수동 메인 루프 — status(stop/reset/done) 반환.

        루프 동안만 wasd 를 무장한다 — 출발 대기/완주 대기 중 눌린 키가
        주행 시작 후 튀어나오지 않게 종료 시 큐도 비운다."""
        with self._pending_lock:
            self.manual_armed = True
        try:
            return self._drive_loop()
        finally:
            with self._pending_lock:
                self.manual_armed = False
                self.manual_cmd = None

    def _drive_loop(self):
        while not self.done:
            if self.stop_on:
                self.hw.stop()
                self.log.log("EMERGENCY_STOP", "NETWORK", source=self.stop_source)
                return "stop"
            if self.reset_on:
                self.hw.stop()
                return "reset"
            if self.paused:
                # paused 모드 publish — 대시보드 게이트가 꺼져 내장 키 복귀.
                self.hw.stop()
                self.publish("paused")
                time.sleep(MANUAL_POLL_S)
                continue
            self.handle_pending()
            snap = self.params.snapshot()

            # 물체를 만나면(초음파 grab_dist 이내) 잡는다 — 그리퍼 층 그대로.
            if (not self.grabbed and
                    self.hw.read_distance_cm() < snap["grab_dist_cm"]):
                self.hw.stop()
                self.hw.grip_close(snap["grip_speed"], GRIP_SEC)
                self.grabbed = True
                self.hw.tone(GRAB_TONE[0], GRAB_TONE[1])   # 파지 삡(비동기)
                self.log.log("GRAB", "ULTRASONIC_NEAR",
                             grab_dist_cm=snap["grab_dist_cm"])

            # 정지 중에도 마커 위면 처리(후진으로 올라탄 경우 등 — 디바운스가
            # 같은 마커의 반복 처리를 막는다).
            color = self.hw.read_center_color_now()
            if self.handle_marker(color, "manual_wait"):
                continue

            with self._pending_lock:
                cmd = self.manual_cmd
                self.manual_cmd = None
            if cmd == "manual_fwd":
                self.manual_forward(snap)
            elif cmd == "manual_rev":
                self.pivot("R", snap, deg=180.0)   # [s] = 180도 유턴
            elif cmd in MANUAL_TURNS:
                deg = snap["turn_deg"] if cmd in MANUAL_FINE else 90.0
                self.pivot(MANUAL_TURNS[cmd], snap, deg=deg)
            else:
                self.publish("manual_wait")
                time.sleep(MANUAL_POLL_S)
        return "done"

    def idle_after_done(self):
        """완주 후 대기 — LCD 기록을 유지한 채 리셋을 기다린다(gg3 그대로)."""
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
        """세션 상태를 전부 버리고 새 세션 준비(시작/reset 공용)."""
        self._init_session_state()
        self.reset_on = False
        self.session += 1
        if self.session == 1:
            self.log.log("SESSION_READY", "STARTUP", session=self.session)
        else:
            self.log.log("SESSION_RESET", "DASHBOARD", source=self.reset_source,
                         session=self.session)

    def run_sessions(self):
        """세션 = 출발 대기 → 수동 주행 → 완주 후 대기. reset 은 새 세션."""
        while not self.stop_on:
            self.new_session()
            status = self.wait_for_start()
            if status == "go":
                status = self.drive_manual()
            if status == "done":
                status = self.idle_after_done()
            if status == "stop":
                break
            # status == "reset" → 루프 상단에서 new_session.


def seed_initial_from_prev(initial):
    """fxck3 첫 실행 승계: fxck3.json 이 없으면 SEED_PATHS(fxck2→fxck1→gg5)
    중 처음 존재하는 저장값에서 겹치는 이름만 initial 에 병합해 반환한다
    (승계된 이름 리스트). 값은 fxck3 한계로 클램프한다."""
    if os.path.exists(SAVE_PATH):
        return []
    saved = None
    for path in SEED_PATHS:
        if not os.path.exists(path):
            continue
        try:
            with open(path, "r") as fp:
                saved = json.load(fp)
        except Exception:
            saved = None
        break
    if not isinstance(saved, dict):
        return []
    seeded = []
    for name in sorted(saved):
        if name in SEED_SKIP or name not in initial:
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
    return seeded


def run():
    from lib.hardware import Ev3Hardware   # ev3dev2 — 브릭에서만 import 가능

    initial = dict(INITIAL_PARAMS)
    seeded = seed_initial_from_prev(initial)
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
        log.log("PARAM_SEED", "PREV_CARRYOVER", names=",".join(seeded))
    print("fxck3 ready. FULL MANUAL: place robot on YELLOW, press the CENTER "
          "button, then drive with dashboard keys [w] fwd (step_mm) / "
          "[s] u-turn 180 / [a] left 90 / [d] right 90 / "
          "[j] left / [l] right (turn_deg each) — taps add "
          "up. markers only trigger sounds/display/gripper (red N, green "
          "good_job+drop, yellow finish); YOU do all the turning. gripper "
          "auto-grabs within grab_dist. pause (Space) restores builtin "
          "dashboard keys incl. [s] STOP. (Ctrl-C or robotctl stop to quit)")
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
    print("fxck3 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
