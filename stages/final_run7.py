#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_run7: final_run3 base + contest start/audio flow + smoother turns.

Key differences from final_run3:
  - final_run3 is left untouched; this file imports it and overrides only the
    stage-7 behavior.
  - Center button workflow:
      1) press center once -> random number 1..4 is displayed/spoken;
      2) calibrate from dashboard if needed;
      3) press center again -> movement starts.
  - PID defaults: kp=0.05, ki=0.065, D=0, deadband=0.5.
  - Pivot turns ramp speed up/down to avoid a sudden motor kick.
  - After a pivot, creep forward slowly under gentle PID to straighten the body.
  - Blue stickers announce "Blue one", "Blue two", ...
  - Delivery announces "Good Job" and displays elapsed time.
"""

import os
import random
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages import final_run3 as base                         # noqa: E402
from lib.shared_params import SharedParams                    # noqa: E402
from lib.telemetry import Telemetry                           # noqa: E402
from lib.decision_log import DecisionLog                      # noqa: E402
from lib.tuning_server import TuningServer                    # noqa: E402


COL_BLUE = 2
STAGE_NAME = "final_run7"
SAVE_PATH = os.path.join(_ROOT, "config", "final_run7.json")

PID_KD = 0.0
POST_TURN_ALIGN_TURN_LIMIT = 5.0
BLUE_DEBOUNCE_MS = 1200


def _replace_param(row, value):
    return (row[0], value, row[2], row[3], row[4], row[5], row[6])


def _build_param_table():
    rows = []
    for row in base.PARAM_TABLE:
        name = row[0]
        if name == "kp":
            row = _replace_param(row, 0.05)
        elif name == "ki":
            row = _replace_param(row, 0.065)
        elif name == "deadband":
            row = _replace_param(row, 0.5)
        rows.append(row)
        if name == "turn_180_factor":
            rows.append(("turn_ramp_deg", 45, 0, 140, 10, 5, "deg"))
            rows.append(("turn_min_speed", 5, 3, 20, 2, 1, "%"))
            rows.append(("post_turn_align_mm", 30, 0, 80, 10, 5, "mm"))
            rows.append(("post_turn_align_speed", 6, 3, 15, 2, 1, "%"))
    return tuple(rows)


PARAM_TABLE = _build_param_table()
INITIAL_PARAMS = dict((r[0], r[1]) for r in PARAM_TABLE)
PARAM_LIMITS = dict((r[0], (r[2], r[3])) for r in PARAM_TABLE)
MAX_STEP = dict((r[0], r[4]) for r in PARAM_TABLE)
UI_STEP = dict((r[0], r[5]) for r in PARAM_TABLE)
UNITS = dict((r[0], r[6]) for r in PARAM_TABLE)
PARAM_ORDER = [r[0] for r in PARAM_TABLE]

ACTIONS = list(base.ACTIONS)


class Stage7PidSteer(base.PidSteer):
    def reset_pd(self):
        self.prev_error = 0.0
        self.prev_t = None
        self.deriv = 0.0

    def reset(self):
        self.reset_pd()
        self.integ *= base.INTEG_RESET_KEEP


class Stage7Runner(base.Runner):
    def __init__(self, hw, params, tele, log):
        base.Runner.__init__(self, hw, params, tele, log)
        self.pid = Stage7PidSteer()

    def _init_session_state(self):
        base.Runner._init_session_state(self)
        self.mission_number = None
        self.mission_started_t = None
        self.segment_started_t = None
        self.blue_count = 0
        self.last_blue_t = -1e9
        self.delivery_count = 0

    def reset_steer_pd(self):
        self.pid.reset_pd()
        self.last_turn = 0.0
        self.lost_since = None

    def _wait_center_press(self, mode, lines):
        self.hw.display_lines(lines)
        while True:
            if self.stop_on:
                return "stop"
            if self.reset_on:
                return "reset"
            if self.paused:
                self.hw.stop()
                self.publish(mode + "_paused")
                time.sleep(0.05)
                continue
            self.handle_pending()
            self.publish(mode, mission_number=self.mission_number or 0)
            if self.hw.center_pressed():
                self.hw.wait_center_release()
                return "pressed"
            time.sleep(0.05)

    def wait_for_start(self):
        """Button 1 chooses the random number; button 2 starts movement."""
        snap = self.params.snapshot()
        self.hw.grip_open(snap["grip_speed"], base.GRIP_SEC)

        status = self._wait_center_press(
            "waiting_random_button",
            ("RUN5 READY", "Calibrate if needed", "Center: RANDOM"))
        if status != "pressed":
            return status

        self.mission_number = random.randint(1, 4)
        self.hw.display_lines(("NUMBER {}".format(self.mission_number),
                               "Place can at {}".format(self.mission_number),
                               "Center: START"))
        self.hw.say_number(self.mission_number)
        self.log.log("MISSION_RANDOM", "START_NUMBER",
                     number=self.mission_number, session=self.session)

        status = self._wait_center_press(
            "waiting_start_button",
            ("NUMBER {}".format(self.mission_number),
             "Calibrate OK?", "Center: START"))
        if status != "pressed":
            return status

        self.mission_started_t = time.monotonic()
        self.segment_started_t = self.mission_started_t
        self.hw.beep_ok()
        self.log.log("START", "CENTER_BUTTON",
                     number=self.mission_number, session=self.session)
        self.straight(base.START_EXIT_MM, base.STRAIGHT_SPEED, mode="start_exit")
        self.last_marker_t = time.monotonic()
        return "go"

    def _turn_speed_at(self, enc, target, snap):
        max_speed = snap["turn_speed"]
        min_speed = min(max_speed, snap.get("turn_min_speed", 5))
        ramp = snap.get("turn_ramp_deg", 0)
        if ramp <= 0 or target <= 0:
            return max_speed
        remaining = max(target - enc, 0.0)
        up = min(1.0, enc / float(ramp))
        down = min(1.0, remaining / float(ramp))
        scale = min(up, down)
        return base.clamp(max(min_speed, max_speed * scale), min_speed, max_speed)

    def turn(self, move):
        self.lost_streak = 0
        if move == "S":
            return
        snap = self.params.snapshot()
        if move == "U":
            self.play(base.TONE_UTURN)
            target = base.BASE_PIVOT_DEG_180 * snap["turn_180_factor"]
        else:
            target = base.BASE_PIVOT_DEG_90 * snap["turn_90_factor"]
        left_dir, right_dir = (-1, 1) if move == "L" else (1, -1)
        self.hw.reset_encoders()
        last_speed = 0.0
        try:
            while self.hw.enc_avg() < target:
                if self.interrupted():
                    break
                if self.paused:
                    self._hold_while_paused("turning")
                    if self.interrupted():
                        break
                enc = self.hw.enc_avg()
                speed = self._turn_speed_at(enc, target, snap)
                last_speed = speed
                self.hw.drive_raw(left_dir * speed, right_dir * speed)
                self.publish("turning", target_deg=round(target, 1),
                             enc_avg=round(enc, 1), speed=round(speed, 1))
                time.sleep(0.005)
        finally:
            self.hw.stop()
        actual = self.hw.enc_avg()
        time.sleep(base.POST_TURN_SETTLE_S)
        self.log.log("TURN",
                     {"L": "TURN_LEFT", "R": "TURN_RIGHT", "U": "UTURN"}[move],
                     target_deg=round(target, 1), enc_avg=round(actual, 1),
                     error_deg=round(actual - target, 1),
                     speed=round(last_speed, 1),
                     stopped_early=self.interrupted())
        self.ex.apply_move(move)
        self.reset_steer()
        if not self.interrupted():
            color = self.hw.read_center_color_now()
            acquired = True
            if color != base.COL_BLACK and color not in base.MARKER_COLORS:
                self.log.log("TURN_ACQUIRE", "CENTER_OFF_LINE_AFTER_TURN",
                             move=move, color=color)
                acquired = self.realign_to_line(self.params.snapshot())
            if acquired and not self.interrupted():
                self.post_turn_align(self.params.snapshot())

    def post_turn_align(self, snap):
        dist_mm = snap.get("post_turn_align_mm", 0)
        if dist_mm <= 0:
            return
        speed = snap.get("post_turn_align_speed", base.CONFIRM_SPEED)
        self.reset_steer_pd()
        self.hw.reset_encoders()
        target_deg = dist_mm / base.MM_PER_DEG
        last_error = 0.0
        last_turn = 0.0
        try:
            while self.hw.enc_avg() < target_deg:
                if self.interrupted():
                    break
                if self.paused:
                    self._hold_while_paused("post_turn_align")
                    if self.interrupted():
                        break
                color = self.hw.read_center_color_now()
                if color in base.MARKER_COLORS:
                    break
                rl = self.hw.read_left_reflect()
                rr = self.hw.read_right_reflect()
                norm_l = base.normalize(rl, snap["cal_l_black"],
                                        snap["cal_l_white"])
                norm_r = base.normalize(rr, snap["cal_r_black"],
                                        snap["cal_r_white"])
                _left, _right, error, turn, _trim = self.pid.step(norm_l, norm_r,
                                                                  snap, speed)
                turn = base.clamp(turn, -POST_TURN_ALIGN_TURN_LIMIT,
                                  POST_TURN_ALIGN_TURN_LIMIT)
                self.hw.drive(speed - turn, speed + turn)
                last_error = error
                last_turn = turn
                self.publish("post_turn_align",
                             dist_mm=round(self.hw.enc_avg() * base.MM_PER_DEG, 1),
                             error=round(error, 2), turn=round(turn, 2))
                time.sleep(base.LOOP_DELAY_S)
        finally:
            self.hw.stop()
        self.reset_steer_pd()
        self.log.log("POST_TURN_ALIGN", "CREEP_PID",
                     dist_mm=round(self.hw.enc_avg() * base.MM_PER_DEG, 1),
                     target_mm=dist_mm, speed=speed,
                     error=round(last_error, 2), turn=round(last_turn, 2))

    def handle_marker(self, color, context):
        if color == COL_BLUE:
            return self._handle_blue_marker(context)
        if color == base.COL_YELLOW and self.ex.mode == "HOME":
            return self._handle_home_yellow_delivery(context)
        return base.Runner.handle_marker(self, color, context)

    def _handle_blue_marker(self, context):
        now = time.monotonic()
        if (now - self.last_blue_t) * 1000 < BLUE_DEBOUNCE_MS:
            return False
        self.hw.stop()
        self.blue_count += 1
        idx = ((self.blue_count - 1) % 4) + 1
        self.hw.display_lines(("BLUE {}".format(idx),
                               "mission {}".format(self.mission_number or "-")))
        self.hw.say_blue(idx)
        self.log.log("BLUE_MARKER", "COLOR_BLUE",
                     index=idx, context=context, session=self.session)
        self.last_blue_t = now
        time.sleep(0.05)
        return False

    def _finish_delivery_segment(self, label):
        self.delivery_count += 1
        elapsed = 0.0
        if self.segment_started_t is not None:
            elapsed = time.monotonic() - self.segment_started_t
        self.hw.say_good_job()
        self.hw.display_lines(("Good Job", "{} {:.1f}s".format(label, elapsed),
                               "delivery {}".format(self.delivery_count)))
        self.log.log("DELIVERY_DONE", "GOOD_JOB_" + label.upper(),
                     delivery=self.delivery_count,
                     elapsed_s=round(elapsed, 1),
                     number=self.mission_number)
        time.sleep(1.0)
        return elapsed

    def deliver(self):
        base.Runner.deliver(self)
        if self.interrupted():
            return
        self._finish_delivery_segment("green")
        self.segment_started_t = time.monotonic()

    def _handle_home_yellow_delivery(self, context):
        if (time.monotonic() - self.last_marker_t) * 1000 < base.MARKER_DEBOUNCE_MS:
            return False
        self.hw.stop()
        time.sleep(base.MARKER_PAUSE_S)
        self.hw.beep_ok()
        self.hw.beep_ok()
        snap = self.params.snapshot()
        self.hw.grip_open(snap["grip_speed"], base.GRIP_SEC)
        self.done = True
        route_left = self.ex.route_left()
        missed = max(0, self.ex.home_red_total - self.home_revisit)
        on_plan = (not self.ex.home_fallback and route_left <= 1 and missed == 0)
        self.log.log("HOME_REACHED",
                     "COLOR_YELLOW" if on_plan else "EARLY_OR_FALLBACK_MISSED_REDS",
                     home_revisit=self.home_revisit,
                     home_total=self.ex.home_red_total,
                     missed=missed, route_left=route_left,
                     fallback=self.ex.home_fallback,
                     context=context)
        self._finish_delivery_segment("yellow")
        self.last_marker_t = time.monotonic()
        return True


def run():
    base.PID_KD = PID_KD

    from lib.hardware import Ev3Hardware

    params = SharedParams(INITIAL_PARAMS, PARAM_LIMITS, MAX_STEP, SAVE_PATH,
                          ui_step=UI_STEP, units=UNITS, param_order=PARAM_ORDER)
    params.load_saved_into_defaults()

    tele = Telemetry()
    log = DecisionLog(telemetry=tele)
    hw = Ev3Hardware()
    hw.read_center_color(base.COLOR_MODE_SETTLE_S, 1)
    runner = Stage7Runner(hw, params, tele, log)

    server = TuningServer(params, tele, do_handler=runner.on_do,
                          stop_handler=runner.on_stop,
                          pause_handler=runner.on_pause,
                          actions=ACTIONS, stage=STAGE_NAME)
    server.start()

    print("final_run7 ready. Center button: random number, calibrate if needed, "
          "center button again: start. PID defaults kp=0.05 ki=0.065 kd=0.")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        try:
            hw.stop()
        finally:
            server.stop()
    print("final_run7 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
