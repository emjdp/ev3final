#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aplus(원격 조종 결정 모드) 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

aplus 는 자율 판단이 없다 — 남은 순수 로직을 검증한다:
  - CommandQueue: FIFO 다칸 큐(연속 입력) / 가득 차면 거부 / clear / peek 비소비.
  - on_do 디스패치: 이동 명령은 큐, GO/clear/reset 은 즉시, 모터 액션은 FIFO.
  - MOVE_ACTIONS/ACTIONS 정합: w/s/a/d/q/e/p 키 규약(사용자 지정 키맵).
  - confirm/slow 속도: base_speed 비례 + 하한(fxck2 승계) — base 15 기준.
  - adaptive_base / PidSteer 속도 비례 조향 / node_bits / on_line(gg5 승계).
  - 조종 패드(tools/aplus_pad.py) 키맵이 로봇 manifest 와 1:1 인지.

실행: python3 tests/test_aplus_logic.py
"""

import os
import sys
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from stages.aplus import (CommandQueue, PidSteer, Runner, adaptive_base,
                          confirm_speed, slow_speed, node_bits, on_line,
                          corner_min_speed, accel_start_speed,
                          straight_decel_speed, STRAIGHT_DECEL_MM,
                          STRAIGHT_DECEL_MIN,
                          INITIAL_PARAMS, MOVE_ACTIONS, DIAG_MOVES, ACTIONS,
                          SPEED_REF, STEER_SCALE_MAX, PID_TURN_LIMIT,
                          SOUND_RED, SOUND_GOOD_JOB, NUMBER_SOUNDS,
                          RED_SAY_MAX,
                          COL_BLACK, COL_GREEN, COL_YELLOW, COL_RED,
                          COL_WHITE, COL_BROWN)
from lib.decision_log import DecisionLog
from lib.telemetry import Telemetry


class CommandQueueCase(unittest.TestCase):
    """FIFO 다칸 큐 — 연속 입력이 순서대로 소비된다."""

    def setUp(self):
        self.q = CommandQueue()

    def test_empty_take(self):
        move, waited = self.q.take()
        self.assertIsNone(move)
        self.assertIsNone(waited)

    def test_fifo_order(self):
        for token in ("DL", "DL", "S"):
            accepted, _snap = self.q.push(token)
            self.assertTrue(accepted)
        self.assertEqual(self.q.take()[0], "DL")
        self.assertEqual(self.q.take()[0], "DL")
        self.assertEqual(self.q.take()[0], "S")
        self.assertIsNone(self.q.take()[0])

    def test_push_returns_snapshot(self):
        self.q.push("L")
        accepted, snap = self.q.push("U")
        self.assertTrue(accepted)
        self.assertEqual(snap, ["L", "U"])

    def test_full_rejects_new(self):
        for _i in range(CommandQueue.MAXLEN):
            accepted, _snap = self.q.push("S")
            self.assertTrue(accepted)
        accepted, snap = self.q.push("L")
        self.assertFalse(accepted)
        self.assertEqual(len(snap), CommandQueue.MAXLEN)
        self.assertNotIn("L", snap)

    def test_peek_all_does_not_consume(self):
        self.q.push("R")
        self.q.push("S")
        self.assertEqual(self.q.peek_all(), ["R", "S"])
        self.assertEqual(self.q.peek_all(), ["R", "S"])
        self.assertEqual(self.q.take()[0], "R")

    def test_clear_returns_all(self):
        self.q.push("S")
        self.q.push("U")
        self.assertEqual(self.q.clear(), ["S", "U"])
        self.assertEqual(self.q.peek_all(), [])
        self.assertIsNone(self.q.take()[0])

    def test_take_reports_wait_time(self):
        self.q.push("S")
        _move, waited = self.q.take()
        self.assertGreaterEqual(waited, 0.0)


class OnDoDispatchCase(unittest.TestCase):
    """네트워크 스레드 경로(on_do) — 이동 명령은 큐, GO/clear/reset 은 즉시,
    모터 액션은 제어 루프 FIFO. hw 를 건드리지 않아 PC 에서 검증 가능."""

    def setUp(self):
        tele = Telemetry()
        self.runner = Runner(None, None, tele, DecisionLog(telemetry=tele))

    def test_move_action_queues_fifo(self):
        self.assertEqual(self.runner.on_do("fwd", {})["queued_move"], "S")
        self.assertEqual(self.runner.on_do("left", {})["queued_move"], "L")
        self.assertEqual(self.runner.cmd.peek_all(), ["S", "L"])

    def test_diag_actions_queue_tokens(self):
        self.runner.on_do("diag_left", {})
        self.runner.on_do("diag_right", {})
        self.assertEqual(self.runner.cmd.peek_all(), ["DL", "DR"])

    def test_queue_full_rejected(self):
        for _i in range(CommandQueue.MAXLEN):
            self.runner.on_do("fwd", {})
        resp = self.runner.on_do("uturn", {})
        self.assertEqual(resp.get("rejected"), "U")
        self.assertEqual(resp.get("reason"), "queue_full")

    def test_clear_action(self):
        self.runner.on_do("right", {})
        self.runner.on_do("uturn", {})
        resp = self.runner.on_do("clear", {})
        self.assertEqual(resp["cleared"], ["R", "U"])
        self.assertEqual(self.runner.cmd.peek_all(), [])

    def test_go_action_sets_flag(self):
        self.assertFalse(self.runner.go_on)
        self.runner.on_do("go", {})
        self.assertTrue(self.runner.go_on)

    def test_reset_action_sets_flag(self):
        self.runner.on_do("reset", {"source": "test"})
        self.assertTrue(self.runner.reset_on)
        self.assertEqual(self.runner.reset_source, "test")

    def test_motor_actions_queue_pending(self):
        """grip/calibrate 는 CommandQueue 가 아니라 제어 루프 FIFO 로 간다."""
        self.runner.on_do("grip_close", {})
        self.runner.on_do("grip_open", {})
        self.runner.on_do("calibrate", {})
        self.assertEqual(self.runner._pending,
                         ["grip_close", "grip_open", "calibrate"])
        self.assertEqual(self.runner.cmd.peek_all(), [])

    def test_goal_drop_queues_pending(self):
        """[7] 도착 폴백은 모터 액션 — 이동 FIFO 가 아니라 제어 루프로 간다."""
        resp = self.runner.on_do("goal_drop", {})
        self.assertEqual(resp["queued"], "goal_drop")
        self.assertEqual(self.runner._pending, ["goal_drop"])
        self.assertEqual(self.runner.cmd.peek_all(), [])


class SayRedCase(unittest.TestCase):
    """패드 [1]~[6] 수동 음성 — 네트워크 스레드에서 즉시 오디오 큐로 재생,
    이동 FIFO 를 쓰지 않고 red N 카운터도 건드리지 않는다."""

    class _FakeAudioHw(object):
        def __init__(self):
            self.wavs = []

        def play_wav(self, path):
            self.wavs.append(path)

        def beep_ok(self):
            self.wavs.append("beep")

    def setUp(self):
        tele = Telemetry()
        self.hw = self._FakeAudioHw()
        self.runner = Runner(self.hw, None, tele, DecisionLog(telemetry=tele))

    def test_plays_red_then_number(self):
        resp = self.runner.on_do("say_red_3", {})
        self.assertEqual(resp["queued"], "say_red_3")
        self.assertEqual(self.hw.wavs, [SOUND_RED, NUMBER_SOUNDS[3]])

    def test_does_not_touch_fifo_or_counters(self):
        self.runner.on_do("say_red_1", {})
        self.assertEqual(self.runner.cmd.peek_all(), [])
        self.assertEqual(self.runner.out_red_spoken, 0)
        self.assertEqual(self.runner.return_red_spoken, 0)

    def test_clamps_out_of_range_number(self):
        resp = self.runner.on_do("say_red_9", {})
        self.assertEqual(resp["queued"], "say_red_%d" % RED_SAY_MAX)
        self.assertEqual(self.hw.wavs,
                         [SOUND_RED, NUMBER_SOUNDS[RED_SAY_MAX]])

    def test_bad_suffix_rejected(self):
        resp = self.runner.on_do("say_red_x", {})
        self.assertIs(resp.get("ok"), False)
        self.assertEqual(self.hw.wavs, [])

    def test_manifest_declares_all_six(self):
        keys = dict((a["name"], a["key"]) for a in ACTIONS)
        for n in range(1, RED_SAY_MAX + 1):
            self.assertEqual(keys["say_red_%d" % n], str(n))


class ActionManifestCase(unittest.TestCase):
    """사용자 지정 키맵 규약 — w/s/a/d/q/e/p, 전부 유일."""

    def test_user_specified_keys(self):
        keys = dict((a["name"], a.get("key")) for a in ACTIONS)
        self.assertEqual(keys["fwd"], "w")
        self.assertEqual(keys["uturn"], "s")
        self.assertEqual(keys["left"], "a")
        self.assertEqual(keys["right"], "d")
        self.assertEqual(keys["diag_left"], "q")
        self.assertEqual(keys["diag_right"], "e")
        self.assertEqual(keys["grip_close"], "p")
        self.assertEqual(keys["goal_drop"], "7")

    def test_all_actions_have_unique_keys(self):
        keys = [a.get("key") for a in ACTIONS]
        self.assertTrue(all(keys))
        self.assertEqual(len(keys), len(set(keys)))

    def test_move_actions_map(self):
        self.assertEqual(MOVE_ACTIONS,
                         {"fwd": "S", "left": "L", "right": "R", "uturn": "U",
                          "diag_left": "DL", "diag_right": "DR"})
        self.assertEqual(DIAG_MOVES, ("DL", "DR"))

    def test_move_actions_all_in_manifest(self):
        names = set(a["name"] for a in ACTIONS)
        for action in MOVE_ACTIONS:
            self.assertIn(action, names)


class PadKeymapCase(unittest.TestCase):
    """조종 패드(tools/aplus_pad.py) 키맵이 로봇 manifest 키와 1:1."""

    def test_pad_keys_match_robot_manifest(self):
        from tools.aplus_pad import KEY_ACTIONS
        manifest = dict((a["name"], a["key"]) for a in ACTIONS
                        if a["name"] != "reset")    # reset 은 패드 [r](확인)
        self.assertEqual(dict((v, k) for k, v in KEY_ACTIONS.items()),
                         manifest)


class SpeedHelpersCase(unittest.TestCase):
    """confirm/slow 는 base_speed 비례 + 하한(fxck2 승계)."""

    def test_default_base_is_15(self):
        self.assertEqual(INITIAL_PARAMS["base_speed"], 15)

    def test_curve_auto_default_on(self):
        # 커브는 기본 자동 통과(fxck2) — 0 으로 내리면 gg8 식 정지 대기.
        self.assertEqual(INITIAL_PARAMS["curve_auto"], 1)

    def test_confirm_floor_at_base_15(self):
        self.assertEqual(confirm_speed(15), 5.0)    # 15×0.25=3.75 → 하한 5

    def test_slow_floor_at_base_15(self):
        self.assertEqual(slow_speed(15), 8.0)       # 15×0.40=6.0 → 하한 8

    def test_proportional_above_floor(self):
        self.assertEqual(confirm_speed(40), 10.0)
        self.assertEqual(slow_speed(40), 16.0)


class StraightDecelCase(unittest.TestCase):
    """직진 종점 감속(순수) — 남은 거리가 감속 구간이면 저속, 밖이면 그대로.
    배달 전·후진/노드 전진이 급브레이크 없이 부드럽게 서게 한다."""

    def test_cruise_outside_zone(self):
        self.assertEqual(straight_decel_speed(15, STRAIGHT_DECEL_MM + 1), 15.0)
        self.assertEqual(straight_decel_speed(-15, STRAIGHT_DECEL_MM + 1), -15.0)

    def test_slows_inside_zone(self):
        self.assertEqual(straight_decel_speed(15, 10), 7.5)     # 15×0.5

    def test_backward_keeps_sign(self):
        self.assertEqual(straight_decel_speed(-15, 10), -7.5)

    def test_floor_applies(self):
        # 10×0.5=5 → 하한 6.
        self.assertEqual(straight_decel_speed(10, 5), float(STRAIGHT_DECEL_MIN))

    def test_never_faster_than_original(self):
        # 이미 하한보다 느린 저속(confirm 5%)은 그대로 — 가속 금지.
        self.assertEqual(straight_decel_speed(5, 5), 5.0)
        self.assertEqual(straight_decel_speed(-5, 5), -5.0)


class ManualGoalCase(unittest.TestCase):
    """[7] 수동 도착 처리 — 초록 미인식 폴백이 도착 절차를 그대로 시행:
    감속 정지 → good_job/랜덤 숫자 → 전진 → 그립 오픈 → 후진 → 180도 회전."""

    class _FakeMotionHw(object):
        def __init__(self):
            self.calls = []
            self.enc = 0.0

        def drive(self, left, right):
            self.calls.append(("drive", left, right))

        def drive_raw(self, left, right):
            self.calls.append(("drive_raw", left, right))

        def stop(self):
            self.calls.append(("stop",))

        def coast(self):
            pass

        def set_ramp(self, up_ms, down_ms=0):
            pass

        def reset_encoders(self):
            self.enc = 0.0

        def enc_avg(self):
            self.enc += 40.0        # 호출마다 전진 — 루프가 곧 종료된다
            return self.enc

        def read_center_color_now(self):
            return COL_BLACK        # 회전 후 라인 위 — realign 생략

        def grip_open(self, speed, sec):
            self.calls.append(("grip_open",))

        def grip_close(self, speed, sec):
            self.calls.append(("grip_close",))

        def play_wav(self, path):
            self.calls.append(("wav", path))

        def tone(self, freq, ms):
            pass

        def beep_ok(self):
            pass

        def show_final4_display(self, out_s, back_s, number):
            pass

    class _FakeParams(object):
        def snapshot(self):
            return dict(INITIAL_PARAMS)

        def rev(self):
            return 0

    def setUp(self):
        import time
        tele = Telemetry()
        self.hw = self._FakeMotionHw()
        self.runner = Runner(self.hw, self._FakeParams(), tele,
                             DecisionLog(telemetry=tele))
        self.runner.timer_start = time.monotonic() - 5.0

    def _run_via_pending(self):
        self.runner.on_do("goal_drop", {})
        self.runner.handle_pending()

    def test_sequence_and_state(self):
        self._run_via_pending()
        calls = self.hw.calls
        # good_job 재생 + 그립 오픈 + 유턴(drive_raw)이 전부 있었다.
        self.assertIn(("wav", SOUND_GOOD_JOB), calls)
        self.assertIn(("grip_open",), calls)
        raws = [c for c in calls if c[0] == "drive_raw"]
        self.assertTrue(raws)
        # 순서: 전진(drive +) → 그립 오픈 → 후진(drive -) → 회전(drive_raw).
        i_open = calls.index(("grip_open",))
        fwd = [i for i, c in enumerate(calls)
               if c[0] == "drive" and c[1] > 0 and c[2] > 0]
        back = [i for i, c in enumerate(calls)
                if c[0] == "drive" and c[1] < 0 and c[2] < 0]
        i_turn = calls.index(raws[0])
        self.assertTrue(fwd and fwd[0] < i_open)
        self.assertTrue(back and i_open < back[0] < i_turn)
        # 상태: 배달 완료(goal_seen) + 그립 비움 + OUT 시간 기록.
        self.assertTrue(self.runner.goal_seen)
        self.assertFalse(self.runner.grabbed)
        self.assertIsNotNone(self.runner.out_elapsed)
        self.assertGreaterEqual(self.runner.out_elapsed, 5.0)

    def test_out_time_not_overwritten_on_second_run(self):
        self._run_via_pending()
        first = self.runner.out_elapsed
        self._run_via_pending()     # 조종자가 보스 — 두 번째도 그대로 시행
        self.assertEqual(self.runner.out_elapsed, first)


class AdaptiveBaseCase(unittest.TestCase):
    """커브 자동 감속 + 출발 가속 램프 — 하한/시작이 base 비례라 base 15
    에서도 체감된다(gg5 절대값 12 는 base 15 에서 여유 3%뿐이었다)."""

    def test_straight_full_speed_after_ramp(self):
        self.assertEqual(adaptive_base(15, 0.5, 0.0, 700, 700), 15.0)

    def test_corner_floor_proportional_at_base_15(self):
        # 15×0.6=9 — base 15 에서도 커브 감속 폭(15→9)이 살아 있다.
        self.assertEqual(corner_min_speed(15), 9.0)
        self.assertEqual(adaptive_base(15, 0.5, 100.0, 700, 9999), 9.0)

    def test_corner_floor_reproduces_gg5_at_ref_speed(self):
        self.assertEqual(corner_min_speed(20), 12.0)    # gg5 절대값 12 재현

    def test_corner_floor_has_absolute_min(self):
        self.assertEqual(corner_min_speed(10), 8.0)     # 10×0.6=6 → 하한 8
        self.assertEqual(corner_min_speed(7), 7.0)      # 하한이 target 초과 금지

    def test_ramp_starts_proportional(self):
        # 재출발도 base 비례로 천천히 — 15 는 9 부터, 30 은 18 부터.
        self.assertEqual(accel_start_speed(15), 9.0)
        self.assertEqual(adaptive_base(15, 0.5, 0.0, 700, 0), 9.0)
        self.assertEqual(adaptive_base(30, 0.5, 0.0, 700, 0), 18.0)

    def test_ramp_disabled_when_zero(self):
        self.assertEqual(adaptive_base(30, 0.5, 0.0, 0, 0), 30.0)


class SpeedScaledSteerCase(unittest.TestCase):
    """gg5 승계 — 속도 비례 조향."""

    def setUp(self):
        self.snap = dict(INITIAL_PARAMS)

    def _turn_at(self, base):
        pid = PidSteer()
        _l, _r, _e, turn, _t = pid.step(20.0, 80.0, self.snap, base)
        return turn

    def test_ref_speed_unscaled(self):
        expected = self.snap["kp"] * (60.0 - self.snap["deadband"])
        self.assertAlmostEqual(self._turn_at(SPEED_REF), expected)

    def test_turn_scales_with_speed(self):
        self.assertAlmostEqual(self._turn_at(30), self._turn_at(20) * 1.5)

    def test_scale_clamped_at_max(self):
        self.assertAlmostEqual(self._turn_at(60),
                               self._turn_at(20) * STEER_SCALE_MAX)

    def test_turn_limit_scales(self):
        snap = dict(self.snap)
        snap["kp"] = 3.0
        pid = PidSteer()
        _l, _r, _e, turn, _t = pid.step(0.0, 100.0, snap, 30)
        self.assertAlmostEqual(abs(turn), PID_TURN_LIMIT * 1.5)


class OnLineCase(unittest.TestCase):
    """중앙 '라인 위' 판정 — 흰색만 아니면 라인(gg3 실측 승계)."""

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


if __name__ == "__main__":
    unittest.main()
