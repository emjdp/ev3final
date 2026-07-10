#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ffrun1 대시보드 연동 단위 테스트 — PC 단독 실행(ev3dev2 불필요).

검증 항목(§H, §I — stages/ffrun1.py 헤더):
  - 이벤트 링버퍼: 단발 이벤트(COLOR_READ 등)가 이후 프레임들에도 계속
    실려 watcher(4Hz) 폴링이 프레임을 놓쳐도 유실되지 않는다. 상한 EVENT_KEEP.
  - 색상 가이드 캘리브레이션(cal_color 위저드): WHITE→BLACK→RED→GREEN→BLUE
    순서 안내, 잘못된 색 거부(진행 안 됨), 5색 완료 시 rgb_* 자동 산출·저장,
    저장값으로 5색 재분류 검증(OK/NG). separation 실패는 우연히 갈려도 NG.

미로/판단층은 final_run11 과 동일 — tests/test_final_run11_maze.py 가 커버
(본 파일 작성 시점에 ffrun1 로 모듈 스왑해 19/19 통과 확인).

실행: python3 tests/test_ffrun1_wizard.py
"""

import os
import sys
import tempfile
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import stages.ffrun1 as ff                                          # noqa: E402
from lib.decision_log import DecisionLog                            # noqa: E402
from lib.shared_params import SharedParams                          # noqa: E402
from lib.telemetry import Telemetry                                 # noqa: E402


# 2026-07-10 실기 로그값 — 이 센서는 흰 바닥에서 g 채널이 20~27% 강하다
# (g/r 1.22~1.27). 예전 상한 1.15 로는 흰색이 LOOKS_GREEN 으로 거부됐다.
# 파랑은 청록처럼 읽힌다(g>b, b/g≈0.82) — b 지배 축으론 캘리가 불가능했고
# 'r 최약' 축 min(b/r, g/r)≈2.98 로 판정한다(§J).
REAL_WHITE = (205, 260, 162)      # sum 627, green dom 1.27
REAL_GREEN = (30, 108, 9)         # sum 147, green dom 3.6
REAL_BLUE = (40, 146, 119)        # sum 305, blue dom 2.98 (구 축 b/g 0.82)
REAL_BLACK = (23, 36, 10)         # sum 69, bright 11.0% (실측값.md)

# 실기 근사 샘플(EV3 RGB-RAW 대략치) — 흰 바닥은 b 채널이 약간 낮다.
# 파랑은 실기 로그값 그대로(§J — 청록 읽힘이 판정축의 근거라서).
SAMPLES = {
    "white": (270, 300, 180),
    "black": (40, 45, 30),
    "red": (200, 60, 45),
    "green": (60, 170, 55),
    "blue": REAL_BLUE,
}


class FakeHw(object):
    def __init__(self):
        self.rgb = SAMPLES["white"]

    def read_center_rgb_now(self):
        return self.rgb

    def stop(self):
        pass


def make_runner(tmpdir):
    params = SharedParams(ff.INITIAL_PARAMS, ff.PARAM_LIMITS, ff.MAX_STEP,
                          os.path.join(tmpdir, "ffrun1.json"),
                          ui_step=ff.UI_STEP, units=ff.UNITS,
                          param_order=ff.PARAM_ORDER)
    tele = Telemetry()
    log = DecisionLog(telemetry=tele)
    hw = FakeHw()
    return ff.Runner(hw, params, tele, log), hw, tele, params


def latest_events(tele):
    return tele.latest().get("events", [])


def event_names(tele):
    return [e["event"] for e in latest_events(tele)]


class PureFunctionTest(unittest.TestCase):

    def test_median3(self):
        self.assertEqual(ff.median3([5, 1, 9]), 5)
        self.assertEqual(ff.median3([3, 3, 100, 2, 1]), 3)

    def test_check_color_sample_accepts_all_colors(self):
        for name in ff.CAL_COLOR_ORDER:
            ok, reason, _ = ff.check_color_sample(name, SAMPLES[name],
                                                  SAMPLES["white"])
            self.assertTrue(ok, "{}: {}".format(name, reason))

    def test_check_color_sample_rejects_misplacement(self):
        cases = (
            ("white", SAMPLES["black"], "WHITE_TOO_DARK"),   # 검정 위에서 white 차례
            ("white", SAMPLES["red"], "LOOKS_RED"),          # 빨강 스티커 위
            ("white", SAMPLES["green"], "LOOKS_GREEN"),      # 초록 스티커 위
            ("white", SAMPLES["blue"], "LOOKS_BLUE"),        # 파랑 스티커 위
            ("black", SAMPLES["white"], "BLACK_TOO_BRIGHT"),
            ("red", SAMPLES["green"], "NOT_RED_ENOUGH"),
            ("green", SAMPLES["white"], "NOT_GREEN_ENOUGH"),
            # §J 교차 검사: 파랑(청록 읽힘)은 g/r 가 커서 green 차례에
            # MIN_DOM 을 아슬하게 넘는다 — blue 지배비로 거부해야 한다.
            ("green", SAMPLES["blue"], "LOOKS_BLUE"),
            ("blue", SAMPLES["green"], "NOT_BLUE_ENOUGH"),
        )
        for name, rgb, want in cases:
            ok, reason, _ = ff.check_color_sample(name, rgb, SAMPLES["white"])
            self.assertFalse(ok, name)
            self.assertEqual(reason, want)

    def test_greenish_white_sensor_regression(self):
        """실기 회귀(2026-07-10): g 가 강한 센서의 흰 바닥이 LOOKS_GREEN 으로
        거부되면 안 된다. 진짜 초록은 여전히 white 차례에 거부돼야 한다."""
        ok, reason, _ = ff.check_color_sample("white", REAL_WHITE, REAL_WHITE)
        self.assertTrue(ok, reason)
        ok, reason, _ = ff.check_color_sample("white", REAL_GREEN, REAL_WHITE)
        self.assertFalse(ok)        # 실기 초록은 어두워 WHITE_TOO_DARK 로 걸린다

    def test_cyan_reading_blue_regression(self):
        """실기 회귀(2026-07-10): 파랑 스티커가 청록으로 읽혀도(g>b, b/g 0.82)
        blue 차례를 통과해야 한다 — 구 축(min(b/r,b/g))에선 dom 0.82 로
        NOT_BLUE_ENOUGH 가 무한 반복됐다."""
        ok, reason, detail = ff.check_color_sample("blue", REAL_BLUE,
                                                   REAL_WHITE)
        self.assertTrue(ok, (reason, detail))
        self.assertGreater(detail["dom"], 2.5)

    def test_derive_round_trip_with_greenish_white(self):
        """실기 4색(흰/검/초록/파랑) 실측으로 위저드를 끝까지 돌려도
        5색 재분류가 성립(빨강만 근사 샘플)."""
        real = dict(SAMPLES)
        real["white"] = REAL_WHITE
        real["green"] = REAL_GREEN
        real["blue"] = REAL_BLUE
        real["black"] = REAL_BLACK
        params, report = ff.derive_rgb_params(real)
        for name in ff.CAL_COLOR_ORDER:
            r, g, b = real[name]
            got, _bright = ff.classify_rgb(r, g, b, dict(params))
            self.assertEqual(got, ff.CAL_COLOR_EXPECT[name], name)
            if name in report:
                self.assertTrue(report[name]["sep_ok"], (name, report[name]))

    def test_derive_params_round_trip(self):
        """산출 파라미터로 5색 샘플이 전부 제 색으로 재분류돼야 한다."""
        params, report = ff.derive_rgb_params(SAMPLES)
        for name in ff.CAL_COLOR_ORDER:
            r, g, b = SAMPLES[name]
            got, _bright = ff.classify_rgb(r, g, b, dict(params))
            self.assertEqual(got, ff.CAL_COLOR_EXPECT[name], name)
            if name in report:
                self.assertTrue(report[name]["sep_ok"], name)
        # PARAM_TABLE 한계 안(그대로 params.set 가능해야 한다)
        self.assertTrue(100 <= params["rgb_sum_white"] <= 1100)
        self.assertTrue(0 <= params["rgb_black_max"] <= 100)
        self.assertTrue(1.0 < params["rgb_red_ratio"] <= 4.0)
        self.assertTrue(1.0 < params["rgb_green_ratio"] <= 4.0)
        self.assertTrue(1.0 < params["rgb_blue_ratio"] <= 6.0)

    def test_derive_params_reports_bad_separation(self):
        """파랑이 흰 바닥과 거의 같으면 sep_ok=False — 칼날 경계 경고."""
        bad = dict(SAMPLES)
        bad["blue"] = (265, 295, 175)       # 사실상 흰색
        _params, report = ff.derive_rgb_params(bad)
        self.assertFalse(report["blue"]["sep_ok"])


class MigrateSavedParamsTest(unittest.TestCase):
    """§J — 구버전 저장 파일(rgb_yellow_ratio)이 통째로 거부되지 않아야 한다.

    SharedParams 는 알 수 없는/빠진 키가 하나라도 있으면 파일 전체를 무시
    하므로, 마이그레이션이 없으면 cal_* 라인 캘리브레이션까지 날아간다.
    """

    def test_old_yellow_file_migrates_and_loads(self):
        import json
        path = os.path.join(tempfile.mkdtemp(), "ffrun1.json")
        old = dict(ff.INITIAL_PARAMS)
        del old["rgb_blue_ratio"]
        old["rgb_yellow_ratio"] = 2.5       # 구버전 키
        old["cal_l_black"] = 17             # 실기에서 잡은 값이 보존돼야 한다
        with open(path, "w") as fp:
            json.dump(old, fp)
        self.assertTrue(ff.migrate_saved_params(path))
        params = SharedParams(ff.INITIAL_PARAMS, ff.PARAM_LIMITS, ff.MAX_STEP,
                              path, ui_step=ff.UI_STEP, units=ff.UNITS,
                              param_order=ff.PARAM_ORDER)
        ok, msg = params.load_saved_into_defaults()
        self.assertTrue(ok, msg)
        snap = params.snapshot()
        self.assertEqual(snap["cal_l_black"], 17)
        self.assertEqual(snap["rgb_blue_ratio"],
                         ff.INITIAL_PARAMS["rgb_blue_ratio"])
        self.assertNotIn("rgb_yellow_ratio", snap)
        # 이미 새 형식이면 no-op
        self.assertFalse(ff.migrate_saved_params(path))


class EventRingBufferTest(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.runner, self.hw, self.tele, self.params = make_runner(self.tmpdir)

    def test_single_shot_event_survives_later_frames(self):
        """§H — read_rgb 이벤트가 이후 프레임에도 실린다(watcher 폴링 유실 방지)."""
        self.hw.rgb = SAMPLES["red"]
        self.runner.on_do("read_rgb", {})
        self.runner.handle_pending()
        for _ in range(10):
            self.runner.publish("waiting_start")
        reads = [e for e in latest_events(self.tele)
                 if e["event"] == "COLOR_READ"]
        self.assertEqual(len(reads), 1)
        self.assertEqual(reads[0]["name"], "red")

    def test_buffer_capped_at_event_keep(self):
        for i in range(ff.EVENT_KEEP * 3):
            self.runner.log.log("SPAM", "N{}".format(i))
            self.runner.publish("follow")
        self.assertEqual(len(latest_events(self.tele)), ff.EVENT_KEEP)


class CalColorWizardTest(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.runner, self.hw, self.tele, self.params = make_runner(self.tmpdir)
        ff.CAL_COLOR_GAP_S = 0.0    # 테스트 가속(샘플 간 대기 생략)

    def _events(self, name):
        return [e for e in latest_events(self.tele) if e["event"] == name]

    def test_prompt_says_white_first(self):
        self.runner._cal_color_prompt()
        self.runner.publish("waiting_start")
        first = self._events("CAL_COLOR_NEXT")[0]
        self.assertEqual(first["place"], "WHITE")
        self.assertEqual(first["step"], "1/5")

    def test_wrong_color_fails_and_does_not_advance(self):
        self.hw.rgb = SAMPLES["black"]      # white 차례에 검정 위
        self.runner.cal_color_step()
        self.runner.publish("waiting_start")
        self.assertEqual(self.runner.cal_color_idx, 0)
        self.assertTrue(self._events("CAL_COLOR_FAIL"))

    def test_full_wizard_saves_and_verifies(self):
        for name in ff.CAL_COLOR_ORDER:
            self.hw.rgb = SAMPLES[name]
            self.runner.cal_color_step()
        self.runner.publish("waiting_start")
        self.assertEqual(self.runner.cal_color_idx, 5)
        verifies = self._events("CAL_COLOR_VERIFY")
        self.assertEqual(len(verifies), 5)
        self.assertTrue(all(e["reason"] == "OK" for e in verifies))
        done = self._events("CAL_COLOR_DONE")[0]
        self.assertEqual(done["reason"], "ALL_VERIFIED")
        self.assertTrue(done["saved"])
        # 파라미터 반영 + 브릭 저장 파일 생성
        self.assertEqual(self.params.snapshot()["rgb_sum_white"], 750)
        self.assertTrue(os.path.isfile(os.path.join(self.tmpdir, "ffrun1.json")))

    def test_done_then_press_again_hints_restart(self):
        for name in ff.CAL_COLOR_ORDER:
            self.hw.rgb = SAMPLES[name]
            self.runner.cal_color_step()
        self.runner.cal_color_step()        # 완료 후 재누름
        self.runner.publish("waiting_start")
        alldone = [e for e in self._events("CAL_COLOR")
                   if e["reason"] == "ALL_DONE"]
        self.assertTrue(alldone)

    def test_restart_action_resets_to_white(self):
        self.hw.rgb = SAMPLES["white"]
        self.runner.cal_color_step()
        self.assertEqual(self.runner.cal_color_idx, 1)
        self.runner.on_do("cal_color_restart", {})
        self.runner.handle_pending()
        self.assertEqual(self.runner.cal_color_idx, 0)
        self.assertEqual(self.runner.cal_color_samples, {})

    def test_knife_edge_separation_is_ng(self):
        """파랑≈흰 바닥이면 우연히 갈려도 VERIFY 는 NG(§I — 칼날 경계)."""
        seq = dict(SAMPLES)
        seq["blue"] = (265, 295, 175)
        for name in ff.CAL_COLOR_ORDER:
            self.hw.rgb = seq[name]
            self.runner.cal_color_samples[name] = seq[name]
            self.runner.cal_color_idx += 1
        self.runner._cal_color_finish()
        self.runner.publish("waiting_start")
        blue = [e for e in self._events("CAL_COLOR_VERIFY")
                if e["place"] == "BLUE"][0]
        self.assertEqual(blue["reason"], "NG")
        done = self._events("CAL_COLOR_DONE")[0]
        self.assertEqual(done["reason"], "CHECK_NG_COLORS")


if __name__ == "__main__":
    unittest.main()
