"""Stage 1~2 구동층 (ev3dev2).

판단층(순수)과 분리된 구동층이다. 여기만 ev3dev2 에 의존한다.
PC 에는 ev3dev2 가 없으므로 import 는 __init__ 안에서 한다(py_compile 안전).

배선(HARDWARE.md / Stage 0 실기 확정):
  - 주행 좌 라지 모터: outA  (전진 방향 정상)
  - 주행 우 라지 모터: outB  (전진 방향 정상)
  - 중앙 컬러센서:      in2  (Stage 0 OK)
Stage 1 은 중앙센서 1개만 쓴다(in1/in3/in4 는 다음 단계).

Stage 2 추가(2026-06-30): 엔코더 각도 기반 제자리 회전을 위해 아래 메서드를 **추가만** 한다.
  - reset_encoders() / read_encoders() : 누적 회전각(도) 리셋/읽기
  - drive_raw()                        : 트림 미적용 좌/우 명령(회전엔 트림 X — stage2 명세 §5.3)
  - beep_ok()                          : 회전 완료 신호(보정 루프 리듬). best-effort.
Stage 1 확정 메서드(drive/stop/read_center_reflect)와 __init__ 기존 동작은 수정하지 않는다.

Stage 3 추가(2026-06-30): 좌/중/우 3센서 노드 감지를 위해 아래 메서드를 **추가만** 한다.
  - read_left_reflect() / read_right_reflect() / read_reflect() : 좌/중/우 반사광.
  - enc_avg()                          : 좌/우 엔코더 절댓값 평균(도) — dist_mm 환산용.
좌/우 컬러센서(in1/in3)는 Stage 1/2 가 쓰지 않으므로 __init__ 을 건드리지 않고
**첫 사용 시 지연 오픈**한다(_ensure_side_sensors). Stage 1/2 확정 동작 불변.

Stage 4 추가(2026-07-03): 중앙센서 반사광↔컬러 모드 전환을 위해 아래 메서드를 **추가만**
한다(stage4_color.md §2 — 브릿지 후보 B/C/D 공용). 기존 메서드/__init__ 불변.
  - read_center_color(settle_s, dummy_reads) : 컬러 모드 전환 + settle + 더미읽기 후 color 1회.
  - restore_reflect_mode(settle_s)           : 반사광 모드 복귀 + settle.

Stage 4 v2 추가(2026-07-03): 중앙 상시 컬러 모드 트랙(stage4v2_color_follow.md)용으로
아래 메서드를 **추가만** 한다. 기존 메서드/__init__ 불변.
  - read_side_reflect()     : 좌/우 반사광만 — 중앙 모드를 건드리지 않는다.
  - read_center_color_now() : 컬러 모드 유지 전제의 color 1회(전환/settle 없음).

run_maze_v12 추가(2026-07-07): 그리퍼(outC MediumMotor) + 초음파(in4) + 단일 tone.
좌/우 센서와 같은 지연 오픈 패턴 — 이전 스테이지가 안 쓰는 장치는 __init__ 에서 열지 않는다.
  - grip_open() / grip_close() : 그리퍼 열기/닫기(초 단위 구동).
  - read_distance_cm()         : in4 초음파 거리(cm).
  - tone()                     : 주파수/길이 지정 tone(best-effort, beep_ok 와 동급).

final_run4 추가(2026-07-08): 브릭 가운데 버튼 시작 + wav 음성 + LCD 기록 표시.
  - wait_center_button()        : 가운데/enter 버튼 press→release 대기(timeout 지원).
  - play_wav()/tone()/beep_ok() : 백그라운드 큐 재생(비블로킹) — 주행을 막지 않고
                                  넣은 순서대로 재생. Sound 볼륨은 __init__ 에서 100%.
  - show_final4_display()       : OUT/BACK 시간과 하단 랜덤 숫자 표시(best-effort, 락).

규약: 브릭 코드는 Python 3.5 안전 — f-string 금지, .format() 사용.
"""

import os
import subprocess
import threading
import time

# --- 파일 맨 위 상수 (live param 아님; STAGES.md "좌/우 트림은 상수로 시작") ---
LEFT_MOTOR_PORT = "outA"
RIGHT_MOTOR_PORT = "outB"
CENTER_SENSOR_PORT = "in2"
# Stage 3 노드 감지용 좌/우 컬러센서(HARDWARE.md 배선). 지연 오픈한다.
LEFT_SENSOR_PORT = "in1"
RIGHT_SENSOR_PORT = "in3"
# run_maze_v12 배선(정리.md 핀맵). 지연 오픈한다.
GRIPPER_MOTOR_PORT = "outC"
ULTRASONIC_SENSOR_PORT = "in4"

# 곱셈 트림(쏠림 보정). Stage 1 보정②에서 실측해 한쪽만 미세 조정한다.
# 1.0 = 보정 없음. 빠른 쪽을 1.0 미만으로 낮추거나 느린 쪽을 그대로 둔다.
LEFT_MOTOR_TRIM = 1.0
RIGHT_MOTOR_TRIM = 1.0

MAX_SPEED = 100  # SpeedPercent 한계(±)


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


class Ev3Hardware(object):
    """중앙센서 1개 + 좌/우 주행 모터 구동층."""

    def __init__(self):
        # ev3dev2 는 여기서 import (PC py_compile 안전).
        from ev3dev2.motor import LargeMotor, SpeedPercent
        from ev3dev2.sensor.lego import ColorSensor

        self._SpeedPercent = SpeedPercent
        self._left = LargeMotor(LEFT_MOTOR_PORT)
        self._right = LargeMotor(RIGHT_MOTOR_PORT)
        self._center = ColorSensor(CENTER_SENSOR_PORT)

        # Stage 2 회전 완료음(선택). 없거나 실패해도 주행/회전에 영향 없게 best-effort.
        # (Stage 1 도 Ev3Hardware 를 쓰므로 여기서 예외가 나면 안 된다.)
        # final_run8: Sound 생성/재생 실패를 삼키지 않고 기록해 '소리 안 남'을 추적
        # 가능하게 한다(sound_available/sound_error/audio_fail_count).
        self._sound = None
        self._sound_error = None
        self._audio_error = None
        self._audio_fail_count = 0
        try:
            from ev3dev2.sound import Sound
            self._sound = Sound()
        except Exception as exc:
            self._sound = None
            self._sound_error = repr(exc)
        self._buttons = None
        self._display = None

        # final_run4: 소리는 백그라운드 큐에서 순서대로 재생한다 — 주행 루프가
        # 재생 완료를 기다리지 않고 '움직이면서' 소리가 난다. 지연 시작(_ensure_audio).
        self._audio_queue = []
        self._audio_cv = threading.Condition()
        self._audio_thread = None
        self._audio_volume_ready = False
        # final_run4: LCD 를 백그라운드 갱신 스레드와 이벤트 핸들러가 함께 그리므로
        # PIL 이미지 동시 접근을 직렬화한다.
        self._display_lock = threading.Lock()

    def read_center_reflect(self):
        """in2 반사광(0~100). 속성 접근이 모드를 COL-REFLECT 로 맞춘다(Stage 0 과 동일)."""
        return self._center.reflected_light_intensity

    def drive(self, left_speed, right_speed):
        """좌/우 바퀴 속도(%) 명령. 트림 적용 후 ±MAX_SPEED 로 클램프."""
        left = clamp(left_speed * LEFT_MOTOR_TRIM, -MAX_SPEED, MAX_SPEED)
        right = clamp(right_speed * RIGHT_MOTOR_TRIM, -MAX_SPEED, MAX_SPEED)
        self._left.on(self._SpeedPercent(left))
        self._right.on(self._SpeedPercent(right))

    def stop(self):
        """양 바퀴 정지(brake)."""
        self._left.off(brake=True)
        self._right.off(brake=True)

    # --- Stage 2 추가(엔코더 회전). Stage 1 코드는 위에서 건드리지 않았다. ---

    def drive_raw(self, left_speed, right_speed):
        """좌/우 바퀴 속도(%) 명령(트림 미적용). 제자리 회전은 좌우 대칭이어야 하므로
        직진 쏠림 보정용 LEFT/RIGHT_MOTOR_TRIM 을 적용하지 않는다(stage2 명세 §5.3)."""
        left = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
        right = clamp(right_speed, -MAX_SPEED, MAX_SPEED)
        self._left.on(self._SpeedPercent(left))
        self._right.on(self._SpeedPercent(right))

    def reset_encoders(self):
        """좌/우 모터 누적 회전각(도)을 0 으로. position 만 0 으로 둬 다른 상태는 안 건드림."""
        self._left.position = 0
        self._right.position = 0

    def read_encoders(self):
        """좌/우 모터 누적 회전각(도) 튜플. 부호는 회전 방향을 따른다."""
        return self._left.position, self._right.position

    # --- final_run6 추가(회전 시작 '틱틱' 튐 제거). 위 메서드/__init__ 불변. ---

    def set_ramp(self, up_ms, down_ms=0):
        """주행 모터 가속/감속 램프(ms) 설정. 회전 시작 시 속도PID 콜드스타트로
        생기는 '틱틱' 튐을 없애기 위해 회전 프리미티브에서만 켜고 끝나면 0 으로
        되돌린다(라인추종 drive() 에는 램프를 걸지 않는다 — 매 15ms 조향이
        뭉개진다). up_ms 는 0→최대속도 가속 시간, down_ms 는 감속 시간.
        best-effort: 커널에 속성이 없어도 조용히 통과한다."""
        for m in (self._left, self._right):
            try:
                m.ramp_up_sp = int(up_ms)
                m.ramp_down_sp = int(down_ms)
            except Exception:
                pass

    def coast(self):
        """양 바퀴 hold 해제(brake=False, 무회생 정지). 회전 직전 reset_encoders()
        가 brake-hold 상태에서 position 을 0 으로 바꾸며 내는 위치보정 킥을 막는다.
        정지 상태에서 호출하므로 바퀴가 굴러가지는 않는다."""
        self._left.off(brake=False)
        self._right.off(brake=False)

    def beep_ok(self):
        """회전 완료 신호음(best-effort, 비블로킹). Sound 가 없으면 조용히 통과."""
        self._audio_enqueue(("beep",))

    # --- Stage 3 추가(좌/중/우 반사광 + 거리 환산용 엔코더 평균). __init__ 불변. ---

    def _ensure_buttons(self):
        if self._buttons is None:
            from ev3dev2.button import Button
            self._buttons = Button()

    def center_pressed(self):
        """Return True while the EV3 brick center/enter button is pressed."""
        try:
            self._ensure_buttons()
            val = getattr(self._buttons, "enter", False)
            if callable(val):
                val = val()
            if bool(val):
                return True
            pressed = getattr(self._buttons, "buttons_pressed", None)
            if callable(pressed):
                pressed = pressed()
            if pressed:
                return ("enter" in pressed or "center" in pressed)
            checker = getattr(self._buttons, "check_buttons", None)
            if callable(checker):
                try:
                    return bool(checker(buttons=["enter"]))
                except TypeError:
                    return bool(checker(["enter"]))
            return False
        except Exception:
            return False

    def wait_center_release(self):
        while self.center_pressed():
            time.sleep(0.03)

    def _ensure_display(self):
        if self._display is None:
            from ev3dev2.display import Display
            self._display = Display()

    def display_lines(self, lines):
        """Best-effort EV3 screen text. Also prints for SSH/debug logs."""
        try:
            print("\n".join([str(x) for x in lines]))
        except Exception:
            pass
        try:
            self._ensure_display()
            self._display.clear()
            for idx, line in enumerate(lines):
                try:
                    self._display.text_grid(str(line), x=0, y=idx)
                except TypeError:
                    self._display.text_grid(str(line), 0, idx)
            self._display.update()
        except Exception:
            pass

    def speak(self, text):
        if self._sound is None:
            return False
        try:
            self._sound.speak(str(text))
            return True
        except Exception:
            return False

    def play_file(self, path):
        if not path or not os.path.exists(path):
            return False
        try:
            self._play_wav_direct(path)
            return True
        except Exception:
            return False

    def play_named_sound(self, names, fallback_text=None):
        roots = ("/usr/share/sounds/ev3dev", "/usr/share/sounds",
                 "/home/robot/sounds", "/home/robot/sound")
        exts = (".wav", ".WAV")
        for root in roots:
            for name in names:
                _base, ext = os.path.splitext(name)
                candidates = [name] if ext else [name + e for e in exts]
                for candidate in candidates:
                    if self.play_file(os.path.join(root, candidate)):
                        return True
        if fallback_text:
            return self.speak(fallback_text)
        return False

    def say_number(self, number):
        names = {1: "One", 2: "Two", 3: "Three", 4: "Four"}
        text = names.get(int(number), str(number))
        return self.play_named_sound((text, text.lower()), text.lower())

    def say_good_job(self):
        return self.play_named_sound(("Good Job", "Good job", "good_job",
                                      "good-job"), "Good job")

    def say_blue(self, number):
        words = {1: "one", 2: "two", 3: "three", 4: "four"}
        word = words.get(int(number), str(number))
        return self.speak("Blue " + word)

    def _ensure_side_sensors(self):
        """좌/우 컬러센서(in1/in3)를 첫 사용 시에만 연다(지연 오픈).

        Stage 1/2 는 좌/우 센서를 쓰지 않으므로 __init__ 에서 열지 않는다. getattr 로
        존재 여부를 확인해 기존 인스턴스 상태(__init__ 설정값)를 건드리지 않는다.
        """
        from ev3dev2.sensor.lego import ColorSensor
        if getattr(self, "_left_sensor", None) is None:
            self._left_sensor = ColorSensor(LEFT_SENSOR_PORT)
        if getattr(self, "_right_sensor", None) is None:
            self._right_sensor = ColorSensor(RIGHT_SENSOR_PORT)

    def read_left_reflect(self):
        """in1 좌센서 반사광(0~100). 속성 접근이 모드를 COL-REFLECT 로 맞춘다."""
        self._ensure_side_sensors()
        return self._left_sensor.reflected_light_intensity

    def read_right_reflect(self):
        """in3 우센서 반사광(0~100)."""
        self._ensure_side_sensors()
        return self._right_sensor.reflected_light_intensity

    def read_reflect(self):
        """좌/중/우 반사광 튜플 (l, c, r). bits 순서(LCR)와 맞춘다."""
        self._ensure_side_sensors()
        return (self._left_sensor.reflected_light_intensity,
                self.read_center_reflect(),
                self._right_sensor.reflected_light_intensity)

    def enc_avg(self):
        """좌/우 누적 회전각 절댓값 평균(도). 직진 거리 dist_mm 환산용."""
        el, er = self.read_encoders()
        return (abs(el) + abs(er)) / 2.0

    # --- Stage 4 추가(중앙센서 반사광↔컬러 모드 전환, B/C/D 공용). 위 메서드 불변. ---

    def read_center_color(self, settle_s, dummy_reads):
        """in2 컬러 모드 판독(color 정수: 0=없음 1=검정 2=파랑 3=초록 4=노랑 5=빨강 6=흰색 7=갈색).

        color 속성 첫 접근이 모드를 COL-COLOR 로 전환한다. 전환 직후 값이 튀므로
        (stage4_color.md §8 '전환 직후 오판') 전환 트리거 → settle 대기 → dummy_reads 회
        버리고 → 마지막 1회를 반환한다. 전환/settle 비용은 호출부(stage4d bench)가 실측한다.
        """
        _ = self._center.color  # 모드 전환 트리거(이 값은 버린다)
        if settle_s > 0:
            time.sleep(settle_s)
        for _i in range(int(dummy_reads)):
            _ = self._center.color
        return self._center.color

    def _read_center_rgb_raw(self):
        try:
            return (self._center.value(0),
                    self._center.value(1),
                    self._center.value(2))
        except Exception:
            raw = self._center.raw
            return (raw[0], raw[1], raw[2])

    def read_center_rgb(self, settle_s, dummy_reads):
        """Switch in2 to RGB-RAW and return one (red, green, blue) sample."""
        try:
            self._center.mode = "RGB-RAW"
        except Exception:
            pass
        if settle_s > 0:
            time.sleep(settle_s)
        for _i in range(int(dummy_reads)):
            self._read_center_rgb_raw()
        return self._read_center_rgb_raw()

    def read_center_rgb_now(self):
        """in2 RGB-RAW 1회 — 전환/settle 없음(final_run9 상시 RGB 트랙용).

        시작 시 read_center_rgb() 로 RGB-RAW 모드에 들어간 뒤 매 루프 호출용.
        (read_center_color_now 의 RGB 판이다 — 모드가 같으면 재전환 비용 없음.)
        """
        return self._read_center_rgb_raw()

    def restore_reflect_mode(self, settle_s):
        """컬러 모드 → 반사광 모드 복귀 + settle(라인추종 재개 전 안정화)."""
        _ = self._center.reflected_light_intensity  # 모드 복귀 트리거
        if settle_s > 0:
            time.sleep(settle_s)

    # --- Stage 4 v2 추가(중앙 상시 컬러 모드 트랙, stage4v2_color_follow.md §2). 위 불변. ---

    def read_side_reflect(self):
        """좌/우 반사광만 (l, r). 중앙센서를 건드리지 않는다.

        read_reflect() 는 중앙 반사광 속성을 읽어 중앙 모드를 COL-REFLECT 로 되돌리므로
        중앙 상시 컬러 모드 트랙(Stage 4 v2)에서는 반드시 이 메서드를 쓴다.
        """
        self._ensure_side_sensors()
        return (self._left_sensor.reflected_light_intensity,
                self._right_sensor.reflected_light_intensity)

    def read_center_color_now(self):
        """in2 color 1회 — 전환/settle/더미읽기 없음.

        시작 시 read_center_color() 로 컬러 모드에 들어간 뒤 매 루프 호출용.
        ev3dev2 는 모드가 같으면 재전환하지 않으므로 추가 비용이 없다.
        """
        return self._center.color

    # --- run_maze_v12 추가(그리퍼 + 초음파 + tone). 위 메서드/__init__ 불변. ---

    def _ensure_gripper(self):
        """그리퍼 모터(outC)를 첫 사용 시에만 연다(좌/우 센서와 같은 지연 오픈)."""
        from ev3dev2.motor import MediumMotor
        if getattr(self, "_gripper", None) is None:
            self._gripper = MediumMotor(GRIPPER_MOTOR_PORT)

    def grip_open(self, speed, seconds):
        """그리퍼 열기 — speed(%) 정방향 seconds 초. 열림 유지엔 힘이 필요 없어 brake 없음."""
        self._ensure_gripper()
        self._gripper.on_for_seconds(self._SpeedPercent(speed), seconds, brake=False)

    def grip_close(self, speed, seconds):
        """그리퍼 닫기(물체 파지) — 역방향 seconds 초 후 brake 로 파지 유지."""
        self._ensure_gripper()
        self._gripper.on_for_seconds(self._SpeedPercent(-speed), seconds, brake=True)

    def read_distance_cm(self):
        """in4 초음파 거리(cm). 첫 사용 시에만 센서를 연다."""
        from ev3dev2.sensor.lego import UltrasonicSensor
        if getattr(self, "_ultrasonic", None) is None:
            self._ultrasonic = UltrasonicSensor(ULTRASONIC_SENSOR_PORT)
        return self._ultrasonic.distance_centimeters

    def tone(self, freq_hz, dur_ms):
        """단일 tone(비블로킹 큐 재생, best-effort). Sound 가 없으면 조용히 통과."""
        self._audio_enqueue(("tone", freq_hz, dur_ms / 1000.0))

    # --- final_run8: 오디오 자가진단(소리 안 남 원인 추적) ---

    def sound_available(self):
        """Sound 객체 생성에 성공했는지(=소리를 낼 수 있는지)."""
        return self._sound is not None

    def sound_error(self):
        """Sound 생성 실패 시의 예외 문자열(성공이면 None)."""
        return self._sound_error

    def audio_fail_count(self):
        """워커에서 재생이 실패한 누적 횟수(0 이면 재생 계통 정상)."""
        return self._audio_fail_count

    # --- final_run4: 소리 백그라운드 큐(주행을 막지 않는다) ---

    def _ensure_audio(self):
        """오디오 워커 스레드를 첫 재생 시에만 띄운다(daemon)."""
        if self._audio_thread is not None:
            return
        t = threading.Thread(target=self._audio_worker, name="audio")
        t.daemon = True
        self._audio_thread = t
        t.start()

    def _audio_enqueue(self, item):
        """재생 항목을 큐에 넣는다. 워커가 순서대로(직렬) 재생하므로 red→number
        같은 연속 재생의 순서는 보장되고, 호출자(주행 루프)는 막히지 않는다."""
        if self._sound is None and item[0] != "wav":
            return
        self._ensure_audio()
        with self._audio_cv:
            self._audio_queue.append(item)
            self._audio_cv.notify()

    def _play_wav_direct(self, path):
        """Play wav through aplay without ev3dev2.Sound.set_volume().

        On this EV3 the PCM mixer can be left at 0%, which makes aplay
        succeed silently. Restore max wav volume before the first clip.
        """
        self._ensure_wav_volume()
        subprocess.call(("/usr/bin/aplay", "-q", path))

    def _ensure_wav_volume(self):
        if self._audio_volume_ready:
            return
        try:
            with open(os.devnull, "w") as devnull:
                subprocess.call(("/usr/bin/amixer", "cset", "numid=2", "256"),
                                stdout=devnull, stderr=devnull)
        except Exception as exc:
            self._audio_error = repr(exc)
            self._audio_fail_count += 1
        self._audio_volume_ready = True

    def _audio_worker(self):
        while True:
            with self._audio_cv:
                while not self._audio_queue:
                    self._audio_cv.wait()
                item = self._audio_queue.pop(0)
            kind = item[0]
            try:
                if kind == "wav":
                    self._play_wav_direct(item[1])       # 완료까지 기다리되 워커 안에서만
                elif kind == "tone":
                    self._sound.play_tone(item[1], item[2])
                elif kind == "beep":
                    self._sound.beep()
            except Exception as exc:
                # final_run8: 삼키되 흔적은 남긴다(마지막 에러 + 카운트).
                self._audio_error = repr(exc)
                self._audio_fail_count += 1

    # --- final_run4 추가(버튼 시작 + wav 음성 + LCD 표시). 위 메서드 불변. ---

    def _ensure_button(self):
        """브릭 버튼 객체를 첫 사용 시에만 연다."""
        if getattr(self, "_button", None) is None:
            from ev3dev2.button import Button
            self._button = Button()

    def _center_button_pressed(self):
        """ev3dev2 버전 차이를 흡수해 가운데/enter 버튼 눌림 여부를 읽는다."""
        self._ensure_button()
        for name in ("enter", "enter_button"):
            if hasattr(self._button, name):
                value = getattr(self._button, name)
                try:
                    return bool(value() if callable(value) else value)
                except Exception:
                    pass
        try:
            pressed = self._button.buttons_pressed
            if callable(pressed):
                pressed = pressed()
            return "enter" in pressed or "center" in pressed
        except Exception:
            return False

    def wait_center_button(self, stop_cb=None, reset_cb=None, poll_s=0.05,
                           timeout=None):
        """가운데 버튼 press→release 를 기다린다.

        반환: True=눌렀다 뗌 / False=중단·리셋 / None=timeout(그 안에 안 눌림).
        timeout 을 주면 블로킹하지 않아 호출자가 그 사이 대시보드 액션
        (calibrate 등)을 처리할 수 있다(final_run4 시작 대기)."""
        try:
            self._ensure_button()
        except Exception:
            return False
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if stop_cb is not None and stop_cb():
                return False
            if reset_cb is not None and reset_cb():
                return False
            if self._center_button_pressed():
                break
            if deadline is not None and time.monotonic() >= deadline:
                return None
            time.sleep(poll_s)
        while self._center_button_pressed():
            if stop_cb is not None and stop_cb():
                return False
            if reset_cb is not None and reset_cb():
                return False
            time.sleep(poll_s)
        return True

    def play_wav(self, path):
        """wav 파일 재생(비블로킹 큐, best-effort). 주행 루프는 막지 않고,
        큐에 넣은 순서대로(워커 스레드 안에서 완료까지) 재생돼 순서는 보장된다."""
        self._audio_enqueue(("wav", path))

    def show_final4_display(self, out_elapsed=None, return_elapsed=None,
                            bottom_number=None):
        """final_run4 기록 표시(best-effort): 위 OUT, 그 아래 BACK, 맨 아래 숫자."""
        try:
            from ev3dev2.display import Display
            from PIL import ImageDraw, ImageFont
        except Exception:
            return
        # 백그라운드 갱신 스레드와 이벤트 핸들러가 함께 호출하므로 직렬화한다.
        with self._display_lock:
            try:
                self._draw_final4_display(Display, ImageDraw, ImageFont,
                                          out_elapsed, return_elapsed, bottom_number)
            except Exception:
                pass

    def _draw_final4_display(self, Display, ImageDraw, ImageFont,
                             out_elapsed, return_elapsed, bottom_number):
        if getattr(self, "_display", None) is None:
            self._display = Display()
        display = self._display
        display.clear()
        draw = ImageDraw.Draw(display.image)
        # 폰트는 한 번만 로드해 캐시한다 — 유지 스레드가 초당 여러 번 그리므로
        # 매번 디스크에서 truetype 을 읽으면 EV3 CPU 가 버벅인다.
        if getattr(self, "_fonts", None) is None:
            try:
                path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
                self._fonts = (ImageFont.truetype(path, 22),
                               ImageFont.truetype(path, 42))
            except Exception:
                self._fonts = (ImageFont.load_default(), ImageFont.load_default())
        font_big, font_num = self._fonts
        if out_elapsed is not None:
            draw.text((0, 0), "OUT {:.1f}s".format(out_elapsed),
                      font=font_big, fill=0)
        if return_elapsed is not None:
            draw.text((0, 30), "BACK {:.1f}s".format(return_elapsed),
                      font=font_big, fill=0)
        if bottom_number is not None:
            text = str(bottom_number)
            try:
                width, height = draw.textsize(text, font=font_num)
            except Exception:
                width, height = (24, 24)
            draw.text(((178 - width) // 2, max(62, 128 - height - 2)),
                      text, font=font_num, fill=0)
        display.update()
