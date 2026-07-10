#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EV3 공식 사운드(.rsf) → wav 변환기 (PC에서 1회 실행).

블록코딩 프로그램(LEGO MINDSTORMS Edu EV3) 리소스에 내장된 공식 음성
(One/Two/.../Red/Good job)을 꺼내 브릭에서 aplay 로 재생 가능한 wav 로
변환한다 — final_run9 가 espeak 합성음 대신 블록코딩과 동일한 목소리를
쓰게 하기 위함(사용자 문제 3: "오디오 코덱 위치를 찾기 힘듬"의 해답.
코덱은 브릭이 아니라 PC 프로그램 Resources 안에 있었다).

RSF 포맷(EV3 Robot Sound File):
  bytes 0-1: 포맷 ID(0x0100 = 비압축 사운드)
  bytes 2-3: 데이터 길이(big-endian)
  bytes 4-5: 샘플레이트(big-endian, 보통 8000)
  bytes 6-7: 재생 모드(무시)
  bytes 8~ : 8-bit unsigned PCM mono

사용: python tools/rsf2wav.py
  (경로 상수는 아래 — 필요한 파일만 골라 ev3final/sounds/ 에 쓴다)
"""

import os
import struct
import sys
import wave

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 블록코딩 프로그램의 사운드 리소스 위치(교육판).
RSF_ROOT = os.path.join(os.path.dirname(_ROOT), "LEGO MINDSTORMS Edu EV3",
                        "Resources", "BrickResources", "Education",
                        "Sounds", "files")
OUT_DIR = os.path.join(_ROOT, "sounds")

# final_run9 가 쓰는 키 이름 → 원본 rsf 상대경로.
WANTED = {
    "num_1": "Numbers/One.rsf",
    "num_2": "Numbers/Two.rsf",
    "num_3": "Numbers/Three.rsf",
    "num_4": "Numbers/Four.rsf",
    "num_5": "Numbers/Five.rsf",
    "num_6": "Numbers/Six.rsf",
    "num_7": "Numbers/Seven.rsf",
    "num_8": "Numbers/Eight.rsf",
    "num_9": "Numbers/Nine.rsf",
    "num_10": "Numbers/Ten.rsf",
    "red": "Colors/Red.rsf",
    "good_job": "Communication/Good job.rsf",
}


def rsf_to_wav(rsf_path, wav_path):
    with open(rsf_path, "rb") as f:
        raw = f.read()
    fmt, length, rate = struct.unpack(">HHH", raw[:6])
    if fmt != 0x0100:
        raise ValueError("compressed/unknown rsf format 0x{:04x}: {}"
                         .format(fmt, rsf_path))
    pcm = raw[8:8 + length]
    w = wave.open(wav_path, "wb")
    try:
        w.setnchannels(1)
        w.setsampwidth(1)           # 8-bit unsigned PCM (wave 모듈 규약과 일치)
        w.setframerate(rate)
        w.writeframes(pcm)
    finally:
        w.close()
    return rate, len(pcm)


def main():
    if not os.path.isdir(RSF_ROOT):
        print("rsf root not found:", RSF_ROOT)
        return 1
    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)
    for key in sorted(WANTED):
        src = os.path.join(RSF_ROOT, WANTED[key])
        dst = os.path.join(OUT_DIR, key + ".wav")
        rate, n = rsf_to_wav(src, dst)
        print("{}: {} bytes @ {} Hz -> {}".format(key, n, rate,
                                                  os.path.relpath(dst, _ROOT)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
