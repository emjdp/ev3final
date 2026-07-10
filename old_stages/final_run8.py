#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_run8 — 운반 미션(소리 없음) + 마커 확정 + 스무스 턴.

미션(왕복 운반):
  1) 노란점(출발): 노랑 감지 → 그리퍼 오픈(물체 받을 준비) → 출발.
     ※ 물체는 출발 시 잡지 않는다.
  2) 초록점까지 미로 탐색(라인추종 + 분기 탐색). 가는 길에 물체를 만나면
     (초음파 grab_dist 이내) 그때 파지한다.
  3) 초록점: 물체 내려놓기(그리퍼 오픈) → 유턴 → 그 시점 지도로 복귀 계획.
  4) 노란점까지 복귀(지도 기반 최소거리, 모든 빨강 재방문). 초록/커브는 경유.
     오는 길에도 물체를 만나면 파지한다(가는 길과 동일).
  5) 노란점(도착): 그리퍼 오픈으로 물체 해제 → 완주.

final_run3 대비 변경(순수 판단층 Explorer/build_return_route/PID/node_bits 는
그대로 — tests/test_run_maze_v4_logic.py 계속 통과):
  A) 파지: 출발/초록에서 기다렸다 잡지 않고, 미로 이동 중 물체를 만나면
     (초음파) 잡는다(explore 루프, 그립이 비어 있을 때만). 초록은 내려놓고
     유턴만, 재파지는 복귀 이동 중 자동으로.
  B) 마커 오검출 방지: 색은 1프레임이 아니라 정지 후 여러 번 재판독해 다수결로
     확정(_confirm_marker_color). 코너/경계에서 튄 가짜 초록이 배달을 트리거하던
     final_run3 버그 제거.
  C) 소리 전부 제거: 마커/회전/난수/완주 오디오 없음(나중에 재도입 예정).
     LCD/스톱워치도 없음.
  D) 스무스 턴(final_run6): turn() 이 reset_encoders 전에 coast() 로 hold 를
     풀고, 회전 동안만 set_ramp(turn_ramp_ms)로 가속 램프 → 회전 시작 '틱틱'
     튐 제거. finally 에서 ramp 0 복원(라인추종 조향엔 영향 없음).
  E) 중앙 '라인 위' 판정을 검정 → "흰색만 아니면"으로 완화(on_line()).
     증상: 라인 위인데 중앙이 검정으로 안 읽히면 유실 처리로 후진하고 다른
     길을 찾아버린다. 실측(follow 3919 샘플) 중앙색 분포는 BLACK 92.4% /
     WHITE 4.6% / BROWN 2.0% / BLUE 0.9% / NONE 0.03% 인데, BROWN·BLUE·NONE 은
     EV3 컬러센서가 검정↔흰색 경계나 포화 검정에서 내는 오분류이지 흰 바닥이
     아니다. bits 000(=LOST_BITS)으로 읽힌 follow 프레임 221개 중 90개(41%)가
     이 BROWN/BLUE 였다. on_line(color) = (color != COL_WHITE) 하나로 6곳을
     통일한다 — node_bits 중앙 bit / backup_to_line 선 발견 / 회전 후 재획득 /
     handle_node 의 has_straight / _pivot_scan / realign_to_line.
     진짜 유실(양쪽 반사광 흰색 + 중앙 WHITE)은 그대로 잡힌다.
  F) 교차로 접근 가드: 가로선을 한쪽 반사광이 먼저 스치는 순간(원시값이
     노드 임계 35/30 에 닿기 전 하강 구간) PID 가 좌우 차이를 라인 이탈로
     오해해 한계 조향 → 차체가 틀어진 채 진입(대각선/위빙)하던 문제 수정.
     반사광 < 노드임계+NODE_GUARD_MARGIN 이면 조향을 끄고 감속 직진으로
     자세를 유지하며, confirm_node 확정 또는 값 회복까지 붙든다.
     NODE_GUARD_MAX_MS 초과 지속이면 가로선이 아니므로 PID 복귀.

final_run8 이번 변경(PD 라이브 튜닝):
  - soft-deadband 완전 제거(deadband 파라미터 포함). error 를 그대로 P 에
    넣어 조향한다 — 중앙 근처 미세 헌팅은 kp/kd 라이브 조정으로 잡는다.
  - 고정 상수 KD(구 PID_KD=0.05)를 라이브 파라미터 kd 로 승격. 주행 중
    대시보드에서 kp 와 kd 를 실시간으로 조정할 수 있다.
  - 적분(I)은 코드/가드(INTEG_*)를 그대로 두되 기본값만 ki 0.06→0.0.
    ki=0 이면 적분이 완전히 꺼지는 기존 동작이라 필요할 때만 켜서 쓴다.

final_run8 이번 변경(confirm 거리 결정형):
  - confirm creep 종료를 시간(node_confirm_ms) → 거리(node_confirm_mm)로 교체.
    진입 감속 거리가 타이머 창에 섞여 같은 40ms 에서 creep_mm 이 1.5~7.8mm
    로 산포하던 것을, '감지점 + 정확히 X mm'로 결정적으로 만든다.
  - 스톨/걸림 대비 CONFIRM_TIMEOUT_S(1.5s) 상한을 함께 두고, 상한 도달 시에도
    동일하게 정지 후 재판정으로 진행하되 로그에 timeout=True 를 남긴다.

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      정지는 네트워크 stop 또는 Ctrl-C, 재시작은 네트워크 reset.

실행(브릭):   python3 stages/final_run8.py
문법 점검(PC): python3 -m py_compile stages/final_run8.py lib/*.py
단위 테스트:  python3 tests/test_run_maze_v4_logic.py (ev3dev2 불필요)

===== 이하 final_run3 원문 헤더(복귀 계획 알고리즘 설명) =====

final_run3 — final_run2 + 복귀(home) 로직 교체: 지도 기반 최소거리 전노드 재방문.

복귀 사양 변경(채점 기준): 복귀 중에도 모든 빨강 마커를 다시 지나가야 한다.
단, 총 이동거리는 최소. 가는 길(out) 탐색/배달/유실 처리는 final_run2 와
동일하고, 초록 배달 시퀀스(전진→그리퍼 오픈→후진→유턴)가 끝나는 즉시
탐색을 중단하고 그 시점까지 구축된 지도로 복귀 전체 계획을 세운다.

복귀 계획(판단층, 순수 함수 — PC 단독 테스트 가능):
  - explorer_to_graph: Explorer 트리 지도 → 일반 그래프(adj, mark). DONE
    처리된 팔 끝(빨강/초록/막다른길)은 리프 노드로 승격, 루트의 parent_dir
    방향에 가상 리프 "home". OPEN 팔(미탐색)은 간선이 아니다.
  - shortest_path: BFS 최단 경로(트리라 유일) — start→home 트렁크.
  - build_return_route: 트렁크는 1회, 트렁크 밖 모든 가지는 분기 노드에서
    왕복 2회 방문하는 스텝 리스트 [(절대방향, 도착노드ID), ...]. 트리에서
    "모든 노드를 방문하고 home 에서 끝나는" 최소거리 경로다
    (총 스텝 = 2×간선 수 − 트렁크 간선 수). 가지 순서는 현재 heading 기준
    좌>우>직, 트렁크 진행 방향은 항상 마지막(먼저 타면 못 돌아온다).

복귀 실행층:
  - 분기/빨강/초록/막다른길 '도착'마다 계획 스텝을 하나 소비하고 다음
    스텝의 절대방향을 상대 move 로 변환해 회전(리프에선 계획상 자연히 유턴).
    커브(n_exits==1)는 노드가 아니므로 스텝을 소비하지 않는다 — 간선에
    커브가 몇 개든 계획과 어긋나지 않는다.
  - 빨강 재방문: 정지+부저 2번(기존 유지) + home_revisit 카운트 + REVISIT_HOME
    로그 후 계획 계속. 초음파 파지는 HOME 에서 원래 비활성.
  - 도착 종류/가능한 move 가 계획과 어긋나면 RETURN_FALLBACK 로그 후 즉석
    탐색(좌>우>직)으로 전환 — 절대 그 자리에 멈추지 않는다. 노랑을 계획보다
    일찍 만나면 그대로 종료하되 못 본 빨강 수를 경고 로그로 남긴다.
  - 텔레메트리: home_total(빨강 총수)/home_revisit 를 매 프레임 publish.
    새 이벤트 RETURN_PLAN / RETURN_STEP / REVISIT_HOME / RETURN_FALLBACK,
    HOME_REACHED 에 revisit/missed 필드 추가.

초록을 끝내 못 만나 지도가 완성되면(EXPLORE_DONE) 루트에서 같은 계획
함수를 호출한다(트렁크가 루트→home 1간선으로 퇴화할 뿐 동일 알고리즘).

단위 테스트: python3 tests/test_run_maze_v4_logic.py (ev3dev2 불필요)

===== 이하 final_run2 원문 헤더 =====

final_run2 — final_run1 + run_maze_v13 노드 bits 임계값 실측 재튜닝 반영판.

문제(실기): 라인을 따라가긴 하는데 정상 주행 중에도 항상 한쪽으로 기울어
달린다. v13 까지의 조향은 error = 우반사광 - 좌반사광 '원시값' PD 인데,
이 구조엔 치우침의 원인이 두 개 내장돼 있다.

  원인 A — 센서 개체차: 좌/우 컬러센서는 같은 흰 바닥/같은 검정 위에서도
    원시값이 다르다(장착 높이·LED·개체 편차). 그래서 로봇이 기하학적으로
    라인 중앙에 있어도 error != 0 이고, PD 가 수렴하는 'error = 0' 지점은
    중앙에서 한쪽으로 밀린 지점이 된다. (임계값이 좌 66/우 63, 좌 35/우 30
    로 비대칭 튜닝된 것 자체가 두 센서가 다르다는 증거.)
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
  3) (구) P soft-deadband — 중앙 근처 미세 에러를 P 0 으로 두던 밴드는
     final_run8 PD 라이브 튜닝 커밋에서 제거했다. 지금은 error 를 그대로
     kp 배 하고, 미세 헌팅은 kp/kd 라이브 조정으로 잡는다.

사용법: 라인 위(직선 구간, 센서가 라인 근처)에 로봇을 세우고 대시보드
calibrate 액션 실행 → 소각 스윕 후 beep 2번(성공) / 낮은 tone(실패, 기존값
유지). 이후 kp/kd/ki 를 라이브 튜닝. ki=0 이면 적분 완전 off.

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

final_run2 변경(run_maze_v13 이번 커밋 반영, 노드 bits 임계값만):
  12:01 편도성공 로그 실측 근거 — 가로선 위 좌/우 반사광 8~10, 일반주행
  중앙값 74, 하위1% 51~58. 기존 18/14 는 '완전 검정'만 통과해 센서가
  가로선을 스치는 순간의 중간값(15~40)을 놓쳐 T/십자(111) 동시 인식에
  실패했다. left_th_node/right_th_node 를 35/30 으로 올려 반쯤 걸친
  순간도 잡는다 — 일반주행에서 35 아래는 521샘플 중 1~2개뿐이고, 그
  오검출도 confirm 정지 재판정(정지 시 70대 vs 8대)이 걸러낸다.

(이하 final_run3 원문 헤더 — 복귀 계획 알고리즘 설명. 실행/문법 점검 명령은
 위 final_run8 헤더를 따른다.)
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
NODE_GUARD_MARGIN = 12      # 반사광 < 노드임계+마진 이면 가로선 접근으로 본다(§변경 F)
NODE_GUARD_MAX_MS = 700     # 직진 유지 상한 — 초과 지속이면 가로선이 아니라 정렬 불량
NODE_DEBOUNCE_MS = 900      # 노드 '확정' 후 재감지 최소 간격
NODE_CANCEL_DEBOUNCE_MS = 150  # confirm '취소' 후 간격(v13.1)
MARKER_DEBOUNCE_MS = 1500   # 같은 마커 재감지 방지
MARKER_PAUSE_S = 0.08       # 마커 정지 후 잠깐 멈춤
CONFIRM_SETTLE_S = 0.08     # 재판정 전 정지 안정화
CONFIRM_TIMEOUT_S = 1.5     # 거리 기반 confirm 안전 상한(스톨/걸림 대비)
GRIP_SEC = 0.8              # 그리퍼 열기/닫기 구동 시간
LOOP_DELAY_S = 0.015
FOLLOW_LOG_S = 0.25         # LINE_FOLLOW 로그 최소 간격
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)

# 마커 확정(§변경 B) — 색은 1프레임이 아니라 정지 후 재판독 다수결로 확정한다.
# 코너/흑백 경계에서 튄 가짜 초록이 배달을 트리거하던 final_run3 버그 방지.
MARKER_CONFIRM_SAMPLES = 4  # 정지 후 재판독 횟수
MARKER_CONFIRM_MIN = 3      # 같은 마커 색이 이만큼 이상이어야 확정(첫 판독 포함 아님)
MARKER_CONFIRM_GAP_S = 0.015

# PID — KD 는 라이브 파라미터(kd)로 승격, derivative 는 clamp + EMA 로 완화.
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

# (final_run8: 소리 전부 제거 — 마커/회전/난수 오디오 없음. 나중에 재도입 예정.)


# ---------------------------------------------------------------------
# 라이브 파라미터 — 대시보드/robotctl 로 실기에서 튜닝한다.
# ---------------------------------------------------------------------

# 파라미터당 한 줄: (이름, 초기값, min, max, max_step, ui_step, 단위)
# cal_* 4개는 calibrate 액션이 실측치를 직접 기록하므로 max_step=100(제한 없음).
# 기본값 black 0 / white 100 = 정규화 항등(원시값 그대로) → 미보정 시 v13 동작.
PARAM_TABLE = (
    ("base_speed",      20,    5,   45,   5,    1,    "%"),
    ("kp",              0.17,  0.0, 3.0,  0.1,  0.01, ""),
    ("kd",              0.05,  0.0, 1.0,  0.05, 0.005, ""),  # 기존 PID_KD 초기값 — 라이브 승격
    ("ki",              0.0,   0.0, 0.5,  0.05, 0.01, ""),   # 기본 0 = 적분 완전 off(코드는 유지)
    ("turn_speed",      5,     5,   40,   5,    1,    "%"),
    ("turn_ramp_ms",    250,   0,   600,  100,  50,   "ms"),  # 회전 가속 램프(시작 틱틱 튐 방지, v6)
    ("node_confirm_mm", 8.0,   0.0, 25.0, 5.0,  1.0,  "mm"),  # 감지점+정확히 X mm 에서 재판정(거리 결정형)
    ("left_th_steer",   66,    0,   100,  3,    1,    "%"),   # 유실 복구 검정 판정(원시값)
    ("right_th_steer",  63,    0,   100,  3,    1,    "%"),
    # 노드 bits 판정(원시값) — 12:01 편도성공 로그 실측: 가로선 위 8~10 / 일반주행 하위1% 51~58.
    # 18/14 는 '완전 검정'만 통과해 스치는 순간의 중간값(15~40)을 놓쳐 111 미인식.
    # 35/30 이면 반쯤 걸친 순간도 잡고, 일반주행 오검출은 confirm 재판정이 거른다.
    ("left_th_node",    35,    0,   100,  3,    1,    "%"),
    ("right_th_node",   30,    0,   100,  3,    1,    "%"),
    ("cal_l_black",     0,     0,   100,  100,  1,    "%"),   # calibrate 가 기록
    ("cal_l_white",     100,   0,   100,  100,  1,    "%"),
    ("cal_r_black",     0,     0,   100,  100,  1,    "%"),
    ("cal_r_white",     100,   0,   100,  100,  1,    "%"),
    ("node_advance_mm", 60,    0,   120,  10,   10,   "mm"),  # 의심지점 기준 총 전진
    ("goal_advance_mm", 100,   0,   200,  10,   10,   "mm"),  # 배달 전·후진 거리
    ("turn_90_factor",  0.65,  0.3, 2.0,  0.05, 0.01, "x"),
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

SAVE_PATH = os.path.join(_ROOT, "config", "final_run8.json")
STAGE_NAME = "final_run8"

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


def on_line(center_color):
    """중앙 컬러센서가 '라인 위'인가 — 흰 바닥이 아니면 전부 라인으로 본다.

    검정(COL_BLACK)만 라인으로 치면 안 된다. 실측(follow 3919 샘플): 중앙이
    BLACK 92.4% / WHITE 4.6% / BROWN 2.0% / BLUE 0.9% / NONE 0.03%. BROWN·BLUE·
    NONE 은 EV3 컬러센서가 검정↔흰색 경계나 포화 검정에서 내는 오분류이지 흰
    바닥이 아니다. 그런데 bits 000(=LOST_BITS) 로 읽힌 follow 프레임 221개 중
    90개(41%)가 바로 이 BROWN/BLUE 였다 — 라인 위인데 유실로 판정해 후진하고
    다른 길을 찾아버리던 원인.

    마커색(빨강/노랑/초록)도 라인 위다(스티커는 경로 위에 있다). 마커 처리는
    호출부가 bits 보다 먼저 handle_marker 로 하므로 여기선 '라인'이면 충분하고,
    마커 디바운스로 handle_marker 가 넘긴 프레임이 유실로 빠지지도 않는다.
    """
    return center_color != COL_WHITE


def node_bits(reflect_l, center_color, reflect_r, snap):
    """노드 판정 bits(좌,중,우) — 좌/우는 반사광 '원시값', 중앙은 라인 위 여부.

    임계값(left/right_th_node)이 이미 원시값 기준으로 실기 튜닝돼 있으므로
    조향 정규화와 분리해 원시값을 유지한다(헤더 주석 참조).
    중앙은 on_line() — '검정이어야' 가 아니라 '흰색만 아니면' 라인이다.
    """
    return (1 if reflect_l < snap["left_th_node"] else 0,
            1 if on_line(center_color) else 0,
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
# 복귀 계획(순수) — 지도 기반 최소거리 전노드 재방문 경로.
# ---------------------------------------------------------------------

def shortest_path(adj, start, home):
    """BFS 최단 경로(트리라 유일) — [start, ..., home] 노드 리스트, 없으면 None.

    이웃 순회는 방향 문자열 정렬로 고정해 파이썬 3.5(dict 순서 비보장)에서도
    결정적으로 동작하게 한다.
    """
    if start == home:
        return [start]
    prev = {start: None}
    queue = [start]
    while queue:
        node = queue.pop(0)
        for d in sorted(adj.get(node, {})):
            nxt = adj[node][d]
            if nxt in prev:
                continue
            prev[nxt] = node
            if nxt == home:
                path = [home]
                while path[-1] != start:
                    path.append(prev[path[-1]])
                path.reverse()
                return path
            queue.append(nxt)
    return None


def explorer_to_graph(nodes, arm_ends):
    """Explorer 트리 지도 → (adj, mark) 일반 그래프(순수).

    adj: 노드ID → {절대방향: 이웃ID}(양방향). 분기 노드는 Explorer 정수 ID
    그대로, DONE 팔 끝은 "leaf:노드:방향" 리프로 승격, 루트 parent_dir 에
    가상 리프 "home". OPEN 팔(미탐색)은 무엇이 있는지 모르므로 간선이 아니다.
    mark: 리프ID → "red"|"green"|"yellow"|"dead_end"|"home" (arm_ends 기록).
    """
    adj = {}
    mark = {}
    for nid in nodes:
        adj.setdefault(nid, {})
        arms = nodes[nid]["arms"]
        for d in arms:
            val = arms[d]
            if val == "OPEN":
                continue
            if val == "DONE":
                leaf = "leaf:{}:{}".format(nid, d)
                adj[nid][d] = leaf
                adj[leaf] = {turn_heading(d, "U"): nid}
                mark[leaf] = arm_ends.get((nid, d), "dead_end")
            else:                       # 자식 분기 노드 id
                adj[nid][d] = val
    for nid in nodes:
        pd = nodes[nid]["parent_dir"]
        parent = nodes[nid]["parent"]
        if parent is not None:
            adj[nid][pd] = parent
        else:
            adj[nid][pd] = "home"
            adj["home"] = {turn_heading(pd, "U"): nid}
            mark["home"] = "home"
    return adj, mark


def _priority_rank(heading, target_dir, priority):
    rel = rel_move(heading, target_dir)
    if rel in priority:
        return priority.index(rel)
    return len(priority)    # "U"(정면 뒤쪽 팔) — 항상 마지막에 선택


def _visit_branches(adj, node, heading, skip, priority, route):
    """node 에서 skip 제외 모든 이웃 서브트리를 왕복 방문(재귀, 순수).

    스텝을 route 에 append 하고 끝난 시점의 heading 을 반환한다. 가지 하나를
    다녀올 때마다 heading 이 바뀌므로 우선순위(상대방향)는 매번 재계산한다.
    """
    done = set(skip)
    while True:
        cand = [d for d in adj[node] if d not in done]
        if not cand:
            return heading
        cand.sort(key=lambda d: (_priority_rank(heading, d, priority), d))
        d = cand[0]
        child = adj[node][d]
        back = turn_heading(d, "U")
        route.append((d, child))
        _visit_branches(adj, child, d, (back,), priority, route)
        route.append((back, node))
        heading = back
        done.add(d)


def build_return_route(adj, mark, start, home, heading0,
                       priority=("L", "R", "S")):
    """지도 기반 최소거리 전노드 재방문 복귀 계획(순수).

    start→home 유일 경로(트렁크)는 1회만 지나고, 트렁크 밖 모든 가지는
    트렁크상의 각 분기 노드에서 왕복 2회 방문한다 — 트리에서 "모든 노드를
    방문하고 home 에서 끝나는" 최소거리 경로(총 스텝 = 2×간선 − 트렁크 간선).
    분기 노드에선 "왔던 방향"과 "트렁크 진행 방향"을 제외한 이웃을 현재
    heading 기준 priority(기본 좌>우>직)로 순회하고, 트렁크 방향은 항상
    마지막(그 방향을 먼저 타면 다시 못 돌아온다).

    mark 는 경로 계산엔 쓰지 않지만(어차피 전노드 방문) 호출부가 빨강 수
    집계·검증에 쓰도록 시그니처에 유지한다.

    반환: 이동 스텝 리스트 [(절대방향, 도착노드ID), ...]
    """
    trunk = shortest_path(adj, start, home)
    if trunk is None or len(trunk) < 2:
        return []
    route = []
    heading = heading0
    prev = None
    for i in range(len(trunk) - 1):
        node = trunk[i]
        nxt = trunk[i + 1]
        trunk_dir = None
        for d in adj[node]:
            if adj[node][d] == nxt:
                trunk_dir = d
                break
        skip = [trunk_dir]
        if prev is not None:            # 직전 트렁크 노드로 돌아가는 방향
            for d in adj[node]:
                if adj[node][d] == prev:
                    skip.append(d)
                    break
        heading = _visit_branches(adj, node, heading, skip, priority, route)
        route.append((trunk_dir, nxt))
        heading = trunk_dir
        prev = node
    return route


# ---------------------------------------------------------------------
# 탐색 상태머신(순수) — 전역 위치인식 없이 분기 트리를 관리한다.
# ---------------------------------------------------------------------

class Explorer(object):
    """전 분기 방문(우선순위 L>R>S, 보류는 LIFO) 후 부모 체인으로 복귀.

    mode: TO_FIRST(첫 분기 전) / PROBE(미탐색 팔 진입 중) /
          RETURN_TO_WORK(팔 끝에서 유턴 복귀 중) / GOTO_PENDING(보류 자식으로 이동) /
          BACKTRACK(부모로 이동) / HOME(복귀 계획 소비 중)
    판단 결과는 (move, events) — move 는 "L"/"R"/"S"/"U", events 는 로그용.

    final_run3: HOME 은 build_return_route 의 스텝 리스트(route)를 소비한다.
    route[route_pos] = 지금 주행 중인 스텝. 도착(분기/마커/막다른길)마다
    on_arrive_home 이 스텝을 소비하고 다음 move 를 준다.
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
        self.arm_ends = {}      # (노드id, 방위) -> "red"|"green"|"yellow"|"dead_end"
        self.route = []         # HOME: [(절대방향, 도착노드ID), ...]
        self.route_pos = 0
        self.route_mark = {}    # 리프ID -> 종류(explorer_to_graph 의 mark)
        self.home_fallback = False
        self.home_red_total = 0

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
        # 초록을 못 만난 채 지도 완성 — 루트에서 전노드 재방문 복귀 계획.
        self.mode = "HOME"
        events.append(("EXPLORE_DONE", "MAP_COMPLETE",
                       {"nodes": len(self.nodes)}))
        events += self._plan_home(self.work)
        if self.route:
            move = rel_move(self.heading, self.route[0][0])
            events.append(("RETURN_STEP", "PLAN_FIRST",
                           {"move": move, "to": str(self.route[0][1]),
                            "route_left": len(self.route)}))
            return move, events
        self.home_fallback = True
        events.append(("RETURN_FALLBACK", "EMPTY_ROUTE", {}))
        return "U", events

    def on_probe_end(self, kind):
        """PROBE 팔이 분기점 없이 끝났다(마커/막다른길). 팔을 DONE 처리.

        final_run3: 팔 끝 종류(kind)를 arm_ends 에 기록한다 — 복귀 계획의
        빨강 집계용. move 판단에는 영향이 없다(out 행동 동일).
        """
        if self.mode != "PROBE":
            return [("PROBE_END", "IGNORED_MODE_" + self.mode, {"kind": kind})]
        self.arm_ends[(self.work, self.probe_arm)] = kind
        self.nodes[self.work]["arms"][self.probe_arm] = "DONE"
        events = [("PROBE_END", "ARM_DONE",
                   {"work": self.work, "arm": self.probe_arm, "kind": kind})]
        self.probe_arm = None
        self.mode = "RETURN_TO_WORK"
        return events

    def _plan_home(self, start):
        """지도 전체 → 복귀 계획 수립(HOME 전환 공통). RETURN_PLAN 이벤트 반환."""
        adj, mark = explorer_to_graph(self.nodes, self.arm_ends)
        self.route = build_return_route(adj, mark, start, "home",
                                        self.heading, self.PRIORITY)
        self.route_pos = 0
        self.route_mark = mark
        self.home_fallback = False
        self.home_red_total = len([n for n in mark if mark[n] == "red"])
        trunk = shortest_path(adj, start, "home")
        edges = 0
        for n in adj:
            edges += len(adj[n])
        return [("RETURN_PLAN", "MIN_DIST_REVISIT_ALL",
                 {"steps": len(self.route), "edges": edges // 2,
                  "trunk": (len(trunk) - 1) if trunk else 0,
                  "reds": self.home_red_total,
                  "start": str(start), "heading": self.heading})]

    def start_home(self):
        """초록 배달+유턴 직후 호출 — 탐색을 중단하고 복귀 계획을 세운다.

        정상 케이스는 PROBE 팔 끝의 초록: 팔을 green 으로 닫고 그 리프가
        복귀 시작점(로봇은 유턴을 마쳐 heading == route[0] 방향). 첫 분기
        전(TO_FIRST) 초록이면 지도가 없다 — 빈 계획(온 길 직진 = 노랑)으로
        전환. 그 밖의 모드(지도상 위치 불명)는 즉석 탐색 폴백으로 복귀한다.
        """
        prior = self.mode
        self.mode = "HOME"
        if prior == "PROBE":
            self.arm_ends[(self.work, self.probe_arm)] = "green"
            self.nodes[self.work]["arms"][self.probe_arm] = "DONE"
            start = "leaf:{}:{}".format(self.work, self.probe_arm)
            self.probe_arm = None
            events = [("EXPLORE_STOP", "GREEN_DELIVERED",
                       {"nodes": len(self.nodes), "start": start})]
            return events + self._plan_home(start)
        self.route = []
        self.route_pos = 0
        self.route_mark = {}
        self.home_red_total = len([k for k in self.arm_ends
                                   if self.arm_ends[k] == "red"])
        if not self.nodes:
            # 분기를 하나도 못 만난 초록 — 되짚으면 바로 노랑이다.
            self.home_fallback = False
            return [("RETURN_PLAN", "NO_MAP_STRAIGHT_BACK", {"steps": 0})]
        self.home_fallback = True
        return [("RETURN_FALLBACK", "GREEN_IN_" + prior,
                 {"nodes": len(self.nodes)})]

    def on_arrive_home(self, kind, has_left=False, has_right=False,
                       has_straight=False):
        """복귀 중 도착(junction/red/green/yellow/dead_end) 처리 — (move, events).

        kind 가 계획 스텝의 도착 노드 종류와 일치하면 스텝을 소비하고 다음
        스텝의 절대방향을 상대 move 로 변환해 반환한다. 불일치·move 불가·계획
        소진이면 RETURN_FALLBACK 후 즉석 탐색(좌>우>직) — 절대 멈추지 않는다.
        마커/막다른길 도착은 avail 기본값(U만 가능)으로 부르면 된다.
        """
        events = []
        avail = {"L": has_left, "R": has_right, "S": has_straight, "U": True}
        if not self.home_fallback:
            if self.route_pos < len(self.route):
                dest = self.route[self.route_pos][1]
                exp_kind = self.route_mark.get(dest, "junction")
                if exp_kind != kind:
                    events.append(("RETURN_FALLBACK", "ARRIVAL_MISMATCH",
                                   {"expected": str(dest),
                                    "expected_kind": exp_kind, "got_kind": kind,
                                    "route_left":
                                        len(self.route) - self.route_pos}))
                    self.home_fallback = True
                else:
                    self.route_pos += 1
                    if self.route_pos >= len(self.route):
                        # 마지막 스텝 도착지는 home(노랑)뿐이고 노랑은
                        # handle_marker 가 끝낸다 — 여기 왔다면 계획 붕괴.
                        events.append(("RETURN_FALLBACK", "ROUTE_EXHAUSTED",
                                       {"at": str(dest)}))
                        self.home_fallback = True
                    else:
                        nxt_dir = self.route[self.route_pos][0]
                        move = rel_move(self.heading, nxt_dir)
                        if avail[move]:
                            events.append(("RETURN_STEP", "PLAN",
                                           {"at": str(dest), "kind": kind,
                                            "move": move,
                                            "route_left": len(self.route) -
                                                          self.route_pos}))
                            return move, events
                        events.append(("RETURN_FALLBACK", "MOVE_UNAVAILABLE",
                                       {"at": str(dest), "move": move}))
                        self.home_fallback = True
            else:
                events.append(("RETURN_FALLBACK", "PLAN_EMPTY", {}))
                self.home_fallback = True
        for rel in self.PRIORITY:
            if avail[rel]:
                events.append(("RETURN_STEP", "FALLBACK", {"move": rel}))
                return rel, events
        return "U", events

    def route_left(self):
        return len(self.route) - self.route_pos


# ---------------------------------------------------------------------
# PID 조향(순수) — §변경 2, 3
# ---------------------------------------------------------------------

class PidSteer(object):
    """error = 정규화 우반사광 - 정규화 좌반사광.

    P: error 를 그대로 kp 배 — soft-deadband 제거(라이브 튜닝화 커밋). 중앙
       근처 미세 헌팅은 kp/kd 라이브 조정으로 잡는다.
    I: ki > 0 이고 |error| <= INTEG_BAND 일 때만 누적(커브/노드 진입 windup
       방지), 기여는 ±INTEG_TURN_LIMIT 클램프. 계통 편향(모터 개체차)을 학습해
       정상상태 치우침을 없애는 것이 목적이라 시정수는 느려도 된다. 기본
       ki=0 이면 적분은 완전히 꺼진다(코드/가드는 그대로 살아 있음).
    D: kd 라이브 파라미터 + raw derivative clamp + EMA.
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
        self.home_revisit = 0       # 복귀 중 다시 지나간 빨강 수
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
        self.guard_since = None     # 가로선 접근 가드 시작 시각(§변경 F)

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
            "home_total": self.ex.home_red_total,
            "home_revisit": self.home_revisit,
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

    def reset_steer(self):
        self.pid.reset()
        self.last_turn = 0.0
        self.lost_since = None      # 모션 프리미티브 후 stale 유실 타이머 방지(v13)
        self.guard_since = None     # stale 가드 타이머도 함께 리셋(§변경 F)

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
        """제자리 회전(L/R/U) + heading 갱신 + PID 리셋. 유턴은 낮은 tone 2번 선행.
        회전각/속도를 바꾸려면 여기(와 params 의 factor)만 보면 된다.

        스무스 턴(§변경 D, v6): reset_encoders() 전에 coast() 로 직전 brake-hold 를
        풀어 엔코더 리셋 위치보정 킥을 막고, 회전 동안만 set_ramp(turn_ramp_ms)로
        가속을 램프해 속도PID 콜드스타트/백래시 킥을 없앤다. finally 에서 ramp 0
        복원(라인추종 조향엔 영향 없음, 감속=0 이라 target 에서 크리스프 정지)."""
        # 노드/마커 처리를 마치면 새 구간 — 복구 카운트 이월 방지(v13.1).
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
        self.hw.coast()                 # 직전 brake-hold 해제 → 리셋 킥 방지(v6)
        self.hw.reset_encoders()
        self.hw.set_ramp(snap["turn_ramp_ms"])   # 가속만 램프(감속=0), v6
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
            self.hw.set_ramp(0)         # 라인추종 조향엔 램프 미적용(v6)
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
            if not on_line(color):
                # 흰 바닥일 때만 재획득 — 경계 오분류(BROWN/BLUE)나 마커 위에서
                # 쓸데없이 소각 스캔하지 않는다(마커색도 라인 위다).
                self.log.log("TURN_ACQUIRE", "CENTER_OFF_LINE_AFTER_TURN",
                             move=move, color=color)
                self.realign_to_line(self.params.snapshot())

    # ---- 마커 / 노드 처리 ----

    def _confirm_marker_color(self, first):
        """마커 색 확정(§변경 B): 정지 상태에서 여러 번 재판독해 다수결. 가장
        많이 나온 색이 마커색이고 MARKER_CONFIRM_MIN 이상이면 그 색, 아니면 None.
        코너/흑백 경계에서 1프레임 튄 가짜 색이 배달/완주를 트리거하는 것을 막는다.
        (호출부가 이미 정지시킨 뒤 부른다 — 정지 상태라 센서가 안정적이다.)"""
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
        """빨강/노랑/초록 마커 처리(정지 → 색 확정 → 미션 분기). 처리했으면 True.

        노랑@복귀 = 완주(그립 해제) / 초록@탐색 = 드롭·재파지 후 유턴·복귀계획 /
        복귀 중 마커 = 계획 스텝 소비 / 그 외(탐색 중 빨강·노랑) = 유턴."""
        if color not in MARKER_COLORS:
            return False
        if (time.monotonic() - self.last_marker_t) * 1000 < MARKER_DEBOUNCE_MS:
            return False

        self.hw.stop()
        time.sleep(MARKER_PAUSE_S)
        color = self._confirm_marker_color(color)
        if color is None:
            # 1프레임 튄 가짜 마커 — 아무 것도 하지 않고 주행 재개(caller 가 재판독).
            self.log.log("MARKER_REJECT", "UNCONFIRMED", context=context,
                         ex_mode=self.ex.mode)
            return False
        name = MARKER_NAMES[color]
        self.log.log("MARKER", "COLOR_{}_CONFIRMED".format(name.upper()),
                     color=color, context=context, ex_mode=self.ex.mode,
                     session=self.session)

        if color == COL_YELLOW and self.ex.mode == "HOME":
            # 완주: 물체를 노란점에 내려놓고(그립 해제) 종료.
            self._release_at_home()
            self.done = True
            route_left = self.ex.route_left()
            missed = max(0, self.ex.home_red_total - self.home_revisit)
            on_plan = (not self.ex.home_fallback and route_left <= 1
                       and missed == 0)
            self.log.log("HOME_REACHED",
                         "COLOR_YELLOW" if on_plan
                         else "EARLY_OR_FALLBACK_MISSED_REDS",
                         home_revisit=self.home_revisit,
                         home_total=self.ex.home_red_total,
                         missed=missed, route_left=route_left,
                         fallback=self.ex.home_fallback)
            self.last_marker_t = time.monotonic()
            return True

        if self.ex.mode == "HOME":
            # 복귀 중 마커 리프(빨강 재방문/초록 경유 등) — 유턴은 계획에 포함.
            if color == COL_RED:
                self.visits += 1
                self.home_revisit += 1
                self.log.log("REVISIT_HOME", "COLOR_RED_ON_RETURN",
                             revisit=self.home_revisit,
                             total=self.ex.home_red_total)
            move, events = self.ex.on_arrive_home(name)
            self.log_events(events)
            self.turn(move)
            self.last_marker_t = time.monotonic()
            return True

        if color == COL_GREEN:
            # 초록 도착(목적지): 물체 내려놓기(그립 오픈) → 유턴 → 복귀 계획.
            # 재파지는 복귀 중 물체를 만나면 자동으로 한다(가는 길과 동일).
            self.deliver()
            if self.interrupted():
                return True
            self.turn("U")
            if self.interrupted():
                return True
            self.log_events(self.ex.start_home())
            self.last_marker_t = time.monotonic()
            return True

        # 탐색 중 빨강/노랑 = 막다른 표식 → 유턴(팔 DONE 처리).
        if color == COL_RED:
            self.visits += 1
        self.turn("U")
        if self.ex.mode == "PROBE":
            self.log_events(self.ex.on_probe_end(name))
        self.last_marker_t = time.monotonic()
        return True

    def deliver(self):
        """초록점: 운반해 온 물체를 내려놓는다 — 전진→그립 오픈→후진.
        (그립을 열면 grabbed=False → 복귀 중 물체를 다시 만나면 자동 재파지.)"""
        snap = self.params.snapshot()
        self.goal_seen = True
        self.log.log("GOAL_DROP", "COLOR_GREEN",
                     goal_advance_mm=snap["goal_advance_mm"])
        # 전진해 초록선을 넘어 물체를 내려놓고(그립 오픈) 다시 초록으로 후진.
        self.straight(snap["goal_advance_mm"], STRAIGHT_SPEED)
        if self.interrupted():
            return
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
        self.grabbed = False
        self.straight(snap["goal_advance_mm"], -STRAIGHT_SPEED)

    def _release_at_home(self):
        """노란점 완주: 물체를 내려놓는다(그립 해제)."""
        snap = self.params.snapshot()
        if self.grabbed:
            self.hw.grip_open(snap["grip_speed"], GRIP_SEC)
            self.grabbed = False
            self.log.log("DROP_HOME", "COLOR_YELLOW")

    def confirm_node(self, first_bits, snap):
        """의심지점: PID off → 저속 직진(감지점+node_confirm_mm) → 정지 후 재판정.
        (확정 bits 또는 None, 그동안 전진한 mm) 반환. 마커를 만나면 처리 후 None.
        종료는 거리 결정형(이동거리 >= node_confirm_mm) — 진입 감속 거리가 타이머
        창에 섞여 creep_mm 이 산포하던 시간 기반을 대체한다. 스톨/걸림 대비로
        CONFIRM_TIMEOUT_S 시간 상한을 함께 두고, 상한 도달 시에도 동일하게 정지
        후 재판정으로 진행하되 로그에 timeout=True 를 남긴다.
        000 은 여기로 들어오지 않는다 — creep/재판정 중 000 이 보이면 취소(v13)."""
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
                         creep_mm=round(creep_mm, 1), timeout=timeout)
            return bits, creep_mm
        if bits == LOST_BITS and first_bits[0] == 1 and first_bits[2] == 1:
            # 정지 재판정에서 전백 — 위 creep 케이스와 같은 passed-over(v13.1).
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
        has_straight = on_line(color)
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
            self.turn(move)
            return

        if self.ex.mode == "HOME":
            move, events = self.ex.on_arrive_home("junction", has_left,
                                                  has_right, has_straight)
        else:
            move, events = self.ex.on_junction(has_left, has_right, has_straight)
        self.log_events(events)
        self.turn(move)

    # ---- 유실(000) 처리 — v13 ----

    def _dead_end_uturn(self):
        """막다른길 확정 공통 처리: 유턴 + PROBE 팔이면 DONE.

        HOME 이면 계획 스텝을 소비해 동기화한다(계획상 리프 도착 = 유턴).
        예상 밖 위치(유실 오판 등)면 on_arrive_home 이 폴백으로 넘긴다.
        """
        if self.ex.mode == "HOME":
            move, events = self.ex.on_arrive_home("dead_end")
            self.log_events(events)
            self.turn(move)
            return
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
        """복구 직후 재정렬 — 중앙 컬러가 라인(black) 위에 오도록 좌/우 소각 스캔.

        어두운 쪽부터 스캔하고, 양쪽 다 실패하면 원래 방향으로 복원 후
        False(호출부가 막다른길 처리). 소각이라 heading 격자는 안 바꾼다."""
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
        """노란점에 놓일 때까지 대기 → 그리퍼 오픈(가는 길에 물체 받을 준비) →
        출발선 이탈 전진. status('go'|'stop'|'reset'). 물체는 여기서 잡지 않고
        가는 길에 초음파로 만나면 잡는다(explore 루프).

        대기 중에도 handle_pending 이 돌므로 이 상태에서 calibrate([1] 키)를
        먼저 실행하면 된다(로봇을 라인 위에 두고 실행)."""
        snap = self.params.snapshot()
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)   # 가는 길에 물체 받을 준비
        while self.hw.read_center_color_now() != COL_YELLOW:
            if self.stop_on:
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            self.publish("waiting_start")
            time.sleep(0.05)
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

            # 가는/오는 길 중 물체를 만나면(초음파 grab_dist 이내) 잡는다.
            # 그립이 비어 있을 때만(이미 잡았으면 무시). 미로 어디서든 동작.
            if (not self.grabbed and
                    self.hw.read_distance_cm() < snap["grab_dist_cm"]):
                self.hw.stop()
                self.hw.grip_close(snap["grip_speed"], GRIP_SEC)
                self.grabbed = True
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

            # §변경 F — 교차로 접근 가드: 한쪽 반사광이 노드 임계 근처
            # (임계+NODE_GUARD_MARGIN 미만)로 떨어지면 가로선을 스치기 시작한
            # 것이다. bits 확정(35/30) 전의 이 하강 구간에서 PID 를 돌리면
            # 좌우 차이+D 스파이크로 한계 조향이 나와 차체가 틀어진 채
            # 교차로에 진입하므로(대각선 진입/위빙), 값 회복 또는 confirm_node
            # 진입까지 감속 직진으로 자세를 유지한다. 일반주행 하위 1% 가
            # 51~58(12:01 실측)이라 47/42 아래는 사실상 가로선뿐이다.
            # MAX_MS 초과 지속이면 가로선이 아니라 정렬 불량(센서가 세로선
            # 위)이므로 PID 에 되돌린다. confirm 취소 후 디바운스 150ms 동안
            # 센서가 가로선 위에 남는 프레임도 이 가드가 직진으로 붙든다.
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
                                 arrived=self.goal_seen, nodes=len(self.ex.nodes),
                                 route_left=self.ex.route_left(),
                                 lost_streak=self.lost_streak)
                    time.sleep(LOOP_DELAY_S)
                    continue
            else:
                self.guard_since = None

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
                         route_left=self.ex.route_left(),
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

    print("final_run8 ready (SILENT — sound removed). 1) dashboard 'calibrate' on "
          "the line, 2) place robot on YELLOW to start. it grips the object when it "
          "meets one on the way to GREEN. dashboard 'reset' ([r]) restarts. "
          "(Ctrl-C or robotctl stop to quit)")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        try:
            hw.stop()
        finally:
            server.stop()
    print("final_run8 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
