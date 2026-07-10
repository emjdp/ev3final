#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""final_run11 — final_run10 + 스무스 모션 전면 적용(틱틱 튐 제거).

final_run10 대비 변경 — 판단층(Explorer/build_return_route/PidSteer/node_bits/
classify_rgb)은 한 줄도 건드리지 않았다. 구동 계층만 바뀐다.

  F) run6 의 스무스 턴 처리를 turn() 밖으로 꺼내 모든 모션 프리미티브에 적용.

     run6 은 회전 시작 '틱틱' 튐의 원인 둘을 잡았다:
       (1) brake-hold(off(brake=True)) 상태에서 reset_encoders() 로 position 을
           0 으로 쓰면 위치 서보가 그만큼 되돌리려는 킥을 낸다 → coast() 로
           hold 를 먼저 푼다(정지 상태라 바퀴가 구르지는 않는다).
       (2) 정지 상태에서 속도 명령을 주면 속도 PID 가 콜드스타트하며 튄다 →
           set_ramp(가속 램프)로 완화하고 끝나면 0 으로 복원한다.
     그런데 그 처리가 turn() 안에만 있었다. run10 은 회전/노드 직진통과/유실
     복구/출발 직후마다 acquire_edge 가 소각 피벗(_pivot_scan/_pivot_to_edge)을
     도는데, 이들은 hw.stop()(brake) 직후 reset_encoders() 를 그대로 불러 (1)의
     킥을 매번 냈다 — "매끄럽게 돌고 나서 바로 틱틱거린다"의 정체. run8 에도
     같은 코드가 있었지만 _pivot_scan 이 유실 복구에서만 드물게 돌아 안 보였다.

     _begin_motion(ramp_ms)/_end_motion() 헬퍼로 묶어 피벗(_pivot_scan,
     _pivot_to_edge)·캘리브레이션 스윕(_cal_sweep)·직진(straight)·후진
     (backup_to_line)·회전(turn) 전부에 적용한다.

     예외 하나 — confirm_node 의 creep 은 coast 만 걸고 램프는 걸지 않는다.
     다른 프리미티브는 엔코더/조건 기준이라 램프가 최종 위치를 못 바꾸지만,
     creep 은 node_confirm_ms(40ms) '시간' 기준이라 램프를 걸면 그 안에 속도에
     못 올라 실측 튜닝된 creep 거리(handle_node 가 node_advance_mm 에서 빼는
     값)가 달라진다.

     set_ramp 단위 주의: ramp_up_sp 는 0→최대속도(100%) 가속 시간이므로
     실효 램프 = ramp_ms × (명령속도/100). 피벗은 REALIGN_SPEED=6 으로 느려
     turn_ramp_ms(250)를 그대로 쓰면 실효 15ms 뿐이다. 그래서 피벗/직진용
     램프 상수를 따로 둔다(PIVOT_RAMP_MS/STRAIGHT_RAMP_MS).

  G) 라인추종 정밀화 + 노드 판정과의 분리(사용자: "살짝 선 물고 간다").
     세 값만 바꾼다 — 판단 코드(node_bits/NODE_CANDIDATES/confirm_node/
     handle_node/handle_lost/turn/backup_to_line)는 한 줄도 안 건드린다.

       edge_target    50 → 58   경계보다 살짝 흰쪽에 앉아 라인을 덜 문다.
       deadband      2.0 → 1.0  더 작은 오차에도 조향(정밀).
       center_th_node 50 → 70   ← 이게 핵심.

     center_th_node 가 edge_target 과 같은 50 이었다는 게 진짜 결함이다.
     엣지 추종이 잘 될수록 norm_c 는 50 근처에 머무는데, 그게 정확히 중앙
     노드 bit 가 뒤집히는 지점이다. 좌/우가 흰색인 정상 주행에서 중앙 bit 가
     0 으로 넘어가는 순간 bits == (0,0,0) == LOST_BITS(유실 의심)이 된다.
     lost_persist_ms(200ms) 지속 필터가 겨우 막고 있었을 뿐이고, 추종을
     정밀하게 만들수록(=norm_c 가 50 에 더 오래 머물수록) 오판이 더 잘 난다.
     추종 밴드(edge_target±EDGE_ACQUIRE_TOL = 50~66) 전체가 '라인 보임(1)'이
     되도록 70 으로 올려 두 역할을 분리한다.

     부수 수정 — center_th_node 를 밴드 위로 올리면 norm_c 가 66~70 인 구간
     ('밴드보다 밝지만 라인은 보임')이 새로 생긴다. acquire_edge 는 밴드를
     어두운 쪽에서 접근한다고 가정하므로 거기서 흰쪽으로 피벗해 더 멀어진다.
     그 구간에선 라인 쪽으로 되돌아 피벗하도록 방향을 뒤집는다.

실행(브릭):   python3 stages/final_run11.py
문법 점검(PC): python3 -m py_compile stages/final_run11.py lib/*.py
단위 테스트:  python3 tests/test_final_run11_maze.py (ev3dev2 불필요)

===== 이하 final_run10 원문 헤더 =====

final_run10 — 중앙 엣지 팔로잉 + 회전 후 엣지 획득 (inchul_last_dance.md).

미션(inchul_last_dance.md 프로젝트 상세):
  1) 브릭 가운데 버튼으로 시작. 시작과 동시에 난수(1~4)를 LCD 에 크게 표시하고
     숫자음(one~four)을 재생 — 사람이 그 위치에 통을 놓는다.
  2) 초록점까지 미로 탐색(좌>우>직, 분기 정리형 — return_revisit_all_minimal_route.html
     과 동일 논리). 이 미로에서는 초록 도달 전에 빨강 6개를 모두 방문하게 된다
     (tests/test_final_run10_maze.py 로 PC 검증).
  3) 빨강 스티커 확정마다 "red one", "red two", ... 오디오(가는길·복귀 누적 카운트).
  4) 이동 중 통을 초음파로 감지하면 경보음(삐) 후 파지해 운반.
  5) 초록점: 통 내려놓기 → "good job" + 소요시간 LCD 표시 → 1초 대기 → 180도 회전
     → 두 번째 난수(1~4) 표시+숫자음(사람이 두 번째 통 배치) → 복귀 시작.
  6) 복귀는 지도 기반 최소거리 전빨강 재방문(초록 재방문 없음). 오는 길에 두 번째
     통을 만나면 경보음 후 파지.
  7) 노란점(출발지) 복귀: 통 내려놓기 → "good job" + 소요시간 LCD → 완주.

final_run9 대비 변경(§run10 — 엣지 획득, 사용자 요구):
  F) 회전/노드 직진 통과/유실 복구/출발 직후에 acquire_edge 를 실행한다.
     중앙 엣지 팔로잉은 중앙 센서가 steer_sign 이 타기로 한 쪽 엣지에 있어야
     수렴한다 — 흰 바닥인데 라인이 반대쪽이면 조향이 라인에서 멀어지고,
     라인 중앙(포화 검정)이면 큰 에러로 시작해 과도 스윙이 난다. 피벗 오차
     ·노드 전진 뒤가 바로 그 상태라, run10 은 PID 재개 전에 정지 상태에서:
       1) 이미 엣지 밴드(edge_target±EDGE_ACQUIRE_TOL)면 그대로,
       2) 흰색이면 realign_to_line(좌/우 소각 스캔)으로 검정을 먼저 찾고,
       3) 검정에서 edge_exit_dir(steer_sign) 쪽으로 소각 피벗해 엣지 밴드에서
          정지한다. 항상 같은 쪽으로 나가므로 회전 후에도 타는 엣지가 유지된다.
     목적: 매 회전 직후 동체가 라인과 평행하게 서서 곧게 출발(사용자 요구 —
     "회전을 했을 때도 중앙센서가 엣지를 찾아갈 수 있어야 한다").
  G) 오디오 보류(사용자 확인 — 브릭 espeak 미동작): AUDIO_ENABLED=False 로
     espeak 생성/wav·tone 재생/통 감지 경보음을 전부 끈다. LCD 표시(숫자/
     소요시간)와 미션 로직(red_say_count 등 카운트)은 그대로 — 오디오만
     보류. 나중에 켤 땐 AUDIO_ENABLED 한 줄만 바꾼다.

final_run8 대비 변경(run9 에서 승계):
  A) 센서 구성 변경(사용자 요구: 3개 다 반사광 + 색판정은 색상센서):
     - 좌(in1)/우(in3): 반사광 모드 그대로.
     - 중앙(in2): 상시 컬러 모드 → 상시 RGB-RAW 모드. 매 루프 (r,g,b) 1회 읽어
       * 밝기(반사광 등가) = 100*(r+g+b)/rgb_sum_white → PID·노드 bits·재정렬,
       * 색판정 = classify_rgb(비율 기반, 순수 함수) → 빨강/초록/노랑 마커.
       한 번 읽기로 두 용도를 동시에 쓰므로 모드 전환이 없고(병렬 요구 충족),
       마커 확정은 run8 과 같은 정지 후 다수결(_confirm_marker_color)을 유지한다.
  B) 조향을 중앙 센서 엣지 팔로잉으로 교체(사용자 생각 1 — 좌/우 반사광 차이
     폐기): error = steer_sign*(norm_c - edge_target). 중앙 센서를 라인 경계
     위에 놓고 중앙 밝기 하나로만 PID 조향한다. 좌/우 반사광은 조향에 안 쓰고
     노드 판별 전담(생각 2·3과 분리). steer_sign(±1)은 어느 엣지를 타는지에
     따른 조향 방향 부호로, 폭주 시 실기에서 이 값 하나만 뒤집는다.
  C) 복귀 실행 버그 수정(HTML 미로 픽스처에서 발견): 계획 스텝의 절대방향은 '그
     노드 로컬' 라벨인데 run8 은 커브로 갱신된 실주행 heading 과 rel_move 해 굽은
     복도(J4→J5→R2 같은)에서 잘못된 회전을 냈다. on_arrive_home 이 계획 그래프
     (route_adj)의 노드-로컬 라벨만으로 상대 회전을 계산하도록 수정 — 리프에선
     자연히 U 가 나온다. 기존 직선 픽스처 결과는 동일(테스트로 확인).
  D) 소리 재도입: espeak 로 시작 시 wav 를 1회 생성(캐시)해 aplay 큐로 비블로킹
     재생. espeak/wav 실패 시 tone 폴백(run7 방식). 숫자음/red N/good job.
  E) 시작 트리거: 노랑 감지 대기 → 브릭 가운데 버튼(미션 명세 1). LCD 에
     OUT/BACK 소요시간 + 난수 표시(show_final4_display 재사용).

규약: Python 3.5(f-string 금지) / ev3dev2 는 run() 안에서만 import /
      정지는 네트워크 stop 또는 Ctrl-C, 재시작은 네트워크 reset.

실행(브릭):   python3 stages/final_run10.py
문법 점검(PC): python3 -m py_compile stages/final_run10.py lib/*.py
단위 테스트:  python3 tests/test_final_run10_maze.py (ev3dev2 불필요)

===== 복귀 계획 알고리즘(final_run3에서 승계) =====

복귀 사양(채점 기준): 복귀 중에도 모든 빨강 마커를 다시 지나가야 한다.
단, 총 이동거리는 최소. 초록 배달 시퀀스(전진→그리퍼 오픈→후진→유턴)가 끝나는
즉시 탐색을 중단하고 그 시점까지 구축된 지도로 복귀 전체 계획을 세운다.

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
  - 분기/빨강/초록/막다른길 '도착'마다 계획 스텝을 하나 소비하고, 도착 노드의
    로컬 라벨(직전 노드로 되돌아가는 방향의 반대 = 도착 시 바라보는 방향)과
    다음 스텝 라벨로 상대 move 를 계산해 회전(§변경 C — 리프에선 자연히 유턴).
    커브(n_exits==1)는 노드가 아니므로 스텝을 소비하지 않는다 — 간선에
    커브가 몇 개든 계획과 어긋나지 않는다.
  - 빨강 재방문: "red N" 재생 + home_revisit 카운트 + REVISIT_HOME 로그 후
    계획 계속. 초음파 파지는 복귀 중에도 동작한다(두 번째 통 — 명세 7·8).
  - 도착 종류/가능한 move 가 계획과 어긋나면 RETURN_FALLBACK 로그 후 즉석
    탐색(좌>우>직)으로 전환 — 절대 그 자리에 멈추지 않는다. 노랑을 계획보다
    일찍 만나면 그대로 종료하되 못 본 빨강 수를 경고 로그로 남긴다.
  - 텔레메트리: home_total(빨강 총수)/home_revisit 를 매 프레임 publish.
    새 이벤트 RETURN_PLAN / RETURN_STEP / REVISIT_HOME / RETURN_FALLBACK,
    HOME_REACHED 에 revisit/missed 필드 추가.

초록을 끝내 못 만나 지도가 완성되면(EXPLORE_DONE) 루트에서 같은 계획
함수를 호출한다(트렁크가 루트→home 1간선으로 퇴화할 뿐 동일 알고리즘).

===== PID / 캘리브레이션(final_run1~2에서 승계, 센서만 3반사광으로) =====

  1) 센서별 캘리브레이션 + 정규화 — 대시보드 calibrate 액션이 라인 위에서
     좌→우 소각 스윕하며 좌/우 반사광과 중앙 RGB 밝기 각각의 black/white
     실측 min/max 를 기록한다(cal_* 라이브 파라미터, 성공 시 자동 save).
     norm = 100*(raw-black)/(white-black). 미보정 기본값(0/100)은 항등.
  2) PID — 작은 적분항 ki 가 모터 개체차 같은 계통 편향을 학습해 정상상태
     치우침을 제거. windup 가드 3중(INTEG_BAND 밖 동결 / 기여 클램프 /
     reset 때 절반 유지).
  3) P soft-deadband — |e| <= deadband 는 P 항 0(연속형), 직진 유지는 적분
     트림 담당.

사용법: 라인 위(직선 구간)에 로봇을 세우고 대시보드 calibrate 실행 → 이후
kp/ki/deadband 라이브 튜닝. read_rgb 액션으로 마커 위 RGB 를 실측해
rgb_* 판정 파라미터를 맞춘다(빨강/초록/노랑 스티커 각각 위에서 실행).

나머지 동작(노드/마커/유실/탐색/세션)은 final_run8 과 동일:
  - 노드/분기 의심 bits 는 PID off → 저속 직진 confirm → 정지 재판정.
    000 은 노드 후보가 아니라 지속 필터를 거치는 유실 의심.
  - 유실 확정 시 후진 → 재정렬(REALIGN) 복구, 연속 한도 초과면 유턴.
  - 세션 루프: 시작 대기(버튼) → 탐색 → 복귀 → 완주 후 대기. reset 은
    언제든 시작 대기로.
  * 노드 bits / 유실 복구 임계값(left/right_th_*)은 좌/우 '원시값' 기준
    유지(실기 튜닝값). 중앙 노드 bit 만 RGB 밝기 정규화값 기준(center_th_node).
"""

import os
import random
import subprocess
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
# 스무스 모션 램프(§run11). set_ramp 는 0→100% 가속 시간이라 실효 램프는
# ramp_ms×(명령속도/100) — 느린 피벗(6%)일수록 큰 값이 필요하다.
PIVOT_RAMP_MS = 400         # 소각 피벗(재정렬/엣지획득/캘리브레이션): 실효 약 24ms
STRAIGHT_RAMP_MS = 300      # 엔코더 직진/후진: 실효 약 21~45ms
# 엣지 획득(§run10) — 회전/노드 통과 직후 중앙을 라인 엣지 위로 올린다.
EDGE_ACQUIRE_MAX_DEG = 60   # 검정→엣지 탈출 소각 피벗 최대 enc deg
EDGE_ACQUIRE_TOL = 8.0      # |norm_c - edge_target| 이내면 엣지 위로 판정
NODE_DEBOUNCE_MS = 900      # 노드 '확정' 후 재감지 최소 간격
NODE_CANCEL_DEBOUNCE_MS = 150  # confirm '취소' 후 간격(v13.1)
MARKER_DEBOUNCE_MS = 1500   # 같은 마커 재감지 방지
MARKER_PAUSE_S = 0.08       # 마커 정지 후 잠깐 멈춤
CONFIRM_SETTLE_S = 0.08     # 재판정 전 정지 안정화
GRIP_SEC = 0.8              # 그리퍼 열기/닫기 구동 시간
LOOP_DELAY_S = 0.015
FOLLOW_LOG_S = 0.25         # LINE_FOLLOW 로그 최소 간격
COLOR_MODE_SETTLE_S = 0.01  # 시작 시 컬러 모드 전환 settle(정리.md)

# 마커 확정(§변경 B) — 색은 1프레임이 아니라 정지 후 재판독 다수결로 확정한다.
# 코너/흑백 경계에서 튄 가짜 초록이 배달을 트리거하던 final_run3 버그 방지.
MARKER_CONFIRM_SAMPLES = 4  # 정지 후 재판독 횟수
MARKER_CONFIRM_MIN = 3      # 같은 마커 색이 이만큼 이상이어야 확정(첫 판독 포함 아님)
MARKER_CONFIRM_GAP_S = 0.015

# PID — KD 는 notes 대로 고정, derivative 는 clamp + EMA 로 완화.
PID_KD = 0.05
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

# 오디오(§변경 D) — espeak 로 시작 시 1회 생성(캐시)하는 wav 문구들.
# 재생은 Ev3Hardware 의 백그라운드 큐(aplay)라 주행을 막지 않는다.
# espeak/wav 가 없으면 tone 폴백(아래 TONE_*).
AUDIO_ENABLED = False       # 브릭에 espeak 미동작 확인(사용자) — 오디오 없이 우선 검증(§run10)
SOUND_DIR = os.path.join(_ROOT, "sounds")
NUMBER_WORDS = {1: "one", 2: "two", 3: "three", 4: "four"}
RED_WORDS = ("one", "two", "three", "four", "five", "six",
             "seven", "eight", "nine", "ten", "eleven", "twelve")
ESPEAK_ARGS = ("-a", "200", "-s", "120", "-v", "en-us")
TONE_GRAB_ALARM = ((1500, 300),)            # 통 감지 경보음(삐)
TONE_NUMBER_FALLBACK = 500                  # 500+80*n Hz, 250ms
TONE_GOOD_JOB = ((900, 120), (1200, 160), (900, 120))
TONE_RED_FALLBACK = ((700, 120),)           # red wav 없을 때 앞에 붙는 tone
GOAL_PAUSE_S = 1.0                          # 초록 배달 후 180도 회전 전 대기(명세 6)


# ---------------------------------------------------------------------
# 라이브 파라미터 — 대시보드/robotctl 로 실기에서 튜닝한다.
# ---------------------------------------------------------------------

# 파라미터당 한 줄: (이름, 초기값, min, max, max_step, ui_step, 단위)
# cal_* 4개는 calibrate 액션이 실측치를 직접 기록하므로 max_step=100(제한 없음).
# 기본값 black 0 / white 100 = 정규화 항등(원시값 그대로) → 미보정 시 v13 동작.
PARAM_TABLE = (
    ("base_speed",      10,    5,   45,   5,    1,    "%"),
    ("kp",              0.30,  0.0, 3.0,  0.1,  0.01, ""),   # 엣지 팔로잉 에러(±edge_target)
    ("ki",              0.06,  0.0, 0.5,  0.05, 0.01, ""),
    ("deadband",        1.0,   0.0, 20.0, 2.0,  0.5,  ""),   # 2.0→1.0(§run11 G): 더 작은 오차에도 조향
    # 엣지 팔로잉(§변경 B): 중앙 밝기 목표 경계값 + 조향 방향 부호.
    # 58 = 경계에서 흰쪽으로 살짝(§run11 G) — 50(검/백 중간)이면 센서가 라인
    # 가장자리를 계속 물어 동체 절반이 라인 위에 얹힌다("선 물고 감").
    ("edge_target",     58,    20,  80,   10,   5,    "%"),  # 경계보다 살짝 흰쪽
    ("steer_sign",      1,     -1,  1,    2,    2,    ""),    # 폭주하면 뒤집는다(±1)
    ("turn_speed",      10,    5,   40,   5,    1,    "%"),
    ("turn_ramp_ms",    250,   0,   600,  100,  50,   "ms"),  # 회전 가속 램프(시작 틱틱 튐 방지, v6)
    ("node_confirm_ms", 40,    0,   1000, 60,   10,   "ms"),
    ("left_th_steer",   66,    0,   100,  3,    1,    "%"),   # 유실 복구 검정 판정(원시값)
    ("right_th_steer",  63,    0,   100,  3,    1,    "%"),
    # 노드 bits 판정(원시값) — 12:01 편도성공 로그 실측: 가로선 위 8~10 / 일반주행 하위1% 51~58.
    # 18/14 는 '완전 검정'만 통과해 스치는 순간의 중간값(15~40)을 놓쳐 111 미인식.
    # 35/30 이면 반쯤 걸친 순간도 잡고, 일반주행 오검출은 confirm 재판정이 거른다.
    ("left_th_node",    35,    0,   100,  3,    1,    "%"),
    ("right_th_node",   30,    0,   100,  3,    1,    "%"),
    # 중앙 노드 bit: RGB 밝기 정규화값(norm_c)이 이 미만이면 검정(1).
    # 50→70(§run11 G): edge_target 과 같은 값이면 정상 엣지 추종 중 norm_c 가
    # 이 임계값 위아래로 진동해 중앙 bit 가 상시 깜빡이고, 좌/우가 흰색인
    # 정상 주행이 bits 000(=LOST_BITS, 유실 의심)으로 읽힌다. 추종 밴드
    # (edge_target±EDGE_ACQUIRE_TOL = 50~66) 전체가 '라인 보임(1)'이 되도록
    # 밴드 위로 올린다. 판정 로직/후보 패턴/임계값 의미는 그대로.
    ("center_th_node",  70,    0,   100,  5,    1,    "%"),
    ("cal_l_black",     0,     0,   100,  100,  1,    "%"),   # calibrate 가 기록
    ("cal_l_white",     100,   0,   100,  100,  1,    "%"),
    ("cal_r_black",     0,     0,   100,  100,  1,    "%"),
    ("cal_r_white",     100,   0,   100,  100,  1,    "%"),
    ("cal_c_black",     0,     0,   100,  100,  1,    "%"),   # 중앙 RGB 밝기 기준
    ("cal_c_white",     100,   0,   100,  100,  1,    "%"),
    # RGB 색판정(classify_rgb) — read_rgb 액션으로 실측 후 맞춘다.
    # rgb_sum_white: 흰 바닥 위 r+g+b 합(밝기 100% 기준). 브릭에서 실측 필요.
    ("rgb_sum_white",   600,   100, 1100, 200,  10,   ""),
    ("rgb_black_max",   25,    0,   100,  10,   1,    "%"),   # 밝기 이 미만 = 검정
    ("rgb_red_ratio",   1.8,   1.0, 4.0,  0.5,  0.1,  "x"),   # r > x*g, r > x*b
    ("rgb_green_ratio", 1.25,  1.0, 4.0,  0.5,  0.05, "x"),   # g > x*r, g > x*b
    # 노랑은 흰 바닥과 구분이 관건 — 흰 바닥도 b 채널이 약간 낮으므로(전형
    # r/b 1.5~1.8) 여유를 두고 2.5. read_rgb 실측으로 조정.
    ("rgb_yellow_ratio", 2.5,  1.0, 6.0,  0.5,  0.1,  "x"),   # r,g > x*b (빨강 아님)
    ("node_advance_mm", 60,    0,   120,  10,   10,   "mm"),  # 의심지점 기준 총 전진
    ("goal_advance_mm", 100,   0,   200,  10,   10,   "mm"),  # 배달 전·후진 거리
    ("turn_90_factor",  0.66,  0.3, 2.0,  0.05, 0.01, "x"),
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

SAVE_PATH = os.path.join(_ROOT, "config", "final_run11.json")
STAGE_NAME = "final_run11"

ACTIONS = [
    {"name": "calibrate", "label": "Calibrate L/C/R on line (sweep)"},
    {"name": "read_rgb", "label": "Read Center RGB (+classify)"},
    {"name": "read_reflect", "label": "Read L/R Reflect (raw+norm)"},
    {"name": "reset", "label": "Reset to Start (wait button)"},
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


def node_bits(reflect_l, norm_c, reflect_r, snap):
    """노드 판정 bits(좌,중,우) — 좌/우는 반사광 '원시값', 중앙은 RGB 밝기 정규화값.

    좌/우 임계값(left/right_th_node)은 원시값 기준 실기 튜닝값을 유지한다.
    중앙은 §변경 A: 컬러 black 여부 대신 밝기 정규화값 < center_th_node
    (아날로그라 반쯤 걸친 순간도 임계값으로 조절 가능).
    """
    return (1 if reflect_l < snap["left_th_node"] else 0,
            1 if norm_c < snap["center_th_node"] else 0,
            1 if reflect_r < snap["right_th_node"] else 0)


def classify_rgb(r, g, b, snap):
    """중앙센서 RGB-RAW → (색 코드, 밝기%) — 순수 함수(§변경 A).

    밝기% = 100*(r+g+b)/rgb_sum_white — 반사광 등가값(PID/노드 bits 용,
    캘리브레이션 정규화는 호출부가 cal_c_* 로 한다).
    색은 채널 지배 비율로 판정(개체차에 강하도록 절대값 대신 비율):
      검정: 밝기 < rgb_black_max
      빨강: r 가 g,b 둘 다 rgb_red_ratio 배 초과
      초록: g 가 r,b 둘 다 rgb_green_ratio 배 초과
      노랑: r,g 둘 다 b 의 rgb_yellow_ratio 배 초과(빨강/초록 아님)
      흰색: 밝기 >= 60 / 그 외: 없음(경계·회색)
    """
    total = float(r) + float(g) + float(b)
    bright = 100.0 * total / max(float(snap["rgb_sum_white"]), 1.0)
    if bright < snap["rgb_black_max"]:
        return COL_BLACK, bright
    rr = float(r)
    gg = float(g)
    bb = float(b)
    red_x = snap["rgb_red_ratio"]
    if rr > red_x * gg and rr > red_x * bb:
        return COL_RED, bright
    green_x = snap["rgb_green_ratio"]
    if gg > green_x * rr and gg > green_x * bb:
        return COL_GREEN, bright
    yellow_x = snap["rgb_yellow_ratio"]
    if rr > yellow_x * bb and gg > yellow_x * bb:
        return COL_YELLOW, bright
    if bright >= 60.0:
        return COL_WHITE, bright
    return COL_NONE, bright


def bits_str(bits):
    return "{}{}{}".format(bits[0], bits[1], bits[2])


def edge_exit_dir(steer_sign):
    """검정(라인 위)에서 어느 쪽으로 피벗해야 steer_sign 이 타는 엣지로 나가는가(§run10).

    steer_sign=+1: 밝으면(엣지 밖 흰색) 좌로 조향 → 라인이 센서 왼쪽에 있어야
    수렴 = 라인의 오른쪽 엣지를 탄다 → 검정에선 오른쪽("R")으로 나가야 그 엣지다.
    steer_sign=-1 은 반대("L"). 부호 규약이 여기 한 곳에 고정된다.
    """
    return "R" if steer_sign >= 0 else "L"


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
        self.route_adj = {}     # 계획 그래프(§변경 C — 노드 로컬 라벨 계산용)
        self.route_start = None  # 복귀 시작 노드(route[0] 직전 위치)
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
        self.route_adj = adj
        self.route_start = start
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
        self.route_adj = {}
        self.route_start = None
        self.home_red_total = len([k for k in self.arm_ends
                                   if self.arm_ends[k] == "red"])
        if not self.nodes:
            # 분기를 하나도 못 만난 초록 — 되짚으면 바로 노랑이다.
            self.home_fallback = False
            return [("RETURN_PLAN", "NO_MAP_STRAIGHT_BACK", {"steps": 0})]
        self.home_fallback = True
        return [("RETURN_FALLBACK", "GREEN_IN_" + prior,
                 {"nodes": len(self.nodes)})]

    def _arrival_facing(self, dest, prev):
        """계획 그래프 로컬 라벨로 '도착 시 바라보는 방향'을 계산(§변경 C).

        dest 에서 prev 로 돌아가는 간선 라벨의 반대 = 도착 heading. 계획의
        절대방향 라벨은 노드마다 로컬(커브로 굽은 복도에서 전역 격자와 다름)
        이므로, 실주행 heading 이 아니라 이 값과 rel_move 해야 맞는다.
        """
        arms = self.route_adj.get(dest, {})
        for d in arms:
            if arms[d] == prev:
                return turn_heading(d, "U")
        return None

    def on_arrive_home(self, kind, has_left=False, has_right=False,
                       has_straight=False):
        """복귀 중 도착(junction/red/green/yellow/dead_end) 처리 — (move, events).

        kind 가 계획 스텝의 도착 노드 종류와 일치하면 스텝을 소비하고, 도착
        노드의 로컬 라벨 기준(도착 heading vs 다음 스텝 방향)으로 상대 move 를
        계산해 반환한다(§변경 C — 리프에선 자연히 유턴). 불일치·move 불가·계획
        소진이면 RETURN_FALLBACK 후 즉석 탐색(좌>우>직) — 절대 멈추지 않는다.
        마커/막다른길 도착은 avail 기본값(U만 가능)으로 부르면 된다.
        """
        events = []
        avail = {"L": has_left, "R": has_right, "S": has_straight, "U": True}
        if not self.home_fallback:
            if self.route_pos < len(self.route):
                dest = self.route[self.route_pos][1]
                prev = (self.route[self.route_pos - 1][1]
                        if self.route_pos > 0 else self.route_start)
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
                        facing = self._arrival_facing(dest, prev)
                        nxt_dir = self.route[self.route_pos][0]
                        move = (rel_move(facing, nxt_dir)
                                if facing is not None else None)
                        if move is not None and avail[move]:
                            events.append(("RETURN_STEP", "PLAN",
                                           {"at": str(dest), "kind": kind,
                                            "move": move,
                                            "route_left": len(self.route) -
                                                          self.route_pos}))
                            return move, events
                        events.append(("RETURN_FALLBACK", "MOVE_UNAVAILABLE",
                                       {"at": str(dest), "move": str(move)}))
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
    """error = 중앙 센서 엣지 팔로잉(§변경 B): steer_sign*(norm_c - edge_target).

    중앙 반사광 하나로만 조향한다(사용자 생각 1 — 좌/우 반사광 차이 폐기).
    중앙 센서를 라인의 경계(검정↔흰색) 위에 놓고, 정규화 밝기 norm_c 가
    edge_target(경계 = 검/백 중간, 기본 50)에 머물도록 PID 로 몬다. 로봇이
    한쪽으로 쏠리면 중앙이 더 검게(norm_c↓) 또는 희게(norm_c↑) 읽혀 한
    센서만으로 방향 신호가 나온다. 좌/우 반사광은 조향에 쓰지 않고 노드
    판별 전담(생각 2). steer_sign(±1)은 좌/우 어느 엣지를 타는지에 따라
    조향 방향을 뒤집는 라이브 값 — 실기에서 폭주하면 이 값 하나만 뒤집는다.
    회전 후 엣지 획득(acquire_edge)도 같은 부호를 쓴다(edge_exit_dir) —
    부호를 뒤집으면 조향과 획득이 함께 반대 엣지로 일관되게 바뀐다(§run10).

    P: soft-deadband — |error| <= deadband 면 0, 넘으면 밴드 폭만큼 뺀 값
       (경계에서 불연속 점프가 없어 D 항을 자극하지 않는다).
    I: |error| <= INTEG_BAND 일 때만 누적(커브/노드 진입 windup 방지),
       기여는 ±INTEG_TURN_LIMIT 클램프. 계통 편향(모터 개체차)을 학습해
       정상상태 치우침을 없애는 것이 목적이라 시정수는 느려도 된다.
    D: v13 그대로 KD 고정 + raw derivative clamp + EMA.
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

    def step(self, norm_c, snap, base_speed):
        # 엣지 팔로잉: 중앙 밝기가 경계값(edge_target)을 벗어난 만큼이 에러.
        error = snap["steer_sign"] * (float(norm_c) - snap["edge_target"])

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
        turn = clamp(snap["kp"] * p_error + i_term + PID_KD * self.deriv,
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
        # 미션 상태(§변경 D, E)
        self.mission_number = None   # 출발 난수(1~4)
        self.mission_number2 = None  # 복귀 시작 난수(1~4)
        self.red_say_count = 0       # "red one/two/..." 누적 카운트(가는길+복귀)
        self.run_started_t = None    # 버튼 시작 시각(스톱워치)
        self.green_done_t = None     # 초록 배달 완료 시각(복귀 구간 기준점)
        self.out_elapsed = None      # 출발→초록 소요(s)
        self.return_elapsed = None   # 초록→노랑 소요(s)
        self.last_turn = 0.0
        self.last_marker_t = -1e9
        self.last_node_t = -1e9
        self.last_recover_t = -1e9
        self.lost_since = None      # 000 연속 시작 시각(지속 필터, v13)
        self.lost_streak = 0        # 윈도 내 연속 복구 횟수(v13)
        self.node_debounce_ms = 0   # 직전 confirm 결과에 따른 재감지 간격(v13.1)

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
            "mission_number": self.mission_number or 0,
            "mission_number2": self.mission_number2 or 0,
            "red_say_count": self.red_say_count,
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

    # ---- 오디오/LCD(§변경 D, E) ----

    def _sound_file(self, key):
        return os.path.join(SOUND_DIR, key + ".wav")

    def ensure_sounds(self):
        """시작 시 1회: 미션 문구 wav 를 espeak 로 생성(이미 있으면 재사용).

        브릭에 espeak 가 없거나 실패하면 그 문구는 tone 폴백으로 재생된다.
        (사용자 문제 3 — 블록코딩의 오디오 코덱 위치를 파이썬에선 못 찾는
        문제를, 파일을 우리가 직접 만들어 두는 방식으로 해결.)

        AUDIO_ENABLED=False 면 espeak 시도조차 하지 않는다(§run10 — 브릭에서
        espeak 가 안 돼 생성 시도 자체가 무의미하다고 확인, 오디오 없이 우선
        주행 검증). 다시 켤 땐 AUDIO_ENABLED 한 줄만 바꾼다.
        """
        if not AUDIO_ENABLED:
            self.log.log("AUDIO_DIAG", "AUDIO_DISABLED")
            return
        phrases = {"good_job": "good job"}
        for n in NUMBER_WORDS:
            phrases["num_{}".format(n)] = NUMBER_WORDS[n]
        for i, w in enumerate(RED_WORDS):
            phrases["red_{}".format(i + 1)] = "red " + w
        try:
            if not os.path.isdir(SOUND_DIR):
                os.makedirs(SOUND_DIR)
        except Exception as exc:
            self.log.log("AUDIO_DIAG", "SOUND_DIR_FAIL", error=repr(exc))
            return
        made = 0
        cached = 0
        failed = 0
        for key in sorted(phrases):
            path = self._sound_file(key)
            if os.path.isfile(path):
                cached += 1
                continue
            try:
                code = subprocess.call(("espeak",) + ESPEAK_ARGS +
                                       ("-w", path, phrases[key]))
            except Exception:
                code = -1
            if code == 0 and os.path.isfile(path):
                made += 1
            else:
                failed += 1
                if failed == 1:
                    self.log.log("AUDIO_DIAG", "ESPEAK_FAIL",
                                 key=key, code=code)
        self.log.log("AUDIO_DIAG", "SOUND_FILES",
                     made=made, cached=cached, failed=failed, dir=SOUND_DIR)

    def _play_key(self, key, fallback_tones):
        """wav 가 있으면 비블로킹 큐 재생, 없으면 tone 폴백. 주행을 막지 않는다.

        AUDIO_ENABLED=False 면 wav/tone 둘 다 재생하지 않는다(§run10)."""
        if not AUDIO_ENABLED:
            return False
        path = self._sound_file(key)
        if os.path.isfile(path):
            self.hw.play_wav(path)
            self.log.log("AUDIO", "WAV_" + key.upper())
            return True
        for freq, dur in fallback_tones:
            self.hw.tone(freq, dur)
        self.log.log("AUDIO", "TONE_FALLBACK_" + key.upper())
        return False

    def say_number(self, number):
        """난수(1~4) 숫자음(명세 3, 7)."""
        self._play_key("num_{}".format(int(number)),
                       ((TONE_NUMBER_FALLBACK + 80 * int(number), 250),))

    def say_red(self):
        """빨강 방문 누적 카운트 증가 + "red one/two/..." 재생(명세 5)."""
        self.red_say_count += 1
        idx = min(self.red_say_count, len(RED_WORDS))
        self._play_key("red_{}".format(idx),
                       TONE_RED_FALLBACK +
                       ((TONE_NUMBER_FALLBACK + 80 * idx, 250),))

    def say_good_job(self):
        self._play_key("good_job", TONE_GOOD_JOB)

    def show_screen(self):
        """LCD: OUT/BACK 소요시간 + 최근 난수(best-effort)."""
        number = (self.mission_number2 if self.mission_number2 is not None
                  else self.mission_number)
        self.hw.show_final4_display(self.out_elapsed, self.return_elapsed,
                                    number)

    # ---- 센서 읽기(§변경 A) ----

    def read_color(self, snap):
        """중앙 RGB 1회 → (색, 밝기 정규화값). 한 읽기로 색+밝기 동시."""
        r, g, b = self.hw.read_center_rgb_now()
        color, bright = classify_rgb(r, g, b, snap)
        norm_c = normalize(bright, snap["cal_c_black"], snap["cal_c_white"])
        return color, norm_c

    def handle_pending(self):
        """대시보드 calibrate / read_rgb / read_reflect 액션 처리(없으면 no-op).

        센서 읽기 예외를 여기서 막는다(§run10) — run() 은 KeyboardInterrupt 만
        잡으므로, 방어 없이 두면 센서 읽기 1회 실패가 제어 루프 전체를 죽여
        대시보드에는 'queued' 만 남고 그 뒤로 아무 반응이 없게 된다(다른 do
        액션도 같이 멈춤). wait_center_button 과 같은 원칙(v13.1)."""
        with self._pending_lock:
            action = self._pending
            self._pending = None
        if action is None:
            return
        # 큐에서 꺼낸 액션을 무조건 로그(§run10 진단) — 문자열이 세 분기 중
        # 어디에도 안 걸리면 기존 코드는 조용히 아무것도 안 하고 넘어가서,
        # 액션명 불일치(공백/오타/구버전 describe 캐시 등)를 알아챌 방법이
        # 없었다. DO_DEQUEUE 로 실제로 무엇을 받았는지 항상 남긴다.
        self.log.log("DO_DEQUEUE", "PENDING_ACTION", action=str(action))
        try:
            if action == "calibrate":
                self.calibrate_line()
            elif action == "read_rgb":
                snap = self.params.snapshot()
                r, g, b = self.hw.read_center_rgb_now()
                color, bright = classify_rgb(r, g, b, snap)
                norm_c = normalize(bright, snap["cal_c_black"], snap["cal_c_white"])
                self.log.log("COLOR_READ", "DO_TRIGGER", r=r, g=g, b=b,
                             color=color, name=MARKER_NAMES.get(color, str(color)),
                             bright=round(bright, 1), norm_c=round(norm_c, 1))
                self.publish("read_rgb", r=r, g=g, b=b, color=color,
                             bright=round(bright, 1), norm_c=round(norm_c, 1))
            elif action == "read_reflect":
                snap = self.params.snapshot()
                rl = self.hw.read_left_reflect()
                rr = self.hw.read_right_reflect()
                nl = normalize(rl, snap["cal_l_black"], snap["cal_l_white"])
                nr = normalize(rr, snap["cal_r_black"], snap["cal_r_white"])
                self.log.log("REFLECT_READ", "DO_TRIGGER", reflect_l=rl,
                             reflect_r=rr, norm_l=round(nl, 1), norm_r=round(nr, 1))
                self.publish("read_reflect", reflect_l=rl, reflect_r=rr,
                             norm_l=round(nl, 1), norm_r=round(nr, 1))
            else:
                self.log.log("DO_ACTION_FAILED", "UNKNOWN_ACTION",
                             action=str(action))
        except Exception as exc:
            self.log.log("DO_ACTION_FAILED", "SENSOR_READ_EXCEPTION",
                         action=str(action), error=repr(exc))

    def reset_steer(self):
        self.pid.reset()
        self.last_turn = 0.0
        self.lost_since = None      # 모션 프리미티브 후 stale 유실 타이머 방지(v13)

    def _hold_while_paused(self, mode):
        """모션 중 pause: 모터를 세우고 해제/중단까지 대기."""
        self.hw.stop()
        while self.paused and not self.interrupted():
            self.publish(mode + "_paused")
            time.sleep(0.05)

    def read_bits(self, snap):
        color, norm_c = self.read_color(snap)
        rl = self.hw.read_left_reflect()
        rr = self.hw.read_right_reflect()
        return node_bits(rl, norm_c, rr, snap), color, norm_c, rl, rr

    # ---- 캘리브레이션(§변경 1) ----

    def _cal_sweep(self, direction, max_deg, stats, snap):
        """소각 피벗(L/R)하며 좌/우 반사광 + 중앙 RGB 밝기 min/max 수집.
        진행 enc deg 반환."""
        if max_deg <= 0:
            return 0.0
        left_dir, right_dir = (-1, 1) if direction == "L" else (1, -1)
        self._begin_motion(PIVOT_RAMP_MS)
        try:
            self.hw.drive_raw(left_dir * CAL_SPEED, right_dir * CAL_SPEED)
            while self.hw.enc_avg() < max_deg:
                if self.interrupted():
                    break
                rl, rr = self.hw.read_side_reflect()
                r, g, b = self.hw.read_center_rgb_now()
                _color, bright = classify_rgb(r, g, b, snap)
                bright = clamp(bright, 0.0, 100.0)
                if rl < stats["l_min"]:
                    stats["l_min"] = rl
                if rl > stats["l_max"]:
                    stats["l_max"] = rl
                if rr < stats["r_min"]:
                    stats["r_min"] = rr
                if rr > stats["r_max"]:
                    stats["r_max"] = rr
                if bright < stats["c_min"]:
                    stats["c_min"] = bright
                if bright > stats["c_max"]:
                    stats["c_max"] = bright
                time.sleep(0.005)
        finally:
            self._end_motion()
        return self.hw.enc_avg()

    def calibrate_line(self):
        """로봇을 라인(직선 구간) 위에 세워두고 실행 — 세 센서가 라인을
        가로지르도록 좌→우→복귀 소각 스윕하며 좌/우 반사광 + 중앙 RGB 밝기의
        black/white 실측치를 cal_* 파라미터에 기록한다. 성공: params.save /
        실패(어느 쪽이든 span < CAL_MIN_SPAN): 기존 캘리브레이션 유지.
        소각 왕복이라 자세는 대략 원위치."""
        self.hw.stop()
        snap = self.params.snapshot()
        self.log.log("CAL_START", "DO_TRIGGER",
                     half_deg=CAL_HALF_DEG, speed=CAL_SPEED)
        self.publish("calibrating")
        stats = {"l_min": 100.0, "l_max": 0.0, "r_min": 100.0, "r_max": 0.0,
                 "c_min": 100.0, "c_max": 0.0}
        d1 = self._cal_sweep("L", CAL_HALF_DEG, stats, snap)
        d2 = self._cal_sweep("R", d1 + CAL_HALF_DEG, stats, snap)
        self._cal_sweep("L", max(d2 - d1, 0.0), stats, snap)   # 대략 원위치 복귀
        time.sleep(POST_TURN_SETTLE_S)
        self.reset_steer()
        if self.interrupted():
            return
        l_span = stats["l_max"] - stats["l_min"]
        r_span = stats["r_max"] - stats["r_min"]
        c_span = stats["c_max"] - stats["c_min"]
        if (l_span < CAL_MIN_SPAN or r_span < CAL_MIN_SPAN or
                c_span < CAL_MIN_SPAN):
            self.log.log("CAL_FAIL", "SPAN_TOO_NARROW",
                         l_min=stats["l_min"], l_max=stats["l_max"],
                         r_min=stats["r_min"], r_max=stats["r_max"],
                         c_min=round(stats["c_min"], 1),
                         c_max=round(stats["c_max"], 1),
                         min_span=CAL_MIN_SPAN)
            return
        self.params.set("cal_l_black", int(round(stats["l_min"])))
        self.params.set("cal_l_white", int(round(stats["l_max"])))
        self.params.set("cal_r_black", int(round(stats["r_min"])))
        self.params.set("cal_r_white", int(round(stats["r_max"])))
        self.params.set("cal_c_black", int(round(stats["c_min"])))
        self.params.set("cal_c_white", int(round(stats["c_max"])))
        saved, save_msg = self.params.save()
        self.pid.full_reset()   # 에러 스케일이 바뀌었으므로 학습된 트림도 무효
        self.log.log("CAL_OK", "SWEEP_MIN_MAX",
                     l_black=int(round(stats["l_min"])),
                     l_white=int(round(stats["l_max"])),
                     r_black=int(round(stats["r_min"])),
                     r_white=int(round(stats["r_max"])),
                     c_black=int(round(stats["c_min"])),
                     c_white=int(round(stats["c_max"])),
                     saved=saved, save_msg=save_msg)

    # ---- 모션 프리미티브 ----

    def _begin_motion(self, ramp_ms=0):
        """모션 시작 공통(§run11) — hold 해제 → 엔코더 0 → 가속 램프.

        brake-hold 상태에서 reset_encoders() 를 부르면 위치 서보가 킥('틱틱')을
        낸다. coast() 로 먼저 hold 를 푼다(정지 상태라 바퀴는 안 구른다).
        ramp_ms=0 이면 램프 없이 coast+reset 만 — 시간 기준 creep 처럼 램프가
        결과를 바꾸는 곳에서 쓴다.
        """
        self.hw.coast()
        self.hw.reset_encoders()
        self.hw.set_ramp(ramp_ms)

    def _end_motion(self):
        """모션 종료 공통(§run11) — 정지 후 램프 해제.

        라인추종 drive() 에 램프가 남으면 매 15ms 조향이 뭉개진다.
        """
        self.hw.stop()
        self.hw.set_ramp(0)

    def straight(self, dist_mm, speed, mode="advancing"):
        """엔코더 기준 직진(speed<0 후진). 중단 시점까지의 mm 반환."""
        self._begin_motion(STRAIGHT_RAMP_MS)
        if dist_mm <= 0:
            self._end_motion()      # coast 로 풀린 hold 복원(경사에서 굴러감 방지)
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
            self._end_motion()
        return self.hw.enc_avg() * MM_PER_DEG

    def backup_to_line(self, max_mm, snap):
        """선을 다시 찾을 때까지 후진(최대 max_mm). (찾음 여부, 후진 mm)."""
        self._begin_motion(STRAIGHT_RAMP_MS)
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
                _color, norm_c = self.read_color(snap)
                rl = self.hw.read_left_reflect()
                rr = self.hw.read_right_reflect()
                if (norm_c < snap["center_th_node"] or
                        rl < snap["left_th_steer"] or
                        rr < snap["right_th_steer"]):
                    found = True
                    break
                self.publish("lost_backup",
                             dist_mm=round(self.hw.enc_avg() * MM_PER_DEG, 1))
                time.sleep(0.005)
        finally:
            self._end_motion()
        return found, self.hw.enc_avg() * MM_PER_DEG

    def turn(self, move):
        """제자리 회전(L/R/U) + heading 갱신 + PID 리셋. 유턴은 낮은 tone 2번 선행.
        회전각/속도를 바꾸려면 여기(와 params 의 factor)만 보면 된다.

        스무스 턴(§변경 D, v6 → run11 에서 _begin_motion/_end_motion 으로 공통화):
        coast 로 brake-hold 를 풀어 엔코더 리셋 킥을 막고, 회전 동안만
        set_ramp(turn_ramp_ms)로 가속을 램프해 속도PID 콜드스타트/백래시 킥을
        없앤다(감속=0 이라 target 에서 크리스프 정지)."""
        # 노드/마커 처리를 마치면 새 구간 — 복구 카운트 이월 방지(v13.1).
        self.lost_streak = 0
        if move == "S":
            # 직진 통과는 피벗이 없지만 노드 전진 직후라 중앙이 라인 중앙
            # (포화 검정)에 앉기 쉽다 — 엣지만 획득하고 나간다(§run10).
            if not self.interrupted():
                self.acquire_edge(self.params.snapshot(), "straight")
            return
        snap = self.params.snapshot()
        if move == "U":
            target = BASE_PIVOT_DEG_180 * snap["turn_180_factor"]
        else:
            target = BASE_PIVOT_DEG_90 * snap["turn_90_factor"]
        left_dir, right_dir = (-1, 1) if move == "L" else (1, -1)
        speed = snap["turn_speed"]
        self._begin_motion(snap["turn_ramp_ms"])   # coast+reset+가속 램프(v6/run11)
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
            self._end_motion()
        actual = self.hw.enc_avg()
        time.sleep(POST_TURN_SETTLE_S)
        self.log.log("TURN", {"L": "TURN_LEFT", "R": "TURN_RIGHT", "U": "UTURN"}[move],
                     target_deg=round(target, 1), enc_avg=round(actual, 1),
                     error_deg=round(actual - target, 1),
                     stopped_early=self.interrupted())
        self.ex.apply_move(move)
        self.reset_steer()
        # 회전 직후 엣지 획득(§run10) — 피벗 오차로 중앙이 흰 바닥에 있든
        # 라인 중앙(포화 검정)에 있든, PID 재개 전에 steer_sign 쪽 엣지 위로
        # 올려 과도 스윙 없이 라인과 평행하게 출발한다(run9 는 검정 찾기까지만
        # 했고 엣지 단계가 없어 회전 직후 큰 조향 스윙으로 시작했다).
        if not self.interrupted():
            self.acquire_edge(self.params.snapshot(), "after_" + move)

    # ---- 마커 / 노드 처리 ----

    def _confirm_marker_color(self, first):
        """마커 색 확정(§변경 B): 정지 상태에서 여러 번 재판독해 다수결. 가장
        많이 나온 색이 마커색이고 MARKER_CONFIRM_MIN 이상이면 그 색, 아니면 None.
        코너/흑백 경계에서 1프레임 튄 가짜 색이 배달/완주를 트리거하는 것을 막는다.
        (호출부가 이미 정지시킨 뒤 부른다 — 정지 상태라 센서가 안정적이다.)"""
        snap = self.params.snapshot()
        votes = {}
        for _i in range(MARKER_CONFIRM_SAMPLES):
            time.sleep(MARKER_CONFIRM_GAP_S)
            c, _norm = self.read_color(snap)
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
            # 완주(명세 8): 통 내려놓기 → "good job" + 소요시간 LCD → 종료.
            self._release_at_home()
            if self.green_done_t is not None:
                self.return_elapsed = time.monotonic() - self.green_done_t
            elif self.run_started_t is not None:
                self.return_elapsed = time.monotonic() - self.run_started_t
            self.say_good_job()
            self.show_screen()
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
                         fallback=self.ex.home_fallback,
                         out_s=round(self.out_elapsed or 0, 1),
                         back_s=round(self.return_elapsed or 0, 1))
            self.last_marker_t = time.monotonic()
            return True

        if self.ex.mode == "HOME":
            # 복귀 중 마커 리프(빨강 재방문/초록 경유 등) — 유턴은 계획에 포함.
            if color == COL_RED:
                self.visits += 1
                self.home_revisit += 1
                self.say_red()          # "red N" — 복귀 재방문도 누적(명세 5)
                self.log.log("REVISIT_HOME", "COLOR_RED_ON_RETURN",
                             revisit=self.home_revisit,
                             total=self.ex.home_red_total,
                             red_say=self.red_say_count)
            move, events = self.ex.on_arrive_home(name)
            self.log_events(events)
            self.turn(move)
            self.last_marker_t = time.monotonic()
            return True

        if color == COL_GREEN:
            # 초록 도착(명세 6): 통 내려놓기 → "good job" + 소요시간 LCD →
            # 1초 대기 → 180도 회전 → 두 번째 난수 표시+숫자음(명세 7, 사람이
            # 두 번째 통 배치) → 복귀 계획. 재파지는 복귀 중 초음파로 자동.
            self.deliver()
            if self.interrupted():
                return True
            if self.run_started_t is not None:
                self.out_elapsed = time.monotonic() - self.run_started_t
            self.say_good_job()
            self.show_screen()
            self.log.log("DELIVERY_DONE", "GOOD_JOB_GREEN",
                         out_s=round(self.out_elapsed or 0, 1))
            time.sleep(GOAL_PAUSE_S)    # 명세 6: 1초 후 180도 회전
            if self.interrupted():
                return True
            self.turn("U")
            if self.interrupted():
                return True
            self.green_done_t = time.monotonic()
            self.mission_number2 = random.randint(1, 4)
            self.show_screen()
            self.say_number(self.mission_number2)
            self.log.log("MISSION_NUMBER", "RETURN_START",
                         number=self.mission_number2)
            self.log_events(self.ex.start_home())
            self.last_marker_t = time.monotonic()
            return True

        # 탐색 중 빨강 = 경유지(명세 5): "red N" 후 유턴. 노랑 = 막다른 표식.
        if color == COL_RED:
            self.visits += 1
            self.say_red()
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
        """의심지점: PID off → 저속 직진(node_confirm_ms) → 정지 후 재판정.
        (확정 bits 또는 None, 그동안 전진한 mm) 반환. 마커를 만나면 처리 후 None.
        000 은 여기로 들어오지 않는다 — creep/재판정 중 000 이 보이면 취소(v13)."""
        self.reset_steer()
        # creep 은 node_confirm_ms '시간' 기준이라 램프를 걸면 그 안에 속도에 못
        # 올라 실측 튜닝된 creep 거리가 달라진다 — coast(리셋 킥 방지)만 한다.
        self._begin_motion(0)
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
            bits, color, norm_c, rl, rr = self.read_bits(snap)
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
        bits, color, norm_c, rl, rr = self.read_bits(snap)
        if self.handle_marker(color, "node_confirm_stop"):
            return None, 0.0
        if bits in NODE_CANDIDATES:
            self.log.log("NODE_CONFIRMED", "SLOW_STRAIGHT_STOP",
                         first_bits=bits_str(first_bits), bits=bits_str(bits),
                         reflect_l=rl, reflect_r=rr, color=color,
                         creep_mm=round(creep_mm, 1))
            return bits, creep_mm
        if bits == LOST_BITS and first_bits[0] == 1 and first_bits[2] == 1:
            # 정지 재판정에서 전백 — 위 creep 케이스와 같은 passed-over(v13.1).
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
        """확정 노드 처리. creep_mm(확정 중 전진분)을 빼서 총 전진을 40mm 로 맞춘다.
        bits 는 NODE_CANDIDATES 멤버만 온다(000 은 유실 경로로 분리, v13)."""
        snap = self.params.snapshot()
        self.straight(max(0.0, snap["node_advance_mm"] - creep_mm), CONFIRM_SPEED)
        if self.interrupted():
            return

        color, norm_c = self.read_color(snap)
        if self.handle_marker(color, "after_node_advance"):
            return

        has_left = bits[0] == 1
        has_right = bits[2] == 1
        has_straight = norm_c < snap["center_th_node"]
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
            bits, color, norm_c, rl, rr = self.read_bits(snap)
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

    def _pivot_scan(self, direction, max_deg, snap):
        """중앙 밝기가 라인(검정)이 될 때까지 소각 피벗(L/R). (found, 진행 enc deg)."""
        if max_deg <= 0:
            return False, 0.0
        left_dir, right_dir = (-1, 1) if direction == "L" else (1, -1)
        self._begin_motion(PIVOT_RAMP_MS)
        found = False
        try:
            self.hw.drive_raw(left_dir * REALIGN_SPEED, right_dir * REALIGN_SPEED)
            while self.hw.enc_avg() < max_deg:
                if self.interrupted():
                    break
                _color, norm_c = self.read_color(snap)
                if norm_c < snap["center_th_node"]:
                    found = True
                    break
                time.sleep(0.005)
        finally:
            self._end_motion()
        return found, self.hw.enc_avg()

    def realign_to_line(self, snap):
        """복구 직후 재정렬 — 중앙 밝기가 라인(검정) 위에 오도록 좌/우 소각 스캔.

        어두운 쪽부터 스캔하고, 양쪽 다 실패하면 원래 방향으로 복원 후
        False(호출부가 막다른길 처리). 소각이라 heading 격자는 안 바꾼다."""
        _color, norm_c = self.read_color(snap)
        if norm_c < snap["center_th_node"]:
            self.log.log("REALIGN", "ALREADY_ON_LINE")
            return True
        rl = self.hw.read_left_reflect()
        rr = self.hw.read_right_reflect()
        first = "L" if rl <= rr else "R"
        other = "R" if first == "L" else "L"
        found, d1 = self._pivot_scan(first, REALIGN_MAX_DEG, snap)
        if found:
            time.sleep(POST_TURN_SETTLE_S)
            self.reset_steer()
            self.log.log("REALIGN", "SCAN_" + first, deg=round(d1, 1))
            return True
        if self.interrupted():
            return False
        found, d2 = self._pivot_scan(other, d1 + REALIGN_MAX_DEG, snap)
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
        found, _d3 = self._pivot_scan(back_dir, abs(offset), snap)
        time.sleep(POST_TURN_SETTLE_S)
        self.reset_steer()
        if found:
            self.log.log("REALIGN", "FOUND_ON_RESTORE")
            return True
        self.log.log("REALIGN", "NOT_FOUND", scan_deg=round(d1 + d2, 1))
        return False

    # ---- 엣지 획득(§run10) ----

    def _pivot_to_edge(self, direction, max_deg, snap):
        """소각 피벗(L/R)하며 중앙 밝기가 엣지 밴드(edge_target±TOL)에 들어오면
        정지. (found, 진행 enc deg) 반환."""
        if max_deg <= 0:
            return False, 0.0
        left_dir, right_dir = (-1, 1) if direction == "L" else (1, -1)
        self._begin_motion(PIVOT_RAMP_MS)
        found = False
        try:
            self.hw.drive_raw(left_dir * REALIGN_SPEED,
                              right_dir * REALIGN_SPEED)
            while self.hw.enc_avg() < max_deg:
                if self.interrupted():
                    break
                _color, norm_c = self.read_color(snap)
                if abs(norm_c - snap["edge_target"]) <= EDGE_ACQUIRE_TOL:
                    found = True
                    break
                time.sleep(0.005)
        finally:
            self._end_motion()
        return found, self.hw.enc_avg()

    def acquire_edge(self, snap, context):
        """중앙 센서를 steer_sign 이 타는 라인 엣지 위로 올린다(§run10).

        엣지 팔로잉 PID 는 중앙이 '올바른 쪽' 엣지에 있어야 수렴한다 — 흰
        바닥인데 라인이 반대쪽이면 조향이 라인에서 멀어지고, 라인 중앙(포화
        검정)이면 큰 에러 과도 스윙으로 출발한다. 회전(피벗 오차)·노드 통과
        직후가 그 상태라 PID 재개 전에 정지 상태에서 획득한다:
          1) 이미 엣지 밴드 안이면 끝.
          2) 흰색이면 realign_to_line 으로 라인(검정)을 먼저 찾는다.
          3) 검정/회색에서 edge_exit_dir(steer_sign) 쪽으로 소각 피벗해 엣지
             밴드에서 정지 — 항상 같은 쪽으로 나가므로 타는 엣지가 유지된다.
        실패는 False 만 반환하고 주행은 계속한다 — 검정 위 PID 는 에러 부호가
        올바른 엣지 쪽을 가리켜 자기 정합으로 빠져나가고, 흰색 위 실패는 곧
        000 유실 경로가 복구한다. 마커 위에선 아무것도 하지 않는다(색 처리는
        메인 루프 몫 — 여기서 피벗하면 마커 판정을 흐린다).
        """
        color, norm_c = self.read_color(snap)
        if color in MARKER_COLORS:
            self.log.log("EDGE_ACQUIRE", "SKIP_ON_MARKER", context=context,
                         color=color)
            return False
        if abs(norm_c - snap["edge_target"]) <= EDGE_ACQUIRE_TOL:
            self.log.log("EDGE_ACQUIRE", "ALREADY_ON_EDGE", context=context,
                         norm_c=round(norm_c, 1))
            return True
        if norm_c >= snap["center_th_node"]:
            # 흰 바닥 — 라인이 어느 쪽인지 모른다. 좌/우 소각 스캔으로 검정을
            # 먼저 찾는다(양쪽 실패 시 자세 복원까지 realign 이 한다).
            if not self.realign_to_line(snap):
                self.log.log("EDGE_ACQUIRE", "LINE_NOT_FOUND", context=context,
                             norm_c=round(norm_c, 1))
                return False
            _color, norm_c = self.read_color(snap)   # realign 이 피벗했다 — 재판독
        exit_dir = edge_exit_dir(snap["steer_sign"])
        if norm_c > snap["edge_target"] + EDGE_ACQUIRE_TOL:
            # 밴드보다 밝지만 아직 라인이 보이는 구간(center_th_node 를 밴드 위로
            # 올린 §run11 G 이후 생긴다). 여기서 exit_dir(흰쪽)로 피벗하면 밴드에서
            # 더 멀어지므로 라인 쪽으로 되돌아 피벗한다.
            exit_dir = "L" if exit_dir == "R" else "R"
        found, deg = self._pivot_to_edge(exit_dir, EDGE_ACQUIRE_MAX_DEG, snap)
        time.sleep(POST_TURN_SETTLE_S)
        self.reset_steer()
        if found:
            self.log.log("EDGE_ACQUIRE", "ON_EDGE_" + exit_dir,
                         context=context, deg=round(deg, 1))
            return True
        # 밴드를 못 찾음(노드 가로선처럼 넓은 검정 위 등) — 검정 위라면
        # PID 가 자기 정합으로 이어받으므로 그대로 출발한다.
        self.log.log("EDGE_ACQUIRE", "BAND_NOT_FOUND", context=context,
                     exit_dir=exit_dir, deg=round(deg, 1))
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
        # 재정렬은 검정(라인 중앙)에서 멈춘다 — 엣지까지 마저 올라간다(§run10).
        self.acquire_edge(snap, "lost_recover")
        self.lost_streak += 1
        self.last_recover_t = time.monotonic()
        self.log.log("LINE_RECOVER", "BACKUP_REALIGN_OK",
                     dist_mm=round(dist, 1), streak=self.lost_streak)

    # ---- 세션 단계 ----

    def wait_for_start(self):
        """시작 대기(명세 1): 그리퍼 오픈 → 브릭 가운데 버튼 press→release →
        난수(1~4) LCD 표시+숫자음(명세 3, 사람이 통 배치) → 출발선 이탈 전진.
        status('go'|'stop'|'reset'). 물체는 여기서 잡지 않고 이동 중 초음파로
        만나면 잡는다(explore 루프).

        대기 중에도 handle_pending 이 돌므로 이 상태에서 calibrate / read_rgb 를
        먼저 실행하면 된다(로봇을 라인/스티커 위에 두고 실행)."""
        snap = self.params.snapshot()
        self.hw.grip_open(snap["grip_speed"], GRIP_SEC)   # 가는 길에 물체 받을 준비
        self.show_screen()
        while True:
            if self.stop_on:
                return "stop"
            if self.reset_on:
                return "reset"
            self.handle_pending()
            self.publish("waiting_start")
            try:
                pressed = self.hw.wait_center_button(
                    lambda: self.stop_on, lambda: self.reset_on,
                    poll_s=0.03, timeout=0.1)
            except Exception as exc:
                self.log.log("BUTTON", "WAIT_CENTER_FAILED", error=repr(exc))
                pressed = None
                time.sleep(0.2)
            if pressed is True:
                break
            # None = timeout(계속 대기), False = stop/reset(루프 상단에서 처리)
            # 또는 버튼 장치 초기화 실패 — 이때 sleep 없이 돌면 busy-spin 이므로
            # 짧게 쉰다(정상 타임아웃 대기에는 영향 미미).
            if pressed is False:
                time.sleep(0.1)
        self.run_started_t = time.monotonic()
        self.mission_number = random.randint(1, 4)
        self.show_screen()
        self.say_number(self.mission_number)
        self.log.log("START", "CENTER_BUTTON", number=self.mission_number)
        self.straight(START_EXIT_MM, STRAIGHT_SPEED, mode="start_exit")
        # 출발 직후에도 중앙이 라인 중앙에 앉기 쉽다 — 엣지 획득 후 PID(§run10).
        if not self.interrupted():
            self.acquire_edge(self.params.snapshot(), "start_exit")
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
            color, norm_c = self.read_color(snap)
            if self.handle_marker(color, "follow"):
                continue

            # 가는/오는 길 중 물체를 만나면(초음파 grab_dist 이내) 경보음(삐)
            # 후 잡는다(명세 6). 그립이 비어 있을 때만. 미로 어디서든 동작.
            if (not self.grabbed and
                    self.hw.read_distance_cm() < snap["grab_dist_cm"]):
                self.hw.stop()
                if AUDIO_ENABLED:
                    for freq, dur in TONE_GRAB_ALARM:
                        self.hw.tone(freq, dur)
                self.hw.grip_close(snap["grip_speed"], GRIP_SEC)
                self.grabbed = True
                self.log.log("GRAB", "ULTRASONIC_NEAR",
                             grab_dist_cm=snap["grab_dist_cm"])
                self.reset_steer()

            rl = self.hw.read_left_reflect()
            rr = self.hw.read_right_reflect()
            bits = node_bits(rl, norm_c, rr, snap)
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

            # 유실 의심 누적 중에도 감속해 라인 이탈 관성을 줄인다.
            slow = bits in SLOW_ON or self.lost_since is not None
            base = SLOW_SPEED if slow else snap["base_speed"]
            # 조향은 중앙 밝기 엣지 팔로잉(§변경 B). 좌/우 정규화값은 조향엔
            # 안 쓰고 텔레메트리 표시용으로만 계산한다.
            norm_l = normalize(rl, snap["cal_l_black"], snap["cal_l_white"])
            norm_r = normalize(rr, snap["cal_r_black"], snap["cal_r_white"])
            left, right, error, turn, trim = self.pid.step(norm_c, snap, base)
            self.hw.drive(left, right)
            self.last_turn = turn

            if now - last_follow_log >= FOLLOW_LOG_S:
                self.log.log("LINE_FOLLOW", "PID", reflect_l=rl, reflect_r=rr,
                             norm_l=round(norm_l, 1), norm_c=round(norm_c, 1),
                             norm_r=round(norm_r, 1),
                             bits=bits_str(bits), error=round(error, 2),
                             turn=round(turn, 2), trim=round(trim, 2))
                last_follow_log = now
            self.publish("follow", reflect_l=rl, reflect_r=rr, color=color,
                         norm_l=round(norm_l, 1), norm_c=round(norm_c, 1),
                         norm_r=round(norm_r, 1),
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
    hw.read_center_rgb(COLOR_MODE_SETTLE_S, 1)      # 중앙센서 RGB-RAW 모드 진입

    runner = Runner(hw, params, tele, log)

    server = TuningServer(params, tele, do_handler=runner.on_do,
                          stop_handler=runner.on_stop, pause_handler=runner.on_pause,
                          actions=ACTIONS, stage=STAGE_NAME)
    server.start()

    runner.ensure_sounds()      # AUDIO_ENABLED=False 면 즉시 반환(§run10, 오디오 보류)

    print("final_run11 ready. 1) dashboard 'calibrate' on the line / 'read_rgb' on "
          "each sticker, 2) press CENTER button to start (random number + voice). "
          "grips the object when the ultrasonic sees one. dashboard 'reset' ([r]) "
          "restarts. (Ctrl-C or robotctl stop to quit)")
    try:
        runner.run_sessions()
    except KeyboardInterrupt:
        log.log("EMERGENCY_STOP", "KEYBOARD", source="keyboard")
    finally:
        try:
            hw.stop()
        finally:
            server.stop()
    print("final_run11 stopped. sessions={}".format(runner.session))


if __name__ == "__main__":
    run()
