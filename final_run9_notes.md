# final_run9 — 미션 최종판 노트 (inchul_last_dance.md 구현)

한 줄 요약: **3반사광(중앙은 RGB-RAW 겸용) PID + 미션 오디오/LCD/난수 + 복귀 실행
버그 수정** — 미로 논리는 `return_revisit_all_minimal_route.html` 과 동치임을 PC
테스트로 검증했고, 센서/오디오는 **실기 검증 필요**.

## 무엇이 바뀌었나 (final_run8 →)

| 항목 | run8 | run9 |
|---|---|---|
| 중앙센서(in2) | 상시 컬러 모드(색 코드) | **상시 RGB-RAW** — 한 읽기로 밝기(PID·노드)+색판정 동시 |
| 조향 에러 | normR − normL (2센서) | **중앙 센서 엣지 팔로잉** steer_sign·(norm_c − edge_target) — 좌/우는 조향 미사용 |
| 노드 중앙 bit | color==BLACK (이진) | **밝기 정규화값 < center_th_node** (아날로그 임계) |
| 마커 판정 | ev3dev color 코드 | **classify_rgb 비율 판정**(rgb_* 파라미터) + 기존 정지 다수결 유지 |
| 시작 | 노랑 감지 | **브릭 가운데 버튼**(명세 1) |
| 소리 | 없음 | 난수음/red N/good job/경보음 — **espeak 생성 wav 캐시** + tone 폴백 |
| LCD | 없음 | OUT/BACK 소요시간 + 난수(1~4) |
| 복귀 실행 | rel_move(실주행 heading, 계획 라벨) | **노드-로컬 라벨 계산**(아래 버그 참조) |

포트/배선은 정리.md 그대로(변경 없음): outA/outB 주행, outC 그리퍼,
in1/in2/in3 컬러센서, in4 초음파.

## 복귀 실행 버그 (run3~run8 잠재, 이번에 수정)

계획 스텝의 절대방향 라벨은 "그 분기점 출구 기준"의 **노드-로컬** 값인데,
run8 실행층은 커브를 지나며 갱신된 **실주행 heading** 과 rel_move 했다.
직선 복도에선 두 좌표계가 일치해 기존 테스트(직선 픽스처)는 통과했지만,
실제 미로처럼 분기점→마커 복도가 굽어 있으면(J4→J5→R2 등) 복귀 중 잘못된
회전이 나온다 — HTML 미로 픽스처로 재현했고(초록 재방문 발생), run9 는
`Explorer.on_arrive_home` 이 계획 그래프(route_adj)의 로컬 라벨만으로
상대 회전을 계산하도록 고쳤다(리프에선 자연히 U턴).
`tests/test_final_run9_maze.py::HtmlMazeCase` 가 회귀 방지.

## 미션 명세 대응 (inchul_last_dance.md §3)

1. 가운데 버튼 시작 → `wait_for_start`
2. 모든 빨강 방문 후 도착, 도착 재방문 불가 → 좌>우>직 탐색이 이 미로에선
   빨강 6개를 모두 찍은 뒤 초록에 닿고, 복귀 경로에 초록이 없음 — PC 테스트로 검증
3. 시작 난수(1~4) 화면+숫자음 → `wait_for_start` (LCD 큰 숫자 + num_N.wav)
4. 분기 자유 경로 → Explorer(좌>우>직)
5. 빨강 감지 시 "red one/two/…" → `say_red()` (가는길+복귀 누적 카운트)
6. 통 감지 경보음+파지(2cm 이상 리프트는 기존 그리퍼 기구) / 초록에서 내려놓고
   good job+시간 표시, 1초 후 180도 회전 → `handle_marker` COL_GREEN 분기
7. 복귀 시작 시 두 번째 난수 화면+숫자음 → 초록 유턴 직후 실행
   (해석: "시작위치로 반환 시" = 복귀를 시작할 때. 두 번째 통은 복귀 경로에
   배치되어야 로봇이 지나가며 파지할 수 있다)
8. 두 번째 통을 출발지(노랑)까지 운반, good job+시간 → COL_YELLOW@HOME 분기

## 실기 튜닝 절차 (필수 — 전부 실기 검증 필요)

1. **rgb_sum_white**: 흰 바닥 위에서 대시보드 `read_rgb` 실행 → r+g+b 합을
   이 파라미터에 넣는다(밝기 100% 기준점).
2. **스티커 판정**: 빨강/초록/노랑 스티커 위에 중앙센서를 대고 `read_rgb`
   → 로그의 color 가 기대색인지 확인, 아니면 rgb_red/green/yellow_ratio 조정.
   검은 라인 위에서도 실행해 COL_BLACK(1) 확인(rgb_black_max).
3. **calibrate**: 라인 직선 구간 위에서 실행 — 좌/우 반사광 + 중앙 밝기의
   black/white 를 실측 기록(cal_*).
4. 주행(중앙 엣지 팔로잉): **먼저 로봇을 라인 경계에 걸치게 놓고** 천천히 출발해
   방향을 본다 — 라인 반대로 폭주하면 `steer_sign` 을 -1 로 뒤집는다(그게 핵심
   한 값). 이후 kp(기본 0.30)/ki/deadband/base_speed 라이브 튜닝. edge_target 은
   중앙이 타야 할 경계 밝기(기본 50) — calibrate 후 라인 경계에서 read_rgb 의
   norm_c 를 보고 조정. center_th_node(노드 판별용)는 norm_c 로그로 별도 조정.
5. 소리: 첫 실행 때 espeak 가 `sounds/` 에 wav 17개를 생성(수십 초 소요,
   이후 캐시). `AUDIO_DIAG` 로그에서 made/cached/failed 확인.
   espeak 이 없으면 tone 폴백으로만 동작 — `sudo apt-get install espeak`.

## 위험/미검증 항목

- RGB-RAW 모드의 읽기 지연/노이즈는 실기 미확인(컬러 모드와 동급으로 예상).
- rgb_* 기본값은 전형값 기반 추정 — read_rgb 실측 없이는 마커 오판 가능.
- 중앙 엣지 팔로잉은 **중앙 센서가 라인 경계를 타야** 방향 신호가 나온다.
  라인이 얇아 중앙이 라인 정중앙(포화 검정)에 앉으면 신호가 없어 못 따라간다 —
  장착/트랙을 엣지 기준으로 맞추는 게 전제(실기 미검증, 사용자 생각 1 채택).
- steer_sign 초기값 +1 은 임의 — 첫 주행에서 방향 확인 후 확정해야 한다.
- 버튼 시작: ev3dev Button 이 정상일 때만 시작 가능(실패 시 로그 BUTTON).

## 업로드/실행

```bash
ssh robot@ev3dev.local 'mkdir -p ~/ev3test/stages ~/ev3test/lib ~/ev3test/tools ~/ev3test/config'
scp stages/final_run9.py robot@ev3dev.local:~/ev3test/stages/
scp lib/*.py robot@ev3dev.local:~/ev3test/lib/
scp tools/*.py robot@ev3dev.local:~/ev3test/tools/
```

- 브릭 실행: `ssh robot@ev3dev.local` → `cd ~/ev3test && python3 stages/final_run9.py`
- SSH 터널: `ssh -N -L 8765:localhost:8765 robot@ev3dev.local`
- 대시보드: `python3 tools/dashboard.py` (calibrate=[1], read_rgb=[2], reset=[r])
- telemetry: `python3 tools/telemetry_watcher.py`

PC 검증: `python -m py_compile stages/final_run9.py lib/*.py` /
`python tests/test_final_run9_maze.py` (16 tests) — 2026-07-09 통과 확인.
