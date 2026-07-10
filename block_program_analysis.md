# robot_team1.ev3 (팀 블록코딩 프로그램) 분석

한 줄 요약: 이 파일은 **라인트레이싱이 아니라 "병렬 감지/카운트/사운드 태스크 +
시작 난수"** 프로그램이다 — 조향(연속 모터 구동) 블록이 하나도 없다. 대신 색
감지 방식·포트·색코드·디바운스 구조가 그대로 들어 있어 파이썬 구현의 대조
검증 자료로 값지고, 블루 카운트 디바운스에 **버그도 하나** 있었다.

원본: `C:\Users\kkddk\Documents\카카오톡 받은 파일\robot_team1.ev3`
(EV3 Lab 프로젝트 zip — Program.ev3p 147KB XML을 파싱해 전체 블록/배선 복원)

## 프로그램이 실제로 하는 일

```
시작
 ├─ RandomSingle(1~4) → global 'random' 저장
 ├─ random 케이스: One/Two/Three/Four.rsf 재생 (Volume=100, Play Type=0 = 재생 완료까지 대기)
 ├─ LCD DisplayStringGrid(난수, Row=3, Size=2 큰 글씨)
 ├─ 초기화: nodecount=0, branchcount=0, cancountbranch=True, arrived=False, returned=False
 └─ WHILE (arrived == True 가 되면 StopIfTrue 로 종료):
     ├─ detect_red    = 포트1 색==빨강[5] (3회 판독 OR — 포트 1 하나만 사용)
     ├─ detect_blue   = 포트1 OR 포트2 OR 포트3 색==파랑[2]
     ├─ branch_detect = 포트1 AND 포트2 AND 포트3 색==검정[1]  ← 분기 = 111
     ├─ handled=True (한 루프에 이벤트 하나만 처리하는 플래그)
     ├─ [detect_red && handled]  → MoveStop(1.D+A) + arrived=True   ← 빨강 = 도착(정지)
     ├─ [detect_blue && cancountnode && handled]
     │     → nodecount++ , Blue.rsf 재생(Play Type=1 비블로킹),
     │       MoveTank(1.B+C, 50/50, 1회전, 브레이크) ← 스티커 통과 전진(~176mm)
     │       cancountnode=False (디바운스)
     ├─ [NOT detect_blue] → cancountnode=False   ★버그: True(재무장)여야 함
     ├─ [branch_detect && cancountbranch && handled] → branchcount++, cancountbranch=False
     ├─ [NOT branch_detect] → cancountbranch=True (재무장 — 이게 올바른 패턴)
     └─ [NOT handled] → (빈 케이스, 잔재)
```

## 파이썬(final_run10)과 대조 — 일치 확인된 것

| 항목 | 블록 프로그램 | final_run10 | 판정 |
|---|---|---|---|
| 난수 | 1~4, 숫자음, LCD 큰 글씨 | 동일 | ✓ 일치 |
| 색 코드 | black=1, blue=2, red=5 | COL_* 동일 | ✓ 일치 |
| 색 센서 포트 | 1, 2, 3 (in1/in2/in3) | 동일 배선 | ✓ 일치 |
| 분기 판정 | 3센서 모두 검정(111) | NODE_CANDIDATES 의 111 | ✓ 일치 |
| 이벤트 디바운스 | cancount* 플래그 | 시간 기반 debounce + 정지 확정 | 우리가 더 강함 |
| 사운드 | 프로젝트에 One~Four/Blue/Hello.rsf 동봉 | 같은 원본에서 추출한 wav | ✓ 같은 목소리 |

## 새로 얻은 정보/주의점

1. **이 파일에 라인트레이싱이 없다.** 연속 주행 블록이 전혀 없다 — 이 프로그램은
   블록코딩의 "병렬 실행" 중 감지·카운트 태스크였고, 조향은 별도 프로그램에
   있었을 것이다. **조향 게인/목표값은 여기서 얻을 수 없음** — 그 파일이 따로
   있으면 추가 분석 가능.
2. **당시 미션 변형**: 경유지 = 파랑(감지 시 Blue 소리), 빨강 = 도착(정지).
   지금 명세(빨강 = 경유지, red one/two 소리)와 다르다 — 값 이식 시 색 역할을
   혼동하지 말 것.
3. **모터 포트가 지금과 다르다**: 블록 시절 주행 = B+C (MoveStop 은 D+A 로
   찍혀 있는데 이건 그쪽 실수로 보임). 지금 파이썬 로봇은 outA/outB — 즉
   블록 시절 숫자를 이식할 때 포트 관련 값은 무시해야 한다.
4. **마커 통과 전진 = 바퀴 1회전(≈176mm) @ 50%**: 스티커 위에서 감지 후
   벗어나기 위한 실측 거리. 우리 구조(마커=유턴)와 달라 직접 쓸 일은 없지만,
   스티커 재감지 방지에 필요한 거리 감각으로 참고.
5. **당시엔 세 센서 전부 COLOR 모드**로 돌면서 감지만 했다(주행은 별도 태스크).
   빨강 감지가 포트 1(중앙?) 하나뿐인 것도 특징 — 지금 우리 설계(중앙 RGB
   단독 색판정)와 사실상 같은 결론.
6. **발견한 블록 프로그램 버그**: `NOT detect_blue → cancountnode=False`.
   분기 쪽(`NOT branch → cancountbranch=True`)과 달리 재무장을 안 하므로
   블루 카운트가 사실상 처음 1회만 동작한다. 파이썬 구현은 시간 기반
   디바운스라 이 버그가 없다. (블록 시절 "카운트가 가끔 안 됐다"면 이게 원인.)
7. **숫자음은 Play Type=0(완료 대기)**: 블록 프로그램은 숫자음이 다 나온 뒤
   출발했다. 우리는 비블로킹 재생 직후 출발 — 시간상 유리해서 유지하되,
   채점에서 "숫자음을 다 듣고 출발"을 요구하면 wait_for_start 의 출발 직전에
   짧은 대기 하나만 넣으면 된다.

## 분석 방법 (재현용)

.ev3 = zip. 내부 Program.ev3p 가 LabVIEW 계열 XML. 블록 =
`ConfigurableMethodCall`(Target 이 블록 종류), 설정값 =
`ConfigurableMethodTerminal` 의 `ConfiguredValue` 속성, 케이스 조건 =
`PairedStructure` 로 연결된 `CaseSelector_*` 의 입력 와이어를
`<Wire Joints="N(nX:Out) N(nY:In)">` 로 역추적.
