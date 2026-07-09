# final_run10 — 회전 후 엣지 획득 노트

한 줄 요약: **run9 의 중앙 엣지 팔로잉에 빠져 있던 마지막 조각 — 회전/노드 통과/
유실 복구/출발 직후의 "엣지 획득(acquire_edge)"** 을 추가했다. 미로/마커/오디오
논리는 run9 그대로(PC 테스트 19개 통과), 엣지 획득 자체는 **실기 검증 필요**.

## 왜 (사용자 문제 제기)

동체가 라인과 평행하게, 좌우로 기울지 않고 곧게 따라가야 한다. run9 가 이미
좌/우 반사광 차이 조향을 폐기하고 중앙 센서 엣지 팔로잉 PID 로 바꿨지만
(좌/우는 분기 판단 전담), 회전 직후가 문제였다:

- 엣지 팔로잉은 중앙 센서가 **steer_sign 이 타기로 한 쪽 엣지** 위에 있어야
  수렴한다. 흰 바닥인데 라인이 반대쪽에 있으면 조향이 라인에서 **멀어지고**,
  라인 중앙(포화 검정)이면 에러 ±50 에서 시작해 큰 과도 스윙이 난다.
- run9 의 회전 후 처리(TURN_ACQUIRE)는 "중앙이 흰색이면 검정을 찾는다"까지만
  했다 — 검정(라인 중앙)에서 PID 를 켜니 매 회전 직후 몸이 크게 흔들리며 출발.

## 무엇이 바뀌었나 (final_run9 →)

| 시점 | run9 | run10 |
|---|---|---|
| 회전(L/R/U) 직후 | 흰색이면 검정 찾기(realign)만 | **acquire_edge: 검정 찾기 → 엣지 밴드까지 소각 피벗** |
| 노드 직진 통과(S) | 아무것도 안 함 | acquire_edge (노드 전진 뒤 중앙이 포화 검정에 앉는 케이스) |
| 유실 복구 후 | realign(검정에서 정지) | realign 후 acquire_edge 로 엣지까지 |
| 출발 직진 후 | 아무것도 안 함 | acquire_edge |

acquire_edge 절차(정지 상태, 조향 PID off):
1. 이미 엣지 밴드(`edge_target ± EDGE_ACQUIRE_TOL(8)`)면 통과.
2. 흰색(norm_c ≥ center_th_node)이면 기존 realign_to_line(좌/우 소각 스캔,
   어두운 쪽 먼저)으로 검정을 찾는다.
3. 검정/회색에서 `edge_exit_dir(steer_sign)` 쪽으로 소각 피벗(REALIGN_SPEED=6,
   최대 EDGE_ACQUIRE_MAX_DEG=60 enc deg)해 밴드에 들어오면 정지 → PID 재개.

부호 규약(테스트로 고정): **steer_sign=+1 = 라인의 오른쪽 엣지**(밝으면 좌조향
→ 라인이 센서 왼쪽) → 검정에서 **R** 로 탈출. -1 은 반대. 조향과 획득이 같은
값을 쓰므로 실기에서 폭주하면 여전히 **steer_sign 하나만** 뒤집으면 된다.

실패 시 행동(절대 멈추지 않음):
- 마커 색 위 → 아무것도 안 하고 반환(마커 처리는 메인 루프가, 피벗하면 판정 오염).
- 검정을 못 찾음(흰 바닥) → 그대로 출발, 곧 000 유실 경로가 후진+재정렬로 복구.
- 밴드를 못 찾음(노드 가로선처럼 넓은 검정) → 검정 위 PID 는 에러 부호가 올바른
  엣지 쪽을 가리키므로 자기 정합으로 빠져나간다(로그 `EDGE_ACQUIRE BAND_NOT_FOUND`).

새 이벤트: `EDGE_ACQUIRE` (ALREADY_ON_EDGE / ON_EDGE_L·R / SKIP_ON_MARKER /
LINE_NOT_FOUND / BAND_NOT_FOUND, context=after_L·R·U / straight / lost_recover /
start_exit). 라이브 파라미터는 **추가 없음**(EDGE_ACQUIRE_TOL/MAX_DEG 는 상수,
필요해지면 그때 개방 — AGENTS 원칙).

## 실기 튜닝 절차 (run9 절차에 추가되는 것)

run9 노트의 1~5(rgb_sum_white → 스티커 판정 → calibrate → steer_sign 확인 →
소리)는 동일. 추가 확인:

1. **회전 직후 로그**: 매 회전 뒤 `EDGE_ACQUIRE` 가 ON_EDGE_* 또는
   ALREADY_ON_EDGE 로 끝나는지. LINE_NOT_FOUND 가 반복되면 피벗각
   (`turn_90_factor`)이 크게 어긋난 것 — 획득이 아니라 회전각부터 잡는다.
2. **획득 방향**: steer_sign 을 뒤집었으면 획득도 자동으로 반대(L↔R) 탈출로
   바뀐다 — 별도 조정 불필요.
3. **BAND_NOT_FOUND 가 흔하면**: 라인이 넓거나 밴드가 좁은 것.
   EDGE_ACQUIRE_TOL(8) 상수를 브릭 로그의 norm_c 분포를 보고 조정.

## 위험/미검증 항목

- 획득 피벗 60 enc deg(로봇 약 40도) 한도가 실제 라인 폭에서 충분한지 실기 미확인.
- 획득이 매 회전마다 0.3~1초쯤 추가된다(정지 판독+소각 피벗) — 총 주행 시간 증가.
- run9 의 미검증 항목(RGB-RAW 지연, rgb_* 기본값, steer_sign 초기값, 버튼)은
  그대로 남아 있다.

## 업로드/실행

```bash
ssh robot@ev3dev.local 'mkdir -p ~/ev3test/stages ~/ev3test/lib ~/ev3test/tools ~/ev3test/config'
scp stages/final_run10.py robot@ev3dev.local:~/ev3test/stages/
scp lib/*.py robot@ev3dev.local:~/ev3test/lib/
scp tools/*.py robot@ev3dev.local:~/ev3test/tools/
```

- 브릭 실행: `ssh robot@ev3dev.local` → `cd ~/ev3test && python3 stages/final_run10.py`
- SSH 터널: `ssh -N -L 8765:localhost:8765 robot@ev3dev.local`
- 대시보드: `python3 tools/dashboard.py` (calibrate=[1], read_rgb=[2], reset=[r])
- telemetry: `python3 tools/telemetry_watcher.py`

PC 검증: `python -m py_compile stages/final_run10.py lib/*.py` /
`python tests/test_final_run10_maze.py` (19 tests) — 2026-07-10 통과 확인.
