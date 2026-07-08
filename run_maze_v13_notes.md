# run_maze_v13 안정화 정리

## 목표

v12 실기에서 관찰된 문제를 고친다: 라인트레이싱 중 로봇이 조금만 틀어져도
중앙 컬러센서가 라인을 벗어나 `bits=000`(전백)이 뜨고, 이것이 데드엔드로
확정돼 불필요한 유턴이 발생했다.

원인 체인(v12):

```
살짝 틀어짐 → 중앙 컬러만 라인 이탈(좌우 반사센서는 흰색 유지)
→ 000 이 노드 후보(CANDIDATES)로 confirm 진입
→ confirm 이 '저속 전진'이라 라인에서 더 멀어짐 → 000 확정
→ handle_lost → 후진 복구(정렬은 안 함) → 틀어진 채 재출발
→ 4초 안에 재유실 → "LOST_AGAIN_AFTER_RECOVER" → 후진 없이 즉시 유턴
```

구조적 배경: PD 에러는 `우반사광-좌반사광`뿐이라 라인이 두 센서 사이
데드밴드 안에서 흐르는 동안은 보정이 0 이다. 그 사이 중앙 컬러만 먼저
이탈한다. 또 `|turn|>8` 위빙 가드는 완만한 틀어짐(turn≈0)에서는 절대
발동하지 않는다.

## v12 대비 핵심 변경 4가지

1. **000 지속시간 필터** — 000 은 `lost_persist_ms`(기본 100ms, 라이브 파라미터)
   연속 유지될 때만 유실 의심으로 처리한다. 컬러센서 한 프레임 깜빡임
   (0=none, 6=white 순간값)과 데드밴드 스침이 걸러진다. 의심 누적 중에는
   `SLOW_SPEED` 로 감속해 이탈 관성을 줄인다.
2. **000 을 노드 후보에서 제외** — `NODE_CANDIDATES` 에서 000 을 뺐다.
   유실 의심이 확정 단계로 가면 저속 '전진' confirm 대신 **즉시 정지 후
   정지 상태 3샘플 재판정**(`lost_check`)을 한다. 한 샘플이라도 라인이
   보이면 취소하고 주행을 재개한다.
3. **복구 연속 한도 `lost_max_recover`(기본 2, 라이브 파라미터)** —
   "복구 후 4초 내 재유실 = 즉시 유턴" 대신, 윈도(4s) 안에서 후진 복구를
   최대 N회 허용하고 한도를 넘어야 데드엔드 유턴한다. 진짜 막다른길은
   복구해도 곧 다시 유실돼 한도에 걸리므로 감지는 유지된다(막다른길당
   복구 사이클 1회분 정도 느려짐). 대회에서 시간이 급하면 1 로 낮추면
   v12 와 같은 타이밍이 된다.
4. **복구 후 재정렬(REALIGN)** — 후진으로 라인을 찾으면 중앙 컬러가
   black 이 되도록 어두운 쪽부터 좌/우 소각 스캔(한쪽 최대 70 enc deg,
   속도 6%)해 자세를 교정한 뒤 재출발한다. 양쪽 다 실패하면 원위치로
   복원하고 데드엔드 처리한다. 소각이므로 Explorer 의 heading 격자는
   바꾸지 않는다.

## 추가/변경 파라미터

| 항목 | 값 | 비고 |
|---|---:|---|
| `lost_persist_ms` | 100 | 라이브(0~500). 000 연속 유지 시간 필터 |
| `lost_max_recover` | 2 | 라이브(1~3). 유턴 확정 전 복구 허용 횟수 |
| `LOST_CONFIRM_SAMPLES` | 3 | 상수. 정지 재판정 샘플 수(전부 000 이어야 확정) |
| `REALIGN_MAX_DEG` | 70 | 상수. 재정렬 스캔 한쪽 최대 enc deg(로봇 약 50도) |
| `REALIGN_SPEED` | 6 | 상수. 재정렬 스캔 속도(%) |

나머지 주행/노드/마커/소리/세션 파라미터와 동작은 v12 와 동일하다
(`run_maze_v12_notes.md`). 저장 파일은 `config/run_maze_v13.json` 으로
분리되어 v12 튜닝값과 섞이지 않는다 — 첫 실행 후 대시보드에서 다시
save 할 것.

## 새 로그 이벤트

| 이벤트 | 의미 |
|---|---|
| `LOST_SUSPECT PERSIST_STOP_RECHECK` | 000 지속 → 정지 재판정 시작 |
| `LOST_SUSPECT CANCELLED_LINE_SEEN` | 재판정 중 라인 재발견 → 취소 |
| `LINE_LOST CONFIRMED_ALL_WHITE` | 3샘플 전부 000 → 유실 확정 |
| `REALIGN SCAN_L / SCAN_R / ALREADY_ON_LINE / FOUND_ON_RESTORE / NOT_FOUND` | 재정렬 결과 |
| `LINE_RECOVER BACKUP_REALIGN_OK` | 후진+재정렬 복구 완료(streak 포함) |
| `DEAD_END LOST_STREAK_LIMIT / BACKUP_NO_LINE / REALIGN_NO_LINE` | 유턴 사유 구분 |

가짜 유턴이 또 보이면 `DEAD_END` 직전의 `LOST_SUSPECT`/`REALIGN` 로그로
어느 단계가 뚫렸는지 바로 구분할 수 있다.

## 대시보드 리셋 버튼

`tools/dashboard.py` 에 `[r]` 키를 추가했다(모든 스테이지 공용).
누르면 confirm(`y`/`n`)을 거쳐 `{"cmd":"do","action":"reset"}` 을 보내
로봇을 출발 대기(노랑)로 되돌린다 — 탐색 상태를 통째로 버리므로 오타
방지용 confirm 을 넣었다. 기존 액션 목록의 reset(숫자 키)도 그대로 있다.

## 실행

EV3 브릭에서:

```bash
python3 stages/run_maze_v13.py
```

문법 점검(PC): `python3 -m py_compile stages/run_maze_v13.py lib/*.py`
