# 전체 layered graph 공동 선택 — 1회

```text
GATE = UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH
EXECUTION = ONCE
RETRY = 0
PARENT_EXIT = 0
PARENT_WALL_KILL = false
PARENT_ELAPSED_S = 343.7852756610373
OVERALL_WALL_S = 757
ADDRESS_LIMIT_BYTES = 8589934592
CONFIRMED_BEFORE_EXEC = true
THREADS = 24
GRAPH_STATUS = PATH
GRAPH_EVALUATION = COMPLETE
EDGE_EVALUATIONS = 40676352
EDGE_CEILING = 40676352
DISCRETE_PASS = true
CONCLUSION = 저장 후보와 직선 TRANSITION을 사용한 전체 기하 후보 경로의 이산 검사 통과
CONTINUOUS_SAFETY = NOT_CLAIMED
TIME_PARAMETERIZATION = NOT_CLAIMED
PHYSICAL_EXECUTION = NOT_CLAIMED
FREE_MOTION_DENIED = false
INTEGRATED_MANIFEST = UNCHANGED
PASSING_RECORD_COUNT = 6
P-022 = UNCHANGED
```

원본 741개 pose와 저장된 중간 rung 203개를 한 그래프에 넣었다. rung 944, graph edge 943이다. 후보를 새로 풀거나 저장 전에 줄이지 않았다. 공통 필터가 241664개 저장 후보를 모두 본 뒤 194144개가 남았다. 최솟값 160, 최댓값 256, 빈 rung은 0이다.

PROCESS 720과 TRANSITION 20의 원본 의미는 유지했다. 중간 rung 203개는 모두 선택 경로에 있고, 각 점은 원래 PROCESS 구간의 `from_sequence_id`–`to_sequence_id`에 대응한다. 선택 경로의 원본 sequence id는 0부터 740까지 741개이며 빠지거나 중복되지 않는다. TRANSITION 도착점 20개도 그 안에 있다.

## 그래프 완료

| 항목 | 값 |
| --- | ---: |
| filter evaluations | 241664 |
| filter status | PASS |
| edge evaluations | 40676352 |
| edge ceiling | 40676352 |
| ladder status | PATH |
| stored edges | 311920 |
| ladder cost | 6.374169022738957 |
| allocation failure | 없음 |
| parent wall kill | false |

평가 수가 ceiling과 같다. 시간 초과, 미완료, 할당 실패가 아니다. 전체 생존을 가정한 쌍 상한 61,800,448은 필터 이후 인접 survivor 곱이 더 작아 실제 평가 쌍이 40,676,352이 된 결과다. 저장 edge 311920은 검사를 통과해 그래프에 들어간 쌍이다.

## 단계 시간과 메모리

잔여 예산은 다음 단계로 옮기지 않았다. 각 단계는 자기 중단 한도 안에서 끝났다.

| 단계 | 측정 초 | 중단 초 | VmHWM KiB | VmRSS KiB |
| --- | ---: | ---: | ---: | ---: |
| 공통 후보 필터 | 30.108318481 | 35 | 219376 | 134348 |
| layered graph | 308.090032637 | 680 | 244532 | 244532 |
| 독립 사후검사 | 5.143443074 | 22 | 244532 | 244532 |
| 저장 | 시작됨 | 10 | 244532 | 244532 |

자식 `process_elapsed_s`는 343.606564015초다. `getrusage` `ru_maxrss`는 241624 KiB다. 주소 공간 한도 8,589,934,592바이트는 실행 전에 확인했다.

## 독립 검사

선택 경로는 중간 관절을 포함해 사다리 타이머와 따로 검사했다.

| 판정 | PROCESS | TRANSITION |
| --- | --- | --- |
| 상태 | POSTCHECK_PASS | POSTCHECK_PASS |
| 원본 구간 | 720 | 20 |
| 표본 | 93223 | 40 |
| 공정 정확성 | 적용, 실패 0 | 적용하지 않음 |
| 최대 위치 오차 | 0.0009275561673644112 m |  |
| 최대 방향 오차 | 0.007625911618979101 rad |  |
| 관절 한계 실패 | 0 | 0 |
| 자체 충돌 | 0 | 0 |
| coupon 충돌 | 0 | 0 |
| 접합 | 통과, 0 rad | 통과, 0 rad |
| 미평가 충돌 | 0 | 0 |

원본 구간 누락은 없다. 접합, 공정 정확성, 관절 한계, 자체 충돌, coupon 충돌은 각각 통과다. 이 통과는 저장 후보와 직선 TRANSITION을 사용한 전체 기하 후보 경로의 이산 검사 통과다. 연속 안전, 시간 매개변수화, 실물 실행은 주장하지 않는다.

## 입력

R3 JSON 6개의 저장 해시를 실행 전에 확인한 뒤 `R6IN`을 만들었다. `inserts.bin` SHA-256은 `ff1f59d4275cdee2e41f5b8ef0da3182480204eb32d2372e1eb65ef4dd40f153`, 크기는 2768164바이트다.

| 파일 | SHA-256 |
| --- | --- |
| `fullgraph.json` | `f46217366bcd37a80c4363ccee44e9ed36ff7070f5b3f5b9f1f391974601677d` |
| `summary.json` | `2d02c5b8b5acd03707d8a98a9ba70afccea7c3701f7e5ad2427fd92bdbc80901` |
| `parent_limit.json` | `20a6f269ff94b7a5966367cc3da5d6feb448c06e4c3fecae8195baf5a900e230` |
| `parent.log` | `ef0b2195376b25edaca543ba2280699ddb3d2b77ca78d6dad449946c1fcea313` |

재시도, 설정 변경, 범위 확대는 하지 않았다. 통합 manifest, 통과 기록 6개, P-022는 바꾸지 않았다.
