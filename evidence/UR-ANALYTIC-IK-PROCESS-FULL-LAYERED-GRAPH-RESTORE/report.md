# 선택 경로 복원과 구간별 독립 검사

```text
GATE = UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH-RESTORE
EXECUTION = ONCE
RETRY = 0
PARENT_EXIT = 0
PARENT_WALL_KILL = false
PARENT_ELAPSED_S = 36.142768274992704
OVERALL_WALL_S = 180
FILTER_ABORT_S = 90
INSPECTION_ABORT_S = 50
ADDRESS_LIMIT_BYTES = 8589934592
CONFIRMED_BEFORE_EXEC = true
LEFTOVER_TRANSFER = false
FILTER_RESULT = IN_SURVIVORS
INSPECTION = DONE
ORIGINAL_PASS = 740
ORIGINAL_FAIL = 0
ORIGINAL_NOT_EVALUATED = 0
DISCRETE_PASS = true
CONCLUSION = 저장 관절열의 원본 740구간에 대한 고정 표본 이산 검사 통과
CONTINUOUS_SAFETY = NOT_CLAIMED
TIME_PARAMETERIZATION = NOT_CLAIMED
PHYSICAL_EXECUTION = NOT_CLAIMED
INTEGRATED_MANIFEST = NOT_ADOPTED
P-022 = UNCHANGED
PRIOR_AGGREGATE = MAINTAINED
NEW_SEARCH = false
CANDIDATE_REPLACEMENT = false
```

부모 한도는 자식 시작 전에 `parent_limit.json`에 기록했다. 필터 중단 90초는 전체 후보 필터 측정 30.108318481초의 3배다. 구간 검사 중단 50초는 기존 사후검사 5.143443074초보다 크게 잡았다. 합은 180초다. 잔여 예산은 옮기지 않았다. 주소 공간 8,589,934,592바이트는 실행 전에 확인됐다.

재탐색은 하지 않았다. 입력 해시가 대응 검토와 같고, 선택 배열 SHA-256 `eac7465345b5601422bcaa0225a607e885be177f6c5da30d75cd6925e8b066f4`가 맞았다. 944개 rung은 모두 원본 후보 슬롯 224다. 같은 필터에서 944개 모두 `IN_SURVIVORS`였다. 224보다 앞선 생존 후보 중 관절이 같은 경우는 0이다. 다른 후보로 바꾸지 않았다.

복원 관절열은 `restore.json`의 `joints` 944개다. 각 줄에 입력 파일, 해시, sequence, 슬롯 224, class, source, 관절 6개가 있다.

## 원본 740구간

| 구분 | 개수 | 하위 edge | 구간 표본 |
| --- | ---: | ---: | ---: |
| PROCESS 미분할 | 517 | 1 | 101 |
| PROCESS 분할 | 203 | 2 | 반쪽마다 101 |
| TRANSITION | 20 | 1 | 101 |

PROCESS 하위 edge는 923개다. 분할 구간은 반쪽 둘이 모두 `PASS`일 때만 원본 구간이 `PASS`다. 720개 PROCESS와 20개 TRANSITION이 모두 `PASS`다.

PROCESS 최대 위치 오차는 0.0009275561673644112 m, 최대 방향 오차는 0.007625911618979101 rad다. 유한성·한계·자체 충돌·coupon 충돌 위반은 0이다.

TRANSITION은 `t=k/100`, `k` = 0…100이다. 공정 정확성은 적용하지 않았고, `max_position_error_m`, `max_orientation_error_rad`, `accuracy_fail`은 null이다. 유한성·관절 한계·자체 충돌·coupon 충돌 위반은 0이다. 구간마다 내부 표본 `k` = 1…99가 포함된다.

실패 표본이 없어 접촉 기록은 비어 있다. 보간 끝 일치의 최댓값은 0 rad이며, 이 값은 구간 통과 조건에 넣지 않았다.

이 통과는 저장 관절열의 원본 740구간에 대한 고정 표본 이산 검사다. 표본 사이 연속 안전, 시간 매개변수화, 실물 실행은 주장하지 않는다. 기존 합계 결과와 통합 manifest, P-022는 수정하지 않았다.

| 파일 | SHA-256 |
| --- | --- |
| `restore.json` | `35f72af9e3b23b9cb2a4e6133eb24623489ed00762f8ccf40000e618a0998943` |
| `summary.json` | `ad953685194753da9eec10de40d37cac2ce1291c47a09aaa9698a8c7de9e4c52` |
| `parent_limit.json` | `77ea4a156e2224215245991a5123d8bc8baaa3674d382c36aa04eb5b4aabc380` |
| `parent.log` | `04250b32d703bf5a50f1e5585cc3eb4908871d6fa5b31d2e85f75ce2283326a2` |
