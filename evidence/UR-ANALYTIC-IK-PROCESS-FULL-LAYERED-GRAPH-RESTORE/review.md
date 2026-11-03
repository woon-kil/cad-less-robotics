# 복원 검사 코드 리뷰

```text
REVIEW = CODE_AND_REPRODUCIBILITY
MANIFEST_ADOPTION = NOT_DONE
P-022 = UNCHANGED
PRIOR_AGGREGATE = MAINTAINED
CLAIM = 전체 기하 후보 경로 확보 및 고정 표본 이산 검사 통과
CONTINUOUS_SAFETY = NOT_CLAIMED
TOOL_FIXTURE_TABLE = NOT_CHECKED
TIME_PARAMETERIZATION = NOT_CLAIMED
PHYSICAL_EXECUTION = NOT_CLAIMED
```

새 통합 manifest는 채택하지 않았다. `restore.json`을 고치지 않았다. 검사 소스는 실행 때 evidence에 없었고, 재현 확인을 위해 `restore-src.cpp`를 이 디렉터리에 넣었다. SHA-256은 `a260f70938057a25643419ca6aefbb01e0ee515dfa3d2be214923f4eaa08cde1`이다.

## 산출물 대조

`restore.json` SHA-256 `35f72af9e3b23b9cb2a4e6133eb24623489ed00762f8ccf40000e618a0998943`를 다시 읽었다.

| 항목 | 결과 |
| --- | --- |
| 관절열 | 944, 슬롯 224, 관절 6, 필터 `IN_SURVIVORS` |
| 입력 파일 | `runs.bin` 721, `transitions.bin` 20, `inserts.bin` 203 |
| 앞선 동일 관절 | 0 |
| 원본 구간 | `(0,1)` … `(739,740)` 740개, 누락 없음 |
| PROCESS | 720. 미분할 517은 half 0, 분할 203은 half `(1,2)` |
| TRANSITION | 20. 표본 101. `accuracy_applied` false. 오차와 `accuracy_fail`은 null |
| 위반 | 유한성·한계·자체 충돌·coupon·오류·미평가 모두 0 |
| 실패 표본 | 0 |
| PROCESS 최대 오차 | 0.0009275561673644112 m, 0.007625911618979101 rad |
| 보간 끝 일치 | 통과 조건에 포함되지 않음 |

이 구성은 고정한 101표본 규칙, 분할 구간은 반쪽 둘 다 통과해야 하는 규칙, 비적용 항목을 0이나 PASS로 쓰지 않는 규칙과 맞다.

## 코드

구간 검사는 Descartes 탐색을 다시 호출하지 않는다. 슬롯 224의 저장 관절만 읽는다. PROCESS는 위치 선형·방향 slerp·위치 `0.001` m·방향 `0.01` rad 뒤에 충돌을 본다. TRANSITION은 `k` = 0…100의 raw 관절 보간에서 유한성, 한계, 자체 충돌, coupon만 본다.

같은 파일의 `filter_candidate`, `pose_error`, `apply_collision`을 쓴다. 그래서 이번 통과는 사다리 탐색과 분리된 구간 기록이지만, 다른 충돌 구현의 재현은 아니다. PROCESS 최대 오차가 이전 합계와 같은 것은 같은 검사 함수의 일치다.

실패가 0건이라 접촉을 저장하는 분기는 실행되지 않았다. 그 분기가 맞다는 증거는 없다. 통과 기록은 위반 0인 표본의 집계다.

## 채택

기존 manifest와 P-022 `CLOSED_FAIL`은 유지한다. 이 리뷰는 채택 절차의 확인이며 채택 자체가 아니다.
