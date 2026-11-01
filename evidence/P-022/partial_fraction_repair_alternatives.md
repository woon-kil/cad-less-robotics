# partial fraction repair 후보

```text
P022_PARTIAL_FRACTION_REPAIR_ALTERNATIVES = PASS_REPAIR_ALTERNATIVES_REVIEW
RESULT = NO_JUSTIFIABLE_REPAIR
PARTIAL_FRACTION_REPAIR_SELECTED = NO

PATH = CartesianPrecision
VALIDATE_AND_IMPROVE_INTERVAL = KEPT

SCALAR_DISTANCE_CALLBACK = RECONSIDER
SCALAR_DISTANCE_CALLBACK_REJECTED = NO
SCALAR_CALLBACK_THRESHOLD = NOT_JUSTIFIED

ALTERNATIVE_REPAIR = NOT_SELECTED
START_IK_REPAIR = NOT_SELECTED
CALLBACK_IMPLEMENTED = NO
REPAIR_APPLIED = NO
THRESHOLD_T = NOT_DEFINED

P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

비교는 MoveIt 2.12.4의 `CartesianPrecision` 경로 안에서만 한다. `CartesianPrecision`, `max_resolution`, `eef_step`, ProcessPath, datum을 느슨하게 하는 안은 넣지 않는다. `JumpThreshold` overload로 바꿔 중간점 검사를 빼는 안도 같은 repair로 보지 않는다. 시작 IK는 이 비교의 대상이 아니다.

## 후보

| 후보 | 원인 직접성 | 새 threshold | 기존 의미 | 재현 | 침습 | 판정 |
| --- | --- | --- | --- | --- | --- | --- |
| seed 거리 callback | proxy | 필요. 출처 없음 | 경로 IK 채택만 제한 | 비교는 결정적. KDL 검색은 그대로 | 낮음 | 선택하지 않음. `RECONSIDER`이며 `REJECTED`는 아님 |
| 첫 중간점 실패를 바로 거절 | 첫 검사와 같음 | 없음. 기존 0.001 m, 0.01 rad | 0.402594 rad 해도 거절 | 결정적 | 낮음 | 선택하지 않음. 보존 대상과 충돌 |
| 기존 검사가 실패할 해만 거절하고 KDL에 다른 해를 요청 | 종료 조건과 같음 | 없음 | 실패 구간이 즉시 끝나지 않고 재검색 | 다른 해가 국소해인지는 미검증 | 높음. callback은 명령 pose를 받지 않고, 중첩 IK가 됨 | 선택하지 않음 |
| 무작위 재시드 제거 | attempt 1의 점프를 막지 못함 | 없음 | timeout 의미와 겹침 | 미검증 | 중간 | 선택하지 않음 |
| IK cost | 해당 없음 | 해당 없음 | KDL이 cost를 받으면 바로 false | 결정적 실패 | 낮음 | 사용 불가 |
| solver 교체 | 범위가 큼 | 알 수 없음 | 계약이 solver를 고정 | 해당 없음 | 큼 | 선택하지 않음 |
| 관절각 wrapping | 선형 보간 거리를 지움 | 없음 | bounded revolute와 불일치 | 해당 없음 | 중간 | 선택하지 않음 |

seed 거리 callback은 폐기하지 않는다. 다만 `T`의 공식 출처가 없으므로 지금 고를 수 없다.

첫 중간점 실패를 거절하면 call 8의 0.402594 rad처럼 한 번 나눈 뒤 통과한 해를 제거한다.

기존 검사 실패만 다시 검색하게 하면 보존 대상은 남길 수 있다. 그 안은 라이브러리 hook이 아니다. `validCallback`에 넘어오는 adapter는 Cartesian pose를 쓰지 않는다. 구간 검사를 callback 안에서 다시 돌리면 IK가 중첩된다. 다시 받은 해가 국소해인지는 측정하지 않았다.

시도 1에서도 `CartToJnt`가 seed에서 멀리 갈 수 있으므로, 무작위 재시드를 끄는 것만으로 포착된 종료가 사라지지 않는다.

## 결과

선택할 수 있는 repair가 없다. `NO_JUSTIFIABLE_REPAIR`로 멈춘다. callback은 만들지 않았다. `T`는 없다.
