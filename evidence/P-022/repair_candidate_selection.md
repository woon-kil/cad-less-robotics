# repair 후보 비교

```text
P022_REPAIR_CANDIDATE_SELECTION = PASS

SELECTED_CANDIDATE = PATH_IK_SEED_DISTANCE_VALID_CALLBACK
CANDIDATE_STATUS = SELECTED_FOR_EVALUATION
SCOPE = computeCartesianPath ONLY
START_IK = UNCHANGED
JOINT_DISTANCE_LIMIT = NOT_DEFINED
PROXY_VALIDITY = NOT_ESTABLISHED
REPAIR_APPLIED = NO
REPAIR_VALIDATED = NO
START_IK_FAILED_REPAIR = NOT_SELECTED

P022_INTERVAL_CAUSAL_TRACE = PASS_DIAGNOSTIC
CAPTURE_SOURCE = MOVEIT_2_12_4_EQUIVALENT_INSTRUMENTED_COPY
PARTIAL_FAILURE_MECHANISM = PROVEN_FOR_CAPTURED_RUN_7_INTERVAL_22
GENERALIZATION_TO_ALL_PARTIAL_FAILURES = NOT_PROVEN

FAILURE_A_REPAIR = NOT_SELECTED
SHARED_REPAIR_ASSUMED = NO

REPAIR_SELECTED_AS_CODE_CHANGE = NO
P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

run 7 interval 22의 종료 경로는 그 실행에 대해 설명됐다. 기준선 run 20의 0.606061과 run 12의 9/34는 같은 계측으로 설명하지 않았다. 계측은 MoveIt 라이브러리 내부가 아니라 2.12.4와 같은 순서의 복사본이다.

## 후보

| 후보 | Failure B | Failure A | 판정 |
| --- | --- | --- | --- |
| 경로 IK만 seed와의 joint 거리를 검사 | 9.65012 rad 해를 그 호출 안에서 거절할 수 있다 | 시작 pose는 zero seed에서 멀리 있어야 하므로 같은 제약을 걸면 안 된다 | 선택. 아직 적용하지 않음 |
| IK cost function | KDL 2.12.4는 cost overload가 없다. cost를 넘기면 base 구현이 바로 false를 반환한다 | 같은 이유 | 현재 solver에서 사용할 수 없음 |
| solver 교체 | 범위가 크다 | 범위가 크다 | 이번 선택에서 제외 |
| `max_resolution` 또는 midpoint tolerance 완화 | 실패한 검사를 통과로 바꾼다 | 해당 없음 | 선택하지 않음 |
| JumpThreshold overload | `validateAndImproveInterval`을 호출하지 않는다 | 해당 없음 | 선택하지 않음 |
| timeout만 변경 | 이미 timeout-only repair는 지지되지 않았다 | 5 ms 민감도는 관측됐다. 값과 경로 제약을 한 repair로 묶지 않는다 | 이번 선택에서 제외 |

## 선택

선택한 후보는 `CartesianPrecision` 경로의 `validCallback`이다. 이 overload는 `consistency_limits` 인자가 없다. KDL은 그 벡터가 있을 때 해와 원래 seed의 관절 차이를 검사한다. callback은 `setJointGroupPositions` 전에 호출되고, 이때 `RobotState`는 seed를 유지한다. 해가 seed에서 멀면 `NO_IK_SOLUTION`으로 돌려 다음 attempt를 계속하게 할 수 있다.

이 callback은 `computeCartesianPath`에만 넘긴다. 시작 IK는 `setToDefaultValues` seed라서 같은 거리 제한을 받지 않는다.

거리 숫자는 고르지 않았다. datum, ProcessPath, `eef_step`, Cartesian 합격 기준, midpoint tolerance, `max_resolution`은 바꾸지 않는다.
