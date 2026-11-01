# threshold 선정 절차

```text
P022_THRESHOLD_SELECTION_PROTOCOL = PASS_PROTOCOL_FROZEN
PRIOR_REVIEW = REVIEW_REQUIRES_REVISION
REMOVED_RULE = T_EQUALS_OBSERVED_MAXIMUM_M
THRESHOLD = NOT_DEFINED
CALLBACK_IMPLEMENTED = NO
CALLBACK_APPLIED = NO
REPAIR_APPLIED = NO

CALIBRATION_EXECUTED = NO
ACCEPTANCE_EXECUTED_UNDER_THIS_PROTOCOL = NO

P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
START_IK_FAILED_REPAIR = NOT_SELECTED
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

`T`는 이 문서에서 정하지 않는다. acceptance fraction을 보고 `T`를 올리지 않는다. `0.402594`와 `1.85667` 사이의 한가운데 값을 고르지 않는다.

## 데이터 분리

calibration과 acceptance는 다른 실행이다.

calibration에 쓰지 않는 것:

- `cartesian_runs.json`
- run 20 joint trace
- partial failure capture
- interval 22 trace
- 그 실행들의 joint 차이로 `T`를 계산하는 일

그 기록은 계약의 보존 값과 거부 값으로만 남는다. `0.402594 rad` 하나를 정상 분포의 상한으로 쓰지 않는다.

calibration은 나중에, callback 없이, 동결된 설정으로 도는 PROCESS run 중에서 fraction이 1인 run의 경로 IK만 쓴다. 시작 IK 실패 run과 partial fraction run은 로그할 수 있으나 `M`을 올리거나 내리는 데 쓰지 않는다.

```text
CALIBRATION_TRIAL_LIMIT = 20
TRIAL = process_runs.txt의 21개 PROCESS run을 한 번 순서대로 실행
EARLY_STOP_IF_M_IN_RANGE = NO
M_COMPUTED_ONCE_AFTER_THE_FIXED_SET = YES
```

20은 이번 실행을 보기 전에 정한 상한이다. partial capture에 쓰던 진단 상한과 같은 수다. `M`이 어느 구간에 들어와도 20회를 줄이지 않는다.

## 규칙

calibration의 목적은 `T`를 만드는 것이 아니다. 독립된 fraction 1 경로에서, 알려진 통과 해와 포착된 실패 표본 사이의 간격이 유지되는지만 본다.

`R`은 fraction 1 run의 경로 IK 전이 중 최종 결과가 통과인 것이다. 첫 검사 통과와 분할 후 통과를 모두 넣는다. 각 전이의 거리는 한계가 있는 revolute의 최대 절대차다. wrapping은 하지 않는다.

`M`은 `R`의 최대 거리다. `M`은 그 실행에서 본 최댓값이다. 정상 continuation의 상한이 아니다. `T = M`으로 두지 않는다. 관측 최댓값에는 아직 보지 않은 통과 해를 위한 여유가 없다.

```text
IF R IS EMPTY OR M <= 0.402594:
  T = NOT_DEFINED
  RESULT = INSUFFICIENT_CALIBRATION_COVERAGE

IF 0.402594 < M < 1.85667:
  T = NOT_DEFINED
  RESULT = SEPARATION_OBSERVED
  CANDIDATE = SURVIVES
  THRESHOLD_CANDIDATE_ELIGIBLE = YES

IF M >= 1.85667:
  T = NOT_DEFINED
  RESULT = SEPARATION_NOT_SUPPORTED
  SCALAR_DISTANCE_CALLBACK = RECONSIDER
```

`M <= 0.402594`이면 빈 구간에 숫자를 넣지 않는다. `M >= 1.85667`이면 `M`을 낮춰 실패 표본에 맞추지 않고, scalar 거리 callback을 다시 본다.

`0.402594 < M < 1.85667`은 `T`를 만들 수 있는 충분조건이 아니다. 후보는 남고, `T`는 여전히 없다. margin 숫자는 이 절차에서 정하지 않는다.

## 그 다음

callback은 만들지 않는다. acceptance fraction으로 `T`를 고르지 않는다. ProcessPath, datum, `eef_step`, Cartesian 합격 기준, `CartesianPrecision`, `max_resolution`, solver, 시작 IK는 그대로다.
