# P-022 실패 진단

```text
P022_FAILURE_DIAGNOSIS = PASS

BASELINE_FINAL_GATE = FAIL_CARTESIAN_API
BASELINE_PRESERVED = YES

FULL_FRACTION_RUN_COUNT = 17

START_IK_FAILED =
  run 5
  run 7
  run 12

PARTIAL_CARTESIAN_RUN =
  run 20
  20 / 33
  fraction = 0.606061

REPEAT_FAILURE_LOCATION_STABLE = NO
REPEAT_FRACTION_STABLE = NO

GEOMETRIC_UNREACHABILITY_PROVEN = NO

DIAGNOSIS =
NONDETERMINISTIC_TIME_BOUNDED_KDL_IK_FAILURE

REPAIR_SELECTED = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

기준선은 `cartesian_runs.json`이다. 이 진단은 그 파일을 바꾸지 않는다.

같은 binary와 같은 pose에서 실패 위치가 움직였다.

```text
baseline : 5, 7, 12
repeat 1 : 5, 15
repeat 2 : 3, 6
repeat 3 : 5
```

이 evidence로 run 5, 7, 12, 20이 기하학적으로 불가능하다고 결론내지 않는다. 모든 pose가 IK 가능하다고 증명된 것도 아니다.

동결된 5 ms IK search budget에서 KDL numerical IK와 random reseeding을 포함한 탐색이 비결정적으로 종료되는 현상이 관측되었다. 특정 ProcessPath pose의 기하학적 도달 불가능성은 입증되지 않았다.

`0.8172 m` heuristic과 shoulder distance는 IK reachability 증거가 아니다.

시작 `setFromIK`에는 0.005 s를 직접 넘겼다. MoveIt 2.12.4 `CartesianInterpolator`는 경로 IK에 timeout 0.0을 넘기고, `RobotState`는 그 값을 group의 default IK timeout으로 바꾼다. solver는 `KDLKinematicsPlugin`이며 첫 시도 뒤에 `getRandomConfiguration`으로 다시 seed한다. 충돌 검사와 joint jump 검사는 꺼져 있다.

repair는 고르지 않았다. P-023은 열지 않았다.
