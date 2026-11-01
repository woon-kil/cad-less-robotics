# P-022 진단 해석 추가

기준선 `cartesian_runs.json`과 `failure_diagnosis.md`의 관측값은 바꾸지 않는다. `FINAL_GATE`는 `FAIL_CARTESIAN_API`다.

시작 IK 실패와 partial Cartesian fraction은 같은 5 ms timeout 문제가 아니다.

```text
START_IK_FAILED
→ 5 ms explicit start-pose IK search와 관련

PARTIAL_CARTESIAN_FRACTION
→ Cartesian interpolation 내부 IK progression에서 발생
→ effective IK timeout은 별도 group-default semantics를 따름
→ 5 ms start-IK timeout만으로 설명되지 않음
```

기준선 binary는 시작 `setFromIK`에 0.005 s를 넘겼다. `computeCartesianPath` 안의 `setFromIK`는 timeout 0.0을 넘기고, `RobotState`는 그것을 group default로 바꾼다. 그 default의 초기 관측값은 0.5 s다. 기준선은 `setDefaultIKTimeout`을 호출하지 않았다.
