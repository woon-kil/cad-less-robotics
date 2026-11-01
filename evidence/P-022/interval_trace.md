# 실패 interval의 중간점 검사

```text
P022_INTERVAL_CAUSAL_TRACE = PASS_DIAGNOSTIC

CAPTURE_SOURCE = MOVEIT_2_12_4_EQUIVALENT_INSTRUMENTED_COPY
INSTRUMENTATION = SOURCE_EQUIVALENT_COPY
LIBRARY_COMPUTE_CARTESIAN_PATH_CALLED = NO
PREVIOUS_CAPTURE_RUN12 = NOT_REPLAYED

CAPTURED_TRIAL = 0
CAPTURED_RUN_ID = 7
WAYPOINT_COUNT = 34
COMPLETED_INTERVAL_COUNT = 22
FAILING_INTERVAL_INDEX = 22
RETURNED_FRACTION = 22/34 = 0.6470588235294118

PATH_IK_FALSE = NO
PRECISION_VIOLATION_COUNT = 17
MAX_RESOLUTION_FALSE_COUNT = 1
IK_FALSE_COUNT_IN_FAILING_INTERVAL = 0

CAUSAL_CHAIN = OBSERVED
JOINT_DISCONTINUITY_AT_VIOLATION = OBSERVED
JOINT_BRANCH_JUMP_CAUSALITY = OBSERVED_IN_INTERVAL
PARTIAL_FAILURE_MECHANISM = PROVEN_FOR_CAPTURED_RUN_7_INTERVAL_22
GENERALIZATION_TO_ALL_PARTIAL_FAILURES = NOT_PROVEN

JOINT_LIMIT_CAUSE = NOT_SUPPORTED
JOINT_LIMIT_INVOLVEMENT_RULED_OUT = NO
SINGULARITY_CAUSE = NOT_SUPPORTED
SINGULARITY_ACCEPTANCE_THRESHOLD = NOT_DEFINED

REPAIR_SELECTED = NO
P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

이번 실행은 앞의 run 12, 9/34 기록을 재생한 것이 아니다. `validateAndImproveInterval`은 라이브러리 안에 callback 없이 들어 있어서, 같은 순서의 복사본이 `setFromIK`와 중간점 FK를 호출했다. timeout, `eef_step`, precision, `max_resolution` 1e-5는 바꾸지 않았다. group default IK timeout은 0.5 s였다.

## 같은 interval 안의 연결

interval 22의 step 1에서 endpoint IK는 true였다. seed와 반환 joint의 최대 차이는 9.65012 rad였다.

그 두 joint state의 중간 FK와 Cartesian 중간 pose의 차이는 0.934953 m, 3.09003 rad였다. 기준은 0.001 m, 0.01 rad다. 검사는 구간을 나누고 중간 pose의 IK를 다시 호출했다. 그 IK도 true였다.

깊이 3 이후 한쪽 자식은 계속 어긋났다. 위치 오차는 0.1992 m, 방향 오차는 3.14159 rad에 머물렀다. 그 노드의 양끝 joint 차이는 6.28318 rad까지 커졌다. 폭은 0.5에서 시작해 절반씩 줄었다. 깊이 16에서 폭은 7.62939e-6이고, 1e-5보다 작아서 IK 호출 없이 `MAX_RESOLUTION`으로 false가 됐다.

같은 부모의 다른 자식 하나, 깊이 4, 폭 0.03125는 위치 오차 3.34373e-7 m, 방향 오차 1.1913e-6 rad, joint 차이 0.00401617 rad로 `PRECISION_MET`이었다. 실패는 interval 전체가 항상 어긋나서가 아니라, joint 차이가 큰 쪽이 최소 폭까지 줄어든 뒤다.

이 interval의 진행률은 0이다. 앞의 22개 interval이 1.0이었으므로 반환 fraction은 22/34이다.

## 한계

이 숫자는 라이브러리 함수 내부에 심은 기록이 아니다. 복사본의 제어 흐름에서 측정했다. 앞의 9/34 실행과 call 번호를 대응시키지는 않는다.

limit 거리는 이번 계측 항목이 아니다. 실패한 경로 IK가 없으므로 `JOINT_LIMIT_CAUSE = NOT_SUPPORTED`이고, limit 관여를 배제하지는 않는다. Jacobian 합격 기준은 없다.

repair는 고르지 않았다. `max_resolution`은 그대로다.
