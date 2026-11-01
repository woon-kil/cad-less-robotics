# 경로 IK continuity repair 계약

```text
P022_PATH_IK_CONTINUITY_CONTRACT = PASS_CONTRACT_FROZEN
THRESHOLD_SELECTION = SEPARATION_CHECK_ONLY
THRESHOLD = NOT_DEFINED

PURPOSE =
reject clearly non-local IK solutions during
Cartesian interval construction

MUST_PRESERVE =
known recoverable solutions
0.0593845 rad direct PASS
0.402594 rad PASS_AFTER_ONE_SUBDIVISION
run20 repeats 0-3 path transitions
0.00746 .. 0.03915 rad direct PASS

MUST_REJECT =
captured pathological distant solutions
failure-chain samples at or above 1.85667 rad

MUST_NOT_CHANGE =
ProcessPath
datum
eef_step
Cartesian acceptance threshold
CartesianPrecision
max_resolution
solver
start IK behavior

DISTANCE_METRIC =
max bounded-revolute absolute joint difference
ANGLE_WRAPPING = DISABLED

THRESHOLD = NOT_DEFINED

UNOBSERVED_CANDIDATE_REGION =
0.402594 < T < 1.85667
THIS_IS_NOT = VALID_THRESHOLD_RANGE

VALIDATION = independent from baseline acceptance

REPAIR_CANDIDATE = PATH_IK_SEED_DISTANCE_VALID_CALLBACK
CANDIDATE_STATUS = SELECTED_FOR_EVALUATION
PROXY_VALIDITY = NOT_ESTABLISHED
CALLBACK_APPLIED = NO
REPAIR_APPLIED = NO

PARTIAL_FRACTION_REPAIR = this contract
START_IK_FAILED_REPAIR = NOT_SELECTED
THE_TWO_ARE_INDEPENDENT = YES

P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

이 계약은 숫자를 보기 전에 보존 대상과 거부 대상을 고정한다. `0.402594 < T < 1.85667`은 현재 표본이 비어 있는 구간이다. 그 안의 값을 threshold로 써도 된다는 뜻이 아니다.

joint 차이와 Cartesian 중간점 오차의 단조 관계는 성립하지 않았다. 가장 큰 위치 오차는 0.02658 rad에서 나왔고, 0.03915 rad에서는 아니었다.

시작 IK는 이 계약의 대상이 아니다. partial fraction에 대한 변경이 시작 IK 실패를 고친다고 두지 않는다.

callback은 적용하지 않았다.
