# run 20 joint trace

```text
PURPOSE = P022_CARTESIAN_FAILURE_DIAGNOSIS
NOT = P023_IK_ACCEPTANCE

RUN_ID = 20
REPEAT_COUNT = 5
SET_DEFAULT_IK_TIMEOUT = NO
START_IK_TIMEOUT_S = 0.005
PATH_IK_TIMEOUT_S = 0.5
DEFAULT_GROUP_IK_TIMEOUT_S = 0.5

P022_RUN20_JOINT_TRACE = PASS_DIAGNOSTIC

BRANCH_DIVERGENCE = OBSERVED
BRANCH_DIVERGENCE_LOCATION = START_IK
PATH_LOCAL_CONTINUITY = OBSERVED
BRANCH_DIVERGENCE_CAUSED_PARTIAL_FRACTION = NOT_CLAIMED

JOINT_LIMIT_CAUSE = NOT_SUPPORTED
SINGULARITY_CAUSE = NOT_SUPPORTED
SINGULARITY_ACCEPTANCE_THRESHOLD = NOT_DEFINED

PARTIAL_FRACTION_REPRODUCED = NO
MID_PATH_IK_FAILURE_REPRODUCED = NO
REPAIR_SELECTED = NO
P022_FINAL_GATE = FAIL_CARTESIAN_API
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

사전 규칙은 `run20_joint_trace_preregister.md`다. datum, `eef_step`, orientation, ProcessPath, solver, seed policy, threshold, timeout은 바꾸지 않았다. group default IK timeout은 덮어쓰지 않았다.

호출 기록에서 시작 IK timeout은 0.005 s, 경로 IK timeout은 0.5 s였다.

```text
repeat 0  START success  fraction 1  path IK failures 0
repeat 1  START success  fraction 1  path IK failures 0
repeat 2  START success  fraction 1  path IK failures 0
repeat 3  START success  fraction 1  path IK failures 0
repeat 4  START_IK_FAILED          fraction null
```

이 다섯 번은 기준선의 run 20 fraction 0.606061을 다시 만들지 않았다. 경로 IK가 중간에서 실패한 호출은 없다.

## branch

같은 call index의 성공 joint를 비교했다. 사전 규칙인 max absolute difference 0.5 rad를 넘는 쌍이 있다. 가장 큰 값은 call 41에서 repeat 0과 1 사이 8.88166 rad다.

시작 해만 보면 갈라짐은 첫 해에서 이미 있다.

```text
repeat 0 vs 1  max abs 8.6012 rad
repeat 0 vs 2  max abs 6.2832 rad
repeat 0 vs 3  max abs 6.2832 rad
```

repeat 0과 2, 0과 3의 6.2832 rad는 일부 joint의 `2π` 차이다. repeat 1은 그 패턴이 아니다. shoulder, elbow, wrist가 함께 다르다.

경로 IK가 성공한 164회에서 seed 대비 max absolute joint delta는 0.00745–0.03915 rad다. 0.5 rad를 넘는 경로 step은 없다. 한 번 고른 시작 해 위에서 다음 step은 그 joint state 근처에 남는다.

repeat 4는 시작 IK가 실패해서 경로 joint가 없다. 실패 seed는 `setToDefaultValues`이고, limit까지 남은 거리는 3.1416 rad다.

## limit과 Jacobian

성공 해의 URDF limit까지 최소 거리는 0.170778 rad다. `1e-6` rad 이하인 joint는 없다. `JOINT_LIMIT_CAUSE = NOT_SUPPORTED`.

Jacobian 최소 singular value는 기록만 했다. 성공한 반복에서 관측 범위는 0.153495–0.249582다. 합격 기준은 없다. `SINGULARITY_CAUSE = NOT_SUPPORTED`.

KDL 내부 attempt 횟수는 기록하지 않았다. 같은 zero seed에서 서로 다른 시작 joint가 나온 것은 관측이다. 그 호출 안에서 random reseed가 몇 번이었는지는 이 기록이 아니다.
