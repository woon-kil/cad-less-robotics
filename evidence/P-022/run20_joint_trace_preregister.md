# run 20 joint trace 사전 등록

```text
PURPOSE = P022_CARTESIAN_FAILURE_DIAGNOSIS
NOT = P023_IK_ACCEPTANCE
NOT = REPAIR

RUN_ID = 20
REPEAT_COUNT = 5

START_IK_TIMEOUT_S = 0.005
SET_DEFAULT_IK_TIMEOUT = NO
PATH_IK_EFFECTIVE_TIMEOUT = group default, expected 0.5 s

EEF_STEP_M = 0.005
EEF_ROTATION_STEP_RAD = 0
SEED_POLICY = setToDefaultValues
SOLVER = KDL
AVOID_COLLISIONS = false
JUMP_DETECTION = not applied by the CartesianPrecision overload

DATUM = unchanged
PROCESSPATH = unchanged
THRESHOLD = unchanged

BRANCH_DIVERGENCE_RULE =
OBSERVED if any shared IK call index has max absolute joint difference > 0.5 rad
between any pair of successful solutions.
Otherwise NOT_OBSERVED.

AT_LIMIT_RULE =
a joint is AT_LIMIT if distance to its URDF bound is <= 1e-6 rad.
JOINT_LIMIT_CAUSE = SUPPORTED only if the seed of a failed IK call is AT_LIMIT.
Otherwise NOT_SUPPORTED.

SINGULARITY_ACCEPTANCE_THRESHOLD = NOT_DEFINED
SINGULARITY_CAUSE = NOT_SUPPORTED
```

Jacobian의 최소 singular value와 condition은 기록만 한다. 이 숫자로 합격 기준을 만들지 않는다.
