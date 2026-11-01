# partial failure capture 사전 등록

```text
P022_PARTIAL_FAILURE_CAPTURE = DIAGNOSTIC_ONLY
NOT = REPAIR
NOT = P023_IK_ACCEPTANCE

MAX_TRIALS = 20
TRIAL = process_runs.txt의 21개 PROCESS run을 한 번 순서대로 실행

START_IK_TIMEOUT_S = 0.005
SET_DEFAULT_IK_TIMEOUT = NO
PATH_IK_TIMEOUT = group default, expected 0.5 s
EEF_STEP_M = 0.005
EEF_ROTATION_STEP_RAD = 0
SEED_POLICY = setToDefaultValues
SOLVER = KDL

CAPTURE WHEN 0 < fraction < 1
STOP at the first such run

START_IK_FAILED = record, do not capture, continue
fraction == 0 = record as FRACTION_ZERO, do not capture, continue
fraction == 1 = continue

IF captured: PASS_PARTIAL_FAILURE_CAPTURED
IF not captured within 20 trials: PARTIAL_FAILURE_NOT_REPRODUCED_WITHIN_FIXED_TRIALS

SINGULARITY_ACCEPTANCE_THRESHOLD = NOT_DEFINED
AT_LIMIT = distance to URDF bound <= 1e-6 rad
JOINT_LIMIT_CAUSE = SUPPORTED only if the seed of the first failed path IK is AT_LIMIT
Otherwise NOT_SUPPORTED for this capture

DO NOT change datum, eef_step, orientation, ProcessPath, solver, seed policy, timeout
DO NOT overwrite cartesian_runs.json
DO NOT open P-023
DO NOT select a repair
```
