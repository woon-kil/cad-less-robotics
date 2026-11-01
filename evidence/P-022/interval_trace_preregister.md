# interval trace 사전 등록

```text
P022_INTERVAL_TRACE = DIAGNOSTIC_ONLY
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
CARTESIAN_PRECISION = CartesianPrecision 기본값
MAX_RESOLUTION = 그 기본값, 변경하지 않음

INSTRUMENTATION = SOURCE_EQUIVALENT_COPY
LIBRARY computeCartesianPath CALLED = NO
REASON = validateAndImproveInterval은 libmoveit_robot_state.so의
anonymous namespace 안에 있고 callback이 없다.
복사본은 MoveIt 2.12.4 cartesian_interpolator.cpp의
validateAndImproveInterval, 단일 구간 루프, waypoint 누적 루프와
같은 순서로 setFromIK와 FK 중간점 검사를 호출한다.

CAPTURE WHEN 0 < fraction < 1
STOP at the first such run
START_IK_FAILED = record, do not capture, continue
fraction == 0 = FRACTION_ZERO, do not capture, continue
fraction == 1 = continue

PREVIOUS_CAPTURE_RUN12 = NOT_REPLAYED
이 실행의 fraction은 그 9/34 기록의 재생이 아니다.

CAUSAL_CHAIN = OBSERVED only if the captured run has one waypoint interval
where all of the following are in that same interval:
- a check with position error > translational precision
  or orientation error > rotational precision
- a later check with width < max_resolution and validation false
- the returned fraction equals
  completed_intervals / waypoint_count
  when that interval contributes 0

JOINT_DISCONTINUITY_AT_VIOLATION = OBSERVED
only if a precision-violating check in that interval has
max absolute joint difference between its two endpoint states > 0.5 rad
This 0.5 rad rule is the existing branch observation rule.
It is not a new acceptance threshold and not a repair.

JOINT_BRANCH_JUMP_CAUSALITY = OBSERVED_IN_INTERVAL
only if CAUSAL_CHAIN and JOINT_DISCONTINUITY_AT_VIOLATION are both met.
Otherwise NOT_PROVEN.

IF not captured within 20 trials:
PARTIAL_FAILURE_NOT_REPRODUCED_WITHIN_FIXED_TRIALS
CAUSAL_CHAIN = NOT_PROVEN

JOINT_LIMIT_CAUSE = NOT_SUPPORTED unless a failed path IK seed is within 1e-6 rad of a URDF bound
JOINT_LIMIT_INVOLVEMENT_RULED_OUT = NO
SINGULARITY_ACCEPTANCE_THRESHOLD = NOT_DEFINED

DO NOT change datum, eef_step, orientation, ProcessPath, solver, seed policy, timeout, max_resolution
DO NOT overwrite cartesian_runs.json
DO NOT open P-023
DO NOT select a repair
```
