# partial Cartesian failure capture

```text
P022_PARTIAL_FAILURE_CAPTURE = PASS_PARTIAL_FAILURE_CAPTURED

MAX_TRIALS = 20
TRIALS_USED = 2
RUNS_EXECUTED = 34
START_IK_FAILED_COUNT = 9
FULL_FRACTION_COUNT = 24
FRACTION_ZERO_COUNT = 0

CAPTURED_TRIAL = 1
CAPTURED_RUN_ID = 12
RETURNED_FRACTION = 0.264706
EXACT_RATIO = 9/34

START_IK_TIMEOUT_S = 0.005
PATH_IK_TIMEOUT_S = 0.5
SET_DEFAULT_IK_TIMEOUT = NO

FIRST_FAILED_PATH_IK_CALL = NONE
JOINT_LIMIT_CAUSE = NOT_SUPPORTED
SINGULARITY_CAUSE = NOT_SUPPORTED
SINGULARITY_ACCEPTANCE_THRESHOLD = NOT_DEFINED

BASELINE_RUN20_FRACTION_REPRODUCED = NO
REPAIR_SELECTED = NO
P022_FINAL_GATE = FAIL_CARTESIAN_API
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

사전 상한은 20 trial이다. `START_IK_FAILED` 9회는 capture로 세지 않고 넘어갔다. fraction 0은 없었다. 기준선 `cartesian_runs.json`은 바꾸지 않았다.

이번 capture는 기준선 run 20의 0.606061이 아니다. run 12가 fraction 0.264706으로 멈춘 한 번의 실행이다.

시작 IK는 성공했다. zero seed에서 결과 joint까지 최대 차이는 5.70568 rad다. 경로 IK 호출 28개는 모두 `ik_success = true`다. false를 반환한 경로 IK는 없다.

경로 step의 seed 대비 최대 joint 차이는 앞에서 0.0132–0.0464 rad였고, call 11에서 6.2712 rad, call 12에서 9.7871 rad가 된다. call 12 이후 target은 한 구간의 Cartesian 중간점을 반복해서 반으로 나눈 값이고, call 25–28에서 `[0.364471, 0.0118057, 0.162063]` m로 모인다. 그 구간의 joint 차이는 3.1416 rad로 반복된다. waypoint 비율은 9/34에서 멈췄다.

마지막 성공 해의 limit까지 거리는 0.00692409 rad다. 관측된 최소 거리는 call 13의 0.00484 rad다. `1e-6` rad 이하인 joint는 없다. 실패한 경로 IK seed가 없으므로 `JOINT_LIMIT_CAUSE = NOT_SUPPORTED`.

성공 해의 Jacobian 최소 singular value는 0.123654–0.163372다. 합격 기준은 없다. `SINGULARITY_CAUSE = NOT_SUPPORTED`. 이 구간의 값이 낮지 않았다는 뜻이지, singularity-free 증명은 아니다.

repair는 고르지 않았다.

9/34가 IK 호출 수가 아니라는 반환 경로는 `fraction_termination.md`에 있다.
