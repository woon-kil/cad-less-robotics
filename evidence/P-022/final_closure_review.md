# P-022 종료 검토

```text
P022_FINAL_CLOSURE_REVIEW = PASS_FINAL_CLOSURE_REVIEW
P022_CLOSURE_APPLICATION = PASS_CLOSURE_APPLIED

IMPLEMENTATION_EXECUTED = YES
ACCEPTANCE_MET = NO
JUSTIFIABLE_REPAIR_REMAINING = NO_CURRENTLY_SELECTED_REPAIR
NO_CURRENTLY_SELECTED_REPAIR = YES

P022_STATUS = CLOSED_FAIL
RECOMMENDED_CLOSURE = CLOSED_FAIL
CLOSURE_APPLIED = YES
RECORDED_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P022_REVIEWER_CLOSED = true

REPAIR_SEARCH = STOPPED
NEW_PARAMETER_EXPERIMENT = NO

PARTIAL_FRACTION_REPAIR = NO_JUSTIFIABLE_REPAIR
SCALAR_DISTANCE_CALLBACK = RECONSIDER
START_IK_REPAIR = NO_JUSTIFIABLE_START_IK_REPAIR
REPAIR_APPLIED = NO

P023 = NOT_OPENED
P024 = NOT_OPENED
ALTERNATIVE_PLANNING_GATE = NOT_OPENED
SUCCESSOR_GATE_DECISION = NOT_YET_MADE
NEXT_ALLOWED_GATE = NONE
```

새 실험은 하지 않았다. 아래 세 판정은 이미 있는 evidence만 읽은 결과다.

## 1. 구현과 실행

`IMPLEMENTATION_EXECUTED = YES`.

P-013 ProcessPath 741개를 m로 바꾸고, 동결된 simulation datum으로 `base_link`의 `RobotPlanningInput`을 만들었다. UR5e `ur_manipulator`, `tool0`을 적재했다. 연속 PROCESS run 21개에 MoveIt Cartesian planning을 호출했다. TRANSITION 동작, RobotTrajectory, 실물 execute는 만들지 않았다. 이 범위는 P-022 계약의 출력과 같다.

모델 적재, 단위 변환, frame 변환, planning representation은 실행 기록이 PASS다. 이 PASS는 Gate 합격이 아니다.

## 2. 수용 기준

`ACCEPTANCE_MET = NO`.

계약은 21개 PROCESS run마다 fraction이 0.95 이상일 때만 `PASS_ROS2_MOVEIT2_ADAPTER`다. 평균으로 대신하지 않는다.

기준선은 run 5, 7, 12가 `START_IK_FAILED`다. 그 세 run은 fraction이 아니므로 `MIN_PROCESS_RUN_FRACTION`을 계산하지 않았다. fraction이 반환된 18개 중 최솟값은 run 20의 0.606061이고 0.95보다 작다. 기록된 최종 게이트는 `FAIL_CARTESIAN_API`다. 반환된 fraction만 봐도 `FAIL_CARTESIAN_PLANNING`이다.

## 3. 남은 repair

`JUSTIFIABLE_REPAIR_REMAINING = NO_CURRENTLY_SELECTED_REPAIR`.

시작 IK는 timeout 민감도가 관측됐다. 5 ms를 다른 값으로 바꿀 독립 기준은 없다. 20 ms는 진단 관측이고 repair 값이 아니다. 5 ms가 최적이라는 뜻도 아니다.

partial fraction은 종료 경로, interval 원인, 거리 proxy, 독립 calibration, threshold 출처, 대안 비교까지 갔다. 동결된 `CartesianPrecision` 안에서 적용할 repair는 고르지 못했다. scalar 거리 callback은 `RECONSIDER`로 남아 있고, `T`는 없으며 선택된 repair가 아니다.

두 class 모두 계약을 유지한 채 적용할 변경이 없다. 추가 timeout sweep, threshold sweep, precision 변경, solver 교체는 이 Gate에서 하지 않는다.

## 권고

새 사실이 없다. 권고 형태는 `CLOSED/FAIL`이다. PASS로 바꾸지 않는다. 구현이 실행된 것과 수용 기준을 만족하지 못한 것은 별개다.

Reviewer가 `CLOSED_FAIL`을 승인했다. `CLOSURE_APPLIED = YES`다. `reviewer_closed`는 true다. `FAIL_CARTESIAN_API`는 실행 결과의 이름이고, 이 승인으로 바꾸지 않는다. PASS로 바꾸지 않는다.

P-023과 P-024는 열지 않는다. 대체 planning Gate도 열지 않는다. 다음 Gate는 정하지 않았다. `NEXT_ALLOWED_GATE = NONE`은 후속을 거부한 결정이 아니다. 후속 roadmap 결정이 아직 없다는 뜻이다.

closure 적용 검수는 `PASS_CLOSURE_APPLIED`다. 이 Gate의 진단과 측정 기록은 다시 열지 않는다. successor 선택은 이 문서 밖이다.
