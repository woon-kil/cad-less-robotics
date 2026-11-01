# Feasibility final review

```text
FEASIBILITY_FINAL_REVIEW = NO_JUSTIFIABLE_ALTERNATIVE
ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW = CLOSED
FEASIBILITY_REVIEWER_ACCEPTED = true
JUSTIFIABLE_ALTERNATIVE_FOUND = NO
TYPE = EVIDENCE_SYNTHESIS_ONLY
NEW_EXPERIMENT = NOT_ALLOWED
NEW_PACKAGE_INSTALLATION = NOT_ALLOWED
IMPLEMENTATION = NOT_ALLOWED
P022_REOPEN = NO
P023 = NOT_OPENED
P024 = NOT_OPENED
ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED

BRANCH_CONTINUITY_ALTERNATIVE_REVIEW = NO_JUSTIFIABLE_CANDIDATE
BRANCH_CONTINUITY_HYPOTHESIS = NOT_PROVEN
DIAGNOSTIC_GATE_CANDIDATE = NONE_FROM_THIS_REVIEW

IKFAST = NOT_FEASIBLE_FOR_THIS_ENVIRONMENT
PICK_IK_DEFAULT = NO_MECHANISM_IMPROVEMENT
LMA = AVAILABILITY_NOT_ESTABLISHED
KDL_SCALAR_CALLBACK = RECONSIDER

CANDIDATES_MEETING_ALL_SIX = 0
FEASIBILITY_VERDICT = NO_JUSTIFIABLE_ALTERNATIVE
VERDICT_SCOPE = FROZEN_P022_PROBLEM_AND_INVESTIGATED_CANDIDATES

NO_SOLUTION_EXISTS = NOT_CLAIMED
MOVEIT_CANNOT_PLAN_THIS_PATH = NOT_CLAIMED
UR5E_CANNOT_EXECUTE_THIS_PATH = NOT_CLAIMED
CARTESIAN_PATH_IS_IMPOSSIBLE = NOT_CLAIMED
PHYSICAL_EXECUTION_IS_IMPOSSIBLE = NOT_CLAIMED

SUCCESSOR_AFTER_FEASIBILITY = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
NEXT_ALLOWED_GATE = NONE
POST_FEASIBILITY_ROADMAP_DECISION = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
DECISION_OPTION = B
NEXT_ALLOWED_WORK = P025_CONTRACT_REDEFINITION
```

새 측정은 없다. 이미 있는 evidence만 여섯 조건에 대조했다.

`JUSTIFIABLE_ALTERNATIVE_FOUND`는 다음이 모두 YES일 때만 해당한다.

1. Jazzy와 UR5e에서 그 대안을 실제로 쓸 수 있다.
2. 포착된 pathological case가 개선된다.
3. known-good regression이 보존된다.
4. acceptance를 임의로 느슨하게 하지 않는다.
5. P-022 baseline이 보존된다.
6. 구현 경로를 다시 만들 수 있다.

YES가 아닌 칸은 그 조건을 만족한 것이 아니다.

| 후보 | 1 사용 가능 | 2 pathological | 3 known-good | 4 acceptance 유지 | 5 P-022 보존 | 6 재현 경로 | 6/6 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| IKFast | NO. 공식 생성이 OpenRave 검사에서 끝났고 plugin이 없다 | NOT_MEASURED | NOT_MEASURED | YES. acceptance를 바꾸지 않았다 | YES | NO. benchmark solver가 없다 | NO |
| pick_ik default | YES. `PickIkPlugin` 적재와 seed consistency 60/60 | NO. `max_resolution` FAIL | NOT_ESTABLISHED. 단일 실행은 보존, stability는 아니다 | YES. precision, timeout, weight를 결과 뒤에 바꾸지 않았다 | YES | YES. 추출 deb와 같은 호출을 다시 만들 수 있다 | NO |
| LMA | NO. MoveIt 2.12.4 kinematics에 plugin이 없다 | NOT_MEASURED | NOT_MEASURED | NOT_TESTED | YES | NO | NO |
| KDL scalar callback | NO. API는 있으나 정당화된 `T`가 없어 대안으로 쓸 수 없다 | NOT_MEASURED | NOT_MEASURED. 첫 중간점 거절은 0.402594 rad known-good와 충돌한다 | NOT_JUSTIFIED | YES | NO. callback을 만들지 않았다 | NO |
| branch-continuity 검토 | 후보 없음 | 해당 없음 | 해당 없음 | 해당 없음 | YES | NO. diagnostic Gate 후보가 없다 | NO |

`computeCartesianPath`는 이미 이전 state를 다음 IK seed로 넘긴다. previous-q seeding을 새로 넣는 안은 현재 구조와 다른 대안이 아니다.

여섯 칸이 모두 YES인 후보는 없다. 이 조사 범위의 판정은 `NO_JUSTIFIABLE_ALTERNATIVE`다.

이 문장은 좁다. 동결된 P-022 문제, UR5e, Jazzy, MoveIt, `CartesianPrecision`과 acceptance를 유지한 채 조사한 후보 중에서 implementation Gate를 열 대안을 확보하지 못했다. 해가 없다는 뜻도, 이 경로의 계획이 불가능하다는 뜻도, UR5e 실행이 불가능하다는 뜻도, Cartesian path 자체가 불가능하다는 뜻도, 실물 실행이 불가능하다는 뜻도 아니다.

`KDL_SCALAR_CALLBACK = RECONSIDER`는 유지한다. 그 callback이 논리적으로 반증됐다는 뜻이 아니다. 지금 evidence로 구현 Gate를 열 만큼 정당화되지 않았다는 뜻과 함께 둔다.

P-023, P-024, 구현 Gate는 열지 않는다. P-022는 다시 열지 않는다.

이 판정은 검토에서 확정됐다. 이후 roadmap 선택은 B다. P-025는 열지 않는다.
