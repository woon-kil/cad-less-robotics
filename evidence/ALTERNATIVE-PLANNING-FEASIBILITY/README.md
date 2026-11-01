# Alternative planning feasibility

```text
ALTERNATIVE_PLANNING_FEASIBILITY_INITIALIZATION = PASS_FEASIBILITY_INITIALIZATION
SUCCESSOR_GATE_DECISION = ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW
ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW = CLOSED
PURPOSE = FIND_JUSTIFIABLE_SOLUTION_TO_P022_FAILURE
NEXT_ALLOWED_GATE = P025
SUCCESSOR_AFTER_FEASIBILITY = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
POST_FEASIBILITY_ROADMAP_DECISION = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
DECISION_OPTION = B
P025_GATE_OPEN_REVIEW = PASS_GATE_OPEN
P025_IMPLEMENTATION_PLAN_REVIEW = PASS_IMPLEMENTATION_PLAN
P025_IMPLEMENTATION_PLAN = APPROVED
NEXT_ALLOWED_WORK = P025_VISUAL_RENDER_REVIEW
P025_CONTRACT_REVIEW = PASS_CONTRACT_FROZEN
P025_CONTRACT = FROZEN
P025_CONTRACT_FROZEN = YES
P025_GATE = OPEN
P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW
P025_VISUAL_RENDER_REVIEW = IN_PROGRESS
AC09_IMPACT = SUPPORTED
AC09_FIRST_VERDICT = FAIL
AC09_REVIEW_AFTER_REPAIR = PASS
AC09_FINAL = PASS
AC10 = BLOCKED
P025_AC01_AC08 = PASS
P025_AC01_AC10 = REVIEWER_EVALUATED
P025_FIRST_REVIEWER_EVALUATION = COMPLETE
P025_REPAIR_SCOPE = DEFINED
P025_REPAIR_IMPLEMENTATION = IMPLEMENTED
AC09_REPAIR = VIEWER_PRESENTATION_ONLY
AC10_CAPTURE_INPUT = BLOCKED
AC10_UNBLOCK = V10_MANIFEST_MATCH
UPSTREAM_EVIDENCE = READ_ONLY
P025_PASS = NOT_EVALUATED
POST_FEASIBILITY_ROADMAP_DECISION_REVIEW = PASS_POST_FEASIBILITY_ROADMAP_DECISION
P025 = OPEN
P025_DEFINITION_CHANGED = NO
FEASIBILITY_FINAL_REVIEW = NO_JUSTIFIABLE_ALTERNATIVE
FEASIBILITY_REVIEWER_ACCEPTED = true
JUSTIFIABLE_ALTERNATIVE_FOUND = NO
BRANCH_CONTINUITY_ALTERNATIVE_REVIEW = NO_JUSTIFIABLE_CANDIDATE
BRANCH_CONTINUITY_HYPOTHESIS = NOT_PROVEN
DIAGNOSTIC_GATE_CANDIDATE = NONE_FROM_THIS_REVIEW
BRANCH_CONTINUITY_REVIEW_TYPE = SOURCE_API_REVIEW_ONLY
LMA_INSTALL_AND_TEST = NOT_OPENED
KDL_SCALAR_CALLBACK = RECONSIDER
PICK_IK_AVAILABLE_FOR_BENCHMARK = YES
DIAGNOSTIC_BENCHMARK_EXECUTED = YES
PICK_IK_DIAGNOSTIC_BENCHMARK = NO_MECHANISM_IMPROVEMENT
PICK_IK_DIAGNOSTIC_REVIEW = ACCEPTED
PICK_IK_KNOWN_GOOD_SINGLE_RUN_PRESERVED = YES
PICK_IK_KNOWN_GOOD_STABILITY = NOT_ESTABLISHED
PASS_MECHANISM_LEVEL_EVIDENCE = NO

P022 = CLOSED_FAIL_IMMUTABLE_BASELINE
P022_REOPENED = NO
P023 = NOT_OPENED
P024 = NOT_OPENED
ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED

CODE_WRITTEN = DIAGNOSTIC_BENCHMARK_ONLY
PACKAGE_INSTALLED = NO
FEASIBILITY_VERDICT = NO_JUSTIFIABLE_ALTERNATIVE
VERDICT_SCOPE = FROZEN_P022_PROBLEM_AND_INVESTIGATED_CANDIDATES

EXIT_RULE =
JUSTIFIABLE_ALTERNATIVE_FOUND
or
NO_JUSTIFIABLE_ALTERNATIVE
```

P-022를 다시 열지 않는다. 이 문서는 해결 가능성을 조사하는 범위만 연다. solver 교체나 callback 구현은 하지 않는다.

검토 순서는 고정한다.

```text
CANDIDATE-1 = IKFast / analytic multi-solution
CANDIDATE-1_STATUS = NOT_FEASIBLE_FOR_THIS_ENVIRONMENT
CANDIDATE-2 = pick_ik default
CANDIDATE-2_STATUS = NO_MECHANISM_IMPROVEMENT
CANDIDATE-2_LOAD_QUALIFICATION = PASS_LOAD_AND_SEED_CONSISTENCY
CANDIDATE-3 = LMA
CANDIDATE-3_STATUS = AVAILABILITY_NOT_ESTABLISHED
CANDIDATE-4 = KDL + scalar callback
CANDIDATE-4_STATUS = RECONSIDER
```

후보를 늘리지 않는다. apt에 보인 TRAC-IK는 이 목록에 넣지 않는다.

각 후보는 다음 여덟 가지를 본다. 코드로 fraction을 다시 측정하기 전에, 문서와 설치된 패키지만으로 답할 수 있는 것만 적는다.

1. ROS 2 Jazzy / MoveIt에서 실제 쓸 수 있는가
2. UR5e에 적용할 수 있는가
3. seed와 branch continuity를 어떻게 다루는가
4. P-022에서 포착한 failure mechanism을 직접 줄일 근거가 있는가
5. 새 임의 threshold가 필요한가
6. `CartesianPrecision` 검증을 유지할 수 있는가
7. P-022 acceptance를 그대로 재사용할 수 있는가
8. 구현 침습도

판정에 쓰는 oracle은 P-022의 기존 수치다. 그 파일을 고치지 않는다.

```text
KNOWN_GOOD
0.0593845 rad direct PASS
0.402594 rad PASS after one subdivision
calibration fraction-1 runs = 327
calibration transitions = 15572
M = 0.485861 rad
M_IS_THRESHOLD = NO

KNOWN_PATHOLOGICAL
captured interval
large branch divergence
large midpoint FK error
recursive refinement
max_resolution termination
```

대안의 성공은 fraction 1만으로 적지 않는다. 위 known-good가 유지되고, known pathological 종료가 줄며, 같은 acceptance에서 개선돼야 한다. pick_ik 세 fixture 측정은 `pick_ik_diagnostic_benchmark.md`에 있다. pathological 종료는 줄지 않았다.

## 설치 상태

호스트는 p620, ROS 2 Jazzy, `ros-jazzy-moveit-kinematics` 2.12.4다. 이 확인에서 패키지를 설치하지 않았다.

| 후보 | 이 호스트 |
| --- | --- |
| IKFast | 생성 스크립트와 plugin template이 있다. UR5e용 plugin 패키지는 apt 검색에 없었다. 설치된 예제는 PRBT뿐이다 |
| pick_ik | `ros-jazzy-pick-ik`가 apt에 있다. 설치되어 있지 않다 |
| LMA | 설치본 `moveit_kinematics` 파일에 LMA plugin이 없다 |
| KDL | 설치되어 있고, P-022가 사용한 solver다 |

IKFast template `ikfast61_moveit_plugin_template.cpp`는 joint limit 안의 해를 seed와의 거리로 정렬한다. `LimitObeyingSol`의 비교는 `dist_from_seed`다. 이 구조는 P-022의 먼 branch 선택과 같은 종류의 문제에 닿는다. UR5e plugin이 이 호스트에 없으므로, 그 정렬이 포착된 interval을 줄인다는 측정은 아니다.

pick_ik은 MoveIt 2용 IK plugin으로 apt에 있다. global/local optimization과 displacement 설정은 공식 문서의 설명이다. 세 fixture 측정에서 pathological 종료는 줄지 않았다.

LMA는 검토 순서의 세 번째다. 이 Jazzy 설치본에는 plugin 파일이 없다. 없는 파일을 설치된 대체 solver로 적지 않는다.

KDL scalar callback은 `RECONSIDER`다. threshold `T`는 없고, `CartesianPrecision` 경로는 유지한다. P-022에서 그 callback을 정당화할 출처는 찾지 못했다.

## 판정 조건

`JUSTIFIABLE_ALTERNATIVE_FOUND`는 아래가 모두 있을 때만 적는다.

1. UR5e, Jazzy, MoveIt에서 그 대안을 실제로 쓸 수 있다.
2. 포착된 pathological transition이 같은 Cartesian 의미에서 개선된다.
3. known-good가 유지된다. 0.0593845 rad는 바로 PASS, 0.402594 rad는 한 번 subdivision 뒤 PASS다.
4. acceptance를 느슨하게 해서 나온 개선이 아니다.
5. P-022의 동결 evidence를 다시 쓰지 않는다.
6. 구현 경로를 다시 만들 수 있다.

한 후보의 설치 실패만으로 `NO_JUSTIFIABLE_ALTERNATIVE`로 닫지 않는다. joint delta가 줄었다는 사실만으로 PASS하지 않는다. `CartesianPrecision` 결과까지 연결한다.

diagnostic benchmark는 741 waypoint acceptance가 아니다. 같은 UR5e, `base_link`, `tool0`, Cartesian pose, seed에서 KDL과 pick_ik을 비교했다. 측정 항목은 IK 성공, seed 대비 최대 joint delta, midpoint FK 위치·자세 오차, `CartesianPrecision` 결과, subdivision 깊이, `max_resolution` 종료다. 741 waypoint는 실행하지 않았다.

## 최종 대조

여섯 조건 대조는 `feasibility_final_review.md`다. YES가 여섯 칸인 후보는 없다. `FEASIBILITY_VERDICT = NO_JUSTIFIABLE_ALTERNATIVE`다. 범위는 동결된 P-022 문제와 이번 조사 후보다. 해가 없다거나 이 경로의 계획이 불가능하다는 뜻은 아니다. `KDL_SCALAR_CALLBACK = RECONSIDER`는 유지한다. 구현 Gate는 열지 않는다. IKFast 공식 생성은 OpenRave 자체 검사 wrong rate 0.996으로 끝났다. 그 표현은 이 환경에서 benchmark solver를 확보하지 못했다는 뜻이다. IKFast 기술 전체가 불가능하다는 뜻은 아니다.

pick_ik 기본 설정은 적재와 seed consistency는 통과했고, pathological fixture는 KDL과 같은 `max_resolution`으로 끝났다. feasibility 검토는 닫혔다. `NEXT_ALLOWED_GATE = NONE`이다. 계약은 `PASS_CONTRACT_FROZEN`이다. `P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW`다. `P025_PASS = NOT_EVALUATED`다.
