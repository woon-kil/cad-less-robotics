# P-025 contract redefinition

```text
P025_CONTRACT_REVIEW = PASS_CONTRACT_FROZEN
P025_CONTRACT_REDEFINITION = WRITTEN_PENDING_REVIEW
P025_CONTRACT_FROZEN = YES
PASS_CONTRACT_FROZEN = YES
SUPERSESSION_EFFECTIVE = YES
P025_GATE = OPEN
P025_CONTRACT = FROZEN
P025_SCOPE = VERIFIED_END_TO_END_ENGINEERING_EVIDENCE_VISUALIZATION
P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW
P025_V01_V10 = PASS
P025_DATA_FIDELITY = PASS
P025_UPSTREAM_IMMUTABILITY = PASS
P025_VISUAL_RENDER_REVIEW = IN_PROGRESS
P025_RENDER_SCREENSHOTS = CAPTURED
VISUAL_FINDING_01 = ROBOTTRAJECTORY_CREATED_VALUE_NO_OVERLAPS_LABEL
VISUAL_FINDING_01_CLASSIFICATION = FUNCTIONAL_READABILITY_DEFECT
VISUAL_FINDING_02 = VIEWPORT_390_SCENE5_CLIPS_FINAL_GATE_AND_PARTIAL_EVIDENCE
VISUAL_FINDING_02_CLASSIFICATION = FUNCTIONAL_READABILITY_DEFECT
AC09_IMPACT = SUPPORTED
AC09_FINAL = FAIL
AC10 = BLOCKED
P025_AC01_AC08 = PASS
P025_AC01_AC10 = REVIEWER_EVALUATED
P025_FIRST_REVIEWER_EVALUATION = COMPLETE
P025_REPAIR_SCOPE = DEFINED
P025_REPAIR_IMPLEMENTATION = IMPLEMENTED_PENDING_AC09_REREVIEW
AC09_REPAIR = VIEWER_PRESENTATION_ONLY
AC10_CAPTURE_INPUT = BLOCKED
AC10_UNBLOCK = V10_MANIFEST_MATCH
UPSTREAM_EVIDENCE = READ_ONLY
P025_PASS = NOT_EVALUATED
P025_DEFINITION_CHANGED = NO
P025_EXISTING_DEFINITION = SUPERSEDED_TEXT_RETAINED
NEXT_ALLOWED_GATE = P025
P025_GATE_OPEN_REVIEW = PASS_GATE_OPEN
P025_IMPLEMENTATION_PLAN_REVIEW = PASS_IMPLEMENTATION_PLAN
P025_IMPLEMENTATION_PLAN = APPROVED
NEXT_ALLOWED_WORK = P025_VISUAL_RENDER_REVIEW
P023 = NOT_OPENED
P024 = NOT_OPENED

POST_FEASIBILITY_ROADMAP_DECISION_REVIEW = PASS_POST_FEASIBILITY_ROADMAP_DECISION
DECISION_OPTION = B
```

이 문서는 동결된 계약이다. 역사 정의 본문은 roadmap에 그대로 둔다. 삭제하지 않는다.

목적 한 문장: 검증된 CAD-to-robot integration pipeline의 성공 구간과 실패 경계를, 원본 evidence를 바꾸지 않고 시각적으로 전달한다.

이 Gate는 새 robotics validation이 아니다. 예쁜 화면을 만드는 일도 아니다. 합격은 evidence fidelity다.

## 역사 정의와의 관계

roadmap의 `### P-025 Portfolio End-to-End Visualization`은 역사 정의다. 진입은 P-024 PASS였고, 이야기에는 IK, collision validation, RobotTrajectory가 들어 있다. roadmap decision B가 그 정의를 이 계약으로 대체했다. 역사 본문은 남긴다. 계약 동결 뒤 별도 전이로 Gate를 열었다. 구현은 시작됐다. 역사 본문은 바꾸지 않는다.

```text
HISTORICAL_DEFINITION = RETAINED
SUPERSESSION_SOURCE = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
SUPERSESSION_EFFECTIVE = YES
```

## 보여줄 경계

```text
VERIFIED
CAD Geometry
  → 6D ProcessPath
  → RobotPlanningInput
  → MoveIt Integration

NOT ACHIEVED
Accepted Cartesian Planning
RobotTrajectory
Collision Validation
Physical Execution
```

화면이 CAD에서 robot으로 끝나며 RobotTrajectory가 만들어진 것처럼 보이면 실패다.

## Scene

다섯 장면만 쓴다. 각 숫자는 아래 출처 그대로다. 없는 숫자는 만들지 않는다.

### SCENE-1 CAD INPUT

P-011 Complex Surface Test Coupon. 파일은 `assets/cad/complex_surface_coupon.step`다. 역할은 `CONTROLLED_TEST_COUPON`이고 `REFERENCE_GEOMETRY`다. `GROUND_TRUTH = NONE`이다. 측정 표면의 참값이 아니다.

### SCENE-2 GEOMETRY

P-012의 surface, normal, curvature만 보여 준다. 표시 값은 그 Gate evidence에 있는 것만 쓴다. 지원되지 않는 geometry 주장을 넣지 않는다.

### SCENE-3 PROCESS PATH

P-013 6D ProcessPath. `surface_anchor`, `command_position`, tool orientation을 구분한다. `command_position`은 surface anchor에 법선 방향 standoff를 더한 점이다. ProcessWaypoint를 IK goal이나 trajectory point로 보이지 않게 한다.

`command_position - (surface_anchor + surface_normal * 20)`의 최대 잔차는 P-021 보고서의 `1.000017846308765e-09 mm`다. 이 잔차는 계약식과 저장 수치의 일치다. 물리 정확도도, 로봇 위치 오차 0도, ground truth도 아니다. 화면에서 0 mm 물리 정확도로 줄여 적지 않는다.

Waypoint 수는 741이다. 출처는 P-021 보고서가 가리키는 P-013 `cad_process_path.json`이다.

### SCENE-4 ROBOT ADAPTER

P-021은 계약이다. 입력 frame은 `CAD_MODEL_FRAME`, 위치 단위는 mm, 식은 `x_m = x_mm * 0.001`이다. P-021은 그 변환을 실행하지 않았고 `RobotPlanningInput` 파일을 만들지 않았다.

P-022가 그 계약을 동결된 simulation datum으로 `base_link` 입력에 적용했다. 그 datum은 `CONFIGURED_SIMULATION_DATUM`이다. 측정 transform, calibration, ground truth가 아니다. UR5e는 planning reference robot이고, `tool0`은 planning reference TCP다. 실물 공정 공구 TCP가 아니다.

### SCENE-5 PLANNING BOUNDARY

출처는 `evidence/P-022/cartesian_runs.json`과 `evidence/P-022/failure_diagnosis.md`다. 민감도 실험 숫자는 이 기준선으로 쓰지 않는다.

PROCESS run은 run 0부터 run 20까지 21개다.

```text
fraction = 1          17
START_IK_FAILED       run 5, run 7, run 12
partial               run 20, fraction 0.606061, exact ratio 20/33
```

`START_IK_FAILED`는 fraction 0이 아니다. 최종 게이트는 `FAIL_CARTESIAN_API`다. `P022 = CLOSED_FAIL`이다. 합격 조건은 PROCESS run마다 fraction 0.95 이상이고, 평균으로 대신하지 않는다. 0.606061은 그 기준 아래다.

이어서 feasibility를 같은 장면에 둔다. `FEASIBILITY_VERDICT = NO_JUSTIFIABLE_ALTERNATIVE`다. 뜻은 동결된 P-022 문제와 조사한 후보 안에서 구현 Gate를 열 대안이 없다는 것이다. 계획 불가능, UR5e 실행 불가능, Cartesian path 불가능, 실물 실행 불가능이 아니다. `KDL_SCALAR_CALLBACK = RECONSIDER`는 그 판정과 같이 둘 수 있다. 구현되었다는 뜻이 아니다.

P-023과 P-024 결과는 없다. 없는 결과를 암시하지 않는다.

## MUST_NOT_CLAIM

```text
RobotTrajectory generated
trajectory valid
collision-free
all ProcessPath poses reachable
UR5e can physically execute the path
physical calibration completed
physical accuracy validated
production-ready
ground truth established
```

`REFERENCE_GEOMETRY`와 `MEASURED_GEOMETRY`를 한 값으로 섞지 않는다. `GROUND_TRUTH = NONE`을 유지한다.

## Acceptance

열 항목이 모두 맞을 때만 시각화 합격을 검토한다. 하나라도 실패하면 robotics 결과를 고쳐 맞추지 않는다. 화면을 고친다.

```text
AC-01  표시된 정량 값은 모두 동결 evidence로 거슬러 간다.
AC-02  성공 주장이 출처 Gate의 범위를 넘지 않는다.
AC-03  P-022 CLOSED_FAIL이 화면에 드러난다.
AC-04  17 full, 3 START_IK_FAILED, partial 1개 0.606061이 기준선과 같다.
AC-05  NO_JUSTIFIABLE_ALTERNATIVE가 위의 좁은 의미로 보인다.
AC-06  ProcessPath와 RobotTrajectory가 화면에 구분되고, 후자는 NOT ACHIEVED다.
AC-07  REFERENCE_GEOMETRY, MEASURED_GEOMETRY, GROUND_TRUTH=NONE이 유지된다.
AC-08  P-023, P-024 결과가 있는 것처럼 보이지 않는다.
AC-09  repository evidence를 읽지 않아도 성공 구간과 실패 경계가 이해된다.
AC-10  P-011부터 P-022 evidence와 이 roadmap의 역사 P-025 본문을 바꾸지 않는다.
```

AC-09는 면접관이 evidence 파일부터 읽어야 하면 실패다.

## 아직 하지 않는 것

P-023, P-024, 새 실험, datum과 threshold 변경은 이 문서에 없다. 구현은 승인된 계획대로 시작됐다. `P025_PASS`는 아직 평가하지 않는다. 이 Gate는 frozen evidence를 읽기만 한다.
