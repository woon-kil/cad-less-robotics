# PROJECT-ROADMAP

```text
CANONICAL_GATE_PROCEDURE = YES
DOCUMENT_STATUS = WRITTEN_PENDING_REVIEW
EXECUTION_MODEL = BRANCHED
NEXT_ALLOWED_GATE = P025
SUCCESSOR_GATE_DECISION = ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW
ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW = CLOSED
ALTERNATIVE_PLANNING_FEASIBILITY_INITIALIZATION = PASS_FEASIBILITY_INITIALIZATION
FEASIBILITY_FINAL_REVIEW = NO_JUSTIFIABLE_ALTERNATIVE
FEASIBILITY_REVIEWER_ACCEPTED = true
FEASIBILITY_VERDICT = NO_JUSTIFIABLE_ALTERNATIVE
JUSTIFIABLE_ALTERNATIVE_FOUND = NO
VERDICT_SCOPE = FROZEN_P022_PROBLEM_AND_INVESTIGATED_CANDIDATES
SUCCESSOR_AFTER_FEASIBILITY = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
POST_FEASIBILITY_ROADMAP_DECISION = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
POST_FEASIBILITY_ROADMAP_DECISION_REVIEW = PASS_POST_FEASIBILITY_ROADMAP_DECISION
DECISION_OPTION = B
ROBOT_TRAJECTORY_RESEARCH = STOP_CURRENT_BRANCH
ROBOT_TRAJECTORY_RESEARCH_CURRENT_BRANCH = STOPPED
NEW_PLANNING_RESEARCH_BRANCH = NOT_OPENED
P025_DEFINITION_CHANGED = NO
P025_EXISTING_DEFINITION = SUPERSEDED_TEXT_RETAINED
P025_CONTRACT_REVIEW = PASS_CONTRACT_FROZEN
P025_CONTRACT_FROZEN = YES
P025_CONTRACT = FROZEN
SUPERSESSION_EFFECTIVE = YES
P025_GATE = OPEN
P025_SCOPE = VERIFIED_END_TO_END_ENGINEERING_EVIDENCE_VISUALIZATION
P025_IMPLEMENTATION_PLAN_REVIEW = PASS_IMPLEMENTATION_PLAN
P025_IMPLEMENTATION_PLAN = APPROVED
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
P025_PASS = NOT_EVALUATED
UPSTREAM_EVIDENCE = READ_ONLY
NEXT_ALLOWED_WORK = P025_VISUAL_RENDER_REVIEW
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
IKFAST_UR5E_QUALIFICATION = NOT_FEASIBLE_FOR_THIS_ENVIRONMENT
P-014 = DO_NOT_OPEN
P-021 = REVIEW_PASS
P021_FINAL_GATE = PASS_ROBOT_ADAPTER_CONTRACT
P021_REVIEW = PASS
P021_REVIEWER_ACCEPTED = true
P-022 = CLOSED_FAIL
P022_DEFINITION_REVIEW = PASS
P022_DEFINITION_EXECUTABLE = YES
P022_IMPLEMENTED = EVIDENCE_WRITTEN
P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P022_REVIEWER_CLOSED = true
CLOSURE_APPLIED = YES
P022_CLOSURE_APPLICATION = PASS_CLOSURE_APPLIED
P023 = NOT_OPENED
P024 = NOT_OPENED
ALTERNATIVE_PLANNING_GATE = NOT_OPENED
ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED
P025 = OPEN
```

이 파일이 Gate 절차의 정본이다. 전체 순서, 의존, 진입, 종료, 현재 상태, 다음에 열 수 있는 Gate, STOP을 여기에만 둔다.

역할은 나뉜다.

| 문서 | 역할 |
| --- | --- |
| `AGENTS.md` | Agent 운영 규칙 |
| `evidence/PROJECT-ROADMAP.md` | Gate 절차 정본 |
| `evidence/ADR-*` | 당시의 Architecture Decision. 소급 수정하지 않는다 |
| `evidence/P-*` | Gate 실행 증거. 소급 수정하지 않는다 |
| 각 `status.json` | 그 Gate를 기록할 당시의 snapshot. 현재 상태로 다시 쓰지 않는다 |

`docs/`는 git 추적 밖이다. 절차 정본을 거기에 두지 않는다.

```text
GROUND_TRUTH = NONE
CAD = REFERENCE_GEOMETRY
SCAN = MEASURED_GEOMETRY
ProcessPath != RobotTrajectory
```

등록은 correspondence다. CAD 좌표를 측정값에 덮어쓰지 않는다. ProcessPath는 로봇에 독립이다. RobotTrajectory는 joint-space이고 로봇과 시간에 묶인다.

## 현재 위치

절차 문서가 P-011 예약에서 멈춰 있는 동안 evidence는 P-013 구현까지 있다. 이 파일은 그 불일치를 메우기 위해 기존 증거를 등록한다. 새 실험을 시작하지 않는다.

```text
PROJECT_PROCEDURE_STATUS = FAIL_NO_CANONICAL_GATE_ROADMAP
  이 파일 작성 전의 상태다.

IMPLEMENTATION_STATUS = AHEAD_OF_PROCEDURE_DOCUMENTATION
NEXT_ALLOWED_GATE = NONE
```

위 블록은 이 파일을 만들기 전의 상태다. 지금 열 수 있는 Gate는 P-021뿐이다. P-014부터 P-020은 열지 않는다.

## 이미 있는 Evidence

acceptance 파일에 없는 `CLOSED` 문자열은 만들지 않았다. 재오픈은 이 절차에서 허용하지 않는다.

| Gate | 증거에 있는 판정 | 절차 상태 | 근거 |
| --- | --- | --- | --- |
| P-008 계열 | Dataset v2 `PASS_DATASET_V2`. R1 조사는 R1N `FAIL_CUTTING_PLANE_FEATURE_FEASIBILITY`에서 종료 | 재오픈하지 않음. 새 segmentation Gate 없음 | `evidence/P-008B`, `evidence/P-008/R1N` |
| P-009 | `FAIL_SURFACE_ASSOCIATION`. 함께 `FAIL_PATH_CONTINUITY` | 실패를 유지. PASS로 바꾸지 않음. acceptance.json에 CLOSED 토큰은 없음 | `evidence/P-009/acceptance.json` |
| P-009 계약 | `FROZEN` | 수정하지 않음. 파일 안의 P-010·P-011 `NOT_OPENED`는 2026-10-31 snapshot | `evidence/P-009-CONTRACT/status.json` |
| P-010 | `FAIL_CAD_ASSET_QUALIFICATION` | 수리·재자격 없음. acceptance.json에 CLOSED 토큰은 없음 | `evidence/P-010/acceptance.json` |
| P-010R | `CANDIDATE_FOUND`. primary `NONE` | 공개 화형 STEP을 primary로 채택하지 않음 | `evidence/P-010R/acceptance.json` |
| P-010A | `PASS_ARCHITECTURE_DECOUPLING` | 구현 없음. 기록 당시 `p011_opened = false` | `evidence/P-010A/acceptance.json` |
| ADR-011 | `ACCEPTED`. `DECOUPLE_CAD_AND_FOOTWEAR_ASSETS` | 본문 소급 수정 없음. 당시 문장은 Coupon이 아직 없고 P-011을 열지 않는다는 결정 시점의 사실 | `evidence/ADR-011/decision.json` |
| P-011 | `PASS_COMPLEX_SURFACE_CAD_TEST_COUPON` | 입력 STEP 유지. `assets/cad/complex_surface_coupon.step` | `evidence/P-011/acceptance.json` |
| P-012 | `PASS_CAD_GEOMETRY_ENGINE` | ProcessPath 없음. 엔진을 P-013 통과용으로 바꾸지 않음 | `evidence/P-012/acceptance.json` |
| P-013 | `PASS_CAD_6D_PROCESSPATH` | `IMPLEMENTATION = PASS`. `REVIEW = PENDING_REPOSITORY_CLOSURE`. `reviewer_closed = false` | `evidence/P-013/acceptance.json` |
| P-021 | `PASS_ROBOT_ADAPTER_CONTRACT` | `REVIEW = PASS`. `reviewer_accepted = true`. P-022는 허용만 됨 | `evidence/P-021/acceptance.json` |

P-009의 표면 association 실패는 지운 기록이 아니다. 측정 geometry만으로 ProcessPath를 닫지 못했고, 그 다음 구조가 ADR-011의 reference branch와 나중 correspondence다.

ADR-011 본문은 결정 당시다. 그 이후 실행 상태는 이 표다. ADR을 현재 숫자에 맞춰 다시 쓰지 않는다.

## Branch

```text
A. MEASURED_GEOMETRY
   Gemini 335L → point cloud → shoe geometry → scan ProcessPath
   P-008 계열, P-009 FAIL_SURFACE_ASSOCIATION

B. REFERENCE_GEOMETRY
   STEP/BREP → P-012 Geometry Engine → P-013 CAD 6D ProcessPath

C. CORRESPONDENCE
   같은 Coupon의 CAD와 scan. 등록은 correspondence. GROUND_TRUTH 없음

D. ADAPTIVE_PROCESS_GEOMETRY
   CAD path를 scan에 무조건 투영하지 않음
   association, distance, normal, confidence가 될 때만 수정

E. UNIFIED_PROCESSPATH
   두 출처가 같은 로봇 독립 ProcessPath 계약으로 모임

F. ROBOT_ADAPTER
   ProcessPath를 RobotPlanningInput으로 넘기는 계약. 구현과 execute는 아님

G. ROS2 / MOVEIT2

H. IK / COLLISION

I. ROBOTTRAJECTORY
   joint-space. ProcessPath와 다른 산출
```

A와 B는 측정·CAD 증거로 여기까지 있다. F는 P-021 계약 증거까지다. C부터 E, 그리고 G부터 I는 실행하지 않았다.

P-013 이후의 실행 모델은 `BRANCHED`다. CAD↔Scan / adaptive branch와 Robot Integration branch는 서로 상대의 PASS를 기다리지 않는다.

```text
P-013 CAD 6D ProcessPath
        |
        +-------------------------------+
        |                               |
        v                               v
CORRESPONDENCE / ADAPTIVE          ROBOT INTEGRATION
P-014 ~ P-020                      P-021 REVIEW = PASS
DEFINED_NOT_OPENED                 P-022 CLOSED_FAIL FAIL_CARTESIAN_API
DEFERRED_OPTIONAL_EXTENSION        P-023 ~ P-024 NOT OPENED
        |                               |
        v                               v
P-020 Unified Contract             P-025 DEFINED_NOT_OPENED
future producer                    PORTFOLIO_END_TO_END_VISUALIZATION
                                   P-026 NOT OPENED
```

두 branch가 로봇 쪽에 닿을 때의 공통 입력은 아래 `Robot Adapter Input Contract`다. P-013 경로와, 나중에 생기면 P-020 계약이 그 입력을 만드는 producer다. producer가 둘이라고 adapter 계약이 둘로 나뉘지는 않는다.

## 이후 Gate 제안

P-014부터 P-020은 `DEFINED_NOT_OPENED` / `DEFERRED_OPTIONAL_EXTENSION`이다. P-021 검토는 PASS다. P-022는 `CLOSED_FAIL`이다. 최종 게이트는 `FAIL_CARTESIAN_API`다. `reviewer_closed`는 true다. P-022는 다시 열지 않는다. `SUCCESSOR_GATE_DECISION = ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW`다. 이 선택은 구현 Gate가 아니다. `ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED`다. P-023과 P-024는 `NOT_OPENED`다. P-025 Gate는 `OPEN`이다. `P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW`다. P-025는 frozen evidence를 읽는 downstream이다. 화면을 위해 앞 Gate의 수치나 datum을 재계산하지 않는다.

번호는 기존 `evidence/P-014` 이상이 없어서 비어 있는 번호를 썼다. 충돌하는 기존 Gate ID는 없다.

### P-014 Physical Coupon Qualification

```text
P-014_DEFINITION = INCOMPLETE
P014_DEFINITION_EXECUTABLE = NO
DIMENSIONAL_THRESHOLD_STATUS = BLOCKED_INSUFFICIENT_MANUFACTURING_CONTEXT
P014_CURRENT_STATE = DEFINED_NOT_OPENED
P014_P020_STATE = DEFERRED_OPTIONAL_EXTENSION
```

이 절은 정의를 보강한 것이다. Gate는 열지 않는다. 치수 허용오차 숫자는 만들지 않았다.

- PURPOSE: 디지털 STEP과 제작물 사이의 provenance를 먼저 고정한다. 측정 dataset과 등록은 이 Gate가 아니다.
- INPUT: P-011 STEP. 경로 `assets/cad/complex_surface_coupon.step`. hash `2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa`.
- DEPENDENCIES: P-011, P-012, P-013. P-013은 구현 증거가 있고 review closure는 아직 없다. closure가 없어도 이 정의의 치수 기준은 열리지 않는다.
- ENTRY_CRITERIA: `NEXT_ALLOWED_GATE`가 P-014일 것. `DIMENSIONAL_THRESHOLD_STATUS`가 이 차단에서 벗어나 있을 것. 지금은 둘 다 아니다.

CAD와 제작물의 관계:

- 제작물의 설계 원본은 위 STEP 하나다. 다른 CAD로 바꾸지 않는다.
- STEP 단위는 mm다. 슬라이서가 적용한 축척은 출력 기록에 적는다. 1:1은 여기서 가정하지 않는다.
- 물리 Coupon은 `REFERENCE_GEOMETRY`의 제작물이다. `GROUND_TRUTH`가 아니다. `GROUND_TRUTH = NONE`은 이 Gate에서도 유지한다.
- 치수 비교의 기준 길이는 STEP nominal이다. 제작 결과를 본 뒤에 허용오차를 넓히지 않는다.
- Gemini 335L은 이 Gate의 치수 측정기가 아니다.

출력 기록이 가져야 할 provenance:

- `physical_id`. STEP hash와 다른 제작물 식별자다.
- STEP 경로와 hash.
- `PRINTER_MODEL`, `PRINT_PROCESS`, `MATERIAL`, `SLICER`.
- 슬라이서에 넣은 축척.
- 출력 시각과 작업자.
- 공정 파일의 식별. 파일이 없으면 `ABSENT`로 적고 성공으로 바꾸지 않는다.

지금 저장소에 없는 값:

```text
PRINTER_MODEL = NOT_DEFINED
PRINT_PROCESS = NOT_DEFINED
MATERIAL = NOT_DEFINED
SLICER = NOT_DEFINED
MEASUREMENT_INSTRUMENT = NOT_DEFINED
INSTRUMENT_RESOLUTION = NOT_DEFINED
```

W-002E-R3의 캘리퍼스 0.01 mm는 마커 변의 과거 기록이다. 표시 분해능은 정확도가 아니고, 이 Coupon의 측정기로 채택하지 않는다.

치수 허용오차:

```text
DIMENSIONAL_THRESHOLD_STATUS = BLOCKED_INSUFFICIENT_MANUFACTURING_CONTEXT
DIMENSION_X_TOLERANCE_MM = TBD
DIMENSION_Y_TOLERANCE_MM = TBD
DIMENSION_Z_TOLERANCE_MM = TBD
LANDMARK_DIMENSION_TOLERANCE_MM = TBD
TOLERANCE_BASIS = NONE
```

프린터 정확도와 측정 불확도는 다른 항목이다. 둘 중 하나라도 없으면 허용오차를 적지 않는다. 숫자를 나중에 정할 때는 `TOLERANCE_BASIS`에 printer specification, measurement capability, engineering requirement, 또는 그 밖의 명시 근거 중 하나를 적는다.

Landmark:

- P-011의 원통 landmark가 제작물에 있어야 한다. 공정 표면 밖에 있고, 축 방향은 STEP의 +X다.
- 없거나, 부러졌거나, 어느 것인지 구분되지 않으면 `FAIL_LANDMARK_ABSENT`다.
- 위치와 크기의 mm 허용은 `TBD`다. 보이는지만으로 치수 합격을 주지 않는다.

물리적 온전함:

- 공정 표면이 있어야 한다.
- 균열, 결손, 공정 표면 위의 남은 support를 각각 있음 또는 없음으로 적는다.
- 결손이나 파손이 있으면 `FAIL_PHYSICAL_INTEGRITY`다.
- 휨의 mm 한계는 `TBD`다. 그 한계가 생기기 전에 육안만으로 치수 PASS를 주지 않는다.

- OUTPUT: 위 provenance, 온전함 기록, landmark 유무. 치수 표는 허용오차가 생기기 전에는 `NOT_MEASURED`다.
- PASS_GATE: provenance가 빠지지 않고, landmark가 있고, 온전함 실패가 없고, 치수가 동결된 허용 안에 있을 것. 허용이 `TBD`인 동안에는 PASS할 수 없다.
- FAIL_GATE: `FAIL_PROVENANCE`, `FAIL_PHYSICAL_ID`, `FAIL_LANDMARK_ABSENT`, `FAIL_PHYSICAL_INTEGRITY`. 치수 실패 이름은 허용오차가 생기기 전에는 만들지 않는다. 허용이 없는 상태는 `BLOCKED_INSUFFICIENT_MANUFACTURING_CONTEXT`다.
- NEXT_GATE_ON_PASS: P-015. PASS 전에 열지 않는다.
- NON_GOALS: Gemini 촬영, 등록, ProcessPath 수정, 로봇 동작, 허용오차의 사후 완화, Gemini를 치수 측정기로 쓰는 일.

순서는 CAD STEP → 출력 → 물리 Coupon → provenance다. 촬영이 먼저가 아니다.

### P-015 Gemini 335L Coupon Dataset

- PURPOSE: 자격이 끝난 물리 Coupon의 측정 geometry를 만든다.
- INPUT: P-014를 통과한 제작물.
- ENTRY_CRITERIA: P-014 PASS.
- OUTPUT: 식별이 되는 RGB-D 또는 point cloud 세트와 provenance.
- PASS_GATE: 그 Gate가 촬영 전에 고정한 식별·동기·프레임 조건을 만족할 것.
- DEPENDENCIES: P-014.
- NON_GOALS: 등록, CAD에 맞춘 측정값 수정, P-009 재채점.

### P-016 CAD ↔ Scan Registration

- PURPOSE: reference geometry와 measured geometry의 correspondence를 만든다.
- INPUT: P-011 STEP, P-015 측정.
- ENTRY_CRITERIA: P-015 PASS. `VALIDATED_REGISTRATION_TRANSFORM`을 이 문서가 미리 부여하지 않는다.
- OUTPUT: 변환 후보와 잔차. 역할은 correspondence.
- PASS_GATE: 그 Gate의 잔차·독립 기준을 통과할 것. 통과해도 ground truth가 아니다.
- DEPENDENCIES: P-011, P-015.
- NON_GOALS: scan을 CAD에 워프, W-002E 실패 변환의 재사용, P-009 계약 수정.

### P-017 Registration Validation / Geometry Deviation

- PURPOSE: 등록 이후 남은 차이를 원인별로 주장하지 않고 크기로 남긴다.
- INPUT: P-016 변환과 두 geometry.
- ENTRY_CRITERIA: P-016 PASS.
- OUTPUT: 편차 분포. 제조·출력·센서·등록 중 하나를 단독 원인으로 확정하지 않는다.
- PASS_GATE: 편차가 재현되고, 원인 단정이 없을 것.
- DEPENDENCIES: P-016.
- NON_GOALS: adaptive path, 로봇 궤적.

### P-018 CAD Path ↔ Scan Surface Association

- PURPOSE: P-013 경로의 anchor가 측정 표면과 어떻게 대응하는지 본다.
- INPUT: P-013 ProcessPath, P-016 이후의 scan.
- ENTRY_CRITERIA: P-013 구현 PASS, P-017 PASS.
- OUTPUT: association 성공과 실패 개수.
- PASS_GATE: 그 Gate가 정한 association 기준. P-009의 실패 기준을 낮추지 않는다.
- DEPENDENCIES: P-013, P-017.
- NON_GOALS: 무조건 투영, P-009 결과를 PASS로 변경.

### P-019 Adaptive 6D ProcessPath

- PURPOSE: association, distance, normal consistency, confidence가 모두 될 때만 reference path를 고친다.
- INPUT: P-018 결과.
- ENTRY_CRITERIA: P-018 PASS. 하나라도 실패하면 `KEEP_REFERENCE_PATH` 또는 `REJECT_WAYPOINT`.
- OUTPUT: 수정된 waypoint와 유지된 waypoint의 구분.
- PASS_GATE: 조건 없는 투영이 0일 것.
- DEPENDENCIES: P-018, ADR-011 adaptive policy.
- NON_GOALS: 수치 임계를 이 ROADMAP에서 새로 정하는 일. 임계는 그 Gate 계약에서 고정한다.

### P-020 Unified ProcessPath Contract

- PURPOSE: CAD와 scan이 같은 로봇 독립 ProcessPath로 모이는 계약을 고정한다.
- INPUT: P-009 계약, P-013 계약, P-019 결과.
- ENTRY_CRITERIA: P-019 PASS.
- OUTPUT: 통합 계약. 구현 경로는 그 다음이다.
- PASS_GATE: anchor와 command, tool Z와 standoff, ProcessPath와 RobotTrajectory가 문서로 분리될 것.
- DEPENDENCIES: P-009 계약, P-013, P-019.
- NON_GOALS: P-009·P-013 동결 계약의 재작성, IK.

P-020은 삭제하지 않는다. 역할은 CAD, scan, adaptive path가 한 로봇 독립 ProcessPath 계약으로 모이는지를 보는 future extension이다. P-021의 유일한 prerequisite가 아니다. P-020이 PASS하면 그 결과도 아래 Robot Adapter Input Contract를 만족해야 한다.

### Robot Adapter Input Contract

Robot Adapter가 소비하는 최소 입력은 경로의 출처가 아니라 이 필드다. 이름은 P-013 동결 계약 `ProcessWaypoint`와 `evidence/P-013/cad_process_path.json`에 있는 것만 쓴다.

경로 공통:

- `frame_id`. P-013 산출은 `CAD_MODEL_FRAME`이다. 로봇 base도 MoveIt frame도 아니다.
- `position_unit`. P-013 산출은 mm다.
- `quaternion_order`. P-013 산출은 `x, y, z, w`다.
- `source`
- `geometry_role`

waypoint:

- `surface_anchor`
- `command_position`
- `orientation_quaternion`
- `surface_normal`
- `traversal_tangent`
- `tool_tangent`
- standoff. 동결 계약의 이름은 `tool_standoff_mm`다. 생성된 JSON의 이름은 `process_offset_mm`다. 계약은 이 값을 법선 방향 standoff라고 적고, P-009의 접선 `surface_offset`이 아니라고 적는다. P-013 산출의 `process_offset_mm`는 20.0이다.
- `segment_type`

`confidence`는 P-013에서 `NOT_DEFINED`다. CAD라는 이유로 1.0을 만들지 않는다. adapter 합격 조건에 confidence 척도를 새로 넣지 않는다.

P-013은 이 입력을 만드는 현재 producer다. P-020은 같은 입력을 만드는 future producer다.

### P-021 Robot Adapter Contract

```text
P021_DEFINITION_REVIEW = PASS
P021_DEFINITION_EXECUTABLE = YES
P021_DIRECT_PRODUCER = P-013
P021_CURRENT_STATE = REVIEW_PASS
P021_EVIDENCE_GATE = PASS_ROBOT_ADAPTER_CONTRACT
P021_REVIEW = PASS
P021_REVIEWER_ACCEPTED = true
P013_REVIEWER_CLOSED = false
P022_STATE = CLOSED_FAIL
P022_DEFINITION_REVIEW = PASS
P022_DEFINITION_EXECUTABLE = YES
P022_IMPLEMENTED = EVIDENCE_WRITTEN
P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P022_REVIEWER_CLOSED = true
CLOSURE_APPLIED = YES
```

계약 증거는 `evidence/P-021/`에 있다. 검토 수용은 PASS다. P-013의 `reviewer_closed`는 false 그대로다. P-022는 `CLOSED_FAIL`이다. 최종 게이트는 `FAIL_CARTESIAN_API`다. `reviewer_closed`는 true다. P-023은 열지 않는다.

이 승인이 허용하는 다음 증거는 계약이다. `command_position`은 mm이고, planning 쪽 위치는 `x_m = x_mm * 0.001`이다. 741개 waypoint를 m로 바꾼 파일, `RobotPlanningInput` 데이터, MoveIt pose는 P-021 산출이 아니다. 그 적용은 P-022다.

- PURPOSE: robot-independent ProcessPath와 robot-specific motion planning 사이의 software boundary를 정의한다. motion planning 구현 Gate가 아니다. adapter는 `source`가 CAD, scan, adaptive인지에 따라 동작을 분기하지 않는다.
- INPUT: Robot Adapter Input Contract. 현재 producer는 P-013 CAD 6D ProcessPath다. future producer는 P-020 Unified ProcessPath다.
- ENTRY_CRITERIA: 그 producer의 ProcessPath가 저장소에 있고, 입력 계약 필드가 있을 것. P-020 PASS는 요구하지 않는다. `reviewer_closed`는 이 진입 조건에 없다. 물리 로봇은 요구하지 않는다. simulation 또는 planning-only 구현을 허용한다.
- DEPENDENCIES: Robot Adapter Input Contract를 만족하는 ProcessPath producer. 현재는 P-013. P-020은 대체 가능한 future producer이지 유일한 의존이 아니다.
- OUTPUT: `RobotPlanningInput`. RobotTrajectory가 아니다.
- PASS_GATE: `PASS_ROBOT_ADAPTER_CONTRACT`. 아래 여덟 항목이 모두 PASS일 때만 해당한다.
- NEXT_GATE_ON_PASS: P-022. PASS가 나도 자동으로 열지 않는다.
- NON_GOALS: `execute`, 실물 관절 명령, TF의 실제 획득과 적용, ROS 2 / MoveIt 2 연결, IK, 충돌 검사, 시간 매개, RobotTrajectory 생성, P-014부터 P-020의 실행, P-013 계약 수정, 특정 로봇 제조사 API에 대한 종속, ProcessWaypoint와 IK goal의 1:1 강제.

`RobotPlanningInput`은 이 절차 문서의 계약 이름이다. ROS 메시지 타입으로 정하지 않는다.

입력 검증 계약. 소유는 P-021이다. 기존 P-013 수치를 다시 매기거나 고치지 않는다.

- `frame_id`, `position_unit`, `quaternion_order`, `source`, `geometry_role`
- `surface_anchor`, `command_position`, `orientation_quaternion`, `surface_normal`, `traversal_tangent`, `tool_tangent`
- standoff semantic. 계약 이름은 `tool_standoff_mm`다. P-013 JSON 이름은 `process_offset_mm`다. 둘은 법선 방향 standoff이고, P-009의 접선 `surface_offset`이 아니다. 두 숫자가 같다는 이유만으로 같은 필드로 합치지 않는다.
- `segment_type`
- nonfinite 값, quaternion 정규화, 벡터 차원
- `confidence`는 필수 필드가 아니다. CAD라는 이유로 1.0을 만들지 않는다.

`surface_anchor`와 `command_position`은 다른 필드다.

단위 경계. 변환 규칙의 소유는 P-021이다.

```text
INPUT_POSITION_UNIT = mm
PLANNING_POSITION_UNIT = m
mm_to_m = value * 0.001
```

길이 변환은 위치 값에만 적용한다. `orientation_quaternion`, `surface_normal`, `traversal_tangent`, `tool_tangent`에는 적용하지 않는다.

프레임 경계. 변환 의미의 소유는 P-021이다. 실제 획득과 적용의 소유는 P-022다.

```text
INPUT_FRAME = CAD_MODEL_FRAME
PLANNING_FRAME = ROBOT_PLANNING_FRAME
REQUIRED_TRANSFORM = ROBOT_PLANNING_FRAME T CAD_MODEL_FRAME
```

이 변환은 CAD model frame의 점을 robot planning frame으로 옮기는 의미다. 값이 없으면 planning으로 진행하지 않는다. P-021은 그 요구만 적고, lookup이나 행렬 적용은 구현하지 않는다.

`RobotPlanningInput`의 최소 의미:

- 순서가 있는 process pose
- target planning frame
- 위치 단위 m
- orientation
- segment semantics
- 출처 ProcessPath에 대한 reference

표면 metadata는 보존할 수 있다. 그것을 RobotTrajectory로 바꾸지 않는다.

ProcessWaypoint는 공정 형상의 샘플이다. IK goal이 아니고 trajectory point도 아니다. ProcessWaypoint 수와 joint-space trajectory point 수는 같지 않아도 된다. P-013의 741개를 741개 IK goal로 두지 않는다. ProcessPath를 planning representation으로 바꾸는 정책은 P-022가 구현하고 검증한다.

로봇 모델은 P-021에서 requirement와 interface만 정의한다. 특정 제조사 API에 묶지 않는다. 실제 ROS 2 / MoveIt 2 모델 적재는 P-022다. 이 계약은 제조사에 종속되지 않는 interface boundary다. 로봇별 구현은 P-022 이후에 둔다.

책임:

| 책임 | 소유 |
| --- | --- |
| ProcessPath schema validation | P-021 |
| mm → m 변환 규칙 | P-021 |
| `CAD_MODEL_FRAME` → `ROBOT_PLANNING_FRAME` 변환 계약 | P-021 |
| planning-input schema | P-021 |
| transform 획득과 적용 | P-022 |
| robot model 적재와 MoveIt 연결 | P-022 |
| planning representation | P-022 |
| Cartesian planning | P-022 |
| IK 검증 | P-023 |
| collision 검증 | P-023 |
| time parameterization | P-024 |
| RobotTrajectory | P-024 |
| physical execute | 나중 Gate가 명시하기 전에는 `OUT_OF_SCOPE` |

PASS가 되려면 다음이 모두 문서 계약으로 고정돼 있어야 한다.

```text
INPUT_CONTRACT_DEFINED = PASS
P013_COMPATIBILITY = PASS
UNIT_BOUNDARY_DEFINED = PASS
FRAME_BOUNDARY_DEFINED = PASS
OUTPUT_CONTRACT_DEFINED = PASS
RESPONSIBILITY_BOUNDARY_DEFINED = PASS
VENDOR_INDEPENDENCE_DEFINED = PASS
PROCESSPATH_TRAJECTORY_SEPARATION = PASS
```

FAIL:

- `FAIL_PROCESSPATH_CONTRACT`
- `FAIL_UNIT_CONTRACT`
- `FAIL_FRAME_CONTRACT`
- `FAIL_ADAPTER_OUTPUT_CONTRACT`
- `FAIL_RESPONSIBILITY_BOUNDARY`

유지하는 의미: `GROUND_TRUTH = NONE`. CAD는 `REFERENCE_GEOMETRY`, scan은 `MEASURED_GEOMETRY`. ProcessPath는 RobotTrajectory가 아니다.

### P-022 ROS 2 / MoveIt 2 Adapter

```text
P022_STATE = CLOSED_FAIL
P022_DEFINITION_REVIEW = PASS
P022_DEFINITION_EXECUTABLE = YES
P022_IMPLEMENTED = EVIDENCE_WRITTEN
P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P022_REVIEWER_CLOSED = true
CLOSURE_APPLIED = YES
P022_CLOSURE_APPLICATION = PASS_CLOSURE_APPLIED
SUCCESSOR_GATE_DECISION = ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW
```

정의 검수는 PASS다. 이 줄 아래의 구현은 그 수용 다음의 실행이다. Reviewer가 `CLOSED_FAIL`을 승인했다. `FAIL_CARTESIAN_API`는 그대로다. PASS로 바꾸지 않는다. P-022는 다시 열지 않는다. P-023은 열지 않는다. 후속은 타당성 검토이고, 구현 Gate는 열지 않는다.

- PURPOSE: P-021 `RobotPlanningInput`을 ROS 2 Jazzy / MoveIt 2의 reference adapter로 연결한다. UR5e는 이 Gate의 `PLANNING_REFERENCE_ROBOT`이다. 프로젝트의 유일한 로봇이 아니고, P-021 vendor independence를 바꾸지 않는다.
- INPUT: P-021의 `RobotPlanningInput`. 현재 producer는 P-013이다.
- ENTRY_CRITERIA: P-021 `PASS_ROBOT_ADAPTER_CONTRACT`. 그 PASS만으로 구현이 시작되지는 않는다. 이 정의의 검토 수용이 먼저다.
- DEPENDENCIES: P-021. Humble 컨테이너는 이 Gate의 범위 밖이다.
- OUTPUT: 적용된 simulation datum, 적재된 reference model, contiguous PROCESS run의 planning representation, run별 Cartesian fraction. RobotTrajectory가 아니다.
- PASS_GATE: `PASS_ROS2_MOVEIT2_ADAPTER`. 아래 조건이 모두 성립할 때만 해당한다. 평균 fraction으로는 PASS하지 않는다.
- NEXT_GATE_ON_PASS: P-023. PASS가 나도 자동으로 열지 않는다.
- NON_GOALS: physical execute, 실물 관절 명령, 컨트롤러 토픽으로 JointTrajectory 직접 발행, TRANSITION의 joint-space 궤적 생성, collision-free 주장, ProcessWaypoint와 IK goal의 1:1, 741개 pose를 하나의 Cartesian 요청으로 합치는 일, `0.8172 m`를 reach 사양이나 IK 성공으로 읽는 일.

Runtime:

```text
HOST = p620
ROS_DISTRO = Jazzy
REFERENCE_ROBOT = Universal Robots UR5e
ROBOT_ROLE = PLANNING_REFERENCE_ROBOT
UR_DESCRIPTION_VERSION = 3.5.1
UR_MOVEIT_CONFIG_VERSION = 3.9.0
UR_ROBOT_DRIVER_VERSION = 3.9.0
PROVENANCE = ROS_JAZZY_BINARY_PACKAGE
PHYSICAL_ROBOT_REQUIRED = NO
SIMULATION_ALLOWED = YES
PLANNING_ONLY_ALLOWED = YES
PHYSICAL_EXECUTION = OUT_OF_SCOPE
```

`ur_description` 3.5.1의 `default_kinematics.yaml`은 패키지 파라미터다. 실물 UR5e calibration이 아니다.

Group과 frame. 설치된 SRDF chain `base_link` → `tool0`을 따른다.

```text
PLANNING_GROUP = ur_manipulator
BASE_FRAME = base_link
TCP_FRAME = tool0
TCP_ROLE = PLANNING_REFERENCE_TCP
PHYSICAL_PROCESS_TCP = NO
```

`tool0`은 이 테스트의 planning TCP다. 신발 공정 공구의 TCP가 아니다.

단위. 위치만 `value * 0.001`로 mm에서 m로 바꾼다. quaternion, normal, tangent에는 길이 변환을 넣지 않는다.

프레임. `ROBOT_PLANNING_FRAME`은 이 reference adapter에서 `base_link`다.

```text
TRANSFORM_SOURCE = CONFIGURED_SIMULATION_DATUM
TRANSFORM_ROLE = SIMULATION_PLACEMENT
ROTATION = IDENTITY
TRANSLATION_M = [0.3244333936553, -0.0499999998932, 0.11133404082435]
MEASURED_TRANSFORM = NO
CALIBRATION_RESULT = NO
GROUND_TRUTH = NONE
```

이 변환은 CAD `+Z` 법선이 `base_link`의 위쪽과 맞게 두는 배치다. 측정한 hand-eye가 아니다. 링크 길이 합 `0.8172 m`는 upper arm `0.425 m`와 forearm `0.3922 m`의 heuristic이다. 제조사 reach, IK, 충돌 없는 도달, Cartesian 성공이 아니다.

Planning request. `segment_type`은 MoveIt이 알지 못하므로 adapter가 나눈다.

```text
PROCESS_SEGMENT_POLICY = CARTESIAN
PLANNING_REQUEST_PARTITION_POLICY = CONTIGUOUS_PROCESS_RUN
TRANSITION_SEGMENT_POLICY = BOUNDARY_ONLY
P022_TRANSITION_MOTION_GENERATION = OUT_OF_SCOPE
```

연속된 PROCESS 구간마다 Cartesian 요청을 하나 만든다. TRANSITION은 그 구간들의 경계이고, 공정 Cartesian fraction의 대상이 아니다. TRANSITION 동작을 joint-space 궤적으로 만들지 않는다.

```text
CARTESIAN_EEF_STEP = 0.005 m
EEF_STEP_BASIS = PROJECT_PROCESSPATH_NOMINAL_SPACING
```

`0.005 m`는 P-013의 명목 간격 5 mm와 맞춘 프로젝트 기준이다. MoveIt이나 UR의 공식 권장값이 아니다. 의미는 보간의 최대 translation step이다. waypoint를 5 mm마다 새로 만든다는 뜻이 아니다.

```text
JUMP_DETECTION = DISABLED
relative_jump_threshold = 0
revolute_jump_threshold = 0
prismatic_jump_threshold = 0
JOINT_JUMP_SAFETY_VALIDATED = NO
```

jump 숫자는 실행 결과를 보고 맞추지 않는다. 0은 검출을 끈다는 뜻이다. joint jump 안전이 검증됐다는 뜻이 아니다.

```text
P022_AVOID_COLLISIONS = false
COLLISION_VALIDATION = P-023
COLLISION_FREE_CLAIM = NO
```

`avoid_collisions = true`이면 충돌이 Cartesian fraction에 섞인다. P-022는 그 원인을 분리하지 못하므로 false다. 충돌을 무시해도 안전하다는 뜻이 아니다.

합격 fraction. 전체 waypoint의 평균이 아니다. 각 contiguous PROCESS run의 fraction이 모두 `0.95` 이상이어야 한다.

```text
MIN_PROCESS_RUN_FRACTION >= 0.95
CARTESIAN_FRACTION_THRESHOLD = 0.95
THRESHOLD_SOURCE = PROJECT_ACCEPTANCE_CRITERION
PASS = MIN_PROCESS_RUN_FRACTION >= 0.95
FAIL = FAIL_CARTESIAN_PLANNING
```

`0.95`는 프로젝트 수용 기준이다. MoveIt 공식 임계값이 아니다. fraction 미달은 execute 금지가 아니라 `FAIL_CARTESIAN_PLANNING`이다. execute는 이 Gate에서 하지 않는다.

함께 성립해야 하는 조건:

```text
ROBOT_MODEL_LOAD = PASS
UNIT_CONVERSION = PASS
FRAME_TRANSFORM = PASS
ROBOTPLANNINGINPUT = PASS
PLANNING_REPRESENTATION = PASS
MIN_PROCESS_RUN_FRACTION >= 0.95
```

```text
PROCESS_WAYPOINT_EQUALS_IK_GOAL = NO
PROCESS_WAYPOINT_EQUALS_TRAJECTORY_POINT = NO
```

보간 내부의 IK 호출은 MoveIt 구현이다. ProcessWaypoint를 독립 IK goal로 두지 않는다. IK 검증과 충돌 검증은 P-023이다. time parameterization과 RobotTrajectory는 P-024다.

### P-023 IK + Collision Validation

- PURPOSE: P-022가 만든 planning representation의 IK와 충돌을 검사한다. ProcessWaypoint 하나를 IK goal 하나로 세지 않는다.
- INPUT: P-022 planning representation. 그 표현의 원천 ProcessPath가 P-013 producer이면 CAD ProcessPath다. P-020 producer를 쓴 경우에만 Unified ProcessPath가 원천이 된다. P-020 PASS를 이 Gate의 조건으로 두지 않는다.
- ENTRY_CRITERIA: P-022 PASS.
- OUTPUT: IK와 충돌 결과.
- PASS_GATE: 그 Gate의 분율·충돌 기준.
- DEPENDENCIES: P-022.
- NON_GOALS: 실물 실행, force control, time parameterization, RobotTrajectory 생성.

### P-024 RobotTrajectory Generation

- PURPOSE: joint-space, 시간 매개 RobotTrajectory를 만든다.
- INPUT: P-023을 통과한 계획.
- ENTRY_CRITERIA: P-023 PASS.
- OUTPUT: RobotTrajectory.
- PASS_GATE: ProcessPath 파일과 다른 산출임이 기록될 것.
- DEPENDENCIES: P-023.
- NON_GOALS: ProcessPath를 로봇 궤적으로 바꿔 부르는 일. 실물 execute.

### P-025 Portfolio End-to-End Visualization

```text
P025 = DEFINED_NOT_OPENED
P025_ROLE = PORTFOLIO_END_TO_END_VISUALIZATION
P025_NEW_ROBOTICS_ALGORITHM = NO
HISTORICAL_DEFINITION = RETAINED
SUPERSESSION_SOURCE = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
SUPERSESSION_EFFECTIVE = YES
PROPOSED_CONTRACT = evidence/P-025/contract_redefinition.md
```

아래 본문은 역사 정의다. 지우지 않는다. 동결된 계약은 `evidence/P-025/contract_redefinition.md`다. 대체는 효력이 있다. P-025 Gate는 `OPEN`이다. `P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW`다. 역사 본문은 바꾸지 않는다.


이 Gate는 새 robotics algorithm을 만들지 않는다. 검증된 upstream 결과를 새 기술 주장 없이 모아, 면접관과 검토자가 20~30초 안에 Robotics Software Pipeline을 볼 수 있는 시각 산출물을 만든다. 화면을 맞추려고 P-022부터 P-024의 evidence, datum, threshold를 바꾸지 않는다.

PRIMARY STORY:

```text
CAD / STEP
  → OpenCascade BREP
  → Differential Geometry
  → 6D ProcessPath
  → Robot-independent Adapter
  → ROS 2 / MoveIt 2
  → IK / Collision Validation
  → RobotTrajectory
  → End-to-End Visualization
```

DEPENDENCIES: P-011, P-012, P-013, P-021, P-022, P-023, P-024. P-014부터 P-020은 전제가 아니다. CAD↔Scan correspondence, deviation, adaptive path는 `DEFERRED_OPTIONAL_EXTENSION`으로 남긴다.

ENTRY_CRITERIA: P-024 PASS. 그 전이면 열지 않는다.

REQUIRED ARTIFACTS:

1. ARCHITECTURE VISUAL. CAD/STEP → OpenCascade → Reference Geometry와 RGB-D/Gemini 335L → Point Cloud → Measured Geometry가 나란히 있고, 그 아래 6D ProcessPath → Robot Adapter → ROS 2 / MoveIt 2 → IK / Collision → RobotTrajectory가 있다.
2. CAD GEOMETRY VISUAL. P-011 coupon, process surface, surface normal, curvature, positive / negative / near-zero curvature region. 검증된 P-012 evidence에서만 가져온다. 지원되지 않는 geometry 주장을 넣지 않는다.
3. 6D PROCESSPATH VISUAL. CAD surface, serpentine ProcessPath, surface anchor, 선택한 6D tool frame, tool orientation. ProcessPath가 position과 orientation을 가진다는 것을 보여 준다. `ProcessWaypoint = IK goal`이나 `ProcessWaypoint = trajectory point`로 읽히게 하지 않는다.
4. ROBOT-ADAPTER VISUAL. ProcessPath → Robot-independent Contract → Robot Adapter → Robot-specific Planning. reference implementation은 UR5e, ROS 2 Jazzy, MoveIt 2다. P-021 vendor independence는 유지한다.
5. MOVEIT PLANNING VISUAL. UR5e, reference CAD coupon, configured simulation placement, planned process path, 해당 planning result. `PLANNING_REFERENCE_ROBOT = UR5e`를 적는다. 실물 UR5e 실행, 실물 calibration, 측정된 CAD↔robot transform으로 읽히게 하지 않는다.
6. TRAJECTORY VISUAL. P-024 뒤에만 만든다. ProcessPath → planning → RobotTrajectory. Cartesian ProcessPath와 joint-space RobotTrajectory를 구분한다.
7. END-TO-END DEMO. 약 20~30초. CAD model → surface geometry → normal / curvature → 6D ProcessPath → Robot Adapter → UR5e / MoveIt planning → IK / Collision validation → RobotTrajectory. Gate 문서를 읽기 전에 핵심 경로가 보여야 한다.
8. KEY METRICS. 표시 후보는 CAD face 수, valid geometry sample 수, ProcessPath waypoint 수, path spacing, orientation continuity, process-run 수, minimum Cartesian fraction, IK result, collision-validation result, trajectory result다. 표시하는 값은 기존 Gate evidence로 추적되어야 한다. 없는 숫자를 만들거나 올려 적지 않는다.
9. ENGINEERING DECISIONS. 실물 신발 RGB-D → shoe/support 분리 시도 → `NOT_ROBUSTLY_SEPARABLE` / FAIL evidence → ADR-008 → perception과 ProcessPath 책임 분리 → 공개 CAD 자산 qualification 실패 → ADR-011 → CAD reference geometry와 measured footwear geometry의 분리다. 성공 화면만 모으지 않는다.

PORTFOLIO MESSAGE:

```text
PRIMARY = CAD/3D Scan 형상을 공정 의미를 가진 6D ProcessPath로 변환하고, Robot-independent Adapter를 통해 ROS 2 / MoveIt 2 motion-planning pipeline으로 연결한다.
SECONDARY = 실패한 perception 접근을 측정 가능한 Gate로 검증하고, 실패 원인을 기반으로 책임 경계와 architecture를 재설계했다.
```

CLAIM BOUNDARY. 주장하지 않는 것: physical UR5e validation, physical shoe processing success, production-ready footwear manufacturing, zero robot positional error, measured CAD↔robot calibration, physical process-tool calibration, P-023이 증명하지 않은 collision-free, optional branch가 끝나기 전의 CAD↔Scan correspondence, evidence가 없는 manufacturing accuracy. `GROUND_TRUTH = NONE`.

PASS_GATE: `PASS_PORTFOLIO_END_TO_END_VISUALIZATION`. 아키텍처, CAD geometry, 6D ProcessPath, robot-independent boundary, MoveIt planning, P-023 결과, P-024 RobotTrajectory, end-to-end demo가 각각 보이고, 지표가 evidence에 연결되고, 실패를 포함한 engineering decision이 있으며, 주장이 검증 범위를 넘지 않고, 구현 코드를 읽지 않아도 pipeline이 식별될 때만 PASS다.

NON_GOALS: 새 알고리즘, 앞 Gate의 재측정, 화면을 위한 datum·threshold 조정, deferred branch를 완료된 것처럼 그리는 일.

### P-026 Portfolio / Demo Packaging

```text
P026 = NOT_OPENED
```

- PURPOSE: P-025 시각 산출물을 포트폴리오 설명으로 묶는다. P-025가 이미 담은 주장을 넓히지 않는다.
- INPUT: P-025.
- ENTRY_CRITERIA: P-025 `PASS_PORTFOLIO_END_TO_END_VISUALIZATION`.
- OUTPUT: 주장 범위가 증거와 같은 설명.
- PASS_GATE: ground truth, 접촉 공정, 실물 실행을 증거 없이 적지 않을 것.
- DEPENDENCIES: P-025.
- NON_GOALS: 새 로봇 실험. P-022부터 P-024 evidence를 화면용으로 다시 쓰기.

## STOP

- `NEXT_ALLOWED_GATE = ALTERNATIVE_PLANNING_FEASIBILITY_REVIEW`다. P-022는 `CLOSED_FAIL`이고 다시 열지 않는다. P-014부터 P-020, P-023, P-024, P-025, P-026은 열지 않는다. `ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED`다. P-025는 정의만 있다. feasibility 검토는 `CLOSED`다. `FEASIBILITY_VERDICT = NO_JUSTIFIABLE_ALTERNATIVE`이고 `JUSTIFIABLE_ALTERNATIVE_FOUND = NO`다. 범위는 동결 contract 안에서 구현 Gate를 열 대안이 없다는 뜻이다. 이 ProcessPath를 RobotTrajectory로 만드는 방법이 없다는 뜻은 아니다. `KDL_SCALAR_CALLBACK = RECONSIDER`는 유지한다. `NEXT_ALLOWED_GATE = NONE`이다. P-023, P-024, P-025, 구현 Gate는 열지 않는다. roadmap 선택은 B다. RobotTrajectory 연구는 현재 분기에서 멈춘다. P-025는 아직 열지 않고 기존 정의 본문도 바꾸지 않았다. 계약 검토는 `PASS_CONTRACT_FROZEN`이다. `P025_GATE = OPEN`이다. `P025_IMPLEMENTATION_PLAN = APPROVED`다. `P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW`다. `P025_PASS = NOT_EVALUATED`다. 이 Gate는 frozen evidence를 소비한다. `0.606061`, 실패 run, datum, ProcessPath를 보기 좋게 재계산하지 않는다.
- P-022 정의 검수는 PASS다. `CLOSED_FAIL`은 `FAIL_CARTESIAN_API`를 PASS로 바꾸지 않는다. `NEXT_ALLOWED_GATE`를 P-023으로 바꾸지 않는다. 동결된 datum, `eef_step`, fraction threshold를 결과를 보고 바꾸지 않는다. 실물 실행과 RobotTrajectory 생성은 하지 않는다.
- P-014 출력, Gemini 촬영, 등록, adaptive path를 이 변경으로 시작하지 않는다.
- historical evidence와 ADR-011 본문을 현재 표에 맞춰 고치지 않는다.
- P-009 실패를 삭제하거나 PASS로 읽지 않는다.

## AGENTS.md

절차 복구 당시 `AGENTS.md`는 P-011을 열지 않는다고 적고, 공개 화형 primary와 Coupon을 한 문장에 두었다. 그 문장은 운영 규칙에서 빼었다. `AGENTS.md`는 Gate 개폐를 소유하지 않고 이 파일을 정본으로 가리킨다. `NEXT_ALLOWED_GATE`는 `P025`다. `P025_IMPLEMENTATION_PLAN_REVIEW = PASS_IMPLEMENTATION_PLAN`이다. `P025_IMPLEMENTATION = IMPLEMENTED_PENDING_VISUAL_REVIEW`다. `P025_PASS = NOT_EVALUATED`다. upstream evidence는 읽기 전용이다.
