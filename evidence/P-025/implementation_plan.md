# P-025 implementation plan

```text
P025_IMPLEMENTATION_PLAN_REVIEW = PASS_IMPLEMENTATION_PLAN
P025_IMPLEMENTATION_PLAN = APPROVED
P025_GATE_OPEN_REVIEW = PASS_GATE_OPEN
P025_GATE = OPEN
P025_CONTRACT = FROZEN
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
P025_PASS = NOT_EVALUATED
UPSTREAM_EVIDENCE = READ_ONLY
NEXT_ALLOWED_WORK = P025_VISUAL_RENDER_REVIEW
P023 = NOT_OPENED
P024 = NOT_OPENED
```

이 문서는 승인된 구현 계획이다. 아래 경계는 구현 중에도 바꾸지 않는다. `P025_PASS`는 구현이 끝났다는 이유만으로 정하지 않는다.

목적: frozen evidence를 화면까지 추적 가능하게 연결한다. 새 계산 파이프라인을 만들지 않는다.

## 구조

```text
Frozen Evidence
      │
      ▼
Evidence Adapter
      │
      ├── provenance
      ├── verified values
      ├── gate status
      └── claim boundary
      │
      ▼
Visualization Model
      │
      ▼
Overview
      │
      ▼
Scene 1 ─ Scene 2 ─ Scene 3 ─ Scene 4 ─ Scene 5
```

UI는 evidence 파일을 직접 해석하지 않는다. Adapter가 만든 Visualization Model만 읽는다. 모델에 없는 숫자는 화면에 없다.

Adapter는 source→display mapping이다. 값을 다시 계산하는 loader가 아니다. 복사하는 필드의 이름, 출처 경로, 그 필드가 뜻하는 claim boundary를 모델에 같이 넣는다.

## 허용되는 변환과 금지되는 재계산

| 구분 | 예 | 처리 |
| --- | --- | --- |
| visualization transformation | 저장된 waypoint를 화면용 JSON으로 직렬화. 저장된 두 점 사이를 선으로 그림. 저장된 fraction을 칸 색으로 바꿈 | 허용. provenance를 모델에 남긴다 |
| upstream recomputation | normal 재추정, path 재생성, datum 재적용, IK, Cartesian planning, curvature 재계산, fraction 재집계를 합격 숫자로 씀 | 금지 |

직렬화는 저장된 좌표를 단위와 함께 옮기는 일이다. `command_position - surface_anchor`를 화살표로 그리는 것은 저장된 두 점의 표현이다. 그 차이로 standoff를 새로 맞추거나 법선을 고치지 않는다.

## 첫 화면

면접관이 2~3분 안에 경계를 보도록, Scene drill-down 앞에 overview를 둔다.

```text
REFERENCE CAD
    │
    ▼
Geometry Engine
    │
    ▼
6D ProcessPath
    │
    ▼
Robot Adapter
    │
    ▼
MoveIt Integration
    │
    ▼
VERIFIED BOUNDARY
P-022: CLOSED_FAIL

RobotTrajectory
NOT ACHIEVED
```

Overview는 성공한 파이프라인의 끝처럼 끝나면 실패다. `CLOSED_FAIL`과 `RobotTrajectory = NOT ACHIEVED`는 첫 화면에서 보인다.

## Scene mapping

| Scene | 보여줄 것 | Evidence source | 금지 |
| --- | --- | --- | --- |
| 1 CAD | Complex Surface Coupon | P-011 | footwear CAD, ground truth |
| 2 Geometry | surface, normal, curvature | P-012 | 새 geometry 계산 |
| 3 ProcessPath | 741개 6D waypoint, anchor, command, tool frame | P-013, 잔차 출처는 P-021 보고서 | physical accuracy |
| 4 Robot Adapter | P-021 계약과 P-022 simulation datum | P-021, P-022 | P-021이 transform을 실행했다고 표현 |
| 5 Planning Boundary | 17 full, 3 start failure, 1 partial | P-022 baseline, feasibility | RobotTrajectory 성공 |

### SCENE-1

출처: `evidence/P-011/P-011_REPORT.md`, `assets/cad/complex_surface_coupon.step`.

복사할 값:

```text
ROLE = CONTROLLED_TEST_COUPON
GEOMETRY_ROLE = REFERENCE_GEOMETRY
GROUND_TRUTH = NONE
FINAL_GATE = PASS_COMPLEX_SURFACE_CAD_TEST_COUPON
UNIT = mm
SOLID_COUNT = 1
FACE_COUNT = 20
EDGE_COUNT = 40
BOUNDING_BOX_MM = X 180.0, Y 100.0, Z 41.252357465702
STEP_SHA256 = 2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa
```

SHA-256의 출처 문장은 P-012 보고서의 `INPUT_SHA256`이다. Adapter는 STEP 파일을 다시 해시하지 않는다. 표시 문자열을 그 보고서 값과 비교한다.

footwear CAD, 측정 표면의 참값, scan을 이 CAD에 맞춘 결과처럼 보이지 않게 한다.

### SCENE-2

출처: `evidence/P-012/P-012_REPORT.md`. 샘플 좌표를 다시 평가하지 않는다.

복사할 값:

```text
FINAL_GATE = PASS_CAD_GEOMETRY_ENGINE
FACE_COUNT = 20
SURFACE_TYPES = BSpline 14, Plane 5, Cylinder 1
TOTAL_SAMPLE_COUNT = 1620
VALID_SAMPLE_COUNT = 1585
INVALID_SAMPLE_COUNT = 35
NORMAL_FAILURE_COUNT = 0
K_MIN = -0.0011310794
K_MAX = 0.0017847765
```

이 숫자는 곡면 품질의 물리 정확도가 아니다. `GROUND_TRUTH = NONE`을 이 장면에도 둔다.

### SCENE-3

출처: `evidence/P-013/cad_process_path.json`, `evidence/P-013/processpath_contract.md`, 잔차는 `evidence/P-021/P-021_REPORT.md`.

복사할 계약 값:

```text
WAYPOINT_COUNT = 741
FRAME = CAD_MODEL_FRAME
POSITION_UNIT = mm
QUATERNION_ORDER = x, y, z, w
GEOMETRY_ROLE = REFERENCE_GEOMETRY
STANDOFF = surface normal 방향 20 mm
process_offset_mm = 20.0
tool_standoff_mm key = absent
RESIDUAL_MAX_MM = 1.000017846308765e-09
```

잔차는 `command_position - (surface_anchor + surface_normal * 20)`의 저장 수치 일치다. 0 mm로 반올림하지 않는다. 물리 정확도, 로봇 위치 오차, ground truth가 아니다.

화면에 세 기하를 구분한다.

```text
surface_anchor      표면 위의 점. TCP가 아니다.
command_position    anchor에서 법선 방향 standoff를 더한 점.
tool Z              -surface_normal. IK goal도 trajectory point도 아니다.
```

시각은 저장된 anchor, 저장된 command, 저장된 tool 방향을 한 장면에 둔다. ProcessWaypoint, IK goal, trajectory point는 서로 다른 칸으로 두고 뒤의 둘은 이 장면에 없다.

### SCENE-4

한 장면 안에서 두 provenance를 나눈다.

```text
P-021
  RobotPlanningInput contract
  unit/frame conversion requirement
  input frame = CAD_MODEL_FRAME
  unit = mm
  x_m = x_mm * 0.001
  transform executed = NO
  RobotPlanningInput file written = NO

P-022
  configured simulation datum applied
  CAD_MODEL_FRAME → base_link
  MoveIt integration
  FRAME_TRANSFORM = identity rotation
  translation_m = [0.3244333936553, -0.0499999998932, 0.11133404082435]
  datum role = CONFIGURED_SIMULATION_DATUM
  UR5e group = ur_manipulator
  tool0 = PLANNING_REFERENCE_TCP
```

datum 숫자의 출처는 `evidence/P-022/P-022_REPORT.md`다. 다시 맞추지 않는다. calibration, 측정 transform, ground truth, 실물 공정 공구 TCP가 아니다.

`~1e-9 mm`를 이 장면의 물리 오차로 두지 않는다. 그 값은 Scene 3의 계약식 잔차다.

### SCENE-5

출처: `evidence/P-022/cartesian_runs.json`, `evidence/P-022/failure_diagnosis.md`. 민감도 실험 숫자는 쓰지 않는다. feasibility 문구의 출처는 `evidence/ALTERNATIVE-PLANNING-FEASIBILITY/feasibility_final_review.md`다.

```text
source:
  evidence/P-022/cartesian_runs.json

display:
  process_runs: 21
  run_id: 0..20
  full_fraction_runs: 17
  start_ik_failed_runs: [5, 7, 12]
  start_ik_failed_fraction: null
  partial_run: 20
  partial_fraction: 0.606061
  completed_intervals: 20
  total_intervals: 33
  final_gate: FAIL_CARTESIAN_API
  trajectory_written: false

claim:
  P022_STATUS: CLOSED_FAIL

prohibited_inference:
  - trajectory generated
  - collision free
  - physical executable
```

`START_IK_FAILED`를 fraction 0으로 바꾸지 않는다. 합격선은 PROCESS run마다 0.95 이상이다. 평균으로 대신하지 않는다.

Strip의 칸 번호는 `run_id`다. 0부터 20까지 21칸이다. 1부터 21로 다시 세지 않는다. 다시 세면 run 5, 7, 12, 20이 다른 칸이 된다.

```text
run_id  0 █  1 █  2 █  3 █  4 █  5 ×  6 █
        7 ×  8 █  9 █  10 █  11 █  12 ×  13 █
        14 █  15 █  16 █  17 █  18 █  19 █  20 ▒

█  fraction = 1
×  START_IK_FAILED, fraction = null
▒  run 20, fraction = 0.606061, 20/33
```

각 행의 `trajectory_written`은 false다. 칸이 가득 찬 것은 Cartesian fraction이 1이라는 뜻이지 RobotTrajectory가 생겼다는 뜻이 아니다.

화면에 고정할 문구:

```text
동결된 P-022 계약과 검토 환경 내에서 구현을 정당화할 대안을 확보하지 못함. 계획 해법의 부재를 의미하지 않음.
```

`KDL_SCALAR_CALLBACK = RECONSIDER`를 보이면 구현됐다는 뜻으로 두지 않는다. P-023, P-024 결과는 없다.

## Claim boundary

전 장면에 공통으로 두지 않을 주장:

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

`CAD = REFERENCE_GEOMETRY`, `Scan = MEASURED_GEOMETRY`, `GROUND_TRUTH = NONE`을 Scene 1과 overview에 둔다. 이 시각화의 입력은 CAD reference branch다. scan 측정 경로를 이 화면의 성공으로 넣지 않는다.

## Validation

화면이 아니라 Visualization Model과 동결 파일을 비교한다. 불일치는 화면 문구를 고친다. upstream evidence를 고치지 않는다.

```text
V-01  Scene 1 표시 값 == P-011 보고서의 해당 필드
V-02  Scene 2 표시 값 == P-012 보고서의 해당 필드
V-03  Scene 3 waypoint 수, frame, standoff, 잔차 == P-013 파일과 P-021 보고서
V-04  Scene 4의 P-021 실행 여부, 식, P-022 datum == 각 출처
V-05  Scene 5 run 수, run_id 5·7·12, run 20의 0.606061과 20/33 == cartesian_runs.json과 failure_diagnosis.md
V-06  overview와 Scene 5에 CLOSED_FAIL이 있다
V-07  NO_JUSTIFIABLE_ALTERNATIVE의 고정 문구가 있다
V-08  ProcessPath 칸과 RobotTrajectory NOT ACHIEVED 칸이 서로 다르다
V-09  MUST_NOT_CLAIM 문자열이 성공 주장으로 없다
V-10  P-011부터 P-022 evidence와 roadmap의 역사 P-025 본문 해시가 구현 전후 같다
```

V-05는 JSON을 다시 planning하지 않는다. 저장된 `run_id`, `fraction`, `reason`, `trajectory_written`을 읽는다. `fraction = null`인 run을 0으로 집계하면 V-05는 실패다.

## AC 대응

| AC | 계획에서의 검사 |
| --- | --- |
| AC-01 | V-01, V-02, V-03, V-04, V-05. 모델의 모든 정량 필드에 source path가 있다 |
| AC-02 | 각 Scene의 금지 칸. 성공 문구는 그 Gate의 FINAL_GATE를 넘지 않는다 |
| AC-03 | V-06 |
| AC-04 | V-05. 17, [5, 7, 12], run 20, 0.606061, 20/33 |
| AC-05 | V-07. 고정 문구는 계획 해법의 부재를 부정한다 |
| AC-06 | V-08. overview의 마지막 칸 |
| AC-07 | Scene 1과 overview의 세 역할 문구 |
| AC-08 | Scene 5에 P-023, P-024 결과 칸을 만들지 않는다 |
| AC-09 | overview만 봐도 VERIFIED 사슬과 NOT ACHIEVED 경계가 보인다 |
| AC-10 | V-10 |

AC 하나라도 실패하면 robotics 결과를 고쳐 맞추지 않는다.

## 구현으로 넘어갈 때의 순서

이 계획의 승인이 먼저다. 승인 뒤에만 다음 순서를 연다.

```text
1. Evidence Adapter와 Visualization Model
2. V-01..V-10
3. Overview와 Scene 1..5
4. AC-01..AC-10 evidence
```

2가 실패하면 3의 문구를 고친다. 1은 upstream 파일을 쓰지 않는다. renderer 선택은 이 계획의 승인 항목이 아니다. renderer는 승인된 모델만 그린다.
