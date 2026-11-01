# P-021 Robot Adapter Contract

이 문서는 robot-independent ProcessPath와 robot-specific planning 사이의 계약이다. ROS 메시지 타입을 만들지 않는다. 구현 코드, 변환된 경로 파일, MoveIt pose를 만들지 않는다.

```text
GATE = P-021
FINAL_GATE = PASS_ROBOT_ADAPTER_CONTRACT
P021_REVIEW = PASS
P021_REVIEWER_ACCEPTED = true
P-022 = AUTHORIZED_TO_OPEN
```

## 입력

현재 producer는 P-013이다. 산출은 `evidence/P-013/cad_process_path.json`이다. future producer는 P-020이다. adapter는 `source`가 CAD, scan, adaptive인지에 따라 동작을 분기하지 않는다.

`confidence`는 필수 필드가 아니다. P-013 산출의 값은 `NOT_DEFINED`다. 합격 척도로 쓰지 않는다.

필수 필드:

- `frame_id`
- `position_unit`
- `quaternion_order`
- `source`
- `geometry_role`
- `surface_anchor`
- `command_position`
- `orientation_quaternion`
- `surface_normal`
- `traversal_tangent`
- `tool_tangent`
- `process_offset_mm`
- `segment_type`

## 위치

`surface_anchor`와 `command_position`은 다른 필드다.

P-013 계약의 식은 그대로다.

```text
command_position = surface_anchor + surface_normal * tool_standoff_mm
tool_standoff_mm = 20.0
```

생성된 JSON의 이름은 `process_offset_mm`다. 관측값은 20.0이다. 이 값은 법선 방향 standoff다. P-009의 접선 `surface_offset`이 아니다. 두 숫자가 같다는 이유만으로 한 필드로 합치지 않는다.

## 단위

```text
INPUT_POSITION_UNIT = mm
PLANNING_POSITION_UNIT = m
x_m = x_mm * 0.001
y_m = y_mm * 0.001
z_m = z_mm * 0.001
```

이 Gate는 식을 동결한다. 741개 waypoint에 이 식을 적용한 파일을 만들지 않는다.

길이 변환을 적용하지 않는 값:

- `orientation_quaternion`
- `surface_normal`
- `traversal_tangent`
- `tool_tangent`

## 프레임

```text
INPUT_FRAME = CAD_MODEL_FRAME
PLANNING_FRAME = ROBOT_PLANNING_FRAME
REQUIRED_TRANSFORM = ROBOT_PLANNING_FRAME T CAD_MODEL_FRAME
```

이 변환은 CAD model frame의 점을 robot planning frame으로 옮기는 의미다. 값이 없으면 planning으로 진행하지 않는다.

P-021은 요구만 정의한다. 획득, 검증, 적용은 P-022다.

## RobotPlanningInput

`RobotPlanningInput`은 계약 수준의 중간 표현이다. ROS 메시지 타입이 아니다.

```text
RobotPlanningInput != ProcessPath
RobotPlanningInput != RobotTrajectory
```

최소 의미:

- target planning frame. 이름은 `ROBOT_PLANNING_FRAME`이다.
- 위치 단위 m
- 순서가 있는 process pose. 위치는 위 mm→m 식을 나중에 적용한 `command_position`이다.
- orientation. quaternion 순서는 `x, y, z, w`이고 성분에는 길이 변환을 넣지 않는다.
- segment semantics. `segment_type`을 유지한다.
- 출처 ProcessPath reference. 경로 파일과 waypoint의 `sequence_id`를 가리킨다. 점 목록을 복사해 새 데이터로 만들지 않는다.

표면 metadata는 보존할 수 있다. `surface_anchor`, `surface_normal`, `traversal_tangent`, `tool_tangent`, `process_offset_mm`, `face_id`가 그 예다. `surface_anchor`를 보존하면 단위를 mm로 적는다. planning pose의 위치로 쓰지 않는다. metadata를 RobotTrajectory로 바꾸지 않는다.

이 Gate는 그 schema를 적는다. 741-entry `RobotPlanningInput` 데이터 파일은 만들지 않는다.

## Waypoint

```text
ProcessWaypoint != IK Goal
ProcessWaypoint != trajectory point
```

ProcessWaypoint 수와 IK goal 수, joint-space trajectory point 수는 같지 않아도 된다. P-013의 741개를 741개 IK goal이나 741개 RobotTrajectory point로 두지 않는다. 1:1 mapping을 강제하지 않는다. ProcessPath를 planning representation으로 바꾸는 정책은 P-022가 구현하고 검증한다.

## 책임

| 책임 | 소유 |
| --- | --- |
| ProcessPath contract validation | P-021 |
| mm → m 변환 계약 | P-021 |
| frame conversion requirement | P-021 |
| `RobotPlanningInput` schema | P-021 |
| responsibility boundary | P-021 |
| ROS 2 / MoveIt 2 integration | P-022 |
| robot model loading | P-022 |
| frame transform implementation | P-022 |
| planning representation | P-022 |
| Cartesian planning | P-022 |
| IK validation | P-023 |
| collision validation | P-023 |
| time parameterization | P-024 |
| RobotTrajectory | P-024 |
| physical execute | `OUT_OF_SCOPE` |

## 벤더

이 계약은 로봇 제조사에 종속되지 않는다. 특정 SDK나 API를 요구하지 않는다. 물리 로봇은 필요 없다. simulation 또는 planning-only 구현을 허용한다. 실물 관절 명령은 비목표다.

## 실패

- `FAIL_PROCESSPATH_CONTRACT`
- `FAIL_UNIT_CONTRACT`
- `FAIL_FRAME_CONTRACT`
- `FAIL_ADAPTER_OUTPUT_CONTRACT`
- `FAIL_RESPONSIBILITY_BOUNDARY`

## 합격

아래가 모두 PASS일 때만 `PASS_ROBOT_ADAPTER_CONTRACT`다.

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

P-022는 검토자가 이 증거를 본 뒤에만 후보가 된다. 이 문서는 P-022를 열지 않는다.

유지하는 의미: `GROUND_TRUTH = NONE`. CAD는 `REFERENCE_GEOMETRY`, scan은 `MEASURED_GEOMETRY`.
