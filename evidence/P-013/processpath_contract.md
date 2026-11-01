# P-013 CAD 6D ProcessPath 계약

상태: `PASS_CONTRACT_FROZEN`

구현: `NOT_STARTED`

P-014 이후는 열지 않는다. 이 문서는 경로를 만들지 않는다.

입력 STEP은 `assets/cad/complex_surface_coupon.step`이다. SHA-256은 `2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa`다. 이 파일과 P-011 `geometry_contract.md`는 수정하지 않는다.

## Purpose

P-012가 읽은 CAD 곡면에서 robot-independent 6D ProcessPath를 만들기 위한 의미만 고정한다.

CAD 경로는 reference geometry에서 나온 경로다. ground truth path가 아니다. `GROUND_TRUTH = NONE`.

## Inputs

| 항목 | 값 |
| --- | --- |
| P-011 | `PASS_COMPLEX_SURFACE_CAD_TEST_COUPON` / `CLOSED` |
| P-012 | `PASS_CAD_GEOMETRY_ENGINE` / `CLOSED` |
| STEP | 위 SHA-256 |
| 기하 | P-012의 face, UV, 위치, oriented normal |

P-012 샘플을 경로 점으로 그대로 쓰지 않는다. 구현이 허용된 뒤에 같은 곡면 API로 anchor를 평가한다.

## P-009 호환

scan branch 계약은 바꾸지 않는다. 정본은 `evidence/P-009-CONTRACT/`다.

유지하는 의미:

- `surface_anchor`는 표면 위의 기하 점이다. 최종 TCP도 로봇 위치도 아니다.
- `command_position = surface_anchor + tool_standoff * surface_normal`
- 양의 standoff는 가공물에서 멀어지는 방향이고, `surface_normal`을 따라 free space로 간다.
- 표면 거리와 tangent의 평가 대상은 `command_position`이 아니라 `surface_anchor`다.
- `Z = -surface_normal`
- `Y = normalize(Z × X)`, `X`는 `normalize(Y × Z)`로 다시 직교화한다.
- quaternion 순서는 `x, y, z, w`다.
- `q`와 `-q`는 같은 회전이다. 부호를 맞추는 것은 물리 자세를 바꾸지 않는다.
- 법선을 인접 각도를 줄이려고 뒤집지 않는다.

CAD 전용 수치는 scan 계약의 수치를 대체하지 않는다. P-009의 표면 거리 3 mm / 5 mm, orientation step 20°, confidence의 kNN 조건은 scan branch에 그대로 둔다.

## Source

| branch | SOURCE | GEOMETRY_ROLE |
| --- | --- | --- |
| CAD | `CAD` | `REFERENCE_GEOMETRY` |
| Scan | `SCAN` | `MEASURED_GEOMETRY` |

`GROUND_TRUTH = NONE`

## Process Surface

공정 표면은 face 순번으로 고르지 않는다. P-011의 region 이름도 선택 조건으로 넣지 않는다.

면이 공정 표면인 조건은 다음 두 가지를 모두 만족하는 것이다.

1. P-012 surface type이 `BSpline`이다.
2. 그 면의 valid sample에서 oriented normal을 평균하면 `Nz > 0.5`이다.

이 규칙은 옆면, 바닥, landmark 원통과 끝 뚜껑을 제외한다. BREP face의 경계는 공정 경계가 아니다.

동결 STEP에 이 규칙을 적용하면 윗면 4개가 남는다. P-012 inventory에서는 `F10`, `F11`, `F12`, `F13`이다. 이 ID는 규칙의 결과이지 규칙 자체가 아니다.

## Path Strategy

첫 CAD ProcessPath는 raster / serpentine이다.

- 한 pass의 진행은 CAD 모델 `+X`가 기본 major direction이다.
- 짝수 pass는 `+X`, 홀수 pass는 `-X`다.
- pass에서 pass로의 이동은 `+Y`다.
- contour following과 curvature-adaptive spacing은 넣지 않는다.

점과 tangent는 CAD 곡면 위에서 구한다. XY 평면에 경로를 만든 뒤 Z만 덮어쓰지 않는다.

pass 사이 간격 `PASS_PITCH_MM`은 5.0이다. 이 값은 waypoint 간격과 같은 수로 미리 고정한 사양이다. 구현 결과를 보고 바꾸지 않는다.

## Surface Anchor

`SURFACE_ANCHOR = P_surface`

`P_surface`는 위 공정 표면의 union 위에 있어야 한다. 표면 거리는 anchor에 대해 계산한다.

CAD 구현의 합격선은 P95 ≤ 0.10 mm다. 이것은 해석 BREP에 대한 수치 허용이고, P-009 scan 거리 기준을 대체하지 않는다.

## Command Position

`tool_standoff_mm = 20.0`

이것은 P-009 `tool_standoff_m = 0.020`과 같은 양이다. 부호는 P-009와 같다. 양수이면 `surface_normal` 방향으로 가공물에서 멀어진다.

```text
COMMAND_POSITION = SURFACE_ANCHOR + surface_normal * tool_standoff_mm
```

작업 의뢰의 `process_offset`은 이 standoff를 가리킨다. P-009의 `surface_offset`이 아니다. `surface_offset`은 접선 방향 공정 요청이고, 이번 coupon에서는 0이다. ADR-008 본문의 `process_offset_m: 0.010` 예시도 이 standoff가 아니다. 그 문장은 설명이고 P-009 schema를 고정하지 않는다고 적혀 있다.

`tool_z = -surface_normal`과 위의 command 식은 모순이 아니다.

- `offset_direction = +surface_normal`: 비접촉 위치를 표면 밖으로 옮긴다.
- `tool_z = -surface_normal`: 공구 Z가 표면을 향한다.

경로 품질의 표면 연관은 command가 아니라 anchor로 평가한다.

P-013 안에서는 mm를 m로 바꾸지 않는다. 로봇 adapter가 나중에 m로 바꾼다.

## Normal

각 anchor의 `N_surface`는 P-012 oriented CAD normal이다.

- finite
- 단위 벡터
- 고체 바깥을 향하는 face orientation을 유지한다.
- camera를 보도록 뒤집지 않는다.
- 인접 법선의 각도를 줄이려고 뒤집지 않는다.

인접 waypoint의 `dot(N_i, N_i-1)`을 기록한다. face 경계에서 불연속이면 그 사실을 남긴다. `NORMAL_FLIP_COUNT = 0`.

## Path tangent와 tool tangent

serpentine은 pass마다 진행 방향이 180° 바뀐다. P-009는 `tangent follows traversal`이고, scan 경로의 traversal은 `FORWARD` 한 방향이다. 그 문장을 반전 pass에 그대로 넣으면 tool X도 180° 뒤집힌다.

그때 tool frame은 Z축으로 180° 회전한다. quaternion 부호 변경이 아니다. `delta_theta`는 약 180°다. 그래서 진행 방향과 tool roll을 분리한다.

| 벡터 | 의미 |
| --- | --- |
| `traversal_tangent` | 그 pass를 걸어가는 방향. 다음 pass에서 반전된다. 순서와 간격에만 쓴다. |
| `tool_tangent` | tool frame의 X를 만드는 방향. pass가 반전되어도 따라 뒤집지 않는다. |

`tool_tangent`는 CAD 모델 `+X`를 접평면에 투영한 벡터다. 직전 waypoint의 `tool_tangent`와 내적이 음수이면 부호만 맞춘다. 이 부호 맞춤은 진행 방향을 되돌리는 것이 아니다.

P-009 scan branch의 tangent 정의는 수정하지 않는다.

PROCESS pass 안의 traversal tangent는 anchor의 중심 차분이다. pass 끝은 그 pass 안의 one-sided difference만 쓴다. 반대편 pass의 anchor를 끝점 차분에 넣지 않는다.

## Tool Frame

```text
Z = -N_surface
X = normalize(tool_tangent - dot(tool_tangent, Z) * Z)
Y = normalize(Z × X)
X = normalize(Y × Z)
R = [X Y Z]
```

`tool_tangent`가 Z와 평행해 투영 길이가 0이면 그 waypoint는 실패다. 다른 축으로 조용히 대체하지 않는다.

검증:

- `|X|`, `|Y|`, `|Z|`는 1에 가깝다.
- `X·Y`, `Y·Z`, `Z·X`는 0에 가깝다.
- `det(R)`은 `+1`에 가깝다.

수치 허용은 `1e-6`이다. P-009의 `≈` 문장을 바꾸자는 뜻이 아니다.

## Quaternion

`R = [X Y Z]`에서 `q = [qx, qy, qz, qw]`를 만든다.

- finite
- `|q|`는 1에 가깝다. 허용 `1e-6`

순차 waypoint에서 `dot(q_i, q_i-1) < 0`이면 `q_i = -q_i`다. representation continuity다. 물리 회전을 숨기는 smoothing이 아니다.

물리 회전은 따로 잰다.

```text
delta_theta = 2 * acos(clamp(|dot(q_i, q_i-1)|, 0, 1))
```

단위는 degree다. P-009 orientation difference와 같은 식이다.

## Face Boundary

공정 표면은 BREP face 4개다. face마다 경로를 만들어 이어 붙이지 않는다.

하나의 serpentine이 네 면의 union을 지나간다. 면 경계는 CAD topology 경계일 뿐 process path 경계가 아니다. 경계를 넘는 구간도 pass 안이면 `PROCESS`다.

경계마다 다음을 남긴다.

- 경계 양쪽 anchor의 거리
- 법선 사이 각
- traversal tangent의 연속

이 기록이 곧 합격은 아니다. 불연속이면 실패로 남긴다.

## Spacing

`TARGET_WAYPOINT_SPACING_MM = 5.0`

P-009 `spacing_m = 0.005`와 같은 간격이다.

PROCESS 구간의 정상 segment:

```text
4.0 mm <= spacing <= 6.0 mm
```

pass를 잇는 connector는 `TRANSITION`이다. connector 길이를 PROCESS 간격 통계에 넣지 않는다.

## Segment Types

| 타입 | 의미 |
| --- | --- |
| `PROCESS` | 한 pass를 따라 공정 표면을 지나는 구간 |
| `TRANSITION` | pass 끝에서 다음 pass 시작으로 넘어가는 connector |

`APPROACH`와 `RETRACT`는 이번에 만들지 않는다.

표면 거리, PROCESS 간격, PROCESS orientation 45° 기준은 `PROCESS`에만 적용한다. `TRANSITION`은 별도 통계다.

pass 반전 자체를 `PROCESS` segment로 두면 안 된다.

## ProcessWaypoint

```text
ProcessWaypoint
{
    surface_anchor
    command_position
    orientation_quaternion
    surface_normal
    traversal_tangent
    tool_tangent
    tool_standoff_mm
    source
    geometry_role
    face_id
    segment_type
    confidence
}
```

- `source = CAD`
- `geometry_role = REFERENCE_GEOMETRY`
- `tool_standoff_mm = 20.0`
- `face_id`는 anchor가 놓인 BREP 면이다. 공정 경계 ID가 아니다.
- `confidence = NOT_DEFINED`

CAD라는 이유로 confidence를 1.0으로 두지 않는다. P-009 confidence는 측정 이웃과 PCA의 조건이다. 그 숫자를 CAD에 복사하거나 새 척도를 만들지 않는다.

## Units / Frames

| 항목 | 값 |
| --- | --- |
| `frame_id` | `CAD_MODEL_FRAME` |
| `position_unit` | mm |
| `curvature_unit` | 1/mm. 이번 경로 계약의 합격 조건에는 쓰지 않는다. |
| `quaternion_order` | `x, y, z, w` |

`CAD_MODEL_FRAME`은 동결 STEP의 좌표다. `camera_color_optical_frame`도 로봇 base도 MoveIt frame도 아니다.

P-009의 불변은 유지한다. 경로 frame은 그 기하의 frame과 같다. scan 경로는 measured surface frame에 남고, CAD 경로는 CAD model frame에 남는다. 둘을 하나의 변환으로 섞지 않는다.

## Quality Metrics

구현이 허용되면 최소 다음을 잰다.

- `WAYPOINT_COUNT`
- `PROCESS_SEGMENT_COUNT`
- `TRANSITION_SEGMENT_COUNT`
- `PROCESS_PATH_LENGTH_MM`
- `SPACING_MEDIAN_MM`
- `SPACING_P95_ERROR_MM`
- `SURFACE_DISTANCE_MEDIAN_MM`
- `SURFACE_DISTANCE_P95_MM`
- `SURFACE_DISTANCE_MAX_MM`
- `NORMAL_DOT_MIN`
- `ORIENTATION_DELTA_MEDIAN_DEG`
- `ORIENTATION_DELTA_P95_DEG`
- `ORIENTATION_DELTA_MAX_DEG`
- `QUATERNION_NORM_MIN`
- `QUATERNION_NORM_MAX`
- `FRAME_ORTHOGONALITY_MAX_ERROR`
- `ROTATION_DET_MIN`
- `ROTATION_DET_MAX`

PROCESS와 TRANSITION의 orientation delta는 따로 보고한다.

## PASS / FAIL

구현 단계의 기준은 지금 고정한다. 결과를 본 뒤 통과시키려고 바꾸지 않는다.

- `WAYPOINT_COUNT >= 100`
- PROCESS spacing은 5.0 mm의 ±20% 안에 있다.
- NaN과 Inf는 0이다.
- anchor의 표면 거리 P95는 0.10 mm 이하다.
- `NORMAL_FLIP_COUNT = 0`
- quaternion norm은 1 ± `1e-6`
- `det(R)`는 `+1` ± `1e-6`
- PROCESS에서 `delta_theta > 45°`인 경우는 0이다.
- TRANSITION은 별도 통계다. 이 0건 조건을 맞추는 데 쓰지 않는다.

serpentine 반전으로 tool frame이 180° 도는 구조는 위 분리로 만들지 않는다. 반전이 PROCESS 45° 조건을 깨면 계약을 느슨하게 하지 않고 실패다.

## ProcessPath와 RobotTrajectory

```text
ProcessPath = Cartesian task geometry, robot-independent
RobotTrajectory = joint-space, robot-specific, time-parameterized
```

P-013은 RobotTrajectory를 만들지 않는다.

## Non-Goals

이번에 하지 않는 것:

- path 생성 코드
- waypoint 생성
- quaternion 계산 실행
- ProcessPath 결과 파일
- contour following
- curvature-adaptive spacing
- approach / retract
- normal smoothing
- camera-facing flip
- scan adaptation
- Gemini capture
- CAD↔Scan registration
- ROS 2, MoveIt, IK, collision, RobotTrajectory
- P-014

## 계약 점검

```text
P-013_CONTRACT = PASS_CONTRACT_FROZEN
P-013_IMPLEMENTATION = NOT_STARTED

P009_COMPATIBILITY = PASS
PROCESS_SURFACE_DEFINED = PASS
PATH_STRATEGY_DEFINED = PASS
SURFACE_ANCHOR_DEFINED = PASS
COMMAND_POSITION_DEFINED = PASS
NORMAL_CONVENTION_DEFINED = PASS
TOOL_FRAME_DEFINED = PASS
QUATERNION_CONVENTION_DEFINED = PASS
FACE_BOUNDARY_POLICY_DEFINED = PASS
SPACING_DEFINED = PASS
SEGMENT_TYPE_DEFINED = PASS
QUALITY_METRICS_DEFINED = PASS
PASS_THRESHOLDS_FROZEN = PASS
PROCESSPATH_VS_ROBOTTRAJECTORY = PASS
```
