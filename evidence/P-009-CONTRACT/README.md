# P-009 Contract Freeze

상태: `FROZEN`  
날짜: 2026-10-31

```text
CONTRACTS_FROZEN = YES
P-009 = NOT_OPENED
IMPLEMENTATION_AUTHORIZED = NO
```

이 디렉터리는 구현 전에 고정한 계약이다. ProcessPath를 만들지 않았고, 점군을 다시 분석하지 않았으며, 합격 평가를 하지 않았다.

숫자의 정본은 이 디렉터리의 JSON이다. 이 글은 그 파일의 한글 색인이다.

| 계약 | 파일 |
| --- | --- |
| `P009_PROCESS_SPEC_CONTRACT` | `process_spec_contract.json` |
| `P009_MEASURED_SURFACE_CONTRACT` | `measured_surface_contract.json` |
| `P009_PROCESSPATH_CONTRACT` | `process_path_contract.json` |
| `P009_FRAME_CONVENTION` | `frame_convention.json` |
| `P009_NORMAL_CONVENTION` | `normal_convention.json` |
| `P009_TANGENT_CONVENTION` | `tangent_convention.json` |
| `P009_ORIENTATION_CONVENTION` | `orientation_convention.json` |
| 표면 연관 | `surface_association_contract.json` |
| `P009_SPACING_CONTRACT` | `spacing_contract.json` |
| `P009_SMOOTHING_CONTRACT` | `smoothing_contract.json` |
| `P009_CONFIDENCE_CONTRACT` | `confidence_contract.json` |
| `P009_CONTINUITY_CONTRACT` | `continuity_contract.json` |
| `P009_ACCEPTANCE_METRICS` | `acceptance_metrics.json` |

## 위치 세 개

`ProcessPathPoint.position`은 쓰지 않는다.

`base_surface_anchor`는 offset을 적용하기 전의 측정 표면 점이다. `surface_anchor`는 요청한 공정 좌표를 측정 기하에 다시 붙인 점이다. 표면 거리, 접선, 법선 연관, 간격, 연속은 이 점을 본다.

`command_position = surface_anchor + tool_standoff_m * surface_normal`이다. 법선은 표면에서 카메라·빈 공간 쪽이다. 양수 standoff는 작업물에서 멀어진다. 이 위치는 아직 로봇 base 좌표가 아니다.

`surface_offset_m`은 법선 방향 standoff가 아니다. 접선 평면의 `Y_process` 방향 요청이고, 더한 뒤 같은 kNN 평면 투영으로 다시 붙여 `surface_anchor`를 만든다. `base_anchor + offset * Y`를 재연관 없이 anchor로 두지 않는다.

## 연관

방법은 `KNN_LOCAL_PLANE_PROJECTION`이다. K는 20, 유효 이웃은 12개 이상, k번째 이웃 거리는 0.015 m 이하다. 이 거리는 검색 반경을 늘려도 된다는 뜻이 아니다. 투영만으로 유효가 되지 않는다. 실패는 `REJECTED_UNSUPPORTED_GEOMETRY`다. 반경 확대, 다른 K, 최근접 대체, MLS, mesh, 먼 표면 검색, 간격 보간, 수동 대체를 쓰지 않는다.

`SOURCE_SURFACE_DISTANCE`의 P95는 0.003 m 이하, 최대는 0.005 m 이하다. 이것은 anchor와 지지 XYZ의 내부 일치이며, Gemini·TCP·로봇의 절대 정확도가 아니다.

## 프레임과 자세

기하는 측정 표면 frame에 둔다. 개발 데이터는 `camera_color_optical_frame`이다. `ProcessPath.frame_id`는 입력 frame과 같다.

`X_process`는 anchor에서 만든 접선이다. `Z_process = -surface_normal`이다. `Y_process`는 `Z × X`를 정규화한 값이고, X는 `Y × Z`로 다시 직교화한다. Quaternion 순서는 `[x, y, z, w]`다. `lateral`은 이 `Y_process`다.

## 합격 층

간격 목표는 0.005 m, 상대 허용은 ±20%다. 끝이 아닌 구간은 0.004–0.006 m다. smoothing 최대 이동은 anchor 기준 0.003 m다. 법선 유효 비율은 0.95 이상이다. 위치 점프는 0.010 m를 자동으로 잇지 않는다. 설명되지 않은 자세 단계는 20°를 넘지 않는다. 측지각은 `2 * acos(|q_i · q_{i+1}|)`이다. `ORIENTATION_STEP_P95`는 보고만 하고, 추가 합격 임계값은 없다.

경로 길이 0.100 m는 R1L의 자동 안전 표면 기준을 재사용하거나 바꾼 것이 아니다. R1L 기록은 그대로다.

로봇 모델, 관절, MoveIt, base 변환, hand-eye는 모두 필요 없다.

P-009 안에서 신발 분리, 배경 분류, 둘레 검출, 옆면 전환 발견, 접착선 발견, 최적 공정 영역 발견, 다시점 정합을 하지 않는다. 그것이 필요하면 `FAIL_PROCESS_SPEC_CONTRACT`다.

`STANDOFF_ERROR_MAX`와 `LATERAL_COMMAND_RESIDUAL_MAX`는 보고한다. 센서 정확도 임계값으로 새로 정하지 않았다.
