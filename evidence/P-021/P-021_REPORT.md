# P-021 Robot Adapter Contract

계약 Gate다. 코드와 변환 데이터는 없다.

입력 `evidence/P-013/cad_process_path.json`은 741개다. `frame_id`는 `CAD_MODEL_FRAME`, 위치 단위는 mm, quaternion 순서는 `x, y, z, w`, `source`는 CAD, `geometry_role`은 `REFERENCE_GEOMETRY`다.

필수 필드는 빠지지 않았다. nonfinite는 0이고 차원 오류는 0이다. quaternion norm은 0.9999999999464859에서 1.0000000000550346이다. `surface_anchor`와 `command_position`이 같은 waypoint는 0개다. `command_position - (surface_anchor + surface_normal * 20)`의 최대 잔차는 1.000017846308765e-09 mm다. `process_offset_mm`는 전부 20.0이고, JSON에 `tool_standoff_mm` 키는 없다. 의미는 P-013 계약의 법선 standoff다. `confidence`는 741개 모두 `NOT_DEFINED`이며 필수 항목이 아니다.

단위 계약은 `x_m = x_mm * 0.001`이다. 실제 변환은 하지 않았다. quaternion, normal, tangent에는 길이 변환을 넣지 않는다.

필요 변환은 `ROBOT_PLANNING_FRAME T CAD_MODEL_FRAME`이다. 실제 변환은 하지 않았다. 없으면 planning으로 진행하지 않는다.

출력 계약은 `RobotPlanningInput`이다. ProcessPath도 RobotTrajectory도 아니다. 741-entry 데이터 파일은 없다.

ProcessWaypoint는 IK goal도 trajectory point도 아니다. 741개를 741개 IK goal로 두지 않는다.

판정은 `PASS_ROBOT_ADAPTER_CONTRACT`다. 검토 수용은 PASS다. P-022는 허용만 됐고 구현하지 않았다. `1e-9 mm` 잔차는 저장 수치와 계약식의 일치이며 로봇 위치 정확도가 아니다.
