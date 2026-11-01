# P-021

Robot Adapter Contract를 동결했다. 정본은 `robot_adapter_contract.md`다. 대조 수치는 `validation.json`이다.

입력은 P-013의 `cad_process_path.json` 741개 waypoint다. mm→m 식만 적고, 변환된 경로 파일과 `RobotPlanningInput` 데이터는 만들지 않았다. ROS 2, MoveIt, IK, 충돌 검사, RobotTrajectory, 실물 실행은 없다.

구현 측 기록은 `PASS_ROBOT_ADAPTER_CONTRACT`다. 검토 수용은 `reviewer_accepted = true`다. P-022는 허용만 됐고 구현하지 않았다.
