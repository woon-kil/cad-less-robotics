# P-022

P-013 ProcessPath를 UR5e planning reference로 옮기고 MoveIt 2 Cartesian planning을 실행한 기록이다.

- 입력 검증과 단위·frame 변환: `input_validation.json`, `robot_planning_input.json`
- PROCESS run pose: `process_runs.txt`
- run별 fraction과 API 상태: `cartesian_runs.json`
- 패키지 버전: `environment.json`
- 집계: `P-022_REPORT.md`

`robot_planning_input.json`은 RobotTrajectory가 아니다. Cartesian 결과는 fraction만 남겼고 궤적 파일은 만들지 않았다.

최종 게이트는 `FAIL_CARTESIAN_API`다. 반환된 fraction 중 최솟값 0.606061도 threshold 0.95 아래다. datum, `eef_step`, threshold는 이 결과 뒤에 바꾸지 않았다.

원인 진단은 `failure_diagnosis.md`다. 같은 호출의 반복은 실패 run이 달라 수용 결과를 바꾸지 않는다.

timeout만 바꾼 진단은 `timeout_sensitivity.md`다. 검수는 `PASS_DIAGNOSTIC`이고 timeout만으로 repair하지 않는다.

Cartesian IK seed 순서는 `cartesian_ik_progression.md`다. run 20 joint trace는 `run20_joint_trace.md`다.

partial fraction 포착은 `partial_failure_capture.md`다. 기준선 fraction을 다시 만든 기록은 아니다. 9/34의 반환 경로는 `fraction_termination.md`다. 실패 interval의 중간점 오차 연결은 `interval_trace.md`다. repair 후보 비교는 `repair_candidate_selection.md`다. 적용하지 않았다. 경로 IK 거리 정의는 `path_ik_distance_metric.md`다. 0.059 rad와 0.403 rad 표본의 중간점 검사는 `ambiguous_joint_samples.md`다. joint 거리와 중간점 오차의 관계는 `joint_distance_proxy.md`다. 경로 IK continuity 계약은 `path_ik_continuity_contract.md`다. threshold 선정 절차는 `threshold_selection_protocol.md`다. 독립 calibration은 `calibration_distance.md`다. threshold 출처 검토는 `threshold_source.md`다. partial fraction repair 비교는 `partial_fraction_repair_alternatives.md`다. 시작 IK repair 판단은 `start_ik_repair.md`다. 종료 검토는 `final_closure_review.md`다. `T`는 없고, 구현한 repair도 없다. Reviewer가 `CLOSED_FAIL`을 승인했다. closure 적용 검수는 `PASS_CLOSURE_APPLIED`다. `reviewer_closed`는 true다. `FAIL_CARTESIAN_API`는 그대로다. P-023은 열지 않는다. 이 디렉터리의 진단과 측정은 다시 고치지 않는다.
