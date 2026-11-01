# Post-feasibility roadmap decision

```text
POST_FEASIBILITY_ROADMAP_DECISION = CONTINUE_WITH_VERIFIED_EVIDENCE_VISUALIZATION
POST_FEASIBILITY_ROADMAP_DECISION_REVIEW = PASS_POST_FEASIBILITY_ROADMAP_DECISION
DECISION_OPTION = B
DECISION_EXECUTED = YES
NEXT_ALLOWED_GATE = P025
P025_GATE_OPEN_REVIEW = PASS_GATE_OPEN
P025_IMPLEMENTATION_PLAN_REVIEW = PASS_IMPLEMENTATION_PLAN
P025_IMPLEMENTATION_PLAN = APPROVED
NEXT_ALLOWED_WORK = P025_IMPLEMENTATION
P025_GATE = OPEN
P025_CONTRACT = FROZEN
P025_SCOPE = VERIFIED_END_TO_END_ENGINEERING_EVIDENCE_VISUALIZATION
P025_IMPLEMENTATION = STARTED
UPSTREAM_EVIDENCE = READ_ONLY
P025_PASS = NOT_EVALUATED

ROBOT_TRAJECTORY_RESEARCH = STOP_CURRENT_BRANCH
ROBOT_TRAJECTORY_RESEARCH_CURRENT_BRANCH = STOPPED
NEW_PLANNING_RESEARCH_BRANCH = NOT_OPENED
TECHNICAL_EXPERIMENTS = STOPPED

P022 = CLOSED_FAIL_IMMUTABLE_BASELINE
P023 = NOT_OPENED
P024 = NOT_OPENED
P025 = OPEN
P025_DEFINITION_CHANGED = NO
P025_EXISTING_DEFINITION = SUPERSEDED_TEXT_RETAINED
P025_CONTRACT_REVIEW = PASS_CONTRACT_FROZEN
P025_CONTRACT_FROZEN = YES
SUPERSESSION_EFFECTIVE = YES

OPTION_A = NOT_SELECTED
OPTION_B = SELECTED
OPTION_C = NOT_SELECTED
```

feasibility 검토는 닫혔다. 판정은 `NO_JUSTIFIABLE_ALTERNATIVE`다. 이 문서가 그 다음 선택이다. 선택은 B다. RobotTrajectory 연구는 현재 분기에서 멈춘다. P-025는 열지 않고, 기존 정의 본문도 아직 바꾸지 않는다.

선택지는 세 개다.

| 선택 | 의미 | 상태 |
| --- | --- | --- |
| A | RobotTrajectory 분기를 여기서 끝낸다. P-024로 진행하지 않는다 | NOT_SELECTED |
| B | 궤적 연구는 멈추고, 검증된 pipeline과 실패를 시각화한다 | SELECTED |
| C | P-022와 다른 contract의 research 분기를 연다 | NOT_SELECTED |

A는 기술적으로 멈추기에는 깨끗하지만, 확보한 결과를 짧은 설명으로 전달하지 못한다. C는 새 contract의 연구라서 비용과 불확실성이 크다. B는 새 planning 성공을 만들지 않는다. 이미 검증된 경로와 검증된 실패를 보여 주는 선택이다.

기존 P-025가 성공한 RobotTrajectory를 보여주는 정의라면 그 정의 그대로 열 수 없다. 보여 줄 범위는 CAD geometry, 6D ProcessPath, RobotPlanningInput, UR5e MoveIt Cartesian planning, 측정된 성공과 실패다. P-022 `CLOSED_FAIL`, 실패 위치, 측정된 Cartesian fraction, feasibility 결과가 포함된다. RobotTrajectory 생성, collision-free, trajectory-valid, 실물 실행, calibration accuracy, production readiness는 주장하지 않는다.

이 문장을 지금 P-025 본문에 넣지 않는다. 다음은 contract 재정의뿐이다. 그 contract를 검토한 뒤에만 P-025를 연다.

비교 기준은 포트폴리오에서 남은 시간을 어디에 쓸 때 증거가 강한가다. RobotTrajectory를 어떻게든 끝내는 문제가 아니다. 기술 실험은 멈추었다.

| 기준 | A 종료 | B 재정의 후 시각화 | C 새 planning research |
| --- | --- | --- | --- |
| 현재 evidence 활용 | 높음 | 매우 높음 | 중간 |
| 추가 기술 리스크 | 낮음 | 낮음 | 높음 |
| 개발 시간 | 낮음 | 중간 | 높음 |
| Motion Planning 성공 증명 | 없음 | 없음 | 가능하지만 불확실 |
| Geometry와 ProcessPath 전달 | 중간 | 매우 높음 | 중간 |
| 실패 분석 전달 | 높음 | 매우 높음 | 높음 |
| 짧은 설명으로의 이해 | 중간 | 높음 | 낮아질 위험 |
| 기존 evidence 훼손 위험 | 낮음 | 낮음 | 높음 |
| 포트폴리오 쪽 투자 대비 | 중간 | 높음 | 현재는 낮음 |

위 표는 선택 전의 비교다. 선택은 그 뒤에 B로 기록됐다. 기존 정의는 아직 대체되지 않았다. 상태 값은 `TO_BE_SUPERSEDED_BY_EXPLICIT_CONTRACT_REVISION`이다. contract 본문이 검토되기 전에는 `SUPERSEDED`로 적지 않는다.
