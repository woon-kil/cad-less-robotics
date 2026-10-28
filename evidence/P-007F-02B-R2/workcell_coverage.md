# P-007F-02B-R2 — Workspace Coverage

실측 근거: `frames/0000-color.png`, `rgb_annotated_overview.png`,
`roi_measurements.json`.

| 영역 | 확보 여부 |
|---|---|
| robot 앞 table area | 확보(workspace_bbox, 프레임의 32.6%) |
| shoe/last 배치 가능 영역 | checkerboard 자리 포함 개방 영역으로 확보 |
| table plane | 확보(table_bbox, 프레임의 43.8% — R1의 25.8%보다 넓음) |
| robot/workpiece relationship | 확보 — 로봇(상단)과 작업영역(하단)이 동일 프레임에서 자연스럽게 파악됨 |

## PASS 기준 재확인

로봇 bbox(5.8%) 대비 작업영역+테이블(32.6%+43.8%, 중첩 있음 — 실질적으로
테이블 전체가 더 넓게 잡힘)이 충분히 크다 — robot close-up 치우침 아님.
반대로 로봇이 너무 작아 motion witness 가 약화된 것도 아니다(robot bbox
비율 0.0582 는 R1의 0.0543 과 유사한 수준 — 작아지지 않았다).

## 참고 — 배경 변화

R1 대비 프레임 우측에 모니터/키보드 등 사무용 전자기기가 더 많이
보인다(시야각이 더 측면/낮아지며 방 안쪽이 더 보이게 됨). 이 영역은
workspace_bbox 바깥이라 커버리지 평가에 포함하지 않았다.
