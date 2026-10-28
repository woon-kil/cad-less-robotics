# P-007F-02B-R1 — Workcell Coverage

실측 근거: `frames/0000-color.png`, `roi_grid_overlay.png`,
`roi_measurements.json`.

## 확보 여부

| 영역 | 확보 여부 |
|---|---|
| robot 영역 | 확보(x640-840, y85-335, 프레임의 5.4%) |
| robot 앞 작업영역 | **확보**(x555-975, y330-700, 프레임의 16.9%) — checkerboard calibration target 포함 |
| shoe/last 배치 가능 영역 | 작업영역과 겹침 — 현재 checkerboard 가 놓인 바로 그 자리가 후보가 될 수 있음 |
| table plane | 확보(x555-980, y140-700, 프레임의 25.8%) |
| robot↔workpiece 관계 | **확보** — 로봇(상단)과 작업영역(하단, checkerboard 포함)이 동일 프레임에서 동시에 보이고 상대 위치가 자연스럽게 파악됨 |

## PASS 기준 재확인 — 화면이 robot close-up 에 치우쳐 있지 않은가

로봇 bbox 면적 비율(5.4%)이 작업영역+테이블 비율(16.9%+25.8%)에 비해
훨씬 작다 — **robot close-up 에 치우친 구도가 아니다.** 오히려 작업대
전면의 열린 공간이 프레임의 상당 부분을 차지한다. 이 항목은 PASS.

## 참고 — 프레임 좌측의 배경 혼잡

프레임 좌측(x0-550)에는 이불/베개 등 작업과 무관한 물체가 많다 — 이 영역은
workspace/table bbox 바깥이므로 커버리지 평가에 포함하지 않았다. 다만
배경 특징점 추적(DaBai 식 ego-motion 과 유사한 용도로 Gemini 를 쓸 경우)
안정성 평가 시 이 혼잡한 영역이 아니라 table/robot 인접 영역의 특징을
쓰는 것이 바람직하다는 점만 기록한다(이번 gate 범위 밖, 결정하지 않음).
