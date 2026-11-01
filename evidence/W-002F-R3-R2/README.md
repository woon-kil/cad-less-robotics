# W-002F-R3-R2 — 스마트폰 사진의 인쇄 마커

Gemini를 다시 찍지 않았다. 보드도 옮기지 않았다. 프로젝트에 넣은 사진 3장은 저장된 해상도 1024×576이다. 검출기는 R3과 같은 `DICT_5X5_100`, `CORNER_REFINE_NONE`이다. 주변 `DICT_4X4_50` 종이는 조회하지 않았고, 수락된 ID도 없다.

생성 PNG의 마커 ID는 0–16이다. 비스듬한 사진과 가까운 사진은 그 17개가 모두 수락됐다. 위에서 찍은 사진은 16개이고 ID 0만 빠졌다. 수락 ID 중에 0–16 밖은 없다.

같은 ID의 PNG 중심과 사진 중심을 잇는 2D homography의 중앙 잔차는 0.19–0.35 px다. 이 변환은 solvePnP가 아니고 등록 변환도 아니다. 인쇄된 마커가 생성 배치와 같은 자리에 있다는 비교다.

진단용 임계 127 비트 판독기는 비스듬한 사진에서 수락 ID와 어긋난다. 그 판독기는 OpenCV `identify()`가 아니므로 일치 판정에 쓰지 않았다.

```text
INDEPENDENT_ACCEPTED_MARKER_IDS = 0..16 on two photos; 1..16 on the overhead photo
EXPECTED_CELL_PATTERN_MATCH = YES
PHYSICAL_PRINT_PAYLOAD_PRESERVED = YES
PRINT_AS_SOLE_CAUSE = NOT_SUPPORTED
GEMINI_DECODING = UNCHANGED
REPRINT_REQUIRED = NO
REGISTRATION_EXECUTED = NO
NEXT_GATE_OPENED = NO
```
