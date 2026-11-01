# W-002E-R7 — ID2 물리 복구와 재실측

상태: `BLOCKED_ID2_CENTER_UNMEASURED_AND_SIDE_VIEW_ABSENT / CLOSED`

사용자는 ID2를 옮겼다고 했다. 그 다음 카메라, 로봇, 작업판, 다른 마커는 움직이지 않았다. PnP는 실행하지 않았다. 목표 `(450, 336)`은 실측값으로 복사하지 않았다.

## 검출

이동 후 color 한 장에서 DICT_4X4_50, 기본 코너 refinement로 ID 0, 1, 2, 3이 모두 프레임 안에 있다. `DETECTED_COUNT = 4`. `ALL_MARKERS_FULL_VISIBLE = YES`.

ID2의 corner0→corner1 방향은 다른 세 마커 중앙값과 6.3° 차이다. 한계 20°는 이 프레임을 보기 전에 W-002E-R4와 같게 고정했다. `ID2_ORIENTATION_CONTRACT_MATCH = YES`.

## 설치 상태

위쪽 color에서 ID2와 흡착판 사이에 나무 면이 보인다. 흡착판이 종이나 검은 사각형에 닿아 보이지 않는다. `SUCTION_HARDWARE_INTERFERENCE = NO`.

같은 각도는 시트 가장자리를 옆에서 보지 않는다. 종이가 나무에 완전히 붙어 있는지는 이 프레임으로 확정하지 않는다. 평탄과 틈은 `UNRESOLVED`다. 사진으로 mm 높이는 추정하지 않았다.

## 중심

`ID2_HISTORICAL_CENTER_MM = [500, 336]`  
`ID2_TARGET_CENTER_MM = [450, 336]`  
`ID2_MEASURED_CENTER_MM = NOT_MEASURED`

줄자 눈금을 이 세션에서 받지 않았다. 차이도 계산하지 않았다. 재설치 후 측면 사진도 없다. 위쪽 color와 이동 전 R6 측면 사진으로 평탄을 판정하지 않았다. 그래서 PASS로 닫지 않았다.

같은 물리 시트인지, 다시 인쇄했는지는 사실이 없어 적지 않았다.

`REGISTRATION_EXECUTED = NO`  
`VALIDATED_REGISTRATION_TRANSFORM = NONE`  
`W002E_ROOT_CAUSE = NOT_YET_CONFIRMED`

## 파일

- `result.json`
- `visibility.json`
- `id2_before_move.jpg`
- `id2_after_install.jpg`
- `four_marker_visibility.jpg` — 설치 후 원본과 같은 프레임
- `detected_ids.png` — 검출 표시. 원본을 대체하지 않는다.
- `crop_id2.jpg` — 설치 후 원본에서 ID2 주변만 자른 확인용. 원본은 그대로다.
