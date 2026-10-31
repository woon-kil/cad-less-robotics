# W-002B — Workboard fiducial layout과 registration 계약

상태: `PASS_REGISTRATION_CONTRACT / CLOSED`  
날짜: 2026-10-31

설계와 계약만 고정한다. 마커는 붙이지 않았고, registration은 실행하지 않았다. WorkpieceInstance 방식은 고르지 않았다.

W-002A의 6개 조사점은 검출이 된 위치다. 최종 설치 좌표가 아니다. 연속 가시성도 아니다.

## 물리 원점

`workboard`는 오른손 좌표계다. 마커 ID에 기대지 않는다. 마커를 바꿔도 이 정의는 그대로다.

작업판은 801 mm × 400 mm다.

사람이 이미 조사에 쓴 방향과 같다.

- `(0,0,0)`은 카메라 스탠드 쪽 짧은 변과, 카메라에 가까운 긴 변의 교점이다.
- `+X`는 그 가까운 긴 변을 따라 로봇 쪽이다. 길이는 801 mm다.
- `+Y`는 카메라 스탠드 쪽 짧은 변을 따라 먼 긴 변 쪽이다. 길이는 400 mm다.
- `+Z`는 윗면에서 free-space 쪽이다.

`(801,0,0)`, `(0,400,0)`, `(801,400,0)`은 나머지 세 모서리다. 원점은 ArUco 중심이 아니다.

클램프가 그 모서리 점을 덮는지는 이번에 재지 않았다. 기준은 나무판의 두 변이다. 마커가 아니다.

## 후보 배치

사전은 `DICT_4X4_50`, ID는 0, 1, 2, 3이다. 검은 사각형은 60 mm, 흰 quiet zone은 10 mm 이상이다. 중심의 `z`는 0이다. 종이·테이프 두께는 재지 않았고, 무시할 수 있다고 하지 않는다. 두께를 나중에 재서 1 mm를 넘으면 layout 버전을 바꾼 뒤에야 registration에 넣는다.

네 마커의 인쇄 위쪽 변은 `workboard +X`와 평행하다. 회전으로 ID를 나누지 않는다.

로컬 코드 `object_points()`는 검출 순서를 TL, TR, BR, BL로 두고, 꼭짓점 0→1이 마커 `+X`, 위쪽 변이 마커 `+Y` 쪽, `+Z`가 인쇄면에서 바깥이다. 회전 0은 그 세 축을 workboard의 `+X`, `+Y`, `+Z`와 같게 둔다.

후보만 적는다. 설치 동결이 아니다.

| ID | 중심 mm | 회전 deg |
| --- | --- | --- |
| 0 | (150, 70, 0) | 0 |
| 1 | (500, 70, 0) | 0 |
| 2 | (500, 330, 0) | 0 |
| 3 | (150, 330, 0) | 0 |

이 넷은 조사에서 실제로 검출된 위치 중 한 직선에 있지 않고, X와 Y로 가장 넓다. `(300,70)`과 `(300,330)`은 그 사각형의 변 위에 있어 기준 길이를 늘리지 않으므로 쓰지 않는다.

`LAYOUT_STATUS = REQUIRES_VISIBILITY_VALIDATION`

한 장씩 본 조사와, 네 장이 동시에 붙은 상태는 다르다. 관측하지 않은 안쪽 좌표를 더 안전하다고 두고 합격 처리하지 않았다. 설치 전에 이 네 좌표의 동시 가시를 따로 확인해야 한다.

## Registration

입력은 RGB, intrinsics, 검출된 코너, 마커의 알려진 형상과 workboard 자세다. W-002A의 중심 depth만으로 변환을 만들지 않는다. 중심 depth는 관측 증거다.

변환 이름은 `T_camera_workboard` 하나다.

```text
p_camera = T_camera_workboard * p_workboard
```

카메라 프레임은 `camera_color_optical_frame`이다. 역변환은 이 행렬의 역으로 계산한다. 두 번째 calibration 값으로 저장하지 않는다.

방법은 B다. 네 마커의 알려진 3D 코너와 검출된 2D 코너를 하나의 PnP로 푼다.

A는 마커마다 자세를 낸 뒤 평균한다. 하나의 강체 자세를 넷으로 나누고, 나쁜 마커가 평균을 당긴다.

C의 GridBoard는 일정 간격의 격자다. 이 배치는 격자가 아니다.

OpenCV의 custom Board pose도 같은 대응을 하나의 PnP로 넣는 경우에만 B와 같다. 구현이 쉬워서 B를 고른 것이 아니다. 작업판에 퍼진 대응이 하나의 카메라 자세이기 때문이다.

## 합격 수치

registration을 실행하기 전에 고정한다. 결과를 본 뒤 낮추지 않는다.

검출의 정상 조건은 4/4다. 수치가 성립하는 최소는 3개다. 3개면 nominal이 아니다. 기대 ID 밖의 검출은 `MARKER_ID_MISMATCH`다.

Reprojection은 픽셀로 판정한다.

| 지표 | 상한 |
| --- | --- |
| mean | 1.0 px |
| p95 | 2.0 px |
| max | 3.0 px |

환산은 설명이다. R1I color intrinsics의 fx는 610.507 px이고, 숫자가 있는 조사 깊이 중 가장 먼 `(500,330)`은 0.795 m다. 그때 1 px는 1.302 mm다. 1, 2, 3 px는 1.302, 2.604, 3.907 mm다.

작업판 평면은 같은 코너 잔차와 다른 검사다. 점군을 `workboard`로 옮긴 뒤, 마커·로봇·마운트·배경이 아닌 맨 나무면에서 본다. 그 영역은 잔차를 보기 전에 표시한다.

| 지표 | 상한 | 반올림 |
| --- | --- | --- |
| \|z\| 평균 | 2 mm | 1 px 환산 1.302 mm 이상의 최소 정수 mm |
| \|z\| p95 | 4 mm | 3 px 환산 3.907 mm 이상의 최소 정수 mm |
| 법선과 +Z의 각 | 1.0 deg | 350 mm 기준 길이에서 4 mm를 넘는 최소 정수 도. `atan(4/350)`은 0.655 deg |

반복은 카메라와 작업판을 그대로 둔 독립 프레임 10장 이상이다. 기준 자세는 10개의 중앙값이다. 첫 프레임을 기준으로 쓰지 않는다.

| 지표 | 상한 |
| --- | --- |
| 병진이 중앙값에서 떨어진 최대 거리 | 2 mm |
| 회전이 중앙값에서 떨어진 최대 각 | 0.4 deg |

0.4 deg는 `atan(2/350)` = 0.327 deg 이상의 최소 0.1 deg다. 350 mm는 후보 ID0과 ID1의 X 간격이다.

마커 사이 거리를 추정 자세로 다시 재는 것은 PnP에 넣은 3D 길이를 되돌리는 것이다. 독립 검사가 아니다. 독립인 것은 깊이로 본 작업판 평면과, 설치 뒤 자로 잰 중심 거리다.

실패 이름은 다음만 쓴다. 실패한 뒤 threshold를 낮추지 않는다.

`INSUFFICIENT_MARKERS`, `MARKER_ID_MISMATCH`, `FIDUCIAL_LAYOUT_ERROR`, `PNP_FAILURE`, `HIGH_REPROJECTION_ERROR`, `POOR_REPEATABILITY`, `WORKBOARD_PLANE_INCONSISTENT`, `MARKER_PHYSICAL_MEASUREMENT_ERROR`, `FRAME_CONVENTION_ERROR`

나중에 PASS를 기록할 때는 변환만 두지 않는다. 카메라와 intrinsics의 identity, 프레임 identity, layout 버전, 관측 코너, 변환과 그 역, reprojection, 반복, 평면, 시각, 라이브러리 버전, PASS/FAIL이 함께 있어야 한다.

## 하지 않는 일

이 계약은 camera observation과 물리 `workboard`의 관계만 다룬다. 신발 segmentation, 경계, WorkpieceInstance 선택, ProcessDomain, reachability, IK, collision, trajectory는 만들지 않는다. registration이 성공해도 WorkpieceInstance가 정해진 것이 아니다.

ADR-009와 충돌하지 않는다. `camera↔workboard`는 vision registration이고, 로봇 도달로 영역을 정하지 않는다. 동결된 P-009 JSON은 바꾸지 않았다. 이미 끝난 P-009의 `FAIL_SURFACE_ASSOCIATION`도 바꾸지 않았다.

## 아직 아닌 것

- 네 후보가 동시에 보이는지
- 종이·테이프 두께
- 기존 인쇄물의 quiet zone이 10 mm 이상인지의 재측정
- 클램프가 원점 모서리 점을 덮는지
- 평면 검사에 쓸 맨 나무 영역의 실제 표시
