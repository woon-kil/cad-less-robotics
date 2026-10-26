# P-006C-R2.5

상태: **OPERATOR_ANCHOR_REQUIRED**

판정: **IMPLEMENTED / NOT VERIFIED**

이 단계는 `camera_depth_optical_frame`의 점을 `workplane_roi_frame`으로 보내는 rigid transform만 구한다. `T_base_camera`는 만들지 않는다. DOFBOT은 움직이지 않는다. R3와 P-007은 열지 않는다.

R2의 Physical Task ROI를 그대로 쓴다. 테이블은 X 0–400 mm, Y 0–800 mm다. 작업 영역은 X 0–400 mm, Y 300–800 mm다. 원점은 DOFBOT 쪽에서 Gemini와 LED를 볼 때 테이블의 왼쪽 robot-side 모서리다. +X는 오른쪽, +Y는 DOFBOT에서 카메라 쪽, +Z는 작업면 위다.

## 왜 좌표를 비워 두었는가

체크보드를 아무 데나 놓으면 `T_camera_board`는 구할 수 있다. `T_workplane_board`를 모르면 `T_workplane_camera`는 나오지 않는다. 픽셀 크기나 패턴이 종이 한가운데 있다는 가정으로 그 변환을 만들지 않는다.

종이 외곽 210 × 285 mm와 칸 20.0 mm는 다른 값이다. 칸 7 × 10은 140 × 200 mm다. internal corner 사이는 100 × 160 mm다. 종이 중앙 가정이면 짧은 쪽 여백이 35 mm, 긴 쪽 여백이 42.5 mm가 되지만, 그 값은 측정이 아니므로 앵커로 쓰지 않는다. 어느 종이 변이 7칸 쪽인지도 별도로 재지 않았다.

종이 사각형 자체는 ROI 안에 들어갈 수 있다. 210 mm와 285 mm는 400 mm와 500 mm보다 작다. 그것만으로 origin 좌표를 정하지 않는다. origin은 종이 모서리가 아니라 아래 규칙의 internal corner다.

R2의 다섯 위치는 카메라가 보드를 본 증거다. 그 프레임에는 workplane 좌표가 없다. 앞으로 놓을 앵커가 카메라에 보이는지는 **UNRESOLVED**다.

## checkerboard_frame

P-006C에 잠긴 규칙을 숨기지 않는다.

- origin은 canonicalize 이후 corner 0이다. object point는 (0, 0, 0)이다.
- +X는 같은 row에서 column이 증가하는 방향이다.
- +Y는 다음 row 방향이다.
- +Z는 +X와 +Y의 외적이다.
- 검출은 (6, 9)를 먼저 시도하고, 실패하면 (9, 6)을 시도한다.
- 첫 row의 둘째 코너가 첫째보다 이미지 x가 작으면 grid를 180도 돌린다.

이 시작 코너는 종이에 인쇄된 마크가 아니다. 이미지의 x로 고른다. 그래서 배치마다 overlay의 빨간 origin이 줄자로 재는 점과 같은지 사람이 봐야 한다.

`overlays/convention-illustration.png`는 R2 `center-t01-00`을 읽기만 해서 그 프레임의 규칙을 보여 준다. 검출은 (6, 9)였다. 빨간 화살은 첫 row의 6개 코너, 초록 화살은 첫 column의 9개 코너다. 이 사진의 workplane 좌표는 없고, 보드를 돌리면 종이 위의 origin 코너가 바뀐다. 앵커가 아니다.

## 운영자가 한 번만 재는 것

1. 보드를 작업면에 평평하게 둔다. +X를 오른쪽, +Y를 DOFBOT에서 카메라 쪽으로 직각자나 줄자로 맞춘다. 맞춘 뒤에만 `alignment.yaw_deg`와 `operator_confirmed`를 적는다. 코드는 yaw를 0으로 채우지 않는다.
2. 미리보기 overlay에서 origin, 빨간 +X, 초록 +Y를 확인한다.
3. 그 origin에서 보드 축 방향으로 종이 네 가장자리까지 자로 잰다. `paper_margin_from_origin_m`에 미터로 적는다.
4. origin의 workplane X, Y, Z를 테이블 원점부터 자로 잰다. REG_A, REG_B, REG_C와 VAL_A, VAL_B에 `{x_m, y_m, z_m}`로 적는다. VAL_C는 선택이다.
5. 세 REG 점은 서로 100 mm 이상 떨어지고, 삼각형 면적이 0.004 m² 이상이어야 한다. 일직선이면 거절한다.
6. VAL 점은 어떤 REG 점과도 50 mm 이상 떨어져야 한다. fit에는 REG만 넣고 VAL은 검증에만 쓴다.
7. 종이 전체가 X 0–400 mm, Y 300–800 mm 안에 있어야 한다. 여백이 비어 있으면 이 판단을 하지 않는다.
8. 위가 모두 적히기 전에는 `coordinates_recorded`를 true로 해도 촬영 스크립트가 멈춘다.

Z는 운영자가 적는다. 테이블 면을 0으로 볼지, 종이 두께를 더할지는 측정 기록이다. 코드가 0을 넣지 않는다.

## Threshold

캡처 전에 잠갔다. 결과는 아직 없다.

- 위치 RMSE ≤ 5 mm, P95 ≤ 8 mm, max ≤ 12 mm
- 작업면 Z RMSE ≤ 5 mm, 절대값 max ≤ 10 mm
- board +Z와 작업면 +Z의 방향각 평균 ≤ 3°, max ≤ 5°
- REG sample 사이 translation spread ≤ 5 mm, rotation spread ≤ 3°

이 값은 게이트 지시서 19절의 초기값이다. 센서 threshold 파일은 바꾸지 않았다. hash는 `303d1a4067b05d81cbbbb8ff24ae92ddbf34712b2c91ea0c470cb021a007b539`다.

scale은 1로 고정한다. similarity는 쓰지 않는다. det가 음수면 실패이고, 반사를 회전으로 뒤집지 않는다. 잔차가 큰 샘플을 손으로 지우지 않는다.

변환은 `p_workplane = T_workplane_camera_depth * p_camera_depth`다. 컬러에서 보드를 찾고, 실제 Orbbec TF로 depth optical frame에 올린다. lookup이 실패하거나 identity이면 그 프레임은 거절한다.

## 아직 하지 않은 것

촬영하지 않았다. transform 행렬은 없다. R2 evidence는 읽기만 하고 수정하지 않는다. R2.5가 나중에 PASS해도 Camera Visible ROI를 작업 영역 전체라고 부르지 않는다. 그 경계는 R3다.
