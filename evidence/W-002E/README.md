# W-002E — Camera ↔ Workboard Registration

상태: `FAIL_REPROJECTION / CLOSED`

같이 실패한 항목: `FAIL_WORKBOARD_PLANE`

최초 실패 지점은 대표 프레임의 재투영이다. 평균 1.844 px는 동결 기준 1.0 px를 넘고, p95 2.751 px는 2.0 px를 넘는다. 최댓값 2.756 px는 3.0 px 안에 있다. 임계값은 바꾸지 않았다.

## 변환

`p_camera = T_camera_workboard * p_workboard`

`T_workboard_camera`는 그 역행렬이다. 독립 보정은 아니다.

대표 변환은 10개 샘플 중 SE(3) medoid다. 거리는 translation mm와 rotation deg의 합이고, 동점이면 작은 인덱스다. 선택된 프레임은 7이다. 반복성 자체는 기준 안이다. 이번 변환은 합격한 등록이 아니다.

## 입력

PnP 3D 점은 W-002D-R1 실측 중심과 명목 60 mm 검은 사각형이다. 목표 좌표 `(150,70)` 등은 쓰지 않았다. 코너 순서는 `detectMarkers`와 같고 TL, TR, BR, BL이다. 네 마커 pose를 평균내지 않고 16점을 한 번 `SOLVEPNP_SQPNP`로 풀었다.

Z는 명목 평면, yaw는 설치 계약 0°다. `MEASUREMENT_RESOLUTION_MM = 1`은 줄자 눈금이며 중심 불확도 ±1 mm가 아니다.

## 검증

재투영은 같은 16개 대응점을 다시 투영한 것이다. `REPROJECTION_VALIDATION_INDEPENDENT = NO`.

작업판 평면은 대표 프레임의 맨 나무 ROI `u=480..700`, `v=430..560`이다. 이 프레임에서 그 사각형은 나무이고, 마커·신발·펜은 사각형 밖에 있다. 마커 사각형을 1.8배로 키운 픽셀은 제외했다. `|z|` 평균 2.343 mm와 p95 4.019 mm가 기준을 넘는다. 법선 각 0.592°는 1.0° 안에 있다. 법선 부호는 +Z 쪽이 되도록 맞춘 뒤 +Z와 비교했다.

알려진 거리는 실측 중심에서 계산한 값과, 마커 중심 7×7 depth를 workboard로 옮긴 값의 비교다. 독립 실측이 아니다. `KNOWN_DISTANCE_VALIDATION = COMPUTED_NOT_INDEPENDENT`.

회전 행렬과 역행렬 잔차는 수치적으로 통과했다. 마커 점은 카메라 앞이고, 작업판 +Z는 카메라 쪽을 본다.

## 파일

- `result.json`
- `camera_intrinsics.json`
- `installed_layout.json`
- `T_camera_workboard.json`
- `T_workboard_camera.json`
- `detections/frame_00.json` … `frame_09.json`, `detections/medoid_color.png`
- `transforms/frame_00.json` … `frame_09.json`
- `reprojection/frame_00.json` … `frame_09.json`
- `plane_validation/summary.json`, `known_distance.json`, `roi_overlay.png`, `roi_crop.png`, `abs_z_mm.npy`, `medoid_depth.npy`
- `repeatability/summary.json`

카메라, 로봇, 작업판, 마커는 이 실행에서 움직이지 않았다. P-009-R1과 P-010은 열지 않았다. 신발 위치는 이 변환으로 알 수 없다.
