# W-002E-R5 — Workboard Plane / RGB-D Geometry Audit

상태: `PASS_RGBD_PLANE_GEOMETRY_AUDIT / CLOSED`

이 합격은 등록 합격이 아니다. -2.34 mm를 종이 두께, depth 편향, PnP 오차, D2C 오차 중 하나로 정하지 않았다. z 보정은 넣지 않았다.

## 입력 경로

W-002E 평면은 `/camera/depth/image_raw`의 16비트 depth다. 점군은 쓰지 않았다. 실행 중 `depth_registration=true`, `align_mode=SW`이고, 이 토픽의 `frame_id`는 `camera_color_optical_frame`이다. 정렬 전 `/camera/depth/image_unaligned`의 frame은 `camera_depth_optical_frame`이다. `/camera/depth_registered/points`의 frame도 color optical이다. 그래서 D2C는 적용되어 있고 방식은 `SW_D2C`다. 파라미터 이름만으로 추정하지 않았다.

픽셀 raw를 1000으로 나누어 미터로 만들고, color CameraInfo K로 카메라 좌표를 만든 뒤 `p_workboard = T_workboard_camera * p_camera`를 적용했다. 단위를 한 번 더 스케일하지 않는다. 좌표 사슬과 단위는 PASS다. ROI `u=480..700`, `v=430..560`은 color와 같은 1280×720 영역이다. 이 사각형은 나무결이고, 마커·신발·펜·클램프는 밖에 있다.

## 분해

저장된 W-002E depth로 다시 계산한 signed z 평균은 -2.340 mm, `|z|` 평균은 2.343 mm다. 저장된 값과 0.05 mm 안에서 같다.

카메라 좌표에서 나무 평면의 RMS는 0.889 mm, p95는 1.709 mm다. 평면을 z=0에 붙이지 않았다.

같은 ROI를 30프레임 보면 평면 중심 z의 표준편차는 0.014 mm, 범위는 0.052 mm, 법선 각 표준편차는 0.013°다. -2.34 mm는 이 흔들림보다 크다.

workboard에서 signed offset은 -2.340 mm, tilt RMS는 0.452 mm, 평면 맞춤 뒤 남은 RMS는 0.889 mm다. 기울기와 잔여 거칠기보다 일정한 간격이 크다.

RGB PnP가 정한 z=0과 depth 나무 평면의 법선 차이는 0.592°이고, 중심에서의 간격은 -2.340 mm다. 관계는 `OFFSET_DOMINANT`다. 어느 평면이 참인지는 정하지 않았다.

마커 사각형 밖 1.05–1.35배 고리의 나무 평면까지 부호 있는 거리는 ID0 1.41 mm, ID1 1.48 mm, ID2 1.06 mm, ID3 -0.23 mm다. 중앙값은 1.23 mm다. 이 값은 종이 두께가 아니다.

공식 2 m 사양을 이번 거리의 합격 기준으로 쓰지 않았다.

## 파일

- `result.json`
- `runtime_rgbd_config.json`
- `d2c_provenance.json`
- `coordinate_chain.json`
- `plane_camera_space.json`
- `plane_workboard_space.json`
- `temporal_plane_stats.json`
- `rgb_vs_depth_plane.json`
- `marker_wood_comparison.json`
- `bare_wood_roi.png`
- `plane_residual_map.png` — 픽셀 지도가 아니라 수집 순서의 값 배열이다
- `temporal_z_plot.png`
