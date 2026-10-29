# P-007F-08A — 사용한 기존 evidence

이번 문서는 기존 파일을 인용만 한다. 원본은 수정하지 않았다.

## Camera intrinsics

`evidence/P-006/camera-target/color-camera-info.yaml`

- frame: `camera_color_optical_frame`
- 1280×720
- fx 610.5073852539062
- fy 610.349853515625
- cx 641.7918090820312
- cy 364.4433288574219
- distortion_model: `rational_polynomial`

같은 K가 이후 Gemini 프레임에도 반복된다. 이번 Gate에서 카메라를 다시 보정하지 않았다.

## Robot / FK

- `scripts/p007e_r2r4_fk.py` — `full_angles()`가 servo ID2를 `Arm2_Joint`에 넣는다. `arm2_axis_params()`는 그 축을 **로봇 base frame**에서 만든다.
- `evidence/P-007F-07/m2_kinematic_verification/membership_report.json`
- `evidence/P-007F-07/m2_kinematic_verification/report.md`

## M2 영상 위치

- P-007F-07B-R1 attempt-02 검출 17프레임 중심 중앙값 (776.02, 126.48) px.
- P-007F-08 Phase A ROI `(738, 92, 100, 85)`. 같은 역사적 프레임 `000128`과 phase correlation shift 0.163 px, response 0.986. ROI 내용이 그 사이에 움직이지 않았다는 확인이다. 새 마커 중심 측정은 아니다.

## Depth

- P-007F-07C solvePnP: 마커 원점까지 거리 중앙값 0.681 m. 색 카메라 프레임의 마커 위치다. 관절 축의 위치가 아니다. reprojection 평균 0.93 px.
- P-007F-06 `rgb_depth_alignment.json`: depth–color baseline 23.80 mm. 픽셀 단위 정합은 증명되지 않았다. `DEPTH_SPATIAL_LOCALIZATION_LIMITED`.
- P-007F-04의 depth 678 mm는 예전 ARM2 ROI의 근사값이다. 지금 M2 카드의 depth가 아니다.

## Baseline B

P-007F-08 Phase A: 전역 shift 0.450 px, unchanged. 배치가 얼어 있다는 뜻이지, 로봇 base와의 변환이 측정됐다는 뜻이 아니다.

## Controller-reported state

마지막 기록은 P-007F-05 `controller_post.json`. readback 152. 명령값은 151이었다. 그 파일은 이것을 물리 각도로 해석하지 말라고 적는다. 이번 Gate에서 다시 읽지 않았다.

## 채택하지 않은 과거 근사

`evidence/P-007F-04/visual_detectability.md`는 extrinsic이 없어 엄밀 투영이 불가능하다고 이미 적었다. 그 문서의 `in_plane_discount` 0.25/0.5/1.0은 임의 계수이고, 변위는 TCP이지 M2가 아니다. 1° 범위는 0.355–2.464 px로 1.0 px의 양쪽을 모두 포함한다. 이번 판정의 예측값으로 쓰지 않는다.
