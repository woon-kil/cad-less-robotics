# W-002E-R4 — RGB Camera Geometry and Corner Localization Audit

상태: `PASS_RGB_GEOMETRY_AUDIT / CLOSED`

이 합격은 등록 합격이 아니다. 승인된 `T_camera_workboard`는 없다.

## 이미지와 CameraInfo

실행 중 파라미터 `enable_color_undistortion`은 False다. `w002e_register.py`는 검출 전에 undistort를 하지 않고, `solvePnP`와 `projectPoints`에 같은 CameraInfo의 K와 D를 넣는다. `undistortPoints`는 쓰지 않는다. 그래서 사용한 color는 `RAW_DISTORTED`이고, 이미지와 보정 영역은 맞다. 왜곡을 두 번 적용하지 않았다.

지금 받은 CameraInfo는 W-002E에 저장된 K, D와 같다. 모델은 `rational_polynomial`이다. 발행 이미지는 1280×720 `rgb8`이고 CameraInfo 크기와 같다. 런치 파라미터 `color_width`와 `color_height`는 0이며, 이는 드라이버 기본 해상도다. 스크립트는 resize나 crop을 하지 않는다.

시리얼 `CP263630009D`, 펌웨어 1.4.60, SDK 2.10.6은 실행 중 드라이버 로그에서 읽었다. ROS wrapper `orbbec_camera` package.xml도 2.10.6이다. 로그 한 줄에 Gemini 335L이라는 제품 문자열은 없고, launch는 `gemini_330_series.launch.py`다.

`depth_registration=true`, `align_mode=SW`, `align_target_stream=COLOR`이다. 이 값은 depth를 color에 맞춘다는 뜻이며, color 원본의 왜곡을 없앴다는 뜻은 아니다.

## 코너

OpenCV 4.5.4의 `DetectorParameters_create` 기본 `cornerRefinementMethod`는 0, 즉 `CORNER_REFINE_NONE`이다. 창 크기 5, 최대 반복 30, 최소 정확도 0.1이다. W-002E는 이 기본값을 그대로 썼다.

저장된 대표 프레임만 다시 검출했다. 새 촬영은 없다. 마커 크기는 명목 60 mm 그대로다.

| mode | mean px | p95 px | max px | W-002B 재투영 |
| --- | ---: | ---: | ---: | --- |
| NONE | 1.844 | 2.751 | 2.756 | 아님 |
| SUBPIX | 1.038 | 1.874 | 2.852 | 아님 |
| CONTOUR | 1.505 | 2.205 | 2.317 | 아님 |

NONE의 평균은 W-002E 저장값과 같다. SUBPIX와 CONTOUR는 평균을 0.30 px 이상 바꾸므로 코너 위치 민감도는 `MATERIAL`이다. SUBPIX 평균 1.038 px는 동결 기준 1.0 px를 아직 넘는다. 더 낮은 모드를 골라 등록을 합격으로 바꾸지 않았다.

네 마커의 corner0→corner1 방향 차이는 12.3°다. 코드 인덱스는 W-002B의 TL, TR, BR, BL과 같다. 순서 감사는 PASS다. 순서를 고쳐 다시 풀지 않았다.

## 원인

이미지와 K/D 영역이 맞으므로 왜곡 영역 불일치는 이번 실패의 설명으로 지지되지 않는다. `DISTORTION_DOMAIN_EFFECT = NONE`. 코너 refinement는 잔차 규모를 바꾸지만, 그 변화만으로 원인을 확정하지 않는다. `ROOT_CAUSE = INCONCLUSIVE`.

## 문서

검토자가 제시한 문서와 이번 실행을 같이 둔다. OpenCV ArUco 문서는 pose에 필요한 경우 corner refinement를 제공하고, 방식에 따라 코너 좌표가 달라질 수 있다고 한다. 이번 4.5.4 기본값은 NONE이었다. Gemini 330 원본 color에는 왜곡이 있을 수 있고, ROS2 wrapper는 `enable_color_undistortion`을 따로 둔다. 이번 실행에서 그 값은 False였다.

- https://docs.opencv.org/4.11.0/d5/dae/tutorial_aruco_detection.html
- https://github.com/orbbec/OrbbecSDK_v2/blob/main/docs/tutorial/orbbec_camera_distortion.md
- https://github.com/orbbec/OrbbecSDK_ROS2/releases
- https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/sdk_json_config.html

## 파일

- `result.json`
- `runtime_camera_config.json`
- `camera_info.json`
- `code_audit.json`
- `corner_ordering.json`
- `refinement_comparison.json`
- `corner_order_overlay.png`
- `residual_none.png`
- `residual_subpix.png`
- `residual_contour.png`
