# P-008-R1G — RGB↔Depth/PointCloud correspondence

```text
P-008-R1G = FAIL_DEPTH_CLOUD_CORRESPONDENCE
```

capture_01만 사용했다. shoe segmentation은 구현하지 않았다. RGB 투영은 실행하지 않았다.

## 드라이버

OrbbecSDK_ROS2 v2.10.6. 근거는 `method_reference.md`.

## 저장 경로

RGB는 `/camera/color/image_raw`, MJPG를 푼 `rgb8`이다. 왜곡 계수가 0이 아니고 color undistortion 기본값이 false이므로 rectified가 아니다.

Depth는 `/camera/depth/image_raw`, `16UC1`, frame `camera_depth_optical_frame`이다. 정합 depth는 color optical frame으로 바뀌므로 이 저장본은 unaligned raw다.

Cloud는 `/camera/depth/points`, XYZ만, 같은 depth optical frame이다. colored point cloud 기본값은 false다. raw depth projection이다.

## 시각과 동기

color `1793291194.224658`, depth `1793291194.258052`, cloud `1793291194.224676`.

color–depth 0.033394 s, depth–cloud 0.033376 s, color–cloud 0.000018 s. 저장 pairing rule 0.05 s는 만족한다. depth는 color/cloud보다 약 한 프레임 늦다. 같은 hardware frame이라고 하지 않는다.

`HARDWARE_SYNC = NOT_VERIFIED`. `TEMPORAL_ASSOCIATION = SUFFICIENT`는 ROS stamp pairing만 말한다.

## Depth ↔ cloud

depth 왜곡은 전부 0이고 R은 단위행렬이라 pinhole을 썼다. 점군 점을 depth 픽셀로 투영한 뒤, 그 픽셀의 16UC1 값을 1000으로 나눈 거리를 다시 back-project해 3D 거리를 쟀다. 1000으로 나누는 근거는 드라이버가 point filter 출력을 1000으로 나눠 미터로 내는 코드다. 저장 depth에는 scale 필드가 없다.

사전 기준은 유효 표본 100000 이상, 영상 안 비율 0.999 이상, 양수 depth 비율 0.99 이상, 3D 오차 median 0.002 m 이하, p95 0.005 m 이하다. 측정 뒤에 바꾸지 않았다.

| 항목 | 값 | 기준 |
|---|---:|---|
| valid samples | 337715 | 통과 |
| in bounds | 341698 / 341698 = 1.0 | 통과 |
| positive depth ratio | 0.988344 | 실패 |
| mean | 0.001520 m | 보고만 |
| median | 0.001079 m | 통과 |
| p95 | 0.005054 m | 실패 |
| max | 0.778260 m | 보고만 |

z 잔차 median 1.000 mm, p95 4.000 mm, max 706.000 mm. z 잔차가 5 mm를 넘는 점 10890개 중 4513개만 인접 픽셀 2 mm 이내로 설명된다. 이 인접 픽셀 검사는 합격 기준이 아니다.

중앙값은 약 1 mm로 맞지만, 고정한 꼬리 기준 두 개가 실패했다. `DEPTH_CLOUD_CORRESPONDENCE = FAIL`.

## Color ↔ depth extrinsic

capture_01 metadata에 행렬이 없다. color와 depth CameraInfo `P`의 이동 성분은 0이다. TF와 `/camera/depth_to_color`는 저장되지 않았다. 드라이버는 `enable_publish_extrinsic:=true`일 때만 그 토픽을 내고, 기본값은 false다. static TF를 만드는 코드는 있으나 이번 캡처 스크립트는 TF를 구독하지 않았다.

다른 세션의 23.80 mm 값은 가져오지 않았다. 행렬을 추정하지 않았다.

`COLOR_DEPTH_EXTRINSIC = NOT_VERIFIED`.

이 상태에서도 RGB 투영은 하지 않는다. 깊이–점군 기준이 먼저 실패했으므로 verdict는 `FAIL_DEPTH_CLOUD_CORRESPONDENCE`다. extrinsic이 없는 사실은 그대로다.

## 실행하지 않은 것

RGB projection `NOT_RUN`. 신발 대응 진단 `NOT_RUN`. manual ROI 없음. figure 04–06은 만들지 않았다. 공식 alignment 비교는 저장된 캡처만으로 재현할 수 없어 `NOT_AVAILABLE_FROM_SAVED_CAPTURE`.

`UNREGISTERED_RGB_DIAGNOSTIC_LIMITATION`은 해제하지 않았다. registration accuracy는 주장하지 않는다.

## 검사

`scripts/test_p008r1g_correspondence.py` 8개 통과. 알려진 내부파라미터 round-trip, 알려진 이동, 단위변환, 무효 depth, 화면 밖 거절, holdout 경로 거절을 포함한다. 강체변환 함수는 검사에서만 호출했다. capture_01에는 적용하지 않았다. ROS 패키지는 수정하지 않았다.

## Figure

`figures/01_rgb_original.png`, `02_depth_original.png`, `03_cloud_projection_depth.png`. 03은 depth 영상 위에 점군 투영을 깊이 색으로 올린 것이다. RGB 정합 그림이 아니다.
