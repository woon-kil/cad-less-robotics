# P-008A — Gemini 335L Real Shoe Static Dataset

```text
P-008A = PASS
```

정지한 운동화를 현재 Gemini Baseline B에서 RGB, depth, point cloud로 5회 받았다. 필터, 평면 제거, clustering, boundary, 로봇 명령은 하지 않았다. 카메라와 운동화는 움직이지 않았다.

## 카메라

드라이버 로그의 serial `CP263630009D`, firmware `1.4.60`. Baseline B와 같다.

| 스트림 | 프로파일 | frame |
|---|---|---|
| color | 1280×720, 30 fps, MJPG, encoding `rgb8` | `camera_color_optical_frame` |
| depth | 848×480, 30 fps, Y16, encoding `16UC1` | `camera_depth_optical_frame` |
| cloud | `/camera/depth/points`, unorganized, meters, `is_dense=true` | `camera_depth_optical_frame` |

camera info는 이번 세션 메시지다. `evidence/P-008A/camera_info/`. color `rational_polynomial`, fx 610.507, fy 610.350, cx 641.792, cy 364.443. depth `plumb_bob`, fx=fy 410.414, cx 423.669, cy 240.994.

드라이버 로그에 `Enable frame sync`와 `OB_MULTI_DEVICE_SYNC_MODE_STANDALONE`이 있다. hardware sync를 따로 검증하지 않았으므로 `HARDWARE_SYNCHRONIZED`라고 하지 않는다. 저장 시 ROS stamp 차이는 0.05초 이하다.

depth 메시지에는 scale 필드가 없다.

## 장면

촬영 전 미리보기와 dataset `capture_01` 컬러에서 작업대 원판 위 운동화의 앞코부터 뒤축까지 프레임 안에 있다. depth 유효 비율은 미리보기 기준 0.840이고, 중앙 160×200 영역은 0.923이다. 화각 부족으로 멈추지 않았다.

## 다섯 캡처

서로 다른 stamp와 SHA256이다. 같은 파일을 복사하지 않았다. PCD, PNG read-back과 `manifest.sha256` 재계산은 PASS다.

| capture | color stamp | points | color SHA256 | cloud SHA256 |
|---|---:|---:|---|---|
| 01 | 1793291194.224658 | 341698 | `a28550b8b4a5a91b…` | `51b5af9855e01e67…` |
| 02 | 1793291194.691914 | 341152 | `3795c5c2ddbf82f6…` | `17d7c0bc23993679…` |
| 03 | 1793291195.392796 | 341621 | `1439f13204a52131…` | `24baf9800d72e17c…` |
| 04 | 1793291196.127053 | 341493 | `30e31048518f7d5a…` | `8e30fc3deb50a011…` |
| 05 | 1793291196.827937 | 341531 | `6c5060429b66b756…` | `52635bd45afc0e03…` |

전체 hash는 `integrity.json`과 `manifest.sha256`에 있다. color는 전부 1280×720 `rgb8`, depth는 전부 848×480 `16UC1`이다. finite XYZ 수는 point count와 같다.

## 미리보기

`preview/01_color_preview.png`, `02_depth_preview.png`, `03_pointcloud_preview.png`. 점군 그림은 원본 PCD를 대체하지 않는다.

## 하지 않은 것

Voxel, outlier, RANSAC, clustering, PCA, 투영, concave hull, normal, TCP, MoveIt, servo, P-007 진단, extrinsic calibration. P-008 알고리즘은 재개하지 않았다.

```text
ROBOT_COMMAND_COUNT = 0
SERVO_WRITE_COUNT = 0
MOTION_AUTHORIZED = NO
```
