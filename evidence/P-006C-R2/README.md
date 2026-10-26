# P-006C-R2

상태: **CONDITIONAL PASS**

Physical Task ROI는 운영자 실측이다. 테이블은 X 0–400 mm, Y 0–800 mm다. 작업 영역은 X 0–400 mm, Y 300–800 mm, 400 × 500 mm, 200000 mm²다. Y 300 mm 미만은 DOFBOT과 제어기 설치 영역이라 작업 영역이 아니다. 이것은 로봇 도달 영역이 아니다.

`workplane_roi_frame`의 원점은 DOFBOT 쪽에서 Gemini와 LED를 볼 때 테이블의 왼쪽 robot-side 모서리다. +X는 오른쪽, +Y는 DOFBOT에서 카메라 쪽, +Z는 작업면 위다. `base_link`가 아니다.

체크보드 종이 외곽은 210 × 285 mm다. 칸은 20.0 mm, 내부 코너는 6 × 9다.

캡처는 고정된 Gemini 335L에서 했다. 토픽은 `/camera/color/image_raw`, `/camera/depth/image_raw`와 각 `camera_info`다. 프레임은 `camera_color_optical_frame`, `camera_depth_optical_frame`이다.

| 위치 | 저장 프레임 | 54코너 + depth 유효 |
|---|---:|---:|
| CENTER | 12 | 8 |
| LEFT | 12 | 12 |
| RIGHT | 12 | 12 |
| ROBOT_SIDE | 4 | 4 |
| CAMERA_SIDE | 12 | 12 |

다섯 위치 모두 관측됐다. 카메라 포즈는 color optical frame에만 있다. workplane X/Y로 바꾸지 않았다. Camera Visible ROI의 metric 경계는 `METRIC_BOUNDARY_UNRESOLVED` / `UNRESOLVED_PENDING_WORKPLANE_REGISTRATION`이다.

Robot Reachable ROI는 `UNVERIFIED_P007_NOT_OPEN`이다. Process ROI는 `NOT_CONFIRMED`다.

DOFBOT 동작, 컨트롤러 실행, identity TF, `T_base_camera`는 없다. Legacy P-006C와 sensor threshold는 그대로다. ±40 mm 규칙은 쓰지 않는다.
