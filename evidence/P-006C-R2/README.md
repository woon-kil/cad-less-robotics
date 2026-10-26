# P-006C-R2

상태: **IMPLEMENTED / NOT VERIFIED**

Physical Task ROI는 운영자 실측이다. 테이블은 X 0–400 mm, Y 0–800 mm다. 작업 영역은 X 0–400 mm, Y 300–800 mm다. Y 300 mm 미만은 DOFBOT과 제어기 설치 영역이라 작업 영역이 아니다. 이것은 로봇 도달 영역이 아니다.

`workplane_roi_frame`의 원점은 DOFBOT 쪽에서 Gemini와 LED를 볼 때 테이블의 왼쪽 robot-side 모서리다. +X는 오른쪽, +Y는 DOFBOT에서 카메라 쪽, +Z는 작업면 위다. `base_link`가 아니다.

체크보드 종이 외곽은 210 × 285 mm다. 칸은 20.0 mm, 내부 코너는 6 × 9다.

카메라가 본 포즈는 `camera_color_optical_frame`에만 기록한다. workplane 경계로 바꾸지 않는다. 근거가 없으면 `METRIC_BOUNDARY_UNRESOLVED`다. Robot Reachable ROI는 `UNVERIFIED_P007_NOT_OPEN`이다.

Legacy P-006C는 `BLOCKED`로 둔다. ±40 mm 규칙은 쓰지 않는다.
