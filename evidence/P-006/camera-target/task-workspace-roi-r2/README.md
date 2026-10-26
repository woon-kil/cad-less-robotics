# P-006C-R2

상태: **OPERATOR_MEASUREMENT_REQUIRED**

고정된 Gemini 335L과 DOFBOT 작업면에서 `PHYSICAL_TASK_ROI`, `CAMERA_VISIBLE_ROI`, `VALIDATION_ROI`를 미터로 정의하는 게이트다. 카메라 보정은 하지 않는다.

저장소의 evidence에는 이 테이블의 경계 길이가 없다. 사진 픽셀로 mm를 만들지 않았다. 숫자를 받기 전까지 PASS가 아니다.

작업면 좌표 `workplane_roi_frame`의 축 방향만 정해 두었다. 원점의 물리적 위치와 네 경계의 mm는 작업자가 줄자로 잰다. `base_link`로 바꾸지 않는다.

Legacy P-006C는 `BLOCKED`다. ±40 mm 규칙은 여기 합격 조건이 아니다.
