# ROI definition

검증 범위는 카메라 픽셀 사각형이 아니다. 작업면 위의 미터 좌표로 정의한다.

이번 단계에는 `T_base_camera`가 없다. ROI를 `base_link`로 변환하지 않는다. 추정한 변환, 단위 행렬, 합성 TF는 쓰지 않는다.

허용하는 좌표는 둘이다.

- checkerboard 또는 작업면 로컬 미터 좌표
- `camera_color_optical_frame` 또는 `camera_depth_optical_frame`의 미터 좌표

## 경계

이후 측정이 정해야 하는 경계는 다음이다. 숫자는 아직 없다.

- `robot_side`: 로봇 설치 쪽 작업면 경계
- `opposite_side`: 그 반대쪽 작업면 경계
- `left`: 작업면 왼쪽 경계
- `right`: 작업면 오른쪽 경계

구분해서 기록할 영역은 다음이다.

- nominal task center
- 로봇이 실제로 작업할 subset
- 카메라가 관측하는 subset
- 그 둘의 overlap

로봇 subset은 `base_link` 도달 계산으로 만들지 않는다. R2에서는 고정된 설치 위의 물리적 작업 영역을 작업면 좌표로 측정한다. 카메라 subset은 기존 인식 파이프라인이 그 면을 관측하는 범위다. 픽셀 거리를 미터로 환산해 경계를 만들지 않는다.

Metric values assigned: **NO**. 폭, 길이, 중심 좌표, 합격 거리 어느 것도 이 문서에 넣지 않는다. 실제 ROI를 재기 전에는 위치 threshold를 만들 수 없다.
