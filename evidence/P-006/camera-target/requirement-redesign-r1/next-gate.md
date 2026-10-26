# Next gate

이름: `P-006C-R2 — Task-Workspace ROI Measurement`

R1이 검수되기 전에는 R2를 구현하지 않는다. R2가 끝나기 전에는 새로운 위치 diversity threshold를 확정하지 않는다.

## 측정할 것

고정된 카메라와 로봇 설치를 유지한 채, 로봇이 실제로 작업할 평면 ROI를 작업면 미터 좌표로 측정한다.

- `robot_side`, `opposite_side`, `left`, `right` 네 경계의 작업면 좌표
- nominal task center의 작업면 좌표
- 그 사각형 또는 다각형의 폭과 길이
- 카메라가 기존 인식 파이프라인으로 관측하는 작업면 subset
- 작업으로 쓸 영역과 카메라 관측 영역의 overlap

픽셀 간격으로 이 길이를 만들지 않는다. `base_link`로 변환하지 않는다. `T_base_camera`를 추정하지 않는다. 로봇을 움직여 도달 영역을 실행으로 확인하지 않는다. 작업 subset은 테이블 위에 표시한 물리적 작업 영역의 작업면 좌표다. 캘리브레이션된 도달 집합이 아니다.

## 측정 뒤에만 할 일

ROI 안의 `center`, `left`, `right`, `robot_side`, `opposite_side` 목표 위치를 정한다. 그 다음에 위치 합격 거리를 도출한다. 도출 전 숫자는 없다.

센서 품질 threshold는 `thresholds.yaml` 재사용을 먼저 검토한다. 완화는 기본값이 아니다.
