# Requirement traceability

`NO RETROACTIVE PASS`

1. Legacy P-006C는 optical Z ±40 mm를 요구했고 `BLOCKED`다. 그 evidence는 수정하지 않는다.
2. P-006C-R1은 검증 범위를 실제 작업 영역으로 바꾸기로 했다. ROI 숫자는 넣지 않았다.
3. P-006C-R2는 그 ROI를 `workplane_roi_frame`의 줄자 값으로 정의해야 한다. 저장소에 그 값이 없어 여기서 멈춘다.
4. `VALIDATION_ROI`와 R3 후보 점은 두 ROI의 교집합이 생기기 전에 계산하지 않는다.
5. ±40 mm 규칙은 R3 점의 근거로 쓰지 않는다.

P-002는 팔 모델의 dry planning이다. 이 테이블의 도달 영역이 아니다. OMPL의 기본 10 m bound도 작업면 길이가 아니다. hardware-validated reachability는 없다. model-predicted task reachability도 이 게이트에 넣지 않는다.
