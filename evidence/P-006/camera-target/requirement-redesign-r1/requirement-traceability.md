# Requirement traceability

`NO RETROACTIVE PASS`

이 순서는 threshold를 낮춘 기록이 아니다.

1. Initial requirement. camera optical Z ±40 mm와 화면 좌우 20 mm, tilt 8°. `thresholds.yaml`과 Attempt 2 capture plan.
2. Physical experiment. Attempt 1의 고정 자세, Attempt 2–5의 체크보드 이동. 카메라와 로봇은 그 자리에 두었다.
3. Measured failure. near/far가 ±40 mm에 못 미쳐 CT-15 FAIL. Attempt 5는 받침 한계로 BLOCKED. 검출 실패로 CT-08도 FAIL.
4. Geometry diagnosis. Attempt 6. 작업면 100 mm당 optical Z 약 28.4 mm. ±40 mm 예측 이동 약 140.7 mm.
5. Workcell constraint. far 예측 방향이 카메라·로봇 설치 영역과 충돌한다. 분류 `VALIDATION_GEOMETRY_CONFLICTS_WITH_PHYSICAL_WORKCELL`. 센서 정확도 실패가 아니다.
6. Requirement redesign. 이 디렉터리. 검증 범위를 실제 작업 ROI로 바꾼다. 기존 BLOCKED와 FAIL은 유지한다.
7. Task-space validation. 아직 하지 않는다. ROI 미터 값은 R2 측정 전이다.

기존 evidence를 새 라벨로 다시 채점하지 않는다. Attempt 3의 left/right PASS와 tilt PASS도 그대로 두고, 그것만으로 P-006C를 닫지 않는다.
