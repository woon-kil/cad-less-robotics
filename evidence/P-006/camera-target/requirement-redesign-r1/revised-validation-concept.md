# Revised validation concept

## 원칙

기존 출발점 `camera optical Z를 ±40 mm 움직여야 한다`는 새 요구사항의 출발점이 아니다.

> Camera validation shall cover the physical task workspace in which perception-derived geometry will actually be used by the robot.

검증 범위는 카메라 좌표의 임의 이동이 아니라, 인식 결과를 로봇이 실제로 쓸 작업 공간에서 나온다.

## 영역 이름

`near`와 `far`는 새 요구사항에서 optical Z ±40 mm를 뜻하지 않는다. 위치 영역 이름은 작업 공간 기준이다.

- `center`: 작업 ROI의 명목 중심
- `left`: 작업 ROI의 왼쪽
- `right`: 작업 ROI의 오른쪽
- `robot_side`: 로봇 설치 쪽
- `opposite_side`: 그 반대쪽

방향 다양성이 필요하면 `tilt_positive`와 `tilt_negative`를 유지할 수 있다. 각도와 거리의 숫자는 R2가 ROI를 측정한 뒤에 정한다. 이번 문서는 8°나 20 mm를 새 합격선으로 다시 채택하지 않고, 기존 파일의 그 숫자도 바꾸지 않는다.

각 타깃은 측정된 task ROI 안에 있어야 한다. ROI 밖의 체크보드 이동을 합격 조건으로 두지 않는다.

## 센서 품질 threshold

`thresholds.yaml`의 검출, 재투영, 깊이 평면 RMSE, RGB–depth 거리, 법선 각, 20 mm 간격, 고정 자세 반복성은 센서 품질 문제다. 이 revision에서 값을 완화하거나 교체하지 않는다. 새 위치 게이트가 생겨도 먼저 이 값의 재사용을 검토한다. 검토 전에 숫자를 낮추지 않는다.

`thresholds.yaml`의 `diversity.candidate_labels`에는 아직 `near`, `far`가 있다. 그 파일은 legacy 게이트용으로 동결한다. 새 영역 이름은 이 문서에만 있고, yaml을 고쳐 소급 적용하지 않는다.
