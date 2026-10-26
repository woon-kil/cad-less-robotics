# Legacy requirement

원본 파일 `evidence/P-006/camera-target/thresholds.yaml`과 Attempt 2에 잠근 diversity rule은 수정하지 않는다.

## Near / far

고정 자세는 Attempt 1의 `camera_color_optical_frame` 중앙값이다.

- near: `ΔZ_camera < -0.040 m`
- far: `ΔZ_camera > +0.040 m`

부등호는 엄격하다. 40 mm와 같은 값은 합격이 아니다. 35 mm로 낮추지 않는다.

left, right는 화면 좌우 20 mm, tilt는 서로 반대인 8° 이상이다. 그 규칙도 이 revision에서 바꾸지 않는다.

## 결과

기존 P-006C 상태: `BLOCKED`.

CT-15는 near/far가 위 규칙을 충족하지 못해 FAIL이다. Attempt 5의 검출률 8/16은 CT-08 FAIL이다. Attempt 5 near 2차 중앙값 −0.034581 m와 Attempt 3의 최선 −0.038170 m는 둘 다 FAIL로 남는다.

소급 수정: **NO**.
