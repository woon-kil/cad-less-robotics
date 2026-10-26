# P-006C camera to checkerboard

상태: **FAIL**

숫자 한도는 측정 전에 고정한 `thresholds.yaml` 그대로다. 측정 뒤에 바꾸지 않았다. 실패한 항목은 CT-15뿐이다. 보드는 한 자세 `fixed`로만 찍혔고, left, right, near, far, tilt는 없다.

검출과 metric 비교 자체는 그 한 자세에서 한도 안이다. 12프레임 모두 내부 코너 54개. validation 4프레임의 재투영 RMSE는 0.25017064425427665 px다. 깊이 평면 RMSE 최대는 0.0005230457254262224 m다. RGB 평면과 깊이 평면의 거리 최대는 0.0016317241497878915 m, 법선 차 최대는 0.5873161987080323 deg다. 인접 코너 간격 372개의 평균은 0.020086735752688174 m다.

`checkerboard_frame`은 `base_link`가 아니다. `T_color_checkerboard`는 `p_color = T_color_checkerboard * p_checkerboard`다. 깊이 프레임으로는 드라이버 TF를 한 번 더 곱한다. `T_base_camera`는 없다.

로봇은 움직이지 않았다. P-007은 열지 않았다.
