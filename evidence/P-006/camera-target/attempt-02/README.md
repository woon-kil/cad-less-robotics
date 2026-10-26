# P-006C Attempt 2

상태: **FAIL**

실패 항목은 CT-15뿐이다. 검출, 재투영, 깊이 평면, 20 mm 간격은 잠긴 threshold 안이다. threshold 파일과 Attempt 1 `frames/0000`–`0011`은 바꾸지 않았다.

촬영 전에 고정한 분리 조건은 카메라 X ±0.020 m, 카메라 Z ±0.040 m, 보드 +Z 기울기 8 deg와 반대 방향이다. 이름은 이 조건을 통과해야 accepted가 된다.

`left`와 `right`의 원점 X 변화는 +0.051 mm, +0.052 mm다. fixed 자세와 같다. `near`의 Z 변화는 -0.031979 m로, 0.040 m에 못 미친다. 다만 원점 Y는 +0.103408 m 움직였다. `far`의 Z 변화는 +0.002138 m다. `tilt_positive`는 8.354376 deg, `tilt_negative`는 8.264331 deg이고 두 기울기의 방향은 반대다. 두 법선 사이 각은 16.617934 deg다.

`tilt_negative-00`은 드라이버 TF lookup이 없어 `T_depth_color`를 null로 남겼다. identity로 채우지 않았다. 그 프레임은 평면 집계에서 빠졌고, 같은 자세의 나머지 3프레임은 평면에 들어갔다.

Attempt 1의 12장 fixed 반복성은 그대로다. 병진 표준편차 최대 1.4332292680393973e-05 m, 회전 0.00673503855866625 deg.
