# P-006C Attempt 4

상태: **FAIL**

near와 far만 다시 찍었다. left, right는 Attempt 3, 두 tilt는 Attempt 2를 그대로 썼다. 광축 Z 40 mm 조건은 바꾸지 않았다. near와 far가 그 조건에 못 미쳐 CT-15가 실패다.

near 1차 ΔZ -0.024708 m, ΔY +0.093466 m. 2차 ΔZ -0.015693 m. 3차 ΔZ -0.021591 m. 가장 가까운 값도 -40 mm에 15 mm 남는다.

far 1차 ΔZ +0.002981 m, ΔY -0.005800 m. 2차 ΔZ -0.002199 m. 3차 ΔZ +0.001146 m. 화면 위쪽으로 옮긴 이동이 거의 없다.

left ΔX -0.121647 m, right ΔX +0.091005 m. tilt_positive 8.354376°, tilt_negative 8.264331°.

`far-t01-00`과 `near-t03-00`의 내부 TF lookup은 실패했고 identity로 채우지 않았다.
