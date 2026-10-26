# P-006C Attempt 5

상태: **BLOCKED**

카메라와 로봇은 그대로 두고 near와 far만 찍었다. far 1차는 체크보드가 검출되지 않았고, 작업자가 카메라 받침에 닿았다고 기록했다. 카메라를 옮기지 않고 `FAR_PHYSICAL_WORKSPACE_LIMIT`으로 중단했다.

near 2차 ΔZ는 -0.034581 m다. 잠긴 조건은 ΔZ < -0.040 m이고, 이 값은 거기에 못 미친다. near 1차와 far 1차는 4장 모두 검출에 실패했다.

Attempt 2의 left, right `rule_pass=0`은 그 파일에 그대로 있다. Attempt 3 left ΔX -0.121647 m, right ΔX +0.091005 m는 20 mm 조건을 다시 계산해도 통과다. 이번 CT-15가 실패한 이유는 left와 right가 아니라 near와 far다.

`near-t01-00`의 내부 TF lookup은 실패했고 identity로 채우지 않았다.
