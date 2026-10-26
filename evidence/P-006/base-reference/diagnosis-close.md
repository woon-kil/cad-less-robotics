# P-006A 진단 종료

상태: **DIAGNOSIS PASS / CLOSED**

Attempt 3의 실물 yaw 판정은 INCOMPLETE로 유지한다. 게이트의 목적은 물리적 기준만으로 어디까지 정해지는지 밝히는 것이었고, 그 범위는 닫았다.

Checkerboard geometry: PASS. 7×10, 내부 코너 6×9, 칸 20.0 mm.

Arm1 physical identification: PASS

A_AXIS: PASS. `base_link` x=0, y=0 중심선.

base +Z: PHYSICALLY CONSISTENT

base yaw: UNRESOLVED

base z=0: UNRESOLVED

기계 기준만으로 `T_base_target`을 만들지 않는다. yaw가 없으면 체크보드와 `A_AXIS`의 거리만으로 6-DOF를 채울 수 없다.

`T_base_camera`는 계산하지 않았다. P-007은 열지 않았다.
