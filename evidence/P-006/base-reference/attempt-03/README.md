# P-006A Attempt 3

상태: **INCOMPLETE**

분류: **MODEL-DEFINED / PHYSICAL ALIGNMENT REQUIRED**

`A_AXIS`는 실물 Arm1 회전축이고 `base_link` x=0, y=0이다. +Z는 작업판에서 팔 쪽으로 PHYSICALLY CONSISTENT다. 이 둘은 바꾸지 않았다.

Arm1 각 0에서 `Arm2_Joint` 원점의 XY는 `(0, 0)`이다. 어깨 링크는 +Z로 쌓이고, 어깨 힌지 축이 모델의 +X다. 지금 사진의 팔 방향은 zero가 아니므로 +X/+Y로 쓰지 않았다.

하드웨어 영점 표시는 없다. SRDF `up`, `down`, `init`의 `Arm1_Joint=0`은 모델 숫자다.

`T_base_target`과 `T_base_camera`는 없다. 로봇은 움직이지 않았다. P-007은 열지 않았다.
