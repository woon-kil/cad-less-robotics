# P-002 모델 convention

P-002가 고정한 모델은 `atticusrussell/ballbot` 커밋 `906da9f9120c056dd0c8766fe1a72e72a086061c`의 course-family 스냅샷이다. 공식 Yahboom 저장소의 URDF 원본이라고 부르지 않는다. 이 Mac 워크트리에는 `third_party` URDF가 없다. 비교는 P-002 evidence에 적힌 관절 표와, Orin에 설치된 같은 파일명의 URDF를 필드 단위로 맞춘 것이다. 바이트 해시는 이번 Gate에서 pinned blob과 다시 대조하지 않았다.

설치된 URDF `b626a845cd67998cd5ce47f2b3d302d9cb3e5b9da764d9a1e7e1404a277ccc28`의 관절은 P-002 `robot-model.txt` 및 Arm1/Arm2 기록과 같다.

| joint | parent | child | origin xyz | origin rpy | axis | lower | upper |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Arm1_Joint | base_link | Arm1_Link | 0 0 0.0895 | 0 0 0 | 0 0 1 | -1.5708 | 1.5708 |
| Arm2_Joint | Arm1_Link | Arm2_Link | 0 0 0.0329999999999998 | 0 0 0 | 1 0 0 | -1.5708 | 1.5708 |
| Arm3_Joint | Arm2_Link | Arm3_Link | -0.000549999999999828 0 0.0828499999999964 | 0 0 0 | 1 0 0 | -1.5708 | 1.5708 |
| Arm4_Joint | Arm3_Link | Arm4_Link | -5.0000000000161E-05 0 0.0828500000000002 | 1.57079632679386 0 0 | 1 0 0 | -1.5708 | 1.5708 |
| Arm5_Joint | Arm4_Link | Arm5_Link | 0 0.0781490000000018 0.00215000000000008 | 0 0 0 | 0 1 0 | -1.5708 | 3.1416 |

`Gripping_point_Joint`는 `Arm5_Link`의 fixed child이고 xyz는 `0 0.09 0`이다. `grip_joint`는 `Arm5_Link`에서 `rlink1`로 가는 revolute이며 `Arm6_Joint`가 아니다.

URDF 0은 각 관절 좌표의 0 rad다. Arm1~Arm4의 ±90° 한계와 Arm5의 -90°~180° 한계는, 라이브러리 범위 0~180°와 0~270°에서 90°를 뺀 구간과 숫자로 맞다. 이 일치는 단위와 이동량에 대한 것이고, 현재 자세가 그 영점이라는 뜻은 아니다.

P-002 SRDF의 `arm_group`은 Arm1~Arm5다. named pose `up`은 다섯 관절이 0이고, `init`은 Arm2 1.2063, Arm3 -1.5, Arm4 -1.3104다. 이것은 플래닝 자세이지 서보 중위 파일이 아니다.
