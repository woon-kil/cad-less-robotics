# P-006A base_link reference

상태: **INCOMPLETE**. URDF 수치 추출은 끝났고, 베이스 플레이트와 +X/+Y의 실물 대응은 `NOT PHYSICALLY VERIFIED`다.

`T_base_camera`는 계산하지 않았다. `T_base_target`도 없다. 로봇은 움직이지 않았다.

## Provenance

P-002와 같은 모델이다. ballbot `humble` `906da9f9120c056dd0c8766fe1a72e72a086061c`의 `DOFBOT_Pro-V24.urdf`. 오늘 읽은 파일 sha256은 `b626a845cd67998cd5ce47f2b3d302d9cb3e5b9da764d9a1e7e1404a277ccc28`이다.

공식 `YahboomTechnology/DOFBOT-Pro` `57cc436`에는 URDF가 없다는 P-002 기록을 유지한다. 다른 URDF로 바꾸지 않았다.

## base_link

visual origin `0 0 0`, collision origin `0 0 0`. inertial origin은 `(-8.03990775049002e-05, -0.047909102545025, 0.0178929922479837)` m이다. mesh는 `package://dofbot_pro_description/meshes/base_link.STL`이고 scale 요소는 없다.

## Arm1_Joint

parent `base_link`, child `Arm1_Link`, type revolute. origin xyz `0 0 0.0895` m, rpy `0 0 0` rad, axis `0 0 1`.

`T_base_Arm1Joint` 병진은 x 0, y 0, z 0.0895 m이다. roll, pitch, yaw는 0이다.

## 축

모델에서 +Z는 Arm1 회전축이다. +X와 +Y는 그 오른손 좌표계의 나머지 두 축이다. 금속 모서리와의 대응은 확인하지 않았다. 플레이트 높이에 원점이 어디 붙는지도 `NOT PHYSICALLY VERIFIED`다.

RViz 화면은 만들지 않았다. 축 그림은 `base-frame-visualization.svg`다.

## Checkerboard

7 × 10 칸, 내부 코너 6 × 9, 칸 길이 20.0 mm = 0.0200 m. 상태는 이번 요청에서 받은 `PHYSICALLY VERIFIED`다. 이 세션에서 자를 다시 재거나 픽셀로 환산하지 않았다. board pose는 없다.

이전 `evidence/P-006/target.yaml`의 ChArUco `NOT_MEASURED` 기록은 바꾸지 않았다.
