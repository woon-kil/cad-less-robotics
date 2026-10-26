# 실물 Arm1 대응

상태: **INCOMPLETE**. 로봇은 움직이지 않았고, 거리와 `T_base_camera`, `T_base_target`은 만들지 않았다.

## 사진

가진 프레임은 `evidence/P-006A/gemini-color.png` 하나다. 1280×720, `camera_color_optical_frame`, stamp sec 1793006639 nanosec 789987000. 나무 작업판과 3×3 마커 용지가 보인다. Arm1부터 gripper까지의 연결은 화면에 없다.

원본은 수정하지 않았다. 화면에 없는 축을 그려 넣지 않았으므로 annotation 사본은 만들지 않았다. 이 방 사진은 공개 저장소에 넣지 않는다.

따라서 아래의 실물 판정은 모두 사진 위 식별이 아니다.

## 모델에서 Arm1을 구분하는 순서

P-002 URDF `DOFBOT_Pro-V24.urdf`, ballbot `906da9f9120c056dd0c8766fe1a72e72a086061c`.

관절 각 0에서 `base_link`로 본 축은 다음과 같다. 관절 원점의 rpy를 부모에 곱한 뒤 axis를 그 프레임에서 읽었다.

| 관절 | parent → child | 원점 (m) | 축 |
|---|---|---|---|
| Arm1_Joint | base_link → Arm1_Link | (0, 0, 0.0895) | (0, 0, 1) |
| Arm2_Joint | Arm1_Link → Arm2_Link | (0, 0, 0.1225) | (1, 0, 0) |
| Arm3_Joint | Arm2_Link → Arm3_Link | (-0.00055, 0, 0.20535) | (1, 0, 0) |
| Arm4_Joint | Arm3_Link → Arm4_Link | (-0.0006, 0, 0.2882) | (1, 0, 0) |
| Arm5_Joint | Arm4_Link → Arm5_Link | (-0.0006, -0.00215, 0.366349) | (0, 0, 1) |

Arm1만 parent가 `base_link`이고, 그 축만 링크 원점을 지나는 +Z다. Arm2, Arm3, Arm4의 축은 이 자세에서 +X와 평행하다. Arm5의 관절 프레임 축은 `(0, 1, 0)`이고, Arm4 원점의 roll `1.57079632679386` 때문에 `base_link`에서는 +Z와 평행하다. gripper로 가는 `Gripping_point_Joint`는 fixed다.

mesh는 visual origin `0 0 0`, scale 요소 없음.

- `base_link.STL`: 꼭짓점의 대부분은 반경 약 0.04 m 안의 둥근 몸체다. z≥0.08 m 구간은 X, Y가 모두 약 ±0.040 m다. 이 둥근 몸체는 Arm1_Link가 아니다.
- 같은 mesh에서 Y < -0.10 m인 꼭짓점은 전체의 1.77%이고 Z는 -0.015 m ~ 0.009 m다. 긴 쪽은 낮은 판에만 있다.
- `Arm1_Link.STL`: XY 반경 약 0.0275 m, Z는 0 ~ 0.0435 m. 중심은 축 근처다. 관절 원점이 Z=0.0895 m이므로 이 원판은 베이스 둥근 몸체보다 작고, 그 위에서 +Z로 이어진다.
- `Arm2_Link.STL`: Z 방향이 길고(-0.01365 m ~ 0.09335 m), 회전축은 +X다.

큰 원이라는 이유만으로 Arm1을 고르면 베이스 몸체를 집게 된다. 모델에서 Arm1은 그 몸체 다음의 더 작은 원판이고, 다음 관절의 축이 Arm1 축과 수직이다.

## 실물 판정

`PHYSICAL_ARM1_AXIS`는 정의하지 않는다. 위 순서에 맞는 부품이 사진에 없다.

Arm1 축을 `base_link`의 X=0, Y=0 중심선으로 쓰는 대응은 **NOT VERIFIED**다. URDF에서는 그 축이 `(x, y) = (0, 0)`이다. 실물 중심선을 그 축에 맞춘 관찰은 없다.

작업판 법선과 Arm1 축이 평행한지, +Z가 작업판 위쪽인지는 **NOT VERIFIED**다. 각도를 사진에서 재지 않았다.

+X는 **NOT PHYSICALLY VERIFIED**다. 베이스 mesh의 X 범위는 -0.09249952435493469 m ~ 0.09245279431343079 m로 거의 대칭이다. Arm1 원판도 XY로 대칭이다. 관절 각 0에서 Arm2 축이 +X라는 모델 사실만 있고, 실물 관절 각은 보지 못했다. 로봇의 앞쪽, ROS 관례, 케이블 방향으로 정하지 않았다.

+Y는 **NOT PHYSICALLY VERIFIED**다. 모델의 낮은 판은 -Y로 더 길다(-0.22149640321731567 m ~ 0.07850000262260437 m). 그 판을 실물 모서리에 맞춘 비교는 하지 않았다.

`base_link z=0`은 **NOT PHYSICALLY VERIFIED**다. mesh 최소 Z를 바닥으로 두지 않는다. Y < -0.10 m인 낮은 형상은 Z=-0.015 m와 Z=0.009 m 양쪽에 걸쳐 있으므로, z=0은 그 형상의 바깥 면이 아니다.

## Checkerboard

7×10 칸, 내부 코너 6×9, 칸 20.0 mm = 0.0200 m. 상태 `PHYSICALLY VERIFIED`는 유지한다. 보드 좌표 원점과 `T_base_target`은 없다. 카메라 프레임의 3×3 마커 용지를 이 체크보드로 바꾸지 않았다.
