# 첫 가동 관절

```text
FIRST_MOVABLE_JOINT = Arm1_Joint
PARENT_LINK = base_link
CHILD_LINK = Arm1_Link
JOINT_TYPE = revolute
ORIGIN_XYZ = 0 0 0.0895
ORIGIN_RPY = 0 0 0
AXIS = 0 0 1
LIMIT = -1.5708 .. 1.5708 rad
```

축 `0 0 1`은 `base_link` +Z다. rpy가 0이라 부모 프레임과 같다.

origin의 x와 y가 0이다. 이 축은 `base_link`의 XY 원점을 지난다. XY 방향 오프셋은 없다.

```text
base_link origin
  +0.0895 m along +Z → Arm1_Joint origin
  mesh z min -0.015000 m
  mesh z max  0.097800 m
```

Arm1_Joint origin은 mesh 꼭대기(0.097800 m)보다 0.0083 m 아래다. 바닥보다 0.1045 m 위다.

`Arm1_Link.STL`은 자기 프레임에서 x, y가 약 ±0.0275 m, z가 0 .. 0.043500 m다. `base_link`에서는 z 0.0895 .. 0.1330 m에 놓인다. 베이스 둥근 몸체 위의 작은 원판이다.

## q1 = 0

`Arm2_Joint` origin은 Arm1 프레임에서 `0 0 0.0329999999999998`, rpy `0 0 0`, axis `1 0 0`이다.

Arm1 각이 0이면 그 축은 `base_link` +X와 같다. 팔 링크가 +X를 가리키는 것은 아니다. `Arm2_Link.STL`은 자기 프레임에서 z 방향이 더 길다. 각 0에서 위팔은 `base_link` +Z 쪽으로 뻗고, 어깨 힌지 축만 +X다.

SRDF `up`과 `init`도 `Arm1_Joint` 값을 0으로 둔다. 이것은 모델 숫자다. URDF에 servo zero를 이 각과 같게 두는 calibration 태그는 없다.

```text
SERVO_ZERO_PHYSICAL_ALIGNMENT = UNVERIFIED
```
