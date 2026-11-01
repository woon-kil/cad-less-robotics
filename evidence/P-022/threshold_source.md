# threshold 출처

```text
P022_THRESHOLD_SOURCE_REVIEW = PASS_THRESHOLD_SOURCE_REVIEW
OFFICIAL_JOINT_DISPLACEMENT_THRESHOLD = NOT_FOUND
P022_THRESHOLD_SOURCE = NO_JUSTIFIABLE_THRESHOLD_SOURCE

INSTALLED = ros-jazzy-moveit-core 2.12.4-1noble.20260915.180823
UR_MOVEIT_CONFIG = ros-jazzy-ur-moveit-config 3.9.0-1noble.20261006.043609

T = NOT_DEFINED
M_IS_THRESHOLD = NO
MARGIN_RULE = NOT_DEFINED
CALLBACK_IMPLEMENTED = NO
REPAIR_APPLIED = NO

SCALAR_DISTANCE_CALLBACK = RECONSIDER
REPLACEMENT_REPAIR = NOT_SELECTED

P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

설치본과 같은 태그의 헤더를 봤다. Cartesian 경로의 관절 변위에 쓸 공식 숫자가 없다.

`CartesianPrecision`의 기본값은 0.001 m, 0.01 rad, `max_resolution` 1e-5다. 이것은 중간점 검사의 Cartesian 기준이다. 관절 거리 `T`가 아니다. 계약은 이 값을 바꾸지 않는다.

`JumpThreshold`의 기본 생성자는 `disabled()`와 같다. `relative_factor`, `revolute`, `prismatic`은 0이다. `absolute()`는 호출자가 0보다 큰 revolute와 prismatic을 넘겨야 한다. 라이브러리가 라디안 기본값을 주지 않는다. 그 overload는 `Replace JumpThreshold with CartesianPrecision`으로 표시되어 있고, `validateAndImproveInterval`을 호출하지 않는다. `ur_moveit_config`에는 이 관절 거리 값이 없다. 있는 것은 Pilz의 속도·가속도 한계다.

KDL은 `consistency_limits`가 비어 있지 않으면 seed와 해의 절대차를 본다. 그 벡터의 공식 기본 길이는 없다. solution callback은 후보를 거절하고 검색을 계속할 뿐, 몇 라디안이 먼지는 정하지 않는다.

URDF의 관절 한계는 한 호출의 변위 상한이 아니다.

그래서 `T`를 계산하지 않는다. 0.485861 rad를 상한으로 쓰지 않는다. scalar 거리 callback은 다시 본다. 다른 repair는 고르지 않았다.
