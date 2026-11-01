# 경로 IK 거리 정의

```text
P022_PATH_IK_DISTANCE_METRIC = PASS

DISTANCE_METRIC = max_j |q_solution[j] - q_seed[j]|
ANGLE_WRAPPING = DISABLED
RATIONALE = MATCH_MOVEIT_BOUNDED_REVOLUTE_INTERPOLATION

SOURCE_MODEL = /tmp/p022/src/ur5e.urdf
ACTIVE_JOINTS = shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
JOINT_TYPE = REVOLUTE
CONTINUOUS_JOINT_COUNT = 0

LIMITS_RAD =
shoulder_pan   -6.283185307179586 .. 6.283185307179586
shoulder_lift  -6.283185307179586 .. 6.283185307179586
elbow          -3.141592653589793 .. 3.141592653589793
wrist_1        -6.283185307179586 .. 6.283185307179586
wrist_2        -6.283185307179586 .. 6.283185307179586
wrist_3        -6.283185307179586 .. 6.283185307179586

DISTANCE_PER_JOINT = fabs(q_solution - q_seed)
CANDIDATE_DISTANCE = max over the six joints
ANGLE_WRAP = NO

MOVEIT_2_12_4 = RevoluteJointModel::distance
WRAPS_ONLY_WHEN continuous_ IS TRUE
INTERPOLATE_FOR_THESE_JOINTS = linear, not shortest angle

THRESHOLD_CANDIDATE = NOT_SELECTED
REPAIR_APPLIED = NO
P022_FINAL_GATE = FAIL_CARTESIAN_API
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

이 여섯 관절은 URDF에서 `revolute`다. `continuous` 관절은 없다. MoveIt 2.12.4의 `RevoluteJointModel::distance`는 `continuous_`가 아닐 때 `fabs(q1-q2)`다. `interpolate`도 그 차이를 선형으로 나눈다. 중간점 FK가 벗어난 계산이 이 선형 보간이다.

±2π 한계 안에 있는 두 값은 2π 차이여도 같은 명령이 아니다. 예를 들어 interval 22 깊이 3의 양끝은 wrist_2에서 약 6.27727 rad 떨어져 있고, 둘 다 ±6.283185 안에 있다. 최단각으로 접으면 약 0.008 rad가 된다. 그 접기는 선형 보간이 실제로 움직이는 거리를 지운다. 그래서 모든 revolute에 wrapping을 넣지 않는다.

저장된 해 중에서 URDF 한계 밖인 값은 run 20 경로 호출에서 없었다.

## 거리 분포

비교는 저장된 seed와 해의 최대 절대차다. wrapping 열은 채택하지 않은 대조다.

전체 fraction 1이었던 run 20 반복 0–3의 경로 IK 164개:

```text
min = 0.00745487 rad
max = 0.0391469 rad
```

interval 22에서 성공한 IK 17개:

```text
min = 1.85667 rad
max = 10.5494 rad
```

같은 interval에서 `PRECISION_MET`인 자식의 양끝 차이는 0.00401617 rad다.

앞의 partial capture 경로 호출에는 두 집단 사이에 들어가는 값이 있다. call 8은 0.402594 rad이고, call 10은 0.0593845 rad다. 이 둘의 중간점 검사 결과는 `ambiguous_joint_samples.md`에 있다.

wrapping은 큰 점프 일부를 줄인다. interval 22의 endpoint 9.65012 rad는 최단각 2.91625 rad가 된다. 깊이 3의 양끝 6.27727 rad는 최단각 0.00803 rad가 된다. 후자는 실패 중인 보간을 정상 거리에 가깝게 만든다.

## threshold

두 표본의 최종 검사는 모두 통과했다. 그래도 threshold 숫자는 정하지 않았다. 0.402594 rad는 첫 검사를 넘긴 뒤 한 번의 분할로 통과했고, 실패 사슬의 최소 거리는 1.85667 rad다. 그 사이에 다른 표본은 없다. 한 숫자를 그 구간에 넣지 않는다.

callback은 acceptance 설정에 넣지 않았다.
