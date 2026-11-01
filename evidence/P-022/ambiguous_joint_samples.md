# 0.0593845 rad와 0.402594 rad의 중간점 검사

```text
P022_AMBIGUOUS_JOINT_SAMPLES = PASS_DIAGNOSTIC

SOURCE = evidence/P-022/partial_failure_capture.json
RECONSTRUCTION = stored joint states
FK = MoveIt RobotState on the P-022 UR5e model
INTERPOLATION = RobotState::interpolate
COMMANDED_ORIENTATION = NOT_LOGGED

FK_POSITION_MATCHES_LOGGED_TARGET = YES
LARGEST_POSITION_RESIDUAL_M = about 2e-5

POSITION_TOLERANCE_M = 0.001
ORIENTATION_TOLERANCE_RAD = 0.01

THRESHOLD = NOT_DEFINED
VALID_CALLBACK_APPLIED = NO
REPAIR_APPLIED = NO
P022_FINAL_GATE = FAIL_CARTESIAN_API
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

명령 orientation은 기록에 없다. 저장된 해의 `tool0` FK 위치는 기록된 target position과 약 2e-5 m 안에서 같다. 아래 오차는 그 두 해의 FK를 Cartesian 양끝으로 두고, MoveIt 선형 보간의 중간 FK와 비교한 값이다.

## call 10, 0.0593845 rad

최대 차이는 wrist_2다. seed는 call 8의 해와 같다. 다음 호출 call 11의 seed는 call 10의 해와 같다. 그 사이에 중간점 IK는 없다.

```text
max_abs_joint_delta = 0.05938 rad
joint = wrist_2

midpoint_position_error = 4.98455e-05 m
midpoint_orientation_error = 0.000198848 rad

precision_pass = YES
refinement_required = NO
eventual_result = PASS
```

## call 8, 0.402594 rad

최대 차이는 wrist_1이다. seed는 call 7의 해와 같다. call 9의 target position은 call 7과 call 8의 중간이고, call 9의 seed는 그 두 해의 선형 중간과 5e-6 rad 이내다. 그래서 call 9는 이 구간의 중간점 IK다.

첫 검사:

```text
max_abs_joint_delta = 0.4026 rad
joint = wrist_1

midpoint_position_error = 0.00229769 m
midpoint_orientation_error = 0.00561525 rad

precision_pass = NO
refinement_required = YES
```

위치 오차만 0.001 m를 넘는다. 한 번 나눈 두 자식은 추가 IK 없이 기준 안이다.

```text
first_half joint_delta = 0.19433 rad
position_error = 0.000544007 m
orientation_error = 0.00138611 rad
precision_pass = YES

second_half joint_delta = 0.20827 rad
position_error = 0.000603047 m
orientation_error = 0.0014277 rad
precision_pass = YES

eventual_result = PASS
```

call 10이 call 8의 해에서 이어진 것과 맞다. 이 구간은 거기서 끝나지 않았다.

## 같은 기록의 실패 구간

call 10 다음 call 11은 6.27117 rad이고 wrist_3다. 그 구간의 중간점 오차는 1.29619 m, 0.946537 rad다. call 12부터는 그 구간의 중간점 재귀다. 0.05938 rad와 0.4026 rad는 그 실패 사슬의 표본이 아니다.

같은 절대차가 Cartesian 오차와 대응한다고 두지 않는다. 0.4026 rad의 첫 오차는 0.00229769 m였고 한 번의 분할로 통과했다. 6.27117 rad의 오차는 1.29619 m였고 재귀가 이어졌다.

threshold는 정하지 않았다. callback은 적용하지 않았다.
