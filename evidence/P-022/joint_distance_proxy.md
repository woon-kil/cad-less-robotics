# joint 거리와 중간점 검사

```text
P022_JOINT_DISTANCE_PROXY_EVALUATION = PASS_DIAGNOSTIC

QUESTION = IS_JOINT_DISTANCE_A_USEFUL_REPAIR_PROXY
PROXY_VALIDITY = NOT_ESTABLISHED
JOINT_DISTANCE_ERROR_MONOTONICITY = NOT_ESTABLISHED
SCALAR_CALLBACK_DISCARDED = NO

CANDIDATE_STATUS = SELECTED_FOR_EVALUATION
DISTANCE_METRIC = BOUNDED_REVOLUTE_ABSOLUTE_DIFFERENCE
THRESHOLD = NOT_DEFINED
VALID_CALLBACK_APPLIED = NO
REPAIR_APPLIED = NO

P022_FINAL_GATE = FAIL_CARTESIAN_API
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

표본은 fraction 1이었던 run 20 반복 0–3의 경로 IK 164개다. 시작 IK 실패였던 반복 4는 넣지 않았다. 각 호출의 seed와 해를 MoveIt `interpolate`로 나누고, 그 중간 FK를 두 해의 FK가 만드는 Cartesian 중간과 비교했다. IK는 다시 호출하지 않았다.

164개 모두 다음 호출이 이전 해에서 이어지거나, 반복의 첫 호출이다. 중간점 IK는 0개다. 첫 검사에서 분할이 필요했던 전이는 이 성공 경로에 없다.

```text
COUNT = 164
SUBDIVISION_COUNT = 0
FINAL_RESULT = PASS

MAX_JOINT_DELTA_RAD = 0.00746 .. 0.03915
MIDPOINT_POSITION_ERROR_M = 5.03626e-7 .. 2.94816e-5
MIDPOINT_ORIENTATION_ERROR_RAD = 1.81099e-8 .. 0.000175214

POSITION_OVER_0.001 = 0
ORIENTATION_OVER_0.01 = 0
```

위치 오차가 가장 큰 전이는 0.02658 rad다. 가장 큰 joint 차이 0.03915 rad가 아니다. 이 구간 안에서도 joint 차이가 크면 중간점 오차가 크다고 두지 않는다.

1 rad 이상인 성공 전이는 없다. 그래서 성공 경로가 큰 해를 분할로 통과한다는 증거는 없고, 그 이유로 후보를 버리지 않는다.

이미 라벨이 있는 partial capture와 같이 보면 관측은 세 구간이다.

```text
DIRECT_PASS = 0.00746 .. 0.05938 rad
PASS_AFTER_ONE_SUBDIVISION = 0.402594 rad
FAILURE_CHAIN = 1.85667 .. 10.5494 rad
```

0.402594와 1.85667 사이는 비어 있다. 그 빈 구간이 threshold의 존재 증명은 아니다. 숫자는 정하지 않았다. callback은 적용하지 않았다.
