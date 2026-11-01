# fraction 9/34의 반환 경로

```text
SOURCE = MoveIt 2.12.4 cartesian_interpolator.cpp
INSTALLED_PACKAGE = ros-jazzy-moveit-core 2.12.4

CAPTURE = evidence/P-022/partial_failure_capture.json
RETURNED_FRACTION = 0.264706
EXACT_RATIO = 9/34

IK_CALL_COUNT_IS_THE_FRACTION = NO
RELATIONSHIP = WAYPOINT_SEGMENT_COMPLETION

PATH_IK_FALSE_COUNT = 0
TERMINATION_WITHOUT_IK_FALSE = YES

JOINT_BRANCH_JUMP_CAUSED_FRACTION = NOT_CLAIMED
JOINT_LIMIT_CAUSE = NOT_SUPPORTED
JOINT_LIMIT_INVOLVEMENT = NOT_RULED_OUT
SINGULARITY_CAUSE = NOT_SUPPORTED
SINGULARITY_ACCEPTANCE_THRESHOLD = NOT_DEFINED

REPAIR_SELECTED = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

28회 성공 IK와 9/34는 같은 비율이 아니다. fraction은 IK 호출 수가 아니다.

## 9와 34

우리가 호출한 overload는 waypoint 배열을 순서대로 넘긴다. run 12는 pose 35개다. 첫 pose는 시작 IK이고, `computeCartesianPath`에 넘긴 waypoint는 34개다. 그래서 분모는 34다.

각 waypoint 구간이 1.0을 반환하면 누적값은 `(끝난 구간 수) / 34`가 된다. 1.0이 아닌 구간을 만나면 그 구간의 비율을 34로 나눈 값을 더한 뒤 루프를 멈춘다. 반환값이 정확히 9/34이려면, 앞의 9개 구간이 1.0이고 다음 구간이 0이어야 한다.

구간 함수는 step이 끝날 때만 `last_valid_percentage`를 쓴다. `setFromIK`가 성공해도 `validateAndImproveInterval`이 false면 그 값을 쓰지 않고 break한다. 그때 그 구간의 반환값은 0이다.

## IK가 true인데 끝나는 조건

`validateAndImproveInterval`은 joint 공간 중간의 FK와 Cartesian 중간 pose를 비교한다. translation 차이가 0.001 m 이하이고 rotation 차이가 0.01 rad 이하면 그 구간을 받아들인다. 이 두 값은 `CartesianPrecision`의 기본값이다.

차이가 더 크면 폭을 반으로 줄이고 Cartesian 중간에 `setFromIK`를 호출한다. 그 호출이 true여도 더 깊은 단계가 false면 전체가 false다. 폭이 `max_resolution` 1e-5보다 작아지면 IK를 호출하지 않고 false를 반환한다.

그래서 경로 IK가 모두 true인 기록과 fraction 9/34가 같이 있을 수 있다. 이번 capture가 그 경우다. `IK solver returned false → path stopped`는 이 실행의 설명이 아니다.

## 같은 target이 반복된 이유

call 11의 target은 `[0.368354, 0.0126125, 0.163911]` m이다. call 12는 그 점과 call 10 `[0.364471, 0.0118057, 0.162063]` m의 중간인 `[0.366412, 0.0122091, 0.162987]` m이다. 이후 target은 그 중간점들을 계속 반으로 나누며 `0.364471, 0.0118057, 0.162063`으로 모인다. call 25–28은 그 좌표가 같다.

이것은 waypoint를 하나씩 새로 요청한 목록이 아니다. `validateAndImproveInterval`이 한 구간의 Cartesian 중간을 재귀로 나누는 순서와 같다. 어느 call이 몇 번째 waypoint인지는 이 기록에 index로 남아 있지 않다.

## 큰 joint 차이

6.2712 rad, 9.7871 rad, 3.1416 rad는 그 IK 호출의 seed와 반환 joint의 차이다. 중간점 검사의 seed는 구간 양끝 joint를 보간한 상태다. 반환 해가 seed에서 멀리 있으면 joint 공간 중간의 FK가 Cartesian 중간에서 벗어나고, 재귀가 계속된다. 3.1416 rad 차이는 call 14 이후 그 재귀에서 반복된다.

이 대응은 코드의 상태 전환과 맞다. 그 문장만으로 `joint branch jump가 fraction 0.264706의 원인`이라고 확정하지 않는다.

## limit과 Jacobian

가장 가까운 limit 거리는 0.00484 rad다. exact limit hit는 없다. `JOINT_LIMIT_CAUSE = NOT_SUPPORTED`이고, `JOINT_LIMIT_INVOLVEMENT = RULED_OUT`은 아니다.

Jacobian 최소 singular value 0.123654–0.163372는 관측값이다. 합격 기준은 없다.

repair는 고르지 않았다.
