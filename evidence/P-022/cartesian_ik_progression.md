# P-022 Cartesian IK progression

```text
PURPOSE = P022_CARTESIAN_FAILURE_DIAGNOSIS
NOT = P023_IK_ACCEPTANCE

SOURCE = MoveIt 2.12.4 cartesian_interpolator.cpp, robot_state.cpp, kdl_kinematics_plugin.cpp
INSTALLED_PACKAGE = ros-jazzy-moveit-core 2.12.4

P022_CARTESIAN_IK_PROGRESSION_REVIEW = PASS

SUCCESS_STATE_PROPAGATION = VERIFIED
FAILED_IK_STATE_PROPAGATION = NO
NEXT_STEP_SEED = PREVIOUS_SUCCESSFUL_JOINT_STATE
WAYPOINT_BOUNDARY_SEED = PREVIOUS_SEGMENT_LAST_SUCCESSFUL_STATE
PATH_IK_TIMEOUT_ARGUMENT = 0.0
EFFECTIVE_TIMEOUT_SOURCE = JOINT_MODEL_GROUP_DEFAULT_IK_TIMEOUT
OBSERVED_DEFAULT = 0.5 s
KDL_RANDOM_RESEEDING = POSSIBLE_AFTER_INITIAL_FAILURE
CONSISTENCY_LIMITS = NOT_USED

JOINT_TRACE_MEASURED = NO
IK_BRANCH_DIVERGENCE_MEASURED = NO
JOINT_LIMIT_PROXIMITY_MEASURED = NO
SINGULARITY_PROXIMITY_MEASURED = NO
REPAIR_SELECTED = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

이 문서는 라이브러리 호출 순서만 읽은 것이다. 같은 run을 반복해 joint를 기록하지 않았다. branch가 실제로 갈라졌는지는 아직 측정이 아니다.

## 우리가 호출한 overload

`computeCartesianPath(..., MaxEEFStep, CartesianPrecision)`다. 이 overload는 `JumpThreshold`를 받지 않는다. 경로를 joint jump로 자르지 않는다.

waypoint마다 한 구간을 푼 뒤, 다음 waypoint의 시작 포인터를 `traj.back()`으로 바꾼다. 직전 구간의 마지막 성공 `RobotState`가 다음 구간의 시작이다.

## 한 구간의 seed

구간 함수는 시작 state를 복사한다. 보간 step마다 그 복사본에 `setFromIK`를 호출한다. timeout 인자는 `0.0`이다. `consistency_limits`는 넘기지 않는다.

`RobotState::setFromIK`는 현재 group joint 값을 seed로 복사한다. 성공하면 `setJointGroupPositions`로 그 state를 덮어쓴다. 실패하면 joint를 쓰지 않고 false를 반환한다. 구간 루프는 실패하면 break한다. 그래서 실패한 호출 자체는 다음 seed가 되지 않는다.

성공한 호출의 joint는 다음 보간 step의 seed다. waypoint가 넘어갈 때도 마지막 성공 state가 다음 구간의 시작이다.

## random reseed

`KDLKinematicsPlugin::searchPositionIK`는 첫 시도에 넘어온 seed를 쓴다. 첫 시도가 실패하고 timeout이 남아 있으면 `getRandomConfiguration`으로 joint를 다시 뽑는다. `consistency_limits`가 비어 있으면 그 추출은 seed 근처가 아니다.

timeout `0.0`은 solver에 가기 전에 `JointModelGroup::getDefaultIKTimeout()`으로 바뀐다. 생성자의 초기값은 0.5 s다. 기준선 binary는 이 값을 덮어쓰지 않았다. timeout 실험은 시작 IK와 이 default를 같은 값으로 덮어썼다.

따라서 경로 IK는 주석이 말하는 single attempt가 아니다. 첫 seed가 timeout 안에 실패하면 다른 joint configuration이 해가 될 수 있고, 그 해가 다음 step의 seed가 된다.

`validateAndImproveInterval`도 중간 pose에 `setFromIK(..., 0.0)`을 호출한다. 직진 편차가 precision을 넘으면 그 호출이 들어간다. precision 기본값은 translation 0.001 m, rotation 0.01 rad다.

## 아직 측정하지 않은 것

같은 pose sequence의 반복에서 joint solution이 달라지는지, partial fraction의 직전 joint가 반복마다 달라지는지, 실패 직전 configuration이 limit이나 singularity 근처인지는 이 읽기로 답하지 않는다.

라이브러리가 허용하는 진행은 다음이다. 첫 seed가 실패하고 random reseed가 성공하면, 그 다른 branch가 다음 Cartesian step의 seed가 된다. 그 다음 step이 실패하면 fraction은 그 앞에서 멈춘다. 이 경로가 이번 21 run에서 실제로 발생했는지는 `JOINT_TRACE_MEASURED = NO`다.
