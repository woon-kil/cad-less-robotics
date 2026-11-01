# 시작 IK repair

```text
P022_START_IK_REPAIR_REVIEW = PASS_START_IK_REPAIR_REVIEW
RESULT = NO_JUSTIFIABLE_START_IK_REPAIR
FIVE_MS_PROVEN_OPTIMAL = NO

QUESTION =
is there an independent source for changing the 5 ms start IK timeout?

UR_MOVEIT_CONFIG = ros-jazzy-ur-moveit-config 3.9.0
FILE = kinematics.yaml
KINEMATICS_SOLVER_TIMEOUT_S = 0.005
KINEMATICS_SOLVER_SEARCH_RESOLUTION = 0.005

PROJECT_START_IK_TIMEOUT_S = 0.005
MATCHES_UR_MOVEIT_CONFIG = YES

JOINT_MODEL_GROUP_CONSTRUCTOR_DEFAULT_S = 0.5
APPLIES_WHEN_TIMEOUT_ARGUMENT_IS_0 = YES
UR_CONFIG_VALUE = 0.005

SERVO_INCOMING_COMMAND_TIMEOUT_S = 0.1
THIS_IS_IK_SEARCH_BUDGET = NO

SENSITIVITY_20_MS = NOT_A_SOURCE
TIMEOUT_ONLY_REPAIR_SUPPORTED = NO

START_IK_REPAIR_SELECTED = NO
PARTIAL_FRACTION_REPAIR = NO_JUSTIFIABLE_REPAIR
REPAIR_APPLIED = NO

P022_FINAL_GATE = FAIL_CARTESIAN_API
P022_PASS = NO
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

`ur_moveit_config`의 `kinematics.yaml`은 `kinematics_solver_timeout`을 0.005 s로 둔다. 기준선 시작 `setFromIK`에 넘긴 0.005 s와 같다. 이 값은 시작 IK를 늘리라는 근거가 아니다.

`JointModelGroup` 생성자의 0.5 s는 timeout 인자가 0일 때 쓰는 초기값이다. UR 설정 파일의 solver timeout은 0.005 s다. 0.5 s로 바꾸는 공식 기준은 없다.

`ur_servo.yaml`의 0.1 s는 새 명령이 없을 때 servo를 멈추는 시간이다. IK 탐색 예산이 아니다.

timeout 실험에서 20 ms부터 그 다섯 번 안에 시작 IK 실패가 없었다. 그 실험은 repair 값을 고르지 않기로 이미 기록되어 있다. 그 표로 5 ms를 바꾸지 않는다.

partial fraction repair와 시작 IK repair는 따로다. 둘 다 지금 고를 수 있는 변경이 없다. timeout은 적용하지 않았다.
