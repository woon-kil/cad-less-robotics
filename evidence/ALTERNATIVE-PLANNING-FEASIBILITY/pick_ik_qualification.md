# pick_ik qualification

```text
PICK_IK_FEASIBILITY_QUALIFICATION = PASS_LOAD_AND_SEED_CONSISTENCY
PICK_IK_AVAILABLE_FOR_BENCHMARK = YES
DIAGNOSTIC_BENCHMARK_EXECUTED = YES
FULL_741_WAYPOINT_ACCEPTANCE_EXECUTED = NO

SYSTEM_APT_INSTALL = BLOCKED_SUDO
PACKAGE = ros-jazzy-pick-ik
VERSION = 1.1.2-1noble.20260915.185902
DEB_SHA256 = 6a3ed8077658d77a3b403917b6009c0b30301544810a6bcf447af6d1a135ca93
SOURCE = packages.ros.org noble/main
LOAD_PATH = /tmp/pick-ik-qual extracted deb
PLUGIN_CLASS = pick_ik/PickIkPlugin
PLUGINLIB_LOAD = YES

MODE = global
MINIMAL_DISPLACEMENT_WEIGHT = 0.0
PARAMETER_OVERRIDE = NONE
IK_TIMEOUT_S = 0.005
IK_TIMEOUT_SOURCE = ur_moveit_config kinematics.yaml
SEARCH_DISCRETIZATION = 0.005

SAMPLE_COUNT = 20
REPEAT_COUNT = 3
RNG_SEED = 20261101
IK_CALL_COUNT = 60
IK_SUCCESS_COUNT = 60
LIMIT_VIOLATION_COUNT = 0
REPEAT_SUCCESS_MISMATCH_COUNT = 0
MAX_POSITION_ERROR_M = 0
MAX_ORIENTATION_ERROR_RAD = 0
MAX_JOINT_DELTA_RAD = 0
Q_EQUALS_SEED_REQUIRED = NO
SEED_WAS_ALREADY_THE_SOLUTION = YES

FEASIBILITY_VERDICT = NOT_REACHED
P022 = CLOSED_FAIL_IMMUTABLE_BASELINE
```

`sudo apt-get install`은 비밀번호가 필요해 시스템 설치는 하지 않았다. 공식 deb를 받아 pluginlib으로 `pick_ik/PickIkPlugin`을 적재했다. UR5e `ur_manipulator`, `base_link`, `tool0`에서 `initialize`가 성공했다.

pick_ik 파라미터는 패키지 기본값이다. `mode`는 `global`이고 `minimal_displacement_weight`는 0이다. 결과를 본 뒤 바꾸지 않았다. 호출 timeout 0.005 s는 UR `kinematics.yaml`의 solver timeout이다.

표본은 한계 안에서 뽑은 joint 20개다. 각 표본의 FK pose를 seed가 그 joint인 IK에 넣었다. 같은 입력을 3번 반복했다. 60번 모두 성공했고, joint limit 위반은 없었다. 반복 사이 성공 여부는 달라지지 않았다. 위치·자세 잔차와 seed 대비 joint delta는 모두 0이다. seed가 이미 그 pose의 해이므로, 이 결과는 그 해를 다시 돌려준 consistency다. 다른 branch를 고른 증거는 아니다. `q'=q`를 합격 조건으로 두지 않았다.

이 합격은 P-022 pathological transition 비교가 아니다. 그 비교는 `pick_ik_diagnostic_benchmark.md`다. seed가 이미 해인 이 문서는 branch continuity를 증명하지 않는다.
