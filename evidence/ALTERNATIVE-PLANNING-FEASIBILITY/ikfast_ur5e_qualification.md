# UR5e IKFast 생성 가능성

```text
IKFAST_UR5E_QUALIFICATION = NOT_FEASIBLE_FOR_THIS_ENVIRONMENT
IKFAST_PLUGIN_GENERATED = NO
IKFAST_SOLVER_VALIDATED = NO
IKFAST_UR5E_AVAILABLE_FOR_BENCHMARK = NO
DIAGNOSTIC_BENCHMARK_EXECUTED = NO
FULL_PROCESSPATH_ACCEPTANCE_EXECUTED = NO
GENERATION_ATTEMPTED = YES
GENERATION_ACCEPTED = NO
PICK_IK_INSTALLED = NO

P022 = CLOSED_FAIL_IMMUTABLE_BASELINE
ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED
FEASIBILITY_VERDICT = NOT_REACHED
```

MoveIt 2.12.4 `auto_create_ikfast_moveit_plugin.sh`가 생성 경로다. 입력은 URDF, planning group, base link, end-effector link다. solver 생성은 Docker 안의 OpenRave다. 그 다음 `create_ikfast_moveit_plugin.py`가 plugin package를 만든다.

```text
BASE_IMAGE = personalrobotics/ros-openrave
DERIVED_IMAGE = fixed-openrave
IK_TYPE_DEFAULT = Transform6D
IK_TESTS = 1000
DOCKER_DAEMON = USABLE
BASE_IMAGE_LOCAL = YES
FIXED_OPENRAVE_BUILD = YES
URDF = /tmp/p022/src/ur5e.urdf
URDF_SHA256 = a21bf5fb70b3a1745bf2e4816e0f43654539737ba7f2585939ec773d159778d8
PLANNING_GROUP = ur_manipulator
BASE_LINK = base_link
EEF_LINK = tool0
IK_TEST_SUCCESS_RATE = 0.001
IK_TEST_WRONG_RATE = 0.996
IK_TEST_NO_SOLUTION_RATE = 0.003
IK_TEST_MISSING_RATE = 0.001
OPENRAVE_EXIT = wrong rate 0.996000 > 0
INTERMEDIATE_CPP = /tmp/ikfast.tjElT9/.openrave/kinematics.b7688d783066c6a3d1dbd6d20e17657d/ikfast0x10000049.Transform6D.0_1_2_3_4_5.cpp
CPP_ACCEPTED_BY_GENERATOR = NO
```

`personalrobotics/ros-openrave`를 받았고, MoveIt 스크립트의 `fixed-openrave` 이미지도 빌드됐다. 입력은 P-022에 쓰인 `/tmp/p022/src/ur5e.urdf`다. robot 이름은 `ur5e`이고 `base_link`와 `tool0`이 있다.

공식 스크립트는 Collada 변환 뒤 OpenRave `Transform6D` 생성을 실행했다. 1000회 자체 검사의 success rate는 0.001, wrong rate는 0.996이다. 스크립트는 `wrong rate 0.996000 > 0`으로 종료했다. 중간 `.cpp`는 남았지만 generator가 그 해를 받아들이지 않았다. MoveIt plugin은 만들지 않았고, build/load와 FK↔IK qualification도 하지 않았다. 그 `.cpp`를 benchmark에 넣지 않는다.

이 결과는 `PASS_IKFAST_QUALIFICATION`이 아니다. 이 환경의 공식 생성 경로로는 UR5e `base_link`→`tool0` solver를 쓸 수 없다. 후보 IKFast만 `NOT_FEASIBLE_FOR_THIS_ENVIRONMENT`다. UR5e에서 IKFast가 원천적으로 불가능하다는 뜻은 아니다. feasibility 전체는 닫지 않는다. diagnostic benchmark와 전체 acceptance는 실행하지 않았다.

설치된 template이 seed 거리로 해를 정렬한다는 사실은 유지한다. 그 정렬은 UR5e solver가 있고, 포착된 transition의 `CartesianPrecision`이 개선되며, known-good가 유지될 때 비로소 측정 대상이다. joint delta 감소만으로 합격하지 않는다.

이 파일은 생성 성공이 아니다. 생성에 성공하기 전에는 diagnostic benchmark로 넘어가지 않는다.
