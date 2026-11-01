# Branch continuity alternative review

```text
BRANCH_CONTINUITY_ALTERNATIVE_REVIEW = NO_JUSTIFIABLE_CANDIDATE
TYPE = SOURCE_API_REVIEW_ONLY
IMPLEMENTATION = NOT_ALLOWED
PACKAGE_INSTALLATION = NOT_ALLOWED
NEW_EXPERIMENT = NOT_ALLOWED

PURPOSE = 동결된 CartesianPrecision을 유지한 채 joint branch continuity를 명시적으로 고를 수 있는 후보가 있는지 확인

QUESTION_PROVEN = NO
QUESTION_SUFFICIENT_FOR_THIS_REVIEW = YES

MOVEIT_SOURCE = moveit2 tag 2.12.4
MOVEIT_HEADERS = p620 ROS 2 Jazzy install
PICK_IK_SOURCE = PickNikRobotics/pick_ik tag 1.1.2
PICK_IK_SOURCE_CLASS = OFFICIAL_IMPLEMENTATION_VERIFIED

P023 = NOT_OPENED
P024 = NOT_OPENED
ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED
LMA_INSTALL_AND_TEST = NOT_OPENED
DIAGNOSTIC_GATE_DEFINED = NO

JUSTIFIABLE_ALTERNATIVE_FOUND = NO
NO_JUSTIFIABLE_ALTERNATIVE = NO
FEASIBILITY_VERDICT = NOT_REACHED
NEXT_ALLOWED_WORK = FEASIBILITY_VERDICT_REVIEW
```

이 문서는 solver를 새로 돌리지 않는다. P-022 baseline, pick_ik qualification, pick_ik diagnostic benchmark를 입력으로 두고 MoveIt API와 해당 태그의 소스만 본다.

KDL과 pick_ik 기본 설정이 같은 pathological 종료에 닿았다는 것은 측정됐다. 그래서 “KDL만의 수치 IK 특성인가”는 약해졌다. “Cartesian path가 복수해 중 연속 branch를 강제하지 않는가”는 이 검토의 질문이다. 두 solver의 같은 종료만으로 그 질문이 증명되지는 않는다.

## 1. 이전 joint로 branch를 고르는 API

`computeCartesianPath`의 `CartesianPrecision` overload는 `validCallback`, `KinematicsQueryOptions`, `IKCostFn`을 받는다. `consistency_limits` 인자는 없다. 구간 IK의 seed는 직전 `RobotState`다. 그 seed 전달은 이미 P-022 경로에 있었다. KDL 기록과 pick_ik 기본 설정은 그 상태에서도 먼 branch를 반환했다.

`setFromIK`에는 seed와의 원하는 거리를 받는 `consistency_limits`가 있다. 헤더는 그 벡터의 기본 길이를 주지 않는다. Cartesian 경로 API는 그 인자를 넘기지 않는다. 길이를 새로 정하면 P-022에서 출처가 없어 보류된 scalar threshold와 같다.

`IKCostFn`은 해의 비용을 평가하는 hook이다. KDL plugin 헤더는 이 overload를 구현하지 않는다. 비어 있지 않은 cost는 `KinematicsBase` 기본 구현이 바로 false를 반환한다. 이 solver에서 cost로 branch를 고를 수 없다.

pick_ik 1.1.2는 cost overload를 구현한다. 넘긴 `IKCostFn`은 가중 goal로 들어가고, `cost_threshold` 기본값 0.001을 넘으면 pose가 맞아도 해가 아니다. `memetic_stop_on_first_solution` 기본값은 true다. 이것은 복수해를 이전 q와의 거리만으로 정렬해 고르는 API가 아니다. 기본 `minimal_displacement_weight`는 0이라 그 goal은 꺼져 있다. 0보다 크게 바꾸는 것은 결과를 본 뒤의 가중치 조정이라 이 검토에서 후보로 만들지 않는다.

## 2. KDL scalar callback

callback은 해를 거절하고 다음 attempt를 계속하게 할 수 있다. 거절 거리가 없으면 포착된 9.65012 rad 해를 그 자리에서 거절하는 규칙이 되지 않는다. 그 거리 `T`의 공식 출처는 P-022에서 `NOT_DEFINED`다. 첫 중간점 실패를 바로 거절하면 0.402594 rad 뒤 한 번 subdivision으로 통과한 known-good도 거절된다.

그래서 scalar callback은 새 threshold 없이 현재 종료에 직접 대응하지 못한다. `RECONSIDER`를 `REJECTED`로 바꾸지는 않는다. 다음 실험으로 열 근거는 없다.

## 3. LMA

MoveIt 2.12.4 `moveit_kinematics/CMakeLists.txt`의 하위 디렉터리는 cached IK, IKFast, KDL, srv다. LMA 디렉터리와 plugin 설명은 없다. p620의 `ros-jazzy-moveit-kinematics` 파일 목록에도 LMA가 없다.

이 스택 안에 KDL과 branch 선택이 다른 LMA solver는 없다. 없는 구현의 구조 차이를 이유로 설치하거나 시험하지 않는다. `AVAILABILITY_NOT_ESTABLISHED`는 유지하고, qualification은 열지 않는다.

## 4. 별도 branch-selection 층

연속 branch를 고르려면 같은 pose의 복수해가 있어야 한다. IKFast template는 joint limit 안의 해를 seed 거리로 정렬한다. 이 UR5e의 공식 IKFast 생성은 이 환경에서 solver를 만들지 못했다. 그 template를 여기 대안으로 쓰지 않는다.

KDL은 한 번의 수치해이고, 실패 후 재시도는 seed 근처가 아닌 무작위 구성이다. pick_ik global 기본값은 첫 유효해에서 멈춘다. 둘 다 “이전 q에 가장 가까운 이산해”를 돌려주는 구조가 아니다.

`CartesianPrecision` 바깥에 층을 넣어도, 그 층이 다른 해를 만들 출처가 없으면 기존 중간점 검사를 한 번 더 하는 것과 같다. 그 검사는 이미 있고, pathological fixture는 `max_resolution`에서 끝났다. ProcessPath의 의미나 datum, `eef_step`, precision을 바꾸지 않는 삽입 지점만으로는 연속 해를 만들지 못한다.

## 결과

이 조사 범위에서 diagnostic Gate를 열 후보는 없다.

```text
PREVIOUS_STATE_SEED = ALREADY_USED
CONSISTENCY_LIMITS = THRESHOLD_NOT_JUSTIFIED
KDL_IK_COST = UNSUPPORTED
PICK_IK_IK_COST = GATED_BY_COST_THRESHOLD
KDL_SCALAR_CALLBACK_WITHOUT_NEW_THRESHOLD = NO
LMA_IN_MOVEIT_2_12_4 = NO
SEPARATE_BRANCH_LAYER = NO_SOLUTION_SOURCE
```

`NO_JUSTIFIABLE_CANDIDATE`는 이 검토에서 정당화된 branch-continuity 후보가 없다는 뜻이다. feasibility 전체를 `NO_JUSTIFIABLE_ALTERNATIVE`로 닫지 않는다. 그 판정은 아직 `NOT_REACHED`다.
