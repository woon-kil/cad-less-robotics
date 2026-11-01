# pick_ik diagnostic benchmark

```text
PICK_IK_DIAGNOSTIC_BENCHMARK = NO_MECHANISM_IMPROVEMENT
PICK_IK_DIAGNOSTIC_REVIEW = ACCEPTED
PASS_MECHANISM_LEVEL_EVIDENCE = NO
PICK_IK_PATHOLOGICAL_CASE_IMPROVED = NO
PICK_IK_KNOWN_GOOD_SINGLE_RUN_PRESERVED = YES
PICK_IK_KNOWN_GOOD_STABILITY = NOT_ESTABLISHED
TIME_BUDGET_LIMITED_FAILURE = NO
TIMEOUT_RAISED_AFTER_RESULT = NO
PARAMETER_RETUNE_AFTER_RESULT = NO

FULL_741_WAYPOINT_ACCEPTANCE_EXECUTED = NO
JUSTIFIABLE_ALTERNATIVE_FOUND = NO
FEASIBILITY_VERDICT = NOT_REACHED
BRANCH_CONTINUITY_ALTERNATIVE_REVIEW = NOT_OPENED
LMA_INSTALL_AND_TEST = NOT_OPENED
NEXT_ALLOWED_WORK = DECIDE_BRANCH_CONTINUITY_ALTERNATIVE_REVIEW

P023 = NOT_OPENED
P024 = NOT_OPENED
ALTERNATIVE_IMPLEMENTATION_GATE = NOT_OPENED
```

질문은 하나다. P-022에서 관측된 같은 입력에서 pick_ik이 KDL의 pathological joint-branch divergence를 피하면서 known-good를 보존하는가.

비교 단위는 KDL과 같은 joint delta가 아니다. IK 성공, joint limit, seed 대비 최대 joint delta, endpoint FK 잔차, joint 중간점 FK와 Cartesian 중간점의 차, subdivision, `max_resolution`, `CartesianPrecision` 결과다.

## 고정 조건

프로그램은 `scripts/pick_ik_benchmark.cpp`다. 호스트는 p620, ROS 2 Jazzy, MoveIt 2.12.4, UR5e `ur_manipulator`, `base_link` → `tool0`이다. pick_ik은 qualification과 같은 추출 deb의 `pick_ik/PickIkPlugin`이다.

```text
PICK_IK_MODE = global
MINIMAL_DISPLACEMENT_WEIGHT = 0
PARAMETER_OVERRIDE = NONE
PICK_IK_TIMEOUT_S = 0.005
KDL_TIMEOUT_S = 0.5
CARTESIAN_PRECISION_M = 0.001
CARTESIAN_PRECISION_RAD = 0.01
MAX_RESOLUTION = 1e-5
INITIAL_INTERVAL_WIDTH = 0.5
TRIAL_COUNT = 1
```

`CartesianPrecision`은 중간점 FK가 기준을 넘으면 그 Cartesian 중간 pose의 IK를 호출하고 양쪽 자식을 다시 검사한다. 폭이 1e-5보다 작으면 IK 없이 `MAX_RESOLUTION`이다. 시작 폭 0.5는 포착된 pathological step의 폭과 같다. pick_ik의 endpoint IK와 중간점 IK는 모두 0.005 s다. 실패 뒤 timeout을 올리지 않았다.

GOOD-A와 GOOD-B의 명령 자세는 capture에 없다. 그 둘의 target은 기록된 KDL 결과 joint의 FK다. PATHOLOGICAL target은 interval 22의 기록된 `cartesian_end`다. 순서는 x, y, z, qx, qy, qz, qw다. fixture joint는 capture에 남은 반올림 값이다. 그래서 GOOD-A의 재계산 delta는 0.05938 rad이고, 원래 기록 0.0593845 rad와 소수점 아래에서 다르다.

`KDL_RECORDED`는 저장된 endpoint joint를 쓰고, 중간점 IK만 KDL 0.5 s로 다시 호출한다. `KDL_REPLAY`는 endpoint IK도 새로 호출한다. KDL은 시도가 반복되면 seed를 흔들므로 replay는 기록의 재현이 아니다.

## 수정된 검사의 한 번 측정

| Fixture | Solver | IK | joint delta rad | endpoint pos m | endpoint ori rad | first mid pos m | first mid ori rad | midpoint IK | depth | max_resolution | precision |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| GOOD-A | KDL recorded | 1 | 0.05938 | 0 | 0 | 4.985e-5 | 1.988e-4 | 0 | 0 | 0 | PASS |
| GOOD-A | KDL replay | 1 | 0.059380 | 1.249e-7 | 3.737e-8 | 4.987e-5 | 1.989e-4 | 0 | 0 | 0 | PASS |
| GOOD-A | pick_ik | 1 | 0.059646 | 4.858e-4 | 3.420e-4 | 2.001e-4 | 2.174e-4 | 0 | 0 | 0 | PASS |
| GOOD-B | KDL recorded | 1 | 0.40260 | 0 | 0 | 0.00229769 | 0.00561525 | 1 | 1 | 0 | PASS |
| GOOD-B | KDL replay | 1 | 0.402600 | 7.645e-8 | 2.418e-9 | 0.00229765 | 0.00561525 | 1 | 1 | 0 | PASS |
| GOOD-B | pick_ik | 1 | 0.402324 | 4.907e-5 | 1.610e-5 | 0.00227859 | 0.00561168 | 1 | 1 | 0 | PASS |
| PATHOLOGICAL | KDL recorded | 1 | 9.65012 | 1.128e-5 | 1.123e-5 | 0.934952 | 3.09003 | 16 | 16 | 1 | FAIL |
| PATHOLOGICAL | KDL replay | 1 | 6.26583 | 3.304e-10 | 2.848e-11 | 0.520000 | 3.14140 | 16 | 16 | 1 | FAIL |
| PATHOLOGICAL | pick_ik | 1 | 9.18005 | 8.017e-5 | 8.670e-5 | 0.492134 | 3.12402 | 16 | 16 | 1 | FAIL |

모든 해는 joint limit 안이었다. pick_ik 중간점 IK 16번은 0.005 s 안에서 성공했다. timeout 실패는 없다.

GOOD-A는 세 solver 모두 첫 중간점에서 바로 PASS다. GOOD-B는 세 solver 모두 첫 중간점 위치가 0.001 m를 넘고, 중간점 IK 한 번 뒤 PASS다. 이 한 번의 측정에서는 known-good 형태가 유지됐다.

PATHOLOGICAL의 pick_ik 사슬은 다음이다.

```text
endpoint IK success
joint delta 9.18005 rad
midpoint position error 0.492134 m
midpoint orientation error 3.12402 rad
16 midpoint IK successes inside 0.005 s
depth 16
max_resolution
CartesianPrecision FAIL
```

KDL 기록 사슬과 같은 종료다. joint delta가 9.65012에서 9.18005로 줄어든 것은 개선이 아니다. KDL replay도 6.26583 rad에서 같은 `max_resolution`으로 끝났다.

## 앞선 호출

중간점 IK가 빠져 있던 앞 빌드가 같은 입력에 pick_ik을 한 번 호출했다. 그 재귀 결과는 쓰지 않는다. endpoint와 첫 중간점만 남는다.

```text
GOOD-A joint delta 0.059645831768703417 rad
GOOD-A first midpoint 0.000200071 m / 0.000217350 rad
GOOD-B joint delta 9.714731329240289 rad
GOOD-B first midpoint 1.129075280349361 m / 3.084265294999370 rad
PATHOLOGICAL joint delta 6.292697410172863 rad
PATHOLOGICAL first midpoint 0.331722431380811 m / 2.916873730417493 rad
```

GOOD-A는 이후 측정과 같은 해였다. GOOD-B는 같은 입력에서 먼 branch가 나왔다. 그 해의 `CartesianPrecision` 완료는 측정하지 않았다. 수정된 한 번의 PASS는 그 호출을 지우지 않는다. 그 앞 호출을 보고 timeout을 바꾸거나 다시 골라 성공시키지 않았다.

## 판정 경계

검토는 `NO_MECHANISM_IMPROVEMENT`를 수용했다. KDL 기록과 pick_ik 기본 설정은 같은 종료에 닿았다. 먼 joint branch, 중간점 FK와 Cartesian 중간점의 불일치, 재귀 subdivision, `max_resolution`, `CartesianPrecision` FAIL이다. 위치 오차 0.934952 m에서 0.492134 m, joint delta 9.65012 rad에서 9.18005 rad로 줄어든 것은 mechanism 개선이 아니다.

known-good는 둘로 나눈다. 수정된 단일 실행에서는 GOOD-A와 GOOD-B가 보존됐다. solution selection stability는 입증되지 않았다. GOOD-B 직전 호출의 9.71473 rad, 1.12908 m를 유지한다.

5 ms를 시간 부족으로 분리할 관측은 없다. 741 waypoint는 실행하지 않았다. `minimal_displacement_weight`와 timeout은 결과를 본 뒤 바꾸지 않았다. `JUSTIFIABLE_ALTERNATIVE_FOUND`로 닫지 않는다. LMA 설치 시험과 branch-continuity 검토는 열지 않았다. 다음에 그 검토를 열지는 아직 결정하지 않았다.
