# P-022-AB 실험 A·B

```text
GATE = P-022-AB
SCOPE = EXPERIMENT_A_AND_B
EXPERIMENT_C = NOT_EXECUTED
P022_REOPENED = NO
P022_STATUS = CLOSED_FAIL
P022_FINAL_GATE = FAIL_CARTESIAN_API
P023 = NOT_OPENED
P024 = NOT_OPENED
REVIEWER_CLOSED = false
CONCLUSION = 시험한 범위에서 개선 미확인
RUN20_CAUSE = UNCONFIRMED
GEOMETRIC_UNREACHABILITY_PROVEN = NO
```

P-022와 `cartesian_runs.json`은 바꾸지 않았다. 이 디렉터리의 실행은 그 판정을 덮지 않는다. 시작 IK가 timeout 안에서 실패한 것과, 계측 20회에서 `20/33`이 다시 나오지 않은 것은 도달 불가능성의 증거가 아니다.

## 제어 가능한 항목

설치본은 `ros-jazzy-moveit-core` `2.12.4-1noble.20260915.180823`, `ros-jazzy-moveit-kinematics` `2.12.4-1noble.20260915.201542`다. 헤더와 MoveIt 태그 2.12.4 소스를 읽었다. 설치된 `libmoveit_kdl_kinematics_plugin.so`에 `New random configuration` 문자열이 있다.

세 항목은 서로 다르다.

| 항목 | 실제 제어 | 이 실행 |
| --- | --- | --- |
| 첫 IK seed | `setFromIK` 직전 `RobotState` 관절. `RobotState::setFromIK`는 `searchPositionIK`에 그 값을 seed로 넘긴다. | `setToDefaultValues()`가 여섯 관절을 모두 0으로 둔다. 별도 고정값 축이 아니다. |
| 난수 재시드 끄기 | `searchPositionIK`는 첫 시도가 실패하고 timeout이 남아 있으면 `getRandomConfiguration`을 호출한다. 공개 attempt 상한은 없다. `KinematicsQueryOptions`에도 없다. | `RESEED_DISABLE_AT_0_005S = NOT_AVAILABLE`. 우회 구현으로 끄지 않았다. |
| 난수 재시드 고정 | 재시드 RNG는 플러그인 내부 `state_`의 `getRandomNumberGenerator()`다. 생성자는 seed 인자가 없고, 호출자가 RNG를 갈아 끼우는 공개 API가 없다. | `RANDOM_RESEED_SEED_CONTROL = NOT_AVAILABLE`. |

`getPositionIK`는 timeout 0.0으로 `searchPositionIK`를 한 번만 돌리게 하는 기존 API다. 그 호출은 timeout을 0.005 s가 아니게 바꾸므로 실행하지 않았다. `consistency_limits`는 재시드를 끄지 않고 범위만 줄이므로 쓰지 않았다.

기준선과 같이 `setDefaultIKTimeout`은 호출하지 않았다. 관측된 group default는 0.5 s다. 시작 IK timeout만 0.005 s다.

## 기준선 보존

실행 전후 SHA-256이 같다.

```text
cad_process_path.json       04dcd594f09a18d2d996166e9feb1bfe9f20c0282758e732e94ccf59faf28114
robot_planning_input.json   23266fbcd5da565955378585fd38a893670465ca446c2ce10d0a8362bd0dd436
cartesian_runs.json         a1795dbb2a86637779b11c34f2f139210a096dfbaaf60485f2a81cbad176eacd
complex_surface_coupon.step 2a791f83b37439ef59c425422599765c26d273b30d56f624dedfbd9a8d3bb3fa
```

비교 기준은 계속 `evidence/P-022/cartesian_runs.json`이다. 그 파일의 실패 run은 5·7·12, run 20 fraction은 0.606061이다. 이번 실행으로 그 파일을 바꾸지 않았다.

모델 파일은 기존 p620 경로 `/tmp/p022/src/ur5e.urdf`, `ur5e.srdf`, `process_runs.txt`다. `process_runs.txt` SHA-256은 `c1d1538cb0bff30e6741c72ba15346186e71e2b7396183a135c1145e16c6273c`이고 저장소의 `evidence/P-022/process_runs.txt`와 같다.

## 재현 명령

호스트는 p620, ROS 2 Jazzy다. 이 Mac에서 MoveIt을 돌리지 않았다.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S /tmp/p022-ab/src -B /tmp/p022-ab/build
cmake --build /tmp/p022-ab/build -j4
/tmp/p022-ab/build/p022ab a \
  /tmp/p022/src/ur5e.urdf /tmp/p022/src/ur5e.srdf \
  /tmp/p022/src/process_runs.txt /tmp/p022-ab/experiment_a.json
/tmp/p022-ab/build/p022ab b \
  /tmp/p022/src/ur5e.urdf /tmp/p022/src/ur5e.srdf \
  /tmp/p022/src/process_runs.txt /tmp/p022-ab/experiment_b.json
/tmp/p022-ab/build/p022ab_trace \
  /tmp/p022/src/ur5e.urdf /tmp/p022/src/ur5e.srdf \
  /tmp/p022/src/process_runs.txt /tmp/p022-ab/experiment_b_instrumented.json
```

소스는 `scripts/p022ab.cpp`, `scripts/p022ab_trace.cpp`다. CMake 목록은 이 디렉터리의 `CMakeLists.txt`와 같다. 궤적 파일은 쓰지 않았다.

## 변경 파일

- `scripts/p022ab.cpp`
- `scripts/p022ab_trace.cpp`
- `evidence/PROJECT-ROADMAP.md` 현재 상태 헤더
- `evidence/P-022-AB/` 이 보고서와 원시 JSON

`scripts/p022_plan.cpp`와 `evidence/P-022/`는 수정하지 않았다.

## 실험 A

조건은 하나다. `SET_TO_DEFAULT_VALUES`와 설치본 `searchPositionIK`다. 재시드를 끈 조건과 RNG seed를 고정한 조건은 `NOT_AVAILABLE`이라 실행하지 않았다. 반복 5, 시작 timeout 0.005 s, 경로 IK는 group default 0.5 s다.

시작 IK가 성공한 호출은 모두 위치 오차 0.001 m 이하, 방향 오차 0.01 rad 이하, URDF limit 안이었다. 그 합격은 run의 Cartesian 합격이 아니다. 시작 IK 실패의 fraction은 null이다.

| 반복 | 시작 IK 실패 run | fraction < 0.95 |
| --- | --- | --- |
| 0 | 5, 14, 18, 19 | 없음 |
| 1 | 3, 5, 8, 10, 11 | 6 = 0.2727272727272727, 9 = 0.6372549019607844, 19 = 0.3333333333333333 |
| 2 | 2, 5, 15, 17 | 8 = 0.20588235294117646, 9 = 0.08823529411764706 |
| 3 | 3, 5, 15, 17, 19 | 4 = 0.5757575757575758 |
| 4 | 3, 13, 15 | 7 = 0.6470588235294118, 12 = 0.2647058823529412 |

105개 run 중 시작 IK 실패 21, fraction 0.95 미만 8, 나머지 76은 fraction 1이다. 다섯 반복 모두 21개 run이 전부 fraction 0.95 이상이지 않다. 실패 run은 반복마다 달라지고, 기준선 파일의 5·7·12와 같지 않다. run 20은 이 다섯 번 모두 fraction 1이다. 관절·시간·pose는 `experiment_a.json`에 있다.

## 실험 B

라이브러리 `computeCartesianPath`로 run 20을 20회 돌렸다. 계측 복사본이 아니다.

| 반복 | 시작 IK | fraction | 20/33 |
| --- | --- | --- | --- |
| 0 | 실패 | null | 아니오 |
| 1 | 실패 | null | 아니오 |
| 2–6 | 성공 | 1 | 아니오 |
| 7 | 성공 | 0.6060606060606061 | 예 |
| 8–13 | 성공 | 1 | 아니오 |
| 14 | 성공 | 0.6060606060606061 | 예 |
| 15–19 | 성공 | 1 | 아니오 |

`20/33`에 해당하는 fraction은 20회 중 2회다. pose 수는 34다. 이 두 실행은 계측 전이라 실패 구간이 없다. 시작 IK가 성공한 실행은 FK 0.001 m / 0.01 rad와 URDF limit을 통과했다. 원시는 `experiment_b.json`이다.

계측은 `scripts/p022_trace_interval.cpp`와 같은 2.12.4 순서 복사본이다. 라이브러리 `computeCartesianPath`는 호출하지 않는다. run 20을 다시 20회 돌렸다. `20/33`은 0회다. 19회는 fraction 1, 1회(trial 13)는 시작 IK 실패다. 실패 구간 기록은 비어 있다. `experiment_b_instrumented.json`의 `cause_status`는 `INSTRUMENTED_20_OVER_33_NOT_REPRODUCED`다.

그래서 run 20이 약 60.6%에서 멈춘 직접 이유는 미확정이다. run 12의 `9/34`를 이 원인으로 쓰지 않는다. 계측 20회에서 재현되지 않은 것을 도달 불가능으로 쓰지 않는다.

## 확인된 사실

- 첫 IK seed는 이 호출에서 여섯 관절 0이다. 난수 재시드는 그 seed와 다른 제어다.
- 0.005 s를 유지한 채 재시드를 끄거나 RNG seed를 고정하는 공개 API는 이 설치본에 없다.
- 같은 기준선 정책의 5회에서 시작 IK 실패 run은 고정되지 않았다. 21개 run이 모두 fraction 0.95 이상인 반복은 없다.
- 라이브러리 경로의 run 20은 20회 중 2회 `20/33`이었다.
- 그 뒤 순서 복사 계측 20회에서는 `20/33`이 없었다.
- P-022 입력 해시 네 개는 실행 뒤에도 같다.

## 미확정

- 라이브러리 실행 7번과 14번이 어느 waypoint 구간에서 0이 됐는지. fraction에서 역산하지 않았다.
- 그 정지가 `setFromIK` false인지 `validateAndImproveInterval` false인지.
- 9/34에서 본 관절 차이가 그 두 실행에도 있는지.
- 재시드를 끄면 시작 IK 실패가 같은 run에 고정되는지는 API가 없어 측정하지 않았다.

## 실험 C

실행하지 않았다. A·B가 새 종료 조건을 보여 주지 않았으므로 구현 후보는 고르지 않는다.

비교만 남긴다.

1. 재시드 제거를 경로 전체에 적용하는 안. A에서 0.005 s 재시드 제거가 `NOT_AVAILABLE`이다. timeout을 0으로 바꾸는 `getPositionIK`는 이 비교의 후보가 아니다.
2. 중간점 실패 해를 버리고 다른 IK를 찾는 안. 계측이 `20/33`을 다시 만들지 못해, 기존 callback 검토와 다른 종료를 보여 주지 않았다. 구현하지 않는다.
3. solver 교체. 이번 승인 밖이다.
4. datum, 목표 자세, ProcessPath 변경. 이번 승인 밖이다.

다음 권고는 하나다. 이 범위에서는 개선이 확인되지 않았고, run 20의 정지 원인도 확정되지 않았다. timeout을 고르거나 solver를 바꾸거나 P-022를 다시 열지 않는다.
