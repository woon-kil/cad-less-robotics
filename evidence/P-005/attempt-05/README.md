# P-005 Attempt #5 — Joint-Space Trajectory

상태: **PASS CANDIDATE**. P-005 gate은 계속 FAIL이다. process fidelity 합격 기준은 정하지 않았다. P-006은 열지 않았다.

Attempt #3의 collision-free, joint-limit-valid, continuous projected joint state를 `q0 → q1 → … → q10`으로 직접 이었다. `q → FK → Pose → IK → q'`는 trajectory 구성에 쓰지 않았다. Cartesian planner도 호출하지 않았다.

## 입력

Snapshot `09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60`.

Cases는 CASE-01 61–71, CASE-02 312–322, CASE-03 572–582. synthetic placement와 projection 설정은 Attempt #3/#4와 같다. 위치 허용오차 0.0005 m, null half-count 4, step 0.1 rad, correction 12, seed `init` `up` `down`.

3회 재현에서 projection 관절 차이와 metric 차이는 0이다.

관절 보간 간격 `max_joint_interpolation_step_rad`는 0.05 rad로 실행 전에 고정했다. 이 값은 branch 문턱 0.5 rad의 1/10이고, Attempt #3 최대 관절 간격 0.4 rad를 8구간으로 나눈다. 결과를 보고 바꾸지 않았다.

시간 배치는 `motion_planner_node::retime`과 같다. `dt = max(|Δq|_max / 0.4, 0.05)`. 새 속도 스케줄러를 만들지 않았다. vendor acceleration limit는 꺼져 있으므로 계산된 가속도는 hardware acceptance가 아니다.

관절 순서는 runtime model과 같다. `Arm1_Joint` `Arm2_Joint` `Arm3_Joint` `Arm4_Joint` `Arm5_Joint`.

## Joint waypoint

33개. 연속 관절 간격의 branch discontinuity는 0이다. 문턱은 0.5 rad다.

| case | waypoints | interpolated | mean / p95 / max step (rad) | limits | collision |
|---|---:|---:|---|---:|---:|
| CASE-01 | 11 | 22 | 0.08362195648953569 / 0.20999999999999983 / 0.30000000000000004 | 0 | 0 |
| CASE-02 | 11 | 26 | 0.10700106660016169 / 0.29667079785890943 / 0.30000000000000004 | 0 | 0 |
| CASE-03 | 11 | 31 | 0.13271026535835512 / 0.30999999999999983 / 0.4 | 0 | 0 |

중간 상태의 self collision과 planning-scene collision은 0이다. 관절 한계 위반도 0이다.

## Time / velocity

| case | duration (s) | max velocity (rad/s) | finite-difference acceleration (rad/s²) |
|---|---:|---:|---:|
| CASE-01 | 2.1197341337093105 | 0.4000000000000004 | 6.399999999999985 |
| CASE-02 | 2.7112396933017004 | 0.40000000000000013 | 7.999999999999987 |
| CASE-03 | 3.3177566339588793 | 0.40000000000000036 | 7.609661355979963 |

속도는 상한 0.4 rad/s에 붙는다. 인쇄값이 0.4보다 큰 부분은 `retime`의 나눗셈 잔차이고, 기존 `withinJointVelocity`의 0.1% 검사 안에 있다. 가속도는 기록만 했다.

## FK

관절 waypoint를 다시 FK하면 projected pose와 수치 오차다.

| case | vs projected position (m) | vs projected orientation (deg) |
|---|---:|---:|
| CASE-01 | 0 | 0 |
| CASE-02 | 0 | 1.7075472925031877e-06 |
| CASE-03 | 0 | 0 |

Desired geometry와의 오차는 projection fidelity다. planner 오차가 아니다. Attempt #3과 같다.

| case | position mean / p95 / max (m) | tool mean / p95 / max (deg) | \|roll\| mean / p95 / max (deg) | full mean / p95 / max (deg) |
|---|---|---|---|---|
| CASE-01 | 0.000137111 / 0.000364187 / 0.000439313 | 18.416522 / 24.811336 / 24.964300 | 73.663143 / 133.029753 / 133.062297 | 76.312836 / 134.110660 / 134.143947 |
| CASE-02 | 0.000081308 / 0.000391008 / 0.000448180 | 12.037494 / 19.598066 / 19.785432 | 48.569831 / 83.035626 / 93.716692 | 50.404612 / 84.739002 / 94.909813 |
| CASE-03 | 0.000187809 / 0.000433817 / 0.000484918 | 1.822902 / 5.251949 / 6.178463 | 32.206077 / 60.205658 / 62.772162 | 32.298644 / 60.286182 / 62.785029 |

공정 합격 각도는 정하지 않았다.

## Cartesian chord

관절 보간이 만드는 Cartesian 움직임이다. 직선 chord에서 벗어난 거리와, 구간 시작 tool +Z에서 가장 멀리 돌아간 각이다.

| case | max chord deviation (m) | max tool-axis excursion (deg) | max segment path length (m) |
|---|---:|---:|---:|
| CASE-01 | 0.00017629035529124522 | 20.480771156177354 | 0.0023083891112458932 |
| CASE-02 | 0.0002045334104926817 | 16.462925974577054 | 0.0022451253566373565 |
| CASE-03 | 0.0000502962444596898 | 23.042878715270863 | 0.002348246456361885 |

위치는 chord에서 0.21 mm 아래다. tool axis는 한 구간에서 최대 23.042878715270863° 돌아간다. 그래서 joint trajectory가 가능해도 surface fidelity는 받아들이지 않는다.

## Attempt #4와 비교

| case | Cartesian planner | fraction | joint trajectory | fidelity |
|---|---|---:|---|---|
| CASE-01 | PLANNING_INFEASIBLE | 0.030303030303030304 | JOINT_TRAJECTORY_FEASIBLE | PROCESS_FIDELITY_NOT_YET_ACCEPTED |
| CASE-02 | PLANNING_INFEASIBLE | 0.049783549783549784 | JOINT_TRAJECTORY_FEASIBLE | PROCESS_FIDELITY_NOT_YET_ACCEPTED |
| CASE-03 | PLANNING_INFEASIBLE | 0.11412783592795282 | JOINT_TRAJECTORY_FEASIBLE | PROCESS_FIDELITY_NOT_YET_ACCEPTED |

Projected pose의 Cartesian 계획은 실패했다. 같은 projected joint state의 관절 궤적은 세 case 모두 충돌 없이 이어진다.

## 재현과 테스트

같은 설정 3회. projected joint, interpolation 개수, collision, joint limit, trajectory class, duration이 같다.

테스트는 vision 17, waypoint_prep 7, motion_contract 8, capability_adapter 19. 합계 51, 실패 0, skip 0. 이전 baseline은 43이다.

## 안전 경계

execute 0. controller send 0. trajectory IK 0. calibration 0. fake TF 0. 실제 로봇 명령 0. P-004 변경 0. waypoint 삭제 0.

Projection 재현에 쓴 `searchPositionIK`는 297회다. 3회 × 3 case × 11점 × seed 3개다. 선택된 q를 궤적으로 만드는 단계에서는 IK를 호출하지 않았다.

`/joint_states`에 0을 내는 pump는 scene monitor용이다. trajectory는 publish하지 않았다.
