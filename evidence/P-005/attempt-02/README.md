# P-005 Attempt #2 — IK feasibility diagnosis

Status: **DIAGNOSIS COMPLETE**

P-005 gate는 그대로 **FAIL**이다. 이번 단계는 고치지 않고, 두 번째 waypoint의 IK가 왜 없는지 나눴다.

P-004 waypoint, 45° 분할, attempt #1의 synthetic placement는 바꾸지 않았다. vendor `kinematics.yaml`도 바꾸지 않았다. 배치 재계산과 attempt #1 기록의 최대 차이는 0이다.

## 로봇

runtime `arm_group`은 조인트 5개다. `Arm1_Joint`–`Arm5_Joint`. 끝 링크는 `Arm5_Link`.

solver는 `kdl_kinematics_plugin/KDLKinematicsPlugin`이다. vendor timeout과 runtime `getDefaultIKTimeout()`은 둘 다 0.005 s다. `position_only_ik`는 production solver에 없다. `max_solver_iterations`와 `epsilon`은 yaml에 없어서 KDL 기본값 500, 1e-5다.

Humble KDL은 검색이 끝나도 해가 없으면 `TIMED_OUT`을 쓴다. CartToJnt가 목표에 못 가도 같은 코드다. timeout인지 아닌지는 이 코드가 아니라 시간을 늘렸을 때 해가 생기는지로 봤다.

## 세 case 공통

init seed, 0.005 s, full pose 순서는 모두 `SFFFFFFFFFF`다. 첫 점은 풀리고, 바로 다음 점부터 끝까지 안 풀린다.

첫 실패 index는 62, 313, 573이다. init, previous, up, down, 그리고 position-only가 찾은 관절해를 seed로 다시 넣어도 0 s부터 1.0 s까지 full pose는 `NO_IK`다. 1.0 s 질의의 측정 시간은 1.00 s다. 시간을 더 줘서 해가 나타난 경우는 없다.

충돌로 거절된 해는 없다. 반환된 해의 관절 한계 위반은 0이다. init seed도 한계 안이다. CASE-01 init의 가장 작은 여유는 `Arm3_Joint` lower margin 0.0708 rad이다.

## 위치와 자세

위치만 푸는 진단은 두 가지다. production solver는 그대로 두고, 같은 KDL 클래스를 `position_only_ik`로 따로 초기화했다. 이 설정은 vendor 파일에 쓰지 않았다. 추가로 Jacobian 위치 행만 쓰는 damped least squares를 돌렸다. Jacobian 위치 행은 0–2행이다.

| Case | source | KDL position error | tool-axis error | full orientation error |
| --- | ---: | ---: | ---: | ---: |
| CASE-01 | 62 | 3.1406604290537334e-08 m | 0.15720768367533811° | 4.6171200059097615° |
| CASE-02 | 313 | 2.1141026640376736e-08 m | 8.1451573541390019° | 25.650729259279512° |
| CASE-03 | 573 | 2.6569073246163241e-08 m | 0.28489063658170261° | 5.1557440325172248° |

모두 `IK_FOUND_CLEAR`다. DLS init도 같은 방향으로 위치 오차 1e-4 m 아래다.

이 자세 오차는 orientation weight를 0으로 둔 solver가 seed 근처 자세를 유지한 결과다. tool axis를 만족하라고 푼 결과가 아니다.

## Roll과 tool axis

요청한 tool +Z를 고정하고 roll을 15° 간격, 이어서 5° 간격으로 돌렸다. 15°는 0.1 s, seed는 init과 previous다. 5°는 init, 0.05 s다. 세 case 모두 360행이 `NO_IK`다.

tool axis를 2°, 5°, 10° 기울이고 방위 90° 간격, roll 30° 간격으로 본 표본도 432행 모두 `NO_IK`다. 이 표본의 timeout은 0.1 s이고, 원뿔 전체를 채운 것은 아니다.

그래서 `position reachable + tool-axis reachable + full quaternion unreachable`은 성립하지 않는다. 위치는 닿는다. full quaternion은 1 s 안에서도 안 나온다. tool axis를 고정하고 roll만 바꾼 표본도 full-pose KDL이 받지 않았다.

## CASE별 직전 점과의 차이

CASE-01 source 61→62는 위치만 0.002 m 움직이고 tool axis, quaternion, roll 변화는 0이다. 같은 자세가 2 mm 옆에서 실패한다.

CASE-02 source 312→313은 위치 0.0018649753602956276 m, tool axis 7.8663038540358468°, quaternion 21.671022246536552°, roll 20.176961503742032°다.

CASE-03 source 572→573은 위치 0.0019654428145425588 m, tool axis 0°, quaternion 9.5189080733156448°, roll -9.5189080733157745°다.

세 case의 label set은 같다. primary는 `TOOL_AXIS_INFEASIBLE`다.

## 다음에 하지 않은 것

timeout을 올리지 않았다. solver를 바꾸지 않았다. production orientation을 완화하지 않았다. P-006을 시작하지 않았다.
