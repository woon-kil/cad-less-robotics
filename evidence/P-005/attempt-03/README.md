# P-005 Attempt #3 — Robot Capability Adapter

상태: **DIAGNOSIS COMPLETE**. P-005 gate은 계속 FAIL이다. production planner에는 넣지 않았다.

P-004 desired pose는 읽기만 했다. `computeCartesianPath` 호출은 0이다.

## 무엇을 재었나

DesiredGeometryPose를 덮어쓰지 않고, 같은 XYZ에서 DOFBOT 5관절이 만드는 자세를 따로 골랐다.

- 위치 허용오차는 탐색 전에 **0.0005 m (0.5 mm)** 로 고정했다. 결과를 보고 넓히지 않았다.
- 탐색은 위치 Jacobian 영공간의 격자다. 반경 4칸, 간격 0.1 rad, 보정 반복 12. seed는 SRDF `init`, `up`, `down`.
- full-pose KDL은 timeout 0의 한 번만 호출했다. 난수 재시작은 없다.
- 후보 순위는 위치 허용오차, tool-axis 오차, full orientation 오차다. 이 순위는 진단용이며 공정 합격 기준이 아니다.
- 1°는 EXACT_POSE라는 수치 일치 라벨이다. 공정 합격 각도가 아니다.
- 한 관절 |Δq|가 0.5 rad를 넘으면 `IK_BRANCH_DISCONTINUITY`다. 이 문턱도 검출기이지 합격 기준이 아니다.

Jacobian의 위치 행은 0–2다.

## 결과 요약

33점 모두 0.5 mm 안에 충돌 없고 관절 한계 안인 해가 있었다. EXACT_POSE는 각 case의 첫 점뿐이고, 그 점은 synthetic placement의 init FK 앵커다. 나머지 30점은 NEAR_POSE다. POSITION_ONLY, NO_POSITION_SOLUTION, COLLISION, JOINT_LIMIT은 0이다. 선택 해의 한계 위반은 0이다. 기각된 충돌 후보는 0이다.

같은 설정을 한 프로세스에서 3회 돌렸다. 분류, 관절, 오차의 차이는 0이다.

| case | position mean / p95 / max (m) | tool-axis mean / p95 / max (deg) | roll mean / p95 / max (deg) | full mean / p95 / max (deg) | joint step mean / max (rad) | branch |
|---|---|---|---|---|---|---|
| CASE-01 61–71 | 0.000137111 / 0.000364187 / 0.000439313 | 18.416522 / 24.811336 / 24.964300 | 73.663143 / 133.029753 / 133.062297 | 76.312836 / 134.110660 / 134.143947 | 0.083622 / 0.300000 | 0 |
| CASE-02 312–322 | 0.000081308 / 0.000391008 / 0.000448180 | 12.037494 / 19.598066 / 19.785432 | 48.569831 / 83.035626 / 93.716692 | 50.404612 / 84.739002 / 94.909813 | 0.107001 / 0.300000 | 0 |
| CASE-03 572–582 | 0.000187809 / 0.000433817 / 0.000484918 | 1.822902 / 5.251949 / 6.178463 | 32.206077 / 60.205658 / 62.772162 | 32.298644 / 60.286182 / 62.785029 | 0.132710 / 0.400000 | 0 |

표는 `capability-summary.json`을 소수 6자리로 반올림한 것이다. roll 평균은 절대값이다. 부호 있는 roll은 CSV에 있다.

## 세 case에서 보인 것

CASE-01의 61과 62는 desired orientation이 같다. desired full step은 0°다. 61 executable은 init FK라 오차가 0이다. 62의 tool-axis 오차는 0.143999657°, roll은 −3.972400064°, full은 3.975008159°다. 2 mm 이동에서 tool axis는 거의 유지되고 roll이 약 4° 움직였다. 창의 뒷부분에서는 desired tool axis도 변하고, executable tool-axis 오차는 약 20–25°까지 올라간다.

CASE-02 313은 desired tool step 7.866303854°, desired roll step 20.208847205°, desired full step 21.671022247°다. 같은 점의 capability 오차는 tool 1.313558744°, roll −23.582000958°, full 23.618039270°다. 기하가 요구한 회전과 로봇이 남긴 오차는 별도 열에 있다. tool axis는 기하 변화보다 작게 남았고, roll 오차는 기하 roll 변화와 비슷한 크기다.

CASE-03 573은 desired tool step 0°, desired roll step −9.518908073°다. capability 오차는 tool 0.292853787°, roll 5.754406071°, full 5.761846967°다. tool axis는 유지에 가깝고, 손실은 roll이다. 창 전체의 roll 오차 최대는 62.772161594°다.

## 연속성

0.5 rad 검출기에 걸린 구간은 없다. 가장 큰 한 관절 변화는 CASE-01 62→63의 0.300000000 rad, CASE-02 320→321의 0.300000000 rad, CASE-03 574→575의 0.400000000 rad이다. 0.4 rad는 검출 문턱 아래다.

## 해석

XYZ는 33점 모두 0.5 mm 안에 남았다. tool axis는 case마다 다르다. CASE-03 최대는 6.178463351°이고 CASE-01 최대는 24.964299593°다. 선택 기준이 tool axis를 먼저 줄였을 때 남은 큰 오차는 roll이다. 관절 해는 0.5 rad 검출 기준으로는 연속이다. 이 투영은 위치를 유지하는 근사로는 존재하고, roll까지 포함한 6D 자세로는 부족하다. production adapter로 쓸지는 이 숫자를 본 뒤의 다음 실험에서 정한다. 이번 측정은 그 실험을 실행하지 않았다.

## 안전 경계

P-004 변경 0. motion contract 변경 0. `computeCartesianPath` 0. calibration 0. fake TF 0. execute 0. controller send 0.
