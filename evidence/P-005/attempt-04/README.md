# P-005 Attempt #4 — Projected Pose Planning

상태: **FAIL**. P-005 gate은 계속 FAIL이다. P-006은 열지 않았다.

Attempt #3 projection은 그대로 재현됐다. 그 FK를 Cartesian planner에 넣어도 `min_fraction` 0.95에 닿지 않았다. planner 성공과 process fidelity는 분리했고, fidelity 합격 각도는 정하지 않았다.

## 입력

Desired geometry는 P-004 그대로다. planner에 넣은 것은 Attempt #3 selected joint의 FK, 즉 projected executable pose다.

탐색 설정은 Attempt #3과 같다. 위치 허용오차 0.0005 m, null half-count 4, step 0.1 rad, correction 12, seed `init` `up` `down`.

3회 재현에서 관절 차이와 metric 차이는 0이다. `attempt3_match yes`.

Cartesian 파라미터는 Attempt #1과 같다. `eef_step` 0.002, `jump_threshold` 1.0, 충돌 검사 켜짐, `min_fraction` 0.95, 관절 속도 상한 0.4 rad/s. 결과를 보고 바꾸지 않았다.

각 case의 시작 관절은 첫 projected joint state다. 세 case 모두 SRDF `init`과 일치한다.

## Attempt #1과 비교

| case | desired 6D fraction | projected fraction | points | planning | fidelity |
|---|---:|---:|---:|---|---|
| CASE-01 | 0.090909090909090912 | 0.030303030303030304 | 2 | PLANNING_INFEASIBLE | PROCESS_FIDELITY_NOT_YET_ACCEPTED |
| CASE-02 | 0.090909090909090912 | 0.049783549783549784 | 2 | PLANNING_INFEASIBLE | PROCESS_FIDELITY_NOT_YET_ACCEPTED |
| CASE-03 | 0.090909090909090912 | 0.11412783592795282 | 12 | PLANNING_INFEASIBLE | PROCESS_FIDELITY_NOT_YET_ACCEPTED |

세 case 모두 0.95 미만이다. CASE-01과 CASE-02는 Attempt #1보다 낮다. CASE-03만 1/11보다 높고, 그래도 0.95에 못 미친다.

계획 시간은 0.005536481 s, 0.005225880 s, 0.006053045 s다.

## Projection fidelity

이 오차는 planner fraction과 별도다. 표는 소수 6자리 반올림이고, roll은 절대값 평균이다.

| case | position mean / p95 / max (m) | tool-axis mean / p95 / max (deg) | roll mean / p95 / max (deg) | full mean / p95 / max (deg) |
|---|---|---|---|---|
| CASE-01 | 0.000137111 / 0.000364187 / 0.000439313 | 18.416522 / 24.811336 / 24.964300 | 73.663143 / 133.029753 / 133.062297 | 76.312836 / 134.110660 / 134.143947 |
| CASE-02 | 0.000081308 / 0.000391008 / 0.000448180 | 12.037494 / 19.598066 / 19.785432 | 48.569831 / 83.035626 / 93.716692 | 50.404612 / 84.739002 / 94.909813 |
| CASE-03 | 0.000187809 / 0.000433817 / 0.000484918 | 1.822902 / 5.251949 / 6.178463 | 32.206077 / 60.205658 / 62.772162 | 32.298644 / 60.286182 / 62.785029 |

CASE-01의 tool-axis 최대 24.964300°는 fraction과 같이 남아 있다. 이것을 합격으로 바꾸지 않았다.

## Trajectory FK

시작 FK는 projected pose와 desired anchor에 대해 오차 0이다.

끝나지 않은 궤적의 마지막 상태를 마지막 요청점과 비교하면:

| case | vs projected position / orientation | vs desired position / tool / roll |
|---|---|---|
| CASE-01 | 0.012030406 m / 30.316639° | 0.012031368 m / 48.648324° / -72.015493° |
| CASE-02 | 0.016223254 m / 26.181218° | 0.016541698 m / 22.602662° / 0.162875° |
| CASE-03 | 0.017097854 m / 28.417422° | 0.017145122 m / 8.645092° / -18.364884° |

충돌 상태 0, 관절 한계 위반 0, 속도 위반 0. CASE-03의 측정 최대 속도는 0.131478448 rad/s로 0.4 rad/s 아래다. CASE-01과 CASE-02는 점이 2개라 측정 속도가 0이다.

## 해석

Projected pose를 넣어도 Cartesian fraction은 0.95가 되지 않는다. 그래서 이번 결과는 “원래 문제가 6D pose와 5자유도 능력 사이의 interface뿐이었다”를 입증하지 않는다. adapter 뒤에도 Cartesian interpolation이 경로를 끝내지 못한다.

동시에 process fidelity는 받아들이지 않는다. planner가 나중에 성공해도 CASE-01의 tool-axis 약 25°와 roll 평균 약 74°는 그대로 보고해야 한다.

## 안전 경계

execute 0. controller send 0. calibration 0. fake TF 0. P-004 변경 0. waypoint 삭제 0. scale 1. `min_fraction` 유지.
