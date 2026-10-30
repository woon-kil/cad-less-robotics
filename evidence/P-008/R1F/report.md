# P-008-R1F — Euclidean Connectivity Diagnosis

판정은 `PASS_DIAGNOSIS`다. 거리만으로 신발을 환경과 나누는 구간은 없다. `DISTANCE_SEPARATION = NOT_FEASIBLE`.

R1E prism을 다시 실행했다. 선택 76248, 진단 원판 2861, 10 mm component 23734점과 원판 2505점이 그대로다. cluster 수는 min size 50 기준으로 18이다.

진단 원판은 R1D/R1E와 같은 정의다. RANSAC inlier 중심에서 평면 거리 0.08 m 미만이다. ground-truth segmentation이 아니다.

10 mm에서 추적점 component는 23734점이고 원판 점은 2505개다. 나머지 원판 356점은 다른 component에 있다. 그림 02에서 이 component는 원형 매트와 그 위의 신발, 그리고 매트 위쪽 구조다.

환경 합류는 새 ROI가 아니다. 추적 component 점 수가 `2 * 2861`을 넘는 첫 tolerance다. 그 값은 3.6 mm다. 직전 3.5 mm에서는 2485점, 3.6 mm에서는 19729점이다. 경계를 넘는 간선 길이는 3.58 mm이고, 양끝의 viewpoint 높이는 둘 다 21.1 mm다. 위치는 원판 bounding box 밖, 매트 위쪽 테두리다. 그림 03의 빨간 덩어리가 매트 본체이고, 파란 호가 그 위쪽 테두리다.

신발 원판 전체가 한 component가 되는 tolerance는 26.7 mm다. 이 값은 전체 prism radius graph 기준이다. seed 점만 쓰면 44.6 mm다. 어느 쪽도 3.6 mm보다 작지 않다. 간격은 없다.

실행 전에 고정한 sweep은 2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 30 mm다. 결과를 보고 칸을 추가하지 않았다.

| mm | cluster 수 | component 점 | 원판 소속 | 환경 합류 | 원판 fragment |
| --- | --- | --- | --- | --- | --- |
| 2 | 63 | 1 | 0.000 | 아니오 | 889 |
| 3 | 52 | 820 | 0.148 | 아니오 | 17 |
| 4 | 21 | 23603 | 0.873 | 예 | 5 |
| 5 | 22 | 23629 | 0.876 | 예 | 2 |
| 6 | 20 | 23645 | 0.876 | 예 | 2 |
| 7 | 20 | 23692 | 0.876 | 예 | 2 |
| 8 | 19 | 23692 | 0.876 | 예 | 2 |
| 10 | 18 | 23734 | 0.876 | 예 | 2 |
| 12 | 17 | 23738 | 0.876 | 예 | 2 |
| 15 | 17 | 23747 | 0.876 | 예 | 2 |
| 20 | 17 | 23829 | 0.876 | 예 | 2 |
| 30 | 11 | 24185 | 1.000 | 예 | 1 |

4 mm부터 매트가 이미 붙어 있고, 원판 전체는 30 mm 칸에서야 한 fragment가 된다.

법선은 clustering에 쓰지 않았다. `NormalEstimation` k=20, viewpoint (0,0,0). seed의 10번째 이웃 거리 중앙값은 4.01 mm이고, 99.97%가 10 mm 안에 이웃 10개를 가진다. 밀도는 충분하다. bridge 양끝 법선 각은 0.0768 rad, 약 4.4도다. 원판 내부 최근접 법선 각의 중앙값은 0.0795 rad, 95백분위는 0.267 rad다. bridge 각이 내부 중앙값 이하라 입력 적합성은 `UNSUITABLE`이다.

self-test는 간격이 있는 synthetic grid를 `FEASIBLE`, 더 가까운 grid를 `NOT_FEASIBLE`로 확인했다. `vision_path_generator`와 `robot_controller` 테스트는 55건, 실패 0이다. 패키지 소스는 바꾸지 않았다.

커밋 SHA는 push 뒤 완료 보고에 적는다.
