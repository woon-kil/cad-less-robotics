# P-004 attempt #3

Status: **DIAGNOSIS COMPLETE**

Attempt #1과 #2의 파일은 덮어쓰지 않았다. 생산 법선·경로 알고리즘은 바꾸지 않았다. 같은 snapshot을 기존 `PathExtractor`로 다시 읽었고, waypoint 수는 673, mode는 `boundary`다.

SHA256 `09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60`.

이 waypoint는 `camera_depth_optical_frame`의 camera-frame 6D geometry waypoints다. 로봇이 실행할 waypoint가 아니다. 법선이 90° 넘게 바뀌는 구간을 로봇 궤적으로 써도 된다는 뜻이 아니다.

## 분류

| index | 각 | 접선 각 | 간격 | camera dot | 분류 |
| --- | ---: | ---: | ---: | --- | --- |
| 43 | 133.67° | 35.57° | 1.893 mm | 0.550 / 0.076 | NORMAL_ESTIMATION_OUTLIER |
| 44 | 146.42° | 21.81° | 1.988 mm | 0.076 / 0.446 | NORMAL_ESTIMATION_OUTLIER |
| 311 | 97.82° | 61.75° | 1.899 mm | 0.450 / 0.732 | GEOMETRIC_DISCONTINUITY |

43과 44는 한 사건이다. `n43·n45 = 0.96969955004281871`이라 44 양쪽이 서로 맞다. 44의 k=20 PCA는 저장된 법선과 0.00016°로 같지만 `λ0/λ1 = 0.809`, planarity `0.0595`라 평면 축이 분리되지 않는다. 44만 뒤집으면 두 내적은 양수가 되고 camera dot은 `-0.076`이 된다.

311은 309–311이 같은 법선이고 312 이후가 그 방향과 약 98°를 유지한다. 같은 자리에서 접선이 61.75° 돈다. 어느 쪽을 뒤집어도 camera dot이 음수가 되고 그 구간의 내적 1.0 또는 0.991이 깨진다. 최근접 점의 k=20은 `λ0/λ1 = 0.945`로 섞여 있다. waypoint 중심 PCA는 저장된 법선과 99.36° 다르고 `n312`와의 내적은 0.998이다.

세 창의 camera dot 최솟값은 0보다 크다. 연속 간격 최댓값은 2.0 mm다.

## AC-11 제안

기존 `dot < 0`을 모두 sign flip으로 보면 311의 모서리를 오류로 세게 된다. 43/44는 그 반대다. 부호를 고치면 시점 조건을 깬다. 추정 이웃이 불안정한 한 점이다.

따라서 음수 내적 3개를 한 번에 허용하는 완화는 근거가 없다. 제안은 지표를 나누는 것이다.

- sign ambiguity: 뒤집어도 camera-facing이 유지되는 불연속. 이번 3개에는 없다.
- estimation outlier: 43/44. 아직 결함이다. P-004 FAIL을 유지한다.
- explained geometric discontinuity: 311. 두 camera-facing 구간과 접선 회전이 측정됐다. `unexplained_normal_discontinuities`에는 넣지 않을 수 있다.

이 정의 변경은 적용하지 않았다. 검수가 결정한다.

## 그림

`normal-discontinuity-043.svg`, `044`, `311`. 회색은 반경 20 mm 표면 점, 파란 선은 경로, 녹색은 법선, 보라 점선은 카메라 방향, 빨간 점은 문제 쌍이다. 왼쪽은 경로와 문제 법선, 오른쪽은 경로와 측면이다.

## 안전 경계

camera→robot TF 0. MoveIt 0. 로봇 실행 0. 모서리 분할이나 후퇴는 만들지 않았다.

## Tests

gtest 19, 실패 0, skip 0. 새 테스트는 없다.
