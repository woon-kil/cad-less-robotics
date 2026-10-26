# P-004 attempt #4

Status: **PASS CANDIDATE**

Attempt #1, #2, #3의 파일은 덮어쓰지 않았다. Snapshot SHA256은 `09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60`이다.

Waypoint는 `camera_depth_optical_frame`의 camera-frame 6D geometry waypoints다. 로봇이 실행할 waypoint가 아니다. 모서리의 `requires_motion_handling`은 기록만 한다. orientation smoothing, retract, re-approach, MoveIt transition은 만들지 않았다.

## 신뢰도

법선을 복사한 원점의 k=20 이웃 공분산에서 고유값을 오름차순 `λ0 ≤ λ1 ≤ λ2`로 둔다.

```text
metric: normal_axis_separation
formula: (λ1 - λ0) / λ1
threshold: 0.5
reason: λ1이 λ0의 두 배 이상일 때만 법선 축이 다음 주성분과 분리된다.
```

임계값은 이 비율의 2배 분리로 코드에 고정했다. waypoint 44만 통과하도록 나중에 맞추지 않았다. 측정 분포는 673개 중 61개(9.1%)가 0.5 미만이다. 중앙값은 0.840이고 0.5는 p05(0.413)와 중앙값 사이다.

|  | separation |
| --- | ---: |
| min | 0.055170493903981509 |
| p01 | 0.15110613777745982 |
| p05 | 0.41324597117828532 |
| median | 0.83982587263118313 |
| p95 | 0.97632219629881989 |
| max | 0.9924266377427533 |

44의 separation은 0.19064645273297784다. 더 작은 값은 9개라 작은 쪽 10번째다. 최솟값 0.055는 309–311 이웃이고, 그 점은 복구하지 않았다.

## 분류와 복구

|  | 개수 |
| --- | ---: |
| total | 673 |
| reliable | 612 |
| unreliable | 61 |
| recovered | 54 |
| unrecoverable | 7 |

복구는 한 번만 한다. 지지점은 원래 reliable인 법선만 쓰고, 이번 패스에서 복구한 점은 지지점으로 쓰지 않는다. 왼쪽과 오른쪽이 모두 있고 둘의 단위 법선 내적이 0보다 크면 합을 카메라 원점 쪽으로 향하게 한다. 내적이 0 이하면 `UNRECOVERABLE_EDGE`로 두고 원래 법선을 유지한다. 한쪽이 없으면 `UNRECOVERABLE_SUPPORT`다. 이웃 하나를 복사하지 않고, 부호만 뒤집지 않는다.

복구하지 않은 7개:

- 0, 1, 2: 왼쪽에 reliable 지지가 없다. 원래 법선을 유지했다.
- 309, 310, 311, 312: 양쪽 지지는 308과 313이고 내적은 -0.3849327475546089다. 평균을 만들지 않았다.

## 불연속

|  | 개수 |
| --- | ---: |
| negative adjacency | 1 |
| geometric discontinuity | 1 |
| normal estimation outlier | 0 |
| sign ambiguity | 0 |
| path ordering error | 0 |
| unknown | 0 |

AC-11은 설명되지 않은 불연속이 0인지를 본다. 기하 불연속 1건은 별도 기록이고 P-004 실패가 아니다.

## Waypoint 44

Attempt #2 법선 `(0.6897690296173096, 0.06521081179380417, -0.7210868000984192)`, camera dot `0.07608951669244246`. 인접 내적은 `n43·n44 = -0.69054071318091226`, `n44·n45 = -0.83315405710049384`.

분리도 0.1906으로 unreliable이다. 이 snapshot에서 가장 가까운 원래 reliable 지지는 39와 47이고 내적은 0.8487968562423934다. 둘의 평균을 시점 쪽으로 맞춘 결과가 저장된 법선과 내적 1이다. `index == 44` 조건은 없다.

복구 후 법선 `(-0.8833685173868717, -0.46377436221640583, -0.0676269431549744)`. `n43·n44 = 1`, `n44·n45 = 1`, camera dot `0.6496711780085473`.

43과 45도 분리도가 0.5 미만이라 같은 지지 평균을 받았다. 44만 고른 결과가 아니다.

## Edge 311

|  | 값 |
| --- | ---: |
| normal angle before | 97.817525203886817° |
| normal angle after | 97.817525203886817° |
| tangent angle | 61.748500293980115° |
| classification | GEOMETRIC_DISCONTINUITY |
| requires_motion_handling | true |

각이 줄지 않았다. `edge-preservation.txt`, `discontinuity-classification.csv`.

## 회귀

위치 최대 차이는 attempt #2 waypoint와 0이다. 바뀐 법선은 54개다.

surface 31555, raw path 573, length 1.7537877446544481 m, waypoints 673.

spacing target 0.002 m, mean 0.0019526813438595124 m, mean abs error 4.7318656140493937e-05 m. index 54의 0.00040948138113350887 m 구간은 그대로다. resampling은 바꾸지 않았다.

hemisphere violations 0. invalid normal 0. invalid quaternion 0. quaternion sign flips 0.

orientation error deg: mean 5.906178900562033e-08, p95 8.537736462515939e-07, max 1.2074182697257333e-06.

3회 hash `b9a254590dcc260b4556dd38c366bdd98a64efe99bd5eee4d614b39834041021`. 위치, 자세, 신뢰도 열이 같은 CSV다.

1회 총 시간 1307.574546 ms. 3회는 1307.574546, 1291.432185, 1282.509883 ms. Attempt #2 1회는 1307.875 ms였다.

## 테스트

gtest 17 (`vision_path_generator`) + 7 (`robot_controller`) = 24. failed 0, skipped 0. 이전 19개에 5개를 더했다. `colcon test-result` 디렉터리 요약은 같은 날의 이전 `Test.xml`까지 세서 18과 8로 나온다. 실패와 건너뜀은 0이다. 정본은 gtest `[ PASSED ]` 줄이다.

## 그림

`normal-angle-before-after.svg`. 회색은 attempt #2, 파랑은 attempt #4의 인접 법선 각이다. 35–55 구간의 90° 초과는 없어졌고 300–320의 모서리는 남아 있다.
