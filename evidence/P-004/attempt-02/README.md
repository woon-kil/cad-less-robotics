# P-004 attempt #2

Status: **FAIL**

Attempt #1 evidence는 `evidence/P-004/`에 그대로 있다. 이 디렉터리는 같은 snapshot의 재검증이다.

SHA256 `09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60`.

Waypoint는 `camera_depth_optical_frame`의 camera-frame 6D geometry waypoints다. 로봇이 실행할 waypoint가 아니다.

## 원인

19개 negative dot 중 18개는 평면 반구 정렬이 카메라 쪽 법선을 180° 되돌린 짧은 구간의 경계다. 길이 138과 119 구간은 이미 camera-facing이었고, 반대 hemisphere가 아니다. 자세한 표는 `root-cause.txt`, `normal-flips-before.csv`, `normal-segments-before.csv`다.

## 수정

표면 법선은 `n · ((0,0,0) - point) >= 0` 만 만족하도록 부호를 고른다. PCL `setViewPoint(0, 0, 0)`를 `setInputCloud` 다음에 호출한다. 이웃 전파와 법선 smoothing은 하지 않았다. 툴 +Z는 그 바깥 법선의 반대다.

## 결과

| 항목 | attempt #1 | attempt #2 |
| --- | ---: | ---: |
| surface points | 31555 | 31555 |
| raw path points | 573 | 573 |
| path length m | 1.7537877446544481 | 1.7537877446544481 |
| waypoints | 673 | 673 |
| position delta m |  | 0 |
| hemisphere violations | 0 (plane rule) | 0 (viewpoint rule) |
| negative adjacent dots | 19 | **3** |
| invalid quaternion | 0 | 0 |
| quaternion sign flips | 0 | 0 |
| mean orientation error deg | 8.8802608079660582e-09 | 6.03303976924726e-08 |
| max orientation error deg | 8.537736462515939e-07 | 1.2074182697257333e-06 |

남은 3개도 양쪽이 camera-facing이다.

- index 43, dot -0.69054071318091226, 133.67292625411935°
- index 44, dot -0.83315405710049384, 146.42411244447976°
- index 311, dot -0.13601860860959203, 97.817525203886817°

index 44 하나를 뒤집으면 viewpoint 규칙이 깨진다. index 311 이후 방향은 모서리 너머로 유지된다. 둘 다 부호 전파로 지우지 않았다.

인접 내적 min -0.8331540571004938, mean 0.9676133994678419, p05 0.8778305908595143. 최대 인접 각 146.42411244447976°.

## Spacing

target 0.002 m. mean 0.0019526813438595124 m. mean absolute error 4.731865614049394e-05 m. 리샘플 알고리즘은 바꾸지 않았다.

가장 짧은 내부 간격은 index 54, gap 0.00040948138113350887 m.

- previous: 0.69253184213356067 -0.029683930963914026 0.76348717810798794
- current: 0.69167519485019635 -0.031401798819971218 0.7629258533703982
- next: 0.69144395354096211 -0.031304886974155323 0.76324959794956393

## Determinism과 시간

3회 hash `42c19259d0c43200de39bcfb4c04c0344fab675b2092014aa41e1dc19cf58430`. 매 실행 flip 3, hemisphere violation 0.

total ms: attempt #1 1290.876, attempt #2 1307.875. 3회는 1307.875, 1292.813, 1286.137.

## Tests

gtest 19, 실패 0, skip 0. vision_path_generator 12, robot_controller 7. 추가된 검사는 4개다.

## Safety

camera→robot TF 0. MoveIt planning 0. robot execution 0.
