# P-004 Evidence

Status: **FAIL**

실패 항목은 연속 waypoint 법선의 부호 불일치 19건이다. 분류는 `NORMAL_ESTIMATION`이다. 나머지 측정 항목은 아래 실측을 만족한다. 허용 기준은 낮추지 않았다.

이 디렉터리의 waypoint는 `camera_depth_optical_frame` 기준의 camera-frame 6D geometry waypoints다. camera→base 보정이 없으므로 로봇이 실행할 waypoint가 아니다.

## 입력

Gemini 335L live stream에서 한 프레임을 캡처했다. 같은 파일을 3회 처리했다.

- topic `/camera/depth/points`
- frame `camera_depth_optical_frame`
- stamp `1792995971.582898000`
- width 254957, height 1, fields `x,y,z` float32
- point count = finite count = 254957
- SHA256 `09fa059c0ae4fbda4a1b90dcc4c769936562ad1040aa8824b370aed8e1a16a60`
- artifact `/home/woonkil/cad-less-robotics/evidence/P-004/artifacts/snapshot.pcd` (gitignored)

## 필터와 평면

| 단계 | 점수 |
| --- | ---: |
| INPUT | 254957 |
| AFTER_VOXEL | 128602 |
| AFTER_OUTLIER_REMOVAL | 128132 |
| plane inliers | 41208 |
| AFTER_PLANE_REMOVAL | 86924 |
| surface points | 31555 |

PCL 1.12 `VoxelGrid`는 이 점군의 `dx*dy*dz`가 int32를 넘어 leaf를 적용하지 않는다. 같은 2 mm leaf의 leaf-key centroid로 내렸고 `voxel_index_fallback=true`다. leaf와 단계 순서는 그대로다.

평면은 물체 쪽으로 맞춰져 있다. `a=0.0492518294983528`, `b=-0.16704475008147307`, `c=-0.98471839058752397`, `d=0.97604728613193159`. inlier ratio `0.32160584397340242`. 표면 좌표 NaN 0, Inf 0.

## 법선과 경로

- normals 31555, invalid 0, length mean `1.0000000000539628`, min `0.99999995164238398`, max `1.0000000482588003`
- plane hemisphere violations 0
- consecutive negative dots **19**
- mode `boundary`, raw path points 573, length `1.7537877446544481` m, open
- waypoints 673

19건은 이웃 법선 내적이 음수인 구간이다. 평면 반구 조건은 이미 0건 위반이다. 앞 waypoint 부호를 뒤로 전파하면 673개 중 354개의 방향이 바뀌고, 그중 138개와 119개짜리 연속 구간이 있다. 그 수정은 넣지 않았다.

## Spacing과 orientation

- target `0.002` m
- internal segments 671
- mean `0.0019526813438595124` m
- `|mean - 0.002| = 4.7318656140493937e-05` m, 0.25 mm 이내
- std `0.00013261315421247513`, min `0.00040948138113350887`, max `0.0020000000000001093`
- mean abs error `4.7318656140493937e-05` m
- max abs error `0.0015905186188664912` m
- remainder `0.0013357134610460935` m (평균에서 제외)

툴 +Z는 바깥 법선의 반대다. 롤은 툴 +Z에 수직인 평면에 경로 접선을 투영해 정한다. invalid quaternion 0, quaternion sign flip 0. orientation error mean `8.8802608079660582e-09` deg, p95 `0`, max `8.5377364625159387e-07` deg.

## Determinism과 시간

3회 waypoint 수, 경로 길이, CSV SHA256이 같다. SHA256 `3549375264079d6d9fc86152a4cf4f512512a221cc523b87d0e90e1d2923bf88`.

1회 기준 ms: voxel 41.080, outlier 335.443, plane 193.064, normal 69.688, path 267.135, waypoint 3.417, total 1290.876. 3회 total은 1290.876, 1284.066, 1287.415 ms. 카메라 약 30 Hz와 별개다.

## Topic

`/vision/waypoints` (`geometry_msgs/PoseArray`) 673개, frame `camera_depth_optical_frame`, CSV와 불일치 0. `require_transform:=false`. camera→robot TF 0, MoveIt 0, 로봇 실행 0.

## Regression

`bytesize-robotics-humble`에서 gtest 15개, 실패 0, skip 0. vision_path_generator 8, robot_controller 7. 새 테스트는 없다.

`path.svg`는 보조 그림이다. PASS 판정은 JSON, CSV, 텍스트 로그로 한다.
