# P-008-R1D — Shoe Isolation Failure Diagnosis

판정: `PASS_DIAGNOSIS`

질문은 작업대 운동화 점이 preprocessing 이후 어디에 있고, 왜 하나의 shoe cluster가 되지 않았는가이다. capture_01만 사용했다. R1 파라미터는 바꾸지 않았다. 새 segmentation은 구현하지 않았다.

## 1. PCD fields

`FIELDS x y z`, `SIZE 4 4 4`, `TYPE F F F`, `WIDTH 341698`, `HEIGHT 1`, `POINTS 341698`, `DATA binary`.

## 2. COLOR_IN_POINTCLOUD

`NO`. capture metadata의 `point_step` 16은 PointCloud2 레이아웃이다. 저장된 PCD에는 rgb 필드가 없다.

## 3. correspondence

진단 영역은 R1 지지평면 inlier의 중심으로 둔 반지름 0.08 m 원판이다. inlier는 반지름 약 0.06 m 안이 비어 있다. 0.06 m와 0.10 m에서도 height gate를 통과한 점은 0이다. 이 원판은 cluster 번호로 고른 것이 아니다.

점군은 depth 내부 파라미터로 `camera_depth_optical_frame` 픽셀에 투영된다. 341698점 전부 848×480 안이다. 유효 depth가 있는 337715점에서 `|z×1000 − depth|` 중앙값은 1 mm, p95는 4 mm다.

## 4. RGB-depth registration

`UNREGISTERED_RGB_DIAGNOSTIC_LIMITATION`. color는 `camera_color_optical_frame` 1280×720이고 depth/cloud는 `camera_depth_optical_frame` 848×480이다. 이번 capture의 camera info에는 검증된 color↔depth 변환이 없다. color 사진은 장면 참고일 뿐, 픽셀을 cloud 좌표로 쓰지 않았다.

## 5–10. 원판 안 점의 단계별 잔존

반지름 0.08 m.

| 단계 | 원판 안 점 | height gate(≥0.003 m)를 통과 |
| --- | ---: | ---: |
| RAW | 4220 | 0 |
| Voxel | 2989 | 0 |
| SOR | 2941 | 0 |
| 평면 inlier로 제거 | 80 | — |
| height filter 이후 | 0 | 0 |
| clustering | 0 | 0 |

Voxel 전체 개수 287710은 R1과 같다. SOR은 원판에서 48점만 제거했다. 평면 inlier는 80점이다. 나머지 2861점은 평면에서 4 mm보다 멀리 있는데도 height gate에서 전부 제거됐다.

원판의 signed distance 중앙값은 -0.0468 m, 범위는 -0.1134 m에서 -0.0026 m다. 카메라 쪽 높이는 약 0.3 cm에서 11.3 cm다.

non-inlier signed sum은 +11232.6이다. 원판은 -144.8, 그 바깥은 +11377.1이다. 유지된 반공간은 배경 다수 쪽이고, 매트 중심 물체는 반대쪽에 있다.

## 11. id 0

35742점. 매트 중심까지 최소 0.176 m, 중앙 0.340 m. 유지된 점군 가운데 원판과 가장 가깝지만 최소 간격은 0.092 m다.

## 12. id 3

13279점, 0.688×0.273×0.118 m. 매트 중심까지 최소 0.272 m. 원판과의 최소 간격은 0.204 m. 작업대 신발이 들어 있는 component가 아니다. R1 직후의 그 추정은 측정으로 기각됐다.

## 13. id 8

1644점. R1이 envelope로 선택한 성분. 매트 중심까지 최소 0.605 m, 원판과의 최소 간격 0.534 m. 운동화가 아니다. boundary 재현은 이 진단에서 다시 하지 않았다.

## 14. id 9

1620점. envelope를 통과했지만 점 수가 id 8보다 적어 선택되지 않았다. 매트 중심까지 최소 0.569 m, 원판과의 간격 0.511 m. 평면 UV에서 id 8 옆에 있다.

## 15. membership

`SHOE_REMOVED_BY_PREPROCESSING`

매트 중심의 볼록한 물체는 RAW부터 SOR까지 남아 있고, clustering 입력에는 0점이다.

## 16. merge / fragment / removal

Euclidean component로 합쳐지거나 여러 cluster로 나뉘지 않았다. height gate가 원판을 통째로 제거했다. 평면 inlier로 흡수된 것은 2941점 중 80점이다.

## 17. bridge

cluster tolerance 0.01 m는 유지했다. 원판에서 유지된 점군까지 최소 0.092 m, 중앙 0.170 m다. 1 cm 연결은 없다. 가장 가까운 큰 component는 id 0이고, id 3은 0.204 m, id 8은 0.534 m 떨어져 있다.

## 18. primary root cause

`HEIGHT_HALFSPACE_REMOVAL`

## 19. secondary root cause

`MAJORITY_NONINLIER_ORIENTATION`

기존 `orientTowardObject`는 non-inlier 전체의 signed sum이 양이 되도록 법선을 둔다. 그 합은 매트 바깥 배경이 결정한다. 그 다음 `keepAbove`가 0.003 m 이상을 남기므로 카메라 쪽의 신발은 들어가지 못한다.

## 20. 다음 후보

구현하지 않았다.

- envelope를 넓혀 id 3을 고르는 방법은 신발을 포함하지 않는다. id 3은 원판에서 0.20 m 밖이다.
- cluster tolerance를 바꾸는 방법도 신발을 되돌리지 않는다. 신발 점은 clustering 전에 없다. 간격도 0.09 m라 0.01 m tolerance의 경계가 아니다.
- alpha, PCA, boundary는 이번 실패의 원인이 아니다.
- PCL Euclidean clustering은 공간적으로 분리된 부분을 나누는 방법이다. 이번 장면에서는 그 입력에 신발이 없었다. 출처: Point Cloud Library cluster extraction tutorial, https://pointclouds.org/documentation/tutorials/cluster_extraction.html
- RANSAC 평면 검출 자체는 유지한다. 검출된 평면은 매트가 있는 지지면이고, 신발은 그 면의 카메라 쪽에 있다. 출처: Point Cloud Library walkthrough, https://pointclouds.org/documentation/tutorials/walkthrough.html
- `RegionGrowingRGB`는 고르지 않는다. point cloud에 color가 없고, RGB↔cloud registration도 없다. 출처: https://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.html
- normal 기반 `RegionGrowing`도 다음 단계로 고르지 않는다. 같은 smooth surface를 성장시키는 방법이고, 현재 유지된 점군에는 신발이 없어 입력 조건을 만족하지 않는다. 출처: https://pointclouds.org/documentation/tutorials/region_growing_segmentation.html
- 다음에 볼 조건은 segmentation 교체가 아니라, 이미 검출된 지지평면의 유지 반공간이다. 다수 non-inlier 합으로 법선 방향을 정하는 기존 규약이 신발을 버리는 쪽을 골랐다.

## 21. evidence

`evidence/P-008/R1D/`

그림: `figures/depth_cluster_overlay.png`, `figures/depth_shoe_zoom.png`, `figures/plane_uv_clusters.png`. 노란 점이 진단 원판, 주황이 id 0, 빨강이 id 3, 흰색이 id 8, 하늘색이 id 9다.

## 22. commit SHA

push 후 보고에 적는다. 작성 직전 HEAD는 `a51b030a5b1dcbe626530ebffe7ec25ca5c0ef16`.

## 23. git status

커밋 후 보고.
