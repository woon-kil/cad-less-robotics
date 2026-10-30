# P-008-R1 파라미터 근거

판정은 `FAIL_SHOE_ISOLATION`이다. 아래 값은 capture_01에 적용해 본 설정이며, R2 baseline으로 동결하지 않았다. `frozen_parameters.json`은 만들지 않았다.

| 파라미터 | 값 | 근거 |
| --- | --- | --- |
| voxel_leaf_size | 0.002 m | `EXISTING_VERIFIED_PROJECT_PARAMETER`. `path_extractor.hpp` ExtractParams |
| SOR_mean_k | 30 | `EXISTING_VERIFIED_PROJECT_PARAMETER` |
| SOR_stddev | 1.0 | `EXISTING_VERIFIED_PROJECT_PARAMETER` |
| RANSAC_distance_threshold | 0.004 m | `EXISTING_VERIFIED_PROJECT_PARAMETER` |
| RANSAC iterations | 200 | `EXISTING_VERIFIED_PROJECT_PARAMETER` |
| RANSAC seed | 1 | `PROJECT_ADAPTATION`. PCL SAC에는 setSeed가 없고, 동일 입력 재현이 필요해서 mt19937 seed를 고정했다. 거리 임계값은 바꾸지 않았다 |
| min_height | 0.003 m | `EXISTING_VERIFIED_PROJECT_PARAMETER` |
| cluster_tolerance | 0.01 m | `EXISTING_VERIFIED_PROJECT_PARAMETER` |
| cluster_min_size | 50 | `EXISTING_VERIFIED_PROJECT_PARAMETER` |
| cluster_max_size | 없음 | 기존 largest-cluster 경로에도 상한이 없다 |
| 신발 envelope | 평면 길이 [0.12, 0.36] m, 폭 [0.05, 0.16] m, 높이 [0.03, 0.18] m, 중심 높이 [0.01, 0.22] m. 통과분 중 점수 최대, 동점이면 centroid x, y, z가 작은 쪽 | `GEOMETRIC_SCALE_DERIVED`. 결과를 보기 전에 고정. 실패 후 창을 바꾸지 않음 |
| PCA 부호 | 최대 고유벡터의 첫 비영 성분을 양으로. axis2는 중심과의 내적이 음이 되게. axis1 = axis2 × axis0, det < 0이면 반전 | `PROJECT_ADAPTATION`. 반복 실행의 부호 고정을 위한 규약 |
| alpha | spacing_scale = max(NN p95, voxel leaf). 후보 배율 2, 3, 4, 6, 8, 12. 가장 작은 topology-valid 값 중 area/convex ≥ 0.55, perimeter/convex ≤ 3 | `LOCAL_POINT_SPACING_DERIVED` + 선택 규칙 `PROJECT_ADAPTATION`. 선택된 성분이 작업물이 아니므로 이 alpha는 동결하지 않음 |
| 3D 재투영 | 평면 경계 정점에서 관측 점군으로의 최근접점 | `PROJECT_ADAPTATION`. support는 그 들어올림 거리. mm 합격선을 만들지 않음 |

`PathExtractor::extract()`는 법선과 waypoint까지 이어지므로 호출하지 않았다. VoxelGrid, SOR, 평면 위 점 유지, Euclidean clustering의 수치 파라미터만 기존 값을 그대로 썼다.
