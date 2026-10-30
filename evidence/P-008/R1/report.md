# P-008-R1 — Shoe Geometry Pipeline / Reference Capture

판정: `FAIL_SHOE_ISOLATION`

capture_01만 사용했다. capture_02~05는 열지 않았다. 파라미터는 동결하지 않았다. P-008-R2와 P-009는 열지 않았다. 로봇 명령 0, servo write 0.

## 1. capture_01 SHA

`51b5af9855e01e67610f0a24bc427ea0d257f68c34a6829ab709084e2517a928`

Mac 원본, P-008A `manifest.sha256` 3행, p620 실행 복사본이 같다. `BLOCKED_INPUT_PROVENANCE_MISMATCH`가 아니다.

## 2. 기존 코드 재사용

`PathExtractor::extract()`는 법선과 waypoint 생성까지 이어지므로 호출하지 않았다. VoxelGrid 0.002 m, SOR mean_k 30 / stddev 1.0, RANSAC 거리 0.004 m / 200회, min_height 0.003 m, cluster tolerance 0.01 m / min size 50은 `path_extractor.hpp`의 기존 값이다. 동일 단계를 `scripts/p008r1_shoe_boundary.cpp`에 둔 이유는 extract()가 이번 Gate에서 금지한 법선·경로까지 가기 때문이다. RANSAC seed 1은 재현을 위한 추가이고 거리 임계값은 그대로다.

## 3. raw point count

341698. finite 전부. frame `camera_depth_optical_frame`. 단위 meter.

## 4. Voxel

leaf 0.002 m. PCL VoxelGrid. index fallback 없음. 341698 → 287710. reduction ratio 0.158.

## 5. SOR

287710 → 259515. 제거 28195. mean_k 30. stddev 1.0.

## 6. Plane

계수 a 0.049105, b 0.719139, c 0.693129, d -0.495848. inlier 20709. inlier ratio 0.0798. 거리 0.004 m. 잔여 238806. 높이 필터 후 124933. 그림 `03_support_plane`의 회색은 작업대 위 원형 매트다. 법선은 영상 +Y(아래)와 +Z(카메라에서 멀어지는 방향) 성분이 크다. 장면의 다수 점이 매트 위 신발보다 배경에 있어서, 기존 orient-toward-remaining 규약이 작업물 쪽이 아니라 배경 쪽을 양으로 두었다. 거리 임계값은 결과 보고 바꾸지 않았다.

## 7. cluster 수

27.

## 8. 주요 cluster

| id | points | 평면 길이 m | 폭 m | 높이 m | 중심 높이 m | envelope | 선택 |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 0 | 35742 | 0.849 | 0.472 | 0.177 | 0.059 | 탈락 |  |
| 1 | 23530 | 0.473 | 0.421 | 0.416 | 0.251 | 탈락 |  |
| 3 | 13279 | 0.688 | 0.273 | 0.118 | 0.048 | 탈락 |  |
| 8 | 1644 | 0.143 | 0.098 | 0.056 | 0.026 | 통과 | 선택 |
| 9 | 1620 | 0.154 | 0.106 | 0.096 | 0.083 | 통과 |  |

전체 표는 `clusters.csv`.

## 9. 선정 rule과 실패

사전 규칙: 평면 길이 [0.12, 0.36] m, 폭 [0.05, 0.16] m, 높이 [0.03, 0.18] m, 중심 높이 [0.01, 0.22] m인 cluster 중 점 수가 가장 큰 것. 사람이 cluster 번호를 고르지 않았다.

통과는 id 8과 9뿐이고, 점 수 때문에 id 8이 선택됐다. 깊이 내부 파라미터(fx=fy 410.414, cx 423.669, cy 240.994)로 중심을 깊이 영상에 투영하면 id 8은 (77, 227), id 9는 (144, 217)이다. 둘 다 848×480 영상의 왼쪽 가장자리다. 검출된 지지 평면 중심은 (374, 277)로 매트 근처다. 그 중심에 가장 가까운 실질 cluster는 id 3(13279점, 길이 0.688 m)이라 envelope 밖이다. 작업대 신발은 분리된 단일 cluster가 아니다. 창을 넓혀 id 3을 넣지 않았다. `MANUAL_SELECTION`도 쓰지 않았다.

## 10. PCA

선택된 성분(작업물 아님)의 값이다. centroid (-0.702480, -0.028626, 0.832053). eigenvalue 8.858e-6, 5.120e-4, 1.071e-3. axis_0 (0.021451, 0.625581, -0.779864). axis_1 (0.943813, 0.244633, 0.222198). axis_2 (0.329783, -0.740812, -0.585184). Python `pca_frame`과 축 차이는 1e-15 이하다. 작업물 frame은 성립하지 않았다.

## 11. 투영 범위

선택 성분 1644점. u [-0.066874, 0.076127] m. v [-0.058646, 0.045406] m.

## 12. local sampling

NN median 0.001586 m, p90 0.001985 m, p95 0.002363 m. spacing_scale은 p95.

## 13. alpha 후보

배율 2는 loop 2라 탈락. 배율 3, 4, 6, 8, 12는 topology를 통과했다. 규칙상 가장 작은 통과값이 배율 3이다. 이 비교는 capture_01의 선택된 성분 안에서만 했다. 작업물이 아니므로 동결하지 않는다.

## 14. frozen alpha

없음. `PARAMETERS_FROZEN = NO`.

## 15–19. 선택 성분의 경계 수치

정점 116. loop 1. closed true. ordered true. self-intersection 0. 둘레 0.419495 m. signed area 0.008282 m². 이 수치는 작업대 신발 경계가 아니다. `BOUNDARY_TOPOLOGY = FAIL`.

## 20. 3D 재투영

평면 경계 정점에서 선택 성분 점군으로의 최근접점. `boundary_3d.csv`.

## 21. support

선택 성분에 대한 boundary-to-observed-cloud support distance. mean 0.003068 m, median 0.002735 m, p95 0.006370 m, max 0.008753 m. 정확도 합격선을 만들지 않았다.

## 22. spacing

mean 0.003948 m, median 0.003338 m, p95 0.010295 m, max 0.012546 m. smoothing 하지 않았다.

## 23. worst gap

index 7. A (-0.729686, -0.051277, 0.842000). B (-0.737238, -0.050668, 0.832000). 거리 0.012546 m.

## 24. determinism

동일 capture_01을 3회 실행. `boundary_3d.csv`, `boundary_2d.csv`, `pca_frame.json`, `clusters.csv`, `preprocessing_metrics.json`의 SHA256이 3회 모두 같다. stdout도 selected_cluster=8, vertices=116, alpha=0.00708958로 같다. 재현된 것은 잘못된 선택이다. multi-frame repeatability가 아니다.

## 25. unit / regression

Mac `PYTHONPATH=scripts python3 -m unittest scripts/test_p008r1_metrics.py`: 5/5. synthetic만 사용. p620 Humble `colcon test --packages-select vision_path_generator robot_controller`: vision_path_generator 17/17, robot_controller 7+8+19 전부 PASS. 기존 패키지 소스는 이 Gate에서 수정하지 않았다.

## 26. figures

`evidence/P-008/R1/figures/01_raw_cloud.png`부터 `12_boundary_gap_diagnostics.png`. `05_clusters`의 흰색이 id 8이며, 장면 왼쪽의 작은 조각이다.

## 27. frozen parameter artifact

만들지 않음. 실패 설정을 R2 입력으로 남기지 않는다.

## 28. evidence path

`evidence/P-008/R1/`

## 29. commit SHA

이 파일을 담는 커밋의 SHA는 push 후 보고에 적는다. 작성 직전 HEAD는 `7ac7436d75e51fb4c85baa1b39f7ac6779459b91`.

## 30. git status

커밋 후 보고.

## 31. final verdict

`FAIL_SHOE_ISOLATION`
