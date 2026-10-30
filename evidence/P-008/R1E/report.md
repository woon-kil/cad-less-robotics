# P-008-R1E — Support-Plane Object Extraction

판정: `FAIL_CLUSTERING`

viewpoint prism은 매트 위 신발을 되살렸다. 같은 1 cm clustering은 그 신발을 매트와 한 component로 묶었다. height와 tolerance는 결과를 보고 바꾸지 않았다. 파라미터는 동결하지 않았다.

## 1. PCL

1.12.1. `ConvexHull` + `ExtractPolygonalPrismData` + `EuclideanClusterExtraction`.

## 2. input SHA

`51b5af9855e01e67610f0a24bc427ea0d257f68c34a6829ab709084e2517a928`

P-008A capture_01과 같다. capture_02~05는 열지 않았다.

## 3. RANSAC plane

계수는 a -0.049105, b -0.719139, c -0.693129, d 0.495848이다. R1이 majority orientation 뒤에 기록한 값의 부호 반대이고 같은 평면이다. inlier 20709. 계수를 하드코드하지 않았다. 추출은 이 계수 부호를 쓰지 않고 hull과 viewpoint를 쓴다.

## 4. hull dimension

2.

## 5. hull vertex

30. 면적 1.1837 m².

## 6. viewpoint

`(0, 0, 0)`. `camera_depth_optical_frame`의 광학 중심이다.

## 7. height convention

진단 원판의 viewpoint signed height는 최소 0.0063 m, 중앙값 0.0515 m, 최대 0.1161 m다. 전부 양수이고 0.18 m 안이다. PCL이 viewpoint 쪽을 양으로 둔 결과와 맞다. 음수라서 한계를 뒤집지 않았다.

## 8. min / max

최소 0 m. PCL 1.12.1 생성자 기본값. 최대 0.18 m. `GEOMETRIC_SCALE_DERIVED`. R1의 작업물 높이 상한이다.

## 9. before prism

2861. R1D의 non-plane 원판 수와 같다.

## 10. after prism

2861.

## 11. retention

1.0.

## 12. non-plane

238806.

## 13. prism selected

76248. selection ratio 0.319.

## 14. rejected

162558. 그림 04에서 바닥과 전경이 빠지고, 신발 자리는 빠지지 않는다. 배경 전체를 되살린 것은 아니다. hull이 1.18 m²라 소파와 작업대 일부는 각기둥 안에 남는다. hull은 줄이지 않았다.

## 15. cluster 수

18. tolerance 0.01 m, min size 50.

## 16. shoe cluster id

1. 진단 원판 점이 가장 많은 component다. size envelope로 고르지 않았다.

## 17. point count

23734. 원판 점 2505개가 여기 있다. 나머지 356개는 cluster 7에 있다.

## 18. bbox / dimensions

centroid (-0.1060, 0.0387, 0.6374). bbox min (-0.4487, -0.2018, 0.5033), max (0.1992, 0.2093, 0.8040). dimensions 0.648 × 0.411 × 0.301 m.

## 19. membership

원판 대비 cluster 1의 비율은 0.876이다. 그림 06에서 노란 신발은 파란 component의 일부이고, 그 component는 원형 매트와 오른쪽 구조로 이어진다. 신발만의 component는 아니다.

## 20. plane-sign invariance

synthetic self-test PASS. 같은 점군에서 평면 계수 부호를 뒤집어도 inlier와 prism 결과가 같다. 먼 쪽에 점이 더 많아도 viewpoint (0,0,0)은 카메라 쪽 물체만 남긴다. viewpoint를 평면 뒤로 옮기면 그 선택이 뒤집힌다.

## 21. unit tests

`p008r1e_prism --self-test`: `PLANE_SIGN_INVARIANCE PASS`, `VIEWPOINT PASS`, `PRISM_FILTER PASS`.

## 22. regression

p620 Humble `colcon test --packages-select vision_path_generator robot_controller` 이후 `colcon test-result`: 55 tests, 0 failures. 기존 패키지 소스는 수정하지 않았다.

## 23. figures

`evidence/P-008/R1E/figures/01_plane_hull.png`부터 `06_shoe_cluster.png`.

## 24. evidence

`evidence/P-008/R1E/`

## 25. commit SHA

push 후 보고에 적는다.

## 26. git status

커밋 후 보고.

## 27. verdict

도구의 사전 수치 화면은 retention, selection ratio, membership만 보고 `PASS_EXTRACTION_FIX`를 찍는다. 게이트 판정은 `FAIL_CLUSTERING`이다. 신발을 담은 component가 매트와 연결된 0.65 m 물체이기 때문이다.

숫자 화면을 통과시키려고 tolerance나 height를 조정하지 않았다.
