# P-008-R1E 방법

분류: `OFFICIAL_IMPLEMENTATION_VERIFIED`

사용한 구현은 PCL 1.12.1의 `pcl::ConvexHull`, `pcl::ExtractPolygonalPrismData`, `pcl::EuclideanClusterExtraction`이다.

컨테이너 헤더 `extract_polygonal_prism_data.hpp`의 `segment`는 hull 점의 PCA 법선을 만든 뒤 `(viewpoint - hull[0]) · normal < 0`이면 법선을 뒤집는다. non-inlier 개수로 부호를 정하지 않는다. 생성자 기본 최소 높이는 0, 기본 viewpoint는 (0, 0, 0)이다.

이 경로는 `scripts/p008r1_shoe_boundary.cpp`의 `orientTowardObject`를 호출하지 않는다. 그 함수는 삭제하지 않았다.

높이 최소 0 m는 PCL 기본값이다. 높이 최대 0.18 m는 R1에서 작업물 높이 상한으로 이미 둔 값이며, 이번 실행 결과를 보고 바꾸지 않았다.

검토자가 제시한 공식 용례는 objects supported by a plane, `pcd_select_object_plane.cpp`, 그리고 ICRA 2011 PCL 논문의 planar-surface clustering이다.
